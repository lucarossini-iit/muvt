import time

import numpy as np
import rospkg
from horizon import problem
from horizon.utils import utils, kin_dyn, resampler_trajectory, plotter, mat_storer
from horizon.transcriptions import integrators
from horizon.transcriptions.transcriptor import Transcriptor
from horizon.ros.replay_trajectory import *
from horizon.solvers import solver
import matplotlib.pyplot as plt
import os, math
from itertools import filterfalse

solver_type = 'ipopt'
transcription_method = 'multiple_shooting'
transcription_opts = dict(integrator='RK4')
load_initial_guess = False
ilqr_plot_iter = False

tf = 4.0
n_nodes = 100
t_jump = (1.0, 1.5)

# load urdf
rospack = rospkg.RosPack()
cogimon_urdf_path = rospack.get_path('cogimon_urdf')
urdffile = os.path.join(cogimon_urdf_path, 'urdf', 'cogimon.urdf')
urdf = open(urdffile, 'r').read()
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

exit()

# joint names
joint_names = kindyn.joint_names()
if 'universe' in joint_names: joint_names.remove('universe')
if 'floating_base_joint' in joint_names: joint_names.remove('floating_base_joint')

# parameters
n_c = 2
n_q = kindyn.nq()
n_v = kindyn.nv()
n_f = 3
dt = tf / n_nodes

# define dynamics
prb = problem.Problem(n_nodes)  # n_nodes sono le transizioni -> nodi = n_nodes+1
q = prb.createStateVariable('q', n_q)
q_dot = prb.createStateVariable('q_dot', n_v)
q_ddot = prb.createInputVariable('q_ddot', n_v)
f_list = [prb.createInputVariable(f'f{i}', n_f) for i in range(n_c)]
x, x_dot = utils.double_integrator_with_floating_base(q, q_dot, q_ddot)
prb.setDynamics(x_dot)
prb.setDt(dt)

# use this to create a reference trajectory that can be changed on the fly without
# generating a new problem
# par = prb.createParameter()
# par.assign()

# contact map
contacts_name = ['l_sole', 'r_sole']
contact_map = dict(zip(contacts_name, f_list))

# initial state and initial guess
# TODO set initial configuration
q_init = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                   0.0, 0.9, -1.5238505,
                   0.0, 0.9, -1.5202315,
                   0.0, 0.9, -1.5300265,
                   0.0, 0.9, -1.5253125])

q.setBounds(q_init, q_init, 0)
q_dot.setBounds(np.zeros(n_v), np.zeros(n_v), 0)

q.setInitialGuess(q_init)

for f in f_list:
    f.setInitialGuess([0, 0, 36])

# transcription
if solver_type != 'ilqr':
    th = Transcriptor.make_method(transcription_method, prb, opts=transcription_opts)

# dynamic feasibility
id_fn = kin_dyn.InverseDynamics(kindyn, contact_map.keys(), cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)
tau = id_fn.call(q, q_dot, q_ddot, contact_map)
prb.createIntermediateConstraint("dynamic_feasibility", tau[:6])

# final velocity is zero
prb.createFinalConstraint('final_velocity', q_dot)

# final pose
q_tgt = q_init.copy()
# q_tgt[0] = 0
# q_tgt[5] = math.sin(math.pi / 4)
# prb.createFinalConstraint('q_fb', q[:6] - q_tgt[:6])
# prb.createFinalConstraint('q_f', q[7:] - q_tgt[7:])

prb.createFinalCost('q_f', cs.sumsqr(10 * (q[7:] - q_tgt[7:])))
# prb.createFinalCost('q_dot_f', 100*cs.sumsqr(q_dot))

# contact handling
# TODO define contact sequence
k_all = range(1, n_nodes + 1)
k_swing = list(range(*[int(t / dt) for t in t_jump]))
k_stance = list(filterfalse(lambda k: k in k_swing, k_all))
lifted_legs = contacts_name.copy()

# contact velocity is zero, and normal force is positive
for frame, f in contact_map.items():
    nodes = k_stance if frame in lifted_legs else k_all

    FK = cs.Function.deserialize(kindyn.fk(frame))
    DFK = cs.Function.deserialize(kindyn.frameVelocity(frame, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))
    DDFK = cs.Function.deserialize(kindyn.frameAcceleration(frame, cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED))

    p = FK(q=q)['ee_pos']
    p_start = FK(q=q_init)['ee_pos']
    v = DFK(q=q, qdot=q_dot)['ee_vel_linear']
    a = DDFK(q=q, qdot=q_dot)['ee_acc_linear']

    # node: on first swing node vel should be zero!
    prb.createConstraint(f"{frame}_vel", v, nodes=list(nodes) + [k_swing[0]])
    # prb.createIntermediateCost(f'{frame}_fn', barrier(f[2] - 25.0)) #, nodes=nodes)

# swing force is zero
for leg in lifted_legs:
    fzero = np.zeros(n_f)
    contact_map[leg].setBounds(fzero, fzero, nodes=k_swing)

# cost
# prb.createCost("min_rot", 10 * cs.sumsqr(q[3:6] - q_init[3:6]))
# prb.createCost("min_xy", 100 * cs.sumsqr(q[0:2] - q_init[0:2]))
prb.createCost("min_q", cs.sumsqr(1e-1 * (q[7:] - q_init[7:])))
# prb.createCost("min_q_dot", 1e-2 * cs.sumsqr(q_dot))
prb.createIntermediateCost("min_q_ddot", cs.sumsqr(1e-3 * (q_ddot)))
for f in f_list:
    prb.createIntermediateCost(f"min_{f.getName()}", cs.sumsqr(1e-3 * (f)))

    fc, fc_lb, fc_ub = kin_dyn.linearized_friction_cone(f, 0.5, np.identity(3))
    i = prb.createIntermediateConstraint("fc", fc)
    i.setBounds(fc_lb, fc_ub, nodes=k_all)

# =============
# SOLVE PROBLEM
# =============
opts = dict()
if solver_type == 'ipopt':
    opts['ipopt.tol'] = 0.001
    opts['ipopt.constr_viol_tol'] = n_nodes * 1e-12
    opts['ipopt.max_iter'] = 2000
    opts['ipopt.linear_solver'] = 'ma57'

if solver_type == 'ilqr':
    opts = {'ilqr.max_iter': 1000,
            'ilqr.integrator': 'RK4',
            'ilqr.closed_loop_forward_pass': True,
            'ilqr.line_search_accept_ratio': 1e-9,
            'ilqr.svd_threshold': 1e-12,
            'ilqr.kkt_decomp_type': 'lu',
            'ilqr.constr_decomp_type': 'qr',
            'ilqr.codegen_enabled': False,
            'ilqr.codegen_workdir': '/tmp/ilqr_spot_jump',
            }

solver = solver.Solver.make_solver(solver_type, prb, opts)
# once the solver is created, I still can change the bounds and parameters without creating again the problem

try:
    solver.set_iteration_callback()
    solver.plot_iter = ilqr_plot_iter
except:
    pass

t = time.time()
solver.solve()
elapsed = time.time() - t
print(f'solved in {elapsed} s')

try:
    solver.print_timings()
except:
    pass

solution = solver.getSolutionDict()
solution_constraints_dict = dict()

# ========================================================
if True:

    hplt = plotter.PlotterHorizon(prb, solution)
    # hplt.plotVariables(show_bounds=True, legend=False)
    hplt.plotVariables([elem.getName() for elem in f_list], show_bounds=True, gather=2, legend=False)
    # hplt.plotFunction('inverse_dynamics', show_bounds=True, legend=True, dim=range(6))

    pos_contact_list = list()
    for contact in contacts_name:
        FK = cs.Function.deserialize(kindyn.fk(contact))
        pos = FK(q=solution['q'])['ee_pos']
        plt.figure()
        plt.title(contact)
        for dim in range(n_f):
            plt.plot(np.array([range(pos.shape[1])]), np.array(pos[dim, :]), marker="x", markersize=3,
                     linestyle='dotted')

    plt.figure()
    for contact in contacts_name:
        FK = cs.Function.deserialize(kindyn.fk(contact))
        pos = FK(q=solution['q'])['ee_pos']

        plt.title(f'plane_xy')
        plt.scatter(np.array(pos[0, :]), np.array(pos[1, :]), linewidth=0.1)

    plt.figure()
    for contact in contacts_name:
        FK = cs.Function.deserialize(kindyn.fk(contact))
        pos = FK(q=solution['q'])['ee_pos']

        plt.title(f'plane_xz')
        plt.scatter(np.array(pos[0, :]), np.array(pos[2, :]), linewidth=0.1)

    plt.show()
# ======================================================

contact_map = dict(zip(contacts_name, [solution['f0'], solution['f1'], solution['f2'], solution['f3']]))

# resampling
if False:

    if isinstance(dt, cs.SX):
        dt_before_res = solution['dt'].flatten()
    else:
        dt_before_res = dt

    dt_res = 0.001
    dae = {'x': x, 'p': q_ddot, 'ode': x_dot, 'quad': 1}
    q_res, qdot_res, qddot_res, contact_map_res, tau_res = resampler_trajectory.resample_torques(
        solution["q"], solution["q_dot"], solution["q_ddot"], dt_before_res, dt_res, dae, contact_map,
        kindyn,
        cas_kin_dyn.CasadiKinDyn.LOCAL_WORLD_ALIGNED)