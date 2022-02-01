#include <optimizer/optimizer.h>

#include <cartesio_planning/planner/cartesio_ompl_planner.h>
#include <cartesio_planning/validity_checker/validity_checker_context.h>
#include <cartesio_planning/interpolator/cartesian_trajectory_interpolation.h>
#include <cartesio_planning/utils/robot_viz.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "optimizer_node");
    ros::NodeHandle nh(""), nhpr("~");

    std::string urdf, srdf, jidmap;

    // create ModelInterface
    XBot::ConfigOptions cfg;
    if(nh.hasParam("robot_description_reduced") && nh.getParam("robot_description_reduced", urdf))
    {
        cfg.set_urdf(urdf);
    }
    else
    {
        throw std::runtime_error("robot_description_reduced parameter not set");
    }

    if(nh.hasParam("robot_description_semantic_reduced") && nh.getParam("robot_description_semantic_reduced", srdf))
    {
        cfg.set_srdf(srdf);
    }
    else
    {
        throw std::runtime_error("robot_description_semantic_reduced parameter not set");
    }

    if(nh.hasParam("robot_description_joint_id_map_reduced") && nh.getParam("robot_description_joint_id_map_reduced", jidmap))
    {
        cfg.set_jidmap(jidmap);
    }
    else
    {
        //success = false;
        if(!cfg.generate_jidmap())
            throw std::runtime_error("robot_description_joint_id_map_reduced parameter not set, failed to auto-generate jid_map");
    }



    std::string model_type;
    bool is_model_floating_base;

    cfg.set_parameter("model_type", nhpr.param<std::string>("model_type", "RBDL"));

    cfg.set_parameter("is_model_floating_base", nhpr.param<bool>("is_model_floating_base", false));

    cfg.set_parameter<std::string>("framework", "ROS");

    auto model = XBot::ModelInterface::getModel(cfg);
    auto start_model = XBot::ModelInterface::getModel(cfg);;
    auto goal_model = XBot::ModelInterface::getModel(cfg);;

    // create robotviz
    XBot::Cartesian::Planning::RobotViz start_viz(start_model, "start_markers", nh);
    start_viz.setRGBA(Eigen::Vector4d(0.0, 0.0, 1.0, 0.5));
    XBot::Cartesian::Planning::RobotViz goal_viz(goal_model, "goal_markers", nh);
    goal_viz.setRGBA(Eigen::Vector4d(0.0, 1.0, 0.0, 0.5));

    // cartesio_planning
    // load planner config file (yaml)
    if(!nhpr.hasParam("planner_config"))
    {
        throw std::runtime_error("Mandatory private parameter 'planner_config' missing");
    }

    std::string planner_config_string;
    nhpr.getParam("planner_config", planner_config_string);
    std::cout << "planner_config correctly loaded" << std::endl;

    auto planner_config = YAML::Load(planner_config_string);
    Eigen::VectorXd qmin(model->getJointNum()), qmax(model->getJointNum());
    model->getJointLimits(qmin, qmax);

    // create planner
    XBot::Cartesian::Planning::OmplPlanner planner(qmin, qmax, planner_config);
    std::cout << "[cartesio_planning]: OmplPlanner object created" << std::endl;

    // set start and goal states
    Eigen::VectorXd q_start(model->getJointNum()), q_goal(model->getJointNum());
//    q_start << 0.7725968000931965, 1.0393687668944975, 1.1028849186182552, -1.1553526468778605, 1.58559912318336, 0.9520093532966886, -0.592055437482859;
//    q_goal << -0.9397985404009509, 0.5779515012388596, -0.5486504436373967, -1.273983806300329, 0.31485743502845864, 1.430082706401041, 0.8996239066615537;

//    q_start << 1.435890845467516, 2.254741141162672, 0.7731359057646159, -1.5987985008636163, 1.79908412773386, 0.8278361450793333, -1.9591460584341358;
//    q_goal << 1.6149999999999998, 3.0480033931954638, 1.1174923437663205, -0.900732528582709, 1.9668444721897773, 1.1049814352510496, -1.8022408305826212;

//    q_start << 0.9540997921225692, 2.8340382435025795, -0.054582959519206296, -1.872864408072525, -0.20117951060631925, -0.33496398579180775, 0.5608591855292404;
//    q_goal << 1.4149801952742422, 2.692255731463967, 0.5485718010333238, -0.6226972234435099, 0.09010596535951508, 0.3892125234054429, -0.2962225593885948;

    q_start << 1.490362884745137, 2.142375030204747, 0.6553685116016005, -2.0249158658745556, -1.922850460486469, -0.6804478136821274, 1.7540936188670038;
    q_goal << 0.5680171961836233, 2.882293977941035, -0.3656079030153727, -0.9495590652054192, 0.08411655928135578, -0.05791731561977906, 0.19953927612950212;

    start_model->setJointPosition(q_start);
    start_model->update();
    goal_model->setJointPosition(q_goal);
    goal_model->update();

    model->setJointPosition(q_start);
    model->update();

    planner.setStartAndGoalStates(q_start, q_goal);

    // set validity checker
    XBot::Cartesian::Planning::ValidityCheckContext vc_context(planner_config, model, nhpr);
    vc_context.planning_scene->startMonitor();

    auto validity_predicate = [&model, &vc_context](const Eigen::VectorXd& q)
    {
        model->setJointPosition(q);
        model->update();
        return vc_context.vc_aggregate.checkAll();
    };

    planner.setStateValidityPredicate(validity_predicate);
    planner.solve(3.0, "RRTstar");

    // interpolate
    auto interpolator = std::make_shared<CartesianTrajectoryInterpolation>(model);

    std::vector<Eigen::VectorXd> raw_trajectory, trajectory;
    std::cout << "[cartesio_planning]: start interpolating" << std::endl;
    if(planner.getPlannerStatus() == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION || planner.getPlannerStatus() == ompl::base::PlannerStatus::EXACT_SOLUTION)
    {
        auto t = ros::Duration(0.);
        raw_trajectory = planner.getSolutionPath();

        interpolator->compute(raw_trajectory);
        double time = 0.;
        const double interpolation_time = 0.05;
        while(time <= interpolator->getTrajectoryEndTime())
        {
            auto sol_eval = interpolator->evaluate(time);
            trajectory.push_back(sol_eval);
            std::cout << sol_eval.transpose() << std::endl;
            time += interpolation_time;
        }
    }

    // trajectory
    XBot::HyperGraph::Optimizer opt(trajectory, model);

    ros::Rate rate(50);
    while (ros::ok())
    {
        start_viz.publishMarkers(ros::Time::now(), {});
        goal_viz.publishMarkers(ros::Time::now(), {});
        opt.run();
        rate.sleep();
        ros::spinOnce();
    }
}
