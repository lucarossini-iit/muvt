#include <iostream>
#include <cmath>
#include <memory>

#include <ros/ros.h>

#include <g2o/config.h>
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

int main(int argc, char** argv)
{    
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    
    g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
//     auto linearSolver = std::make_unique<g2o::BlockSolver_6_3::LinearSolverType>();
    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
//     g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
//     optimizer.setAlgorithm(solver);
//     
//     
//     double focal_length= 1000.;
//     Vector2d principal_point(320., 240.);
//     std::vector<g2o::SE3Quat, aligned_allocator<g2o::SE3Quat> > true_poses;
//     g2o::CameraParameters * cam_params = new g2o::CameraParameters (focal_length, principal_point, 0.);
//     cam_params->setId(0);
//     if (!optimizer.addParameter(cam_params)) 
//     {
//     assert(false);
//     }

        
    return true;
}
