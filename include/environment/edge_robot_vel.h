#ifndef EDGE_ROBOT_VEL_H
#define EDGE_ROBOT_VEL_H

#include <g2o/core/base_binary_edge.h>

#include <XBotInterface/ModelInterface.h>

#include <environment/vertex_robot_pos.h>

 using namespace g2o;

 namespace XBot { namespace HyperGraph {

 // The template argument defines the maximum number of link pairs of the robot and the number of DoFs of the robot
 class EdgeRobotVel : public BaseBinaryEdge<-1, Eigen::VectorXd, VertexRobotPos, VertexRobotPos> {
 public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
     typedef BaseBinaryEdge<-1, Eigen::VectorXd, VertexRobotPos, VertexRobotPos> BaseEdge;

     EdgeRobotVel(XBot::ModelInterface::Ptr model) :
     BaseBinaryEdge<-1, Eigen::VectorXd, VertexRobotPos, VertexRobotPos>(),
     _model(model)
     {
         _model->getVelocityLimits(_vel_max);
         _vel_max = _vel_max / 100;
         _vel_min = -_vel_max;
     }

     bool read(std::istream& is)
     {}

     bool write(std::ostream& os) const
     {}

     void computeError()
     {
         auto v1 = dynamic_cast<const VertexRobotPos*>(BaseEdge::_vertices[0]);
         auto v2 = dynamic_cast<const VertexRobotPos*>(BaseEdge::_vertices[1]);

         Eigen::VectorXd q1(v1->estimateDimension()), q2(v1->estimateDimension()), diff(v1->estimateDimension());
         q1 = v1->estimate();
         q2 = v2->estimate();

         double eps = 0.1;
         double S = 0.05;
         double r = 0.2;
         int n = 2;
         diff = (q2 - q1)/0.01;

         for (int i = 0; i < q1.size(); i++)
         {
             if (diff(i) > _vel_max(i) + eps)
             {
                double value = pow((-diff(i)-(-_vel_max(i)-eps))/S, n);
                _error(i) = value;
             }
             else
                 _error(i) = 0;
         }
     }

 private:
     XBot::ModelInterface::Ptr _model;

     Eigen::VectorXd _vel_min, _vel_max;



 }; } }

#endif
