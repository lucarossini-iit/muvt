#ifndef EDGE_ROBOT_VEL_H
#define EDGE_ROBOT_VEL_H

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>

#include <XBotInterface/ModelInterface.h>

#include <environment/vertex_robot_pos.h>

 using namespace g2o;

 namespace XBot { namespace HyperGraph {

 // The template argument defines the maximum number of link pairs of the robot and the number of DoFs of the robot
 class EdgeRobotVel : public BaseBinaryEdge<-1, Eigen::VectorXd, VertexRobotPos, VertexRobotPos> {
 public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
     typedef BaseBinaryEdge<-1, Eigen::VectorXd, VertexRobotPos, VertexRobotPos> BaseEdge;

     EdgeRobotVel(XBot::ModelInterface::Ptr model);

     bool read(std::istream& is)
     {
         Eigen::VectorXd meas;
         internal::readVector(is, meas);
         setMeasurement(meas);

         return is.good() || is.eof();
     }

     bool write(std::ostream& os) const
     {
         Eigen::VectorXd p = measurement();
         os << p;

         return os.good();
     }

     void resize();

     void computeError();

 private:
     XBot::ModelInterface::Ptr _model;

     Eigen::VectorXd _vel_min, _vel_max;



 };

 class EdgeRobotUnaryVel : public BaseUnaryEdge<-1, Eigen::VectorXd, VertexRobotPos> {
 public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

     EdgeRobotUnaryVel(XBot::ModelInterface::Ptr model);


     bool read(std::istream& is)
     {
         Eigen::VectorXd meas;
         internal::readVector(is, meas);
         setMeasurement(meas);

         return is.good() || is.eof();
     }

     bool write(std::ostream& os) const
     {
         Eigen::VectorXd p = measurement();
         os << p;

         return os.good();
     }

     void resize();

     void setRef(Eigen::VectorXd ref);

     void getRef(Eigen::VectorXd ref) const;

     void computeError();


 private:
     XBot::ModelInterface::Ptr _model;

     Eigen::VectorXd _ref;
 };

 } }

#endif
