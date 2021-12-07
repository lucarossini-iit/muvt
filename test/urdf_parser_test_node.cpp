#include <ros/ros.h>

#include <XBotInterface/ModelInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>

#include <urdf/model.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "urdf_parser_test_node");
    ros::NodeHandle nh("");

    urdf::Model urdf_model;
    if (!urdf_model.initParam("robot_description"))
    {
        ROS_ERROR("Error while parsing the urdf!");
        return -1;
    }
    ROS_INFO("Successfully parsed the urdf!");

    std::map<std::string, double> fixed_joint_map = {{"j_arm2_1", 0.5201490},
                                                     {"j_arm2_2", -0.320865},
                                                     {"j_arm2_3", -0.274669},
                                                     {"j_arm2_4", -2.236040},
                                                     {"j_arm2_5", -0.050082},
                                                     {"j_arm2_6", -0.781461},
                                                     {"j_arm2_7", 0.0567608}};

    for (auto joint_pair : fixed_joint_map)
    {
        auto joint = std::const_pointer_cast<urdf::Joint>(urdf_model.getJoint(joint_pair.first));
        if (joint->type == urdf::Joint::FIXED)
        {
            std::cout << "Joint " << joint->name << " is already fixed, ignoring." << std::endl;
            continue;
        }
        else if (joint->type == urdf::Joint::REVOLUTE)
        {
            joint->type = urdf::Joint::FIXED;

            urdf::Pose fixed_pose = joint->parent_to_joint_origin_transform;
            urdf::Vector3 axis = joint->axis;
            Eigen::Vector3d eigen_axis(axis.x, axis.y, axis.z);
            eigen_axis.norm();
            Eigen::AngleAxisd joint_angle(joint_pair.second, eigen_axis);
            Eigen::Quaternion<double> q_eigen(joint_angle.toRotationMatrix());
            fixed_pose.rotation.setFromQuaternion(q_eigen.x(), q_eigen.y(), q_eigen.z(), q_eigen.w());
            joint->parent_to_joint_origin_transform = fixed_pose;
        }
    }

    auto urdf = urdf::exportURDF(urdf_model);

    TiXmlPrinter printer;
    urdf->Accept(&printer);

    std::string xmltext = printer.CStr();
    nh.setParam("robot_description", xmltext);

    ROS_INFO("URDF modified!");


    return 1;
}
