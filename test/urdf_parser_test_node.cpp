#include <ros/ros.h>

#include <XBotInterface/ModelInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>

#include <urdf/model.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "urdf_parser_test_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nhpr("~");

    // Parse URDF
    urdf::Model urdf_model;
    if (!urdf_model.initParam("robot_description"))
    {
        ROS_ERROR("Error while parsing the urdf!");
        return -1;
    }
    ROS_INFO("Successfully parsed the urdf!");

    // Load map from param server
    auto fixed_joint_map = nhpr.param<std::map<std::string, double>>("fixed_joints", std::map<std::string, double>());
    std::cout << "Loaded fixed_joint_map: " << std::endl;
    for (auto joint_pair : fixed_joint_map)
        std::cout << joint_pair.first << ": " << joint_pair.second << std::endl;

    // Fix joints
    for (auto joint_pair : fixed_joint_map)
    {
        auto joint = std::const_pointer_cast<urdf::Joint>(urdf_model.getJoint(joint_pair.first));
        if (joint->type == urdf::Joint::FIXED)
        {
            ROS_INFO("Joint %s is already fixed, ignoring.", joint->name);
            continue;
        }
        else if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::CONTINUOUS)
        {
            joint->type = urdf::Joint::FIXED;

            urdf::Pose fixed_pose = joint->parent_to_joint_origin_transform;
            urdf::Vector3 axis = joint->axis;
            Eigen::Vector3d eigen_axis(axis.x, axis.y, axis.z);
            eigen_axis.norm();
            Eigen::AngleAxisd joint_angle(joint_pair.second, eigen_axis);
            Eigen::Quaternion<double> q_eigen(joint_angle.toRotationMatrix());
            urdf::Pose rotation_offset;
            rotation_offset.rotation.setFromQuaternion(q_eigen.x(), q_eigen.y(),q_eigen.z(), q_eigen.w());
            fixed_pose.rotation = fixed_pose.rotation * rotation_offset.rotation;
            joint->parent_to_joint_origin_transform = fixed_pose;
        }
        else
        {
            ROS_WARN("Joint %s type not supported, ignoring", joint->name);
            continue;
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
