#include <vision/point_cloud_manager.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visual_node");
    ros::NodeHandle nh("");

    XBot::HyperGraph::Utils::PointCloudManager pc_manager(Eigen::Vector3d(2.0, 0.0, 0.0), nh);

    ros::Rate rate(30);
    while (ros::ok())
    {
        pc_manager.run();

        rate.sleep();
        ros::spin();
    }
}
