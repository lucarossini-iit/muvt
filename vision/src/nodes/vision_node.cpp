#include <teb_test/vision/point_cloud_manager.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visual_node");
    ros::NodeHandle nh("");

    XBot::HyperGraph::Utils::PointCloudManager::Ptr pc_manager;

    if (argc != 2)
        ROS_ERROR("please specify a topic name for the input PointCloud!");

    std::string topic_name(argv[1]);
    pc_manager = std::make_shared<XBot::HyperGraph::Utils::PointCloudManager>(topic_name, nh);

    ros::Rate rate(100);
    while (ros::ok())
    {
        pc_manager->run();

        rate.sleep();
        ros::spinOnce();
    }
}
