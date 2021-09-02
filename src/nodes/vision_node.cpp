#include <vision/point_cloud_manager.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visual_node");
    ros::NodeHandle nh("");

    XBot::HyperGraph::Utils::PointCloudManager::Ptr pc_manager;

    if (argc == 2)
    {
        std::string topic_name(argv[1]);
        pc_manager = std::make_shared<XBot::HyperGraph::Utils::PointCloudManager>(topic_name, nh);
    }
    else
    {
        pc_manager = std::make_shared<XBot::HyperGraph::Utils::PointCloudManager>(Eigen::Vector3d(0.0, 0.0, 0.0), nh);
    }

    ros::Rate rate(30);
    while (ros::ok())
    {
        pc_manager->run();

        rate.sleep();
        ros::spinOnce();
    }
}
