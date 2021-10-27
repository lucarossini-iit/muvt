#include <simulator/optimizer_test.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "optimizer_node");
    
    XBot::HyperGraph::Optimizer optimizer;
    
    ros::Rate rate(std::atof(argv[1]));
    while (ros::ok())
    {
        optimizer.run();
        rate.sleep();
        ros::spinOnce();
    }
}
