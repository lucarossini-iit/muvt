#include <simulator/optimizer.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "optimizer_node");
    
    XBot::HyperGraph::Optimizer optimizer;
    
    ros::Rate rate(100);
    while (ros::ok())
    {
        optimizer.run();
        rate.sleep();
    }
}
