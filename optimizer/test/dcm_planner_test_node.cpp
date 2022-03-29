#include <teb_test/planner/dcm_planner.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dcm_planner_test_node");

    XBot::HyperGraph::Planner::DCMPlanner dcm_planner;
    dcm_planner.GenerateSteps(10);
    dcm_planner.ComputeZMPandCP();

    ros::Rate r(1);
    while(ros::ok())
    {
        dcm_planner.run();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
