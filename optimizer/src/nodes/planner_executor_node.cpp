#include "planner_executor.h"

using namespace XBot::HyperGraph::Planner;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planner_executor");

    PlannerExecutor executor;

    ros::Rate r(100);
    while (ros::ok())
    {
        executor.run();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
