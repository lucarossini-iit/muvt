#include <muvt_ros/joint_planner_executor.h>

#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planner_executor");
    Muvt::JointPlannerExecutor executor;

    while (ros::ok())
    {
        executor.run();
    }

    return 0;
}
