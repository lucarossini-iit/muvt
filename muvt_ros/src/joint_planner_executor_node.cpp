#include <muvt_ros/joint_planner_executor.h>

#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planner_executor");
    Muvt::JointPlannerExecutor executor;

    ros::Rate r(20);
    while (ros::ok())
    {
        executor.run();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
