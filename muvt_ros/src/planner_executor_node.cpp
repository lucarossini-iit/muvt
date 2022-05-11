#include "muvt_ros/planner_executor.h"

using namespace Muvt::HyperGraph::Planner;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planner_executor");
  ros::NodeHandle nhpr("~");

  if(!nhpr.hasParam("dcm_config"))
  {
    throw std::runtime_error("Mandatory private parameter 'dcm_config' missing");
  }

  std::string optimizer_config_string;
  nhpr.getParam("dcm_config", optimizer_config_string);
  auto config = YAML::Load(optimizer_config_string);

  YAML_PARSE_OPTION(config["dcm_planner"], dt, double, 0.01);

  PlannerExecutor executor;

  ros::Rate r(1.0/dt);
  while (ros::ok())
  {
    executor.run();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
