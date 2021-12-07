#include <optimizer/optimizer.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "optimizer_node");

    // create ModelInterface
    auto cfg = XBot::ConfigOptionsFromParamServer();
    auto model = XBot::ModelInterface::getModel(cfg);
    
    // create Optimizer
    // TODO: remove Simulator
    Eigen::VectorXd start(model->getJointNum()), goal(model->getJointNum()), qhome(model->getJointNum());
    model->getRobotState("home", qhome);

    start = qhome;
    start(0) = -1.0;

    goal = qhome;
    goal(0) = 1.0;

    // configurations for cartesio_planning
//    q_start = 0.7725968000931965, 1.0393687668944975, 1.1028849186182552, -1.1553526468778605, 1.58559912318336, 0.9520093532966886, -0.592055437482859;
//    q_goal = -0.9397985404009509, 0.5779515012388596, -0.5486504436373967, -1.273983806300329, 0.31485743502845864, 1.430082706401041, 0.8996239066615537;


    // TODO remove simulator
    auto simulator = std::make_shared<XBot::HyperGraph::Simulator>(100, start, goal, XBot::HyperGraph::Simulator::ScenarioType::ROBOTPOS);
    std::vector<Eigen::VectorXd> q_init;

    // trajectory
    auto configurations =  simulator->getConfigurations();
    for (auto configuration : configurations)
    {
        q_init.push_back(configuration.q);
    }

    // single configuration
//    q_init.push_back(qhome);

    XBot::HyperGraph::Optimizer opt(q_init);

    ros::Rate rate(10);
    while (ros::ok())
    {
        opt.run();
        rate.sleep();
        ros::spinOnce();
    }
}
