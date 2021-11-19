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

    auto simulator = std::make_shared<XBot::HyperGraph::Simulator>(100, start, goal, XBot::HyperGraph::Simulator::ScenarioType::ROBOTPOS);
    std::vector<Eigen::VectorXd> q_init;
    auto configurations =  simulator->getConfigurations();
    for (auto configuration : configurations)
    {
        q_init.push_back(configuration.q);
    }
    XBot::HyperGraph::Optimizer opt(q_init);
//    opt.setVertices(q_init);

    ros::Rate rate(10);
    while (ros::ok())
    {
        opt.run();
        rate.sleep();
        ros::spinOnce();
    }
}
