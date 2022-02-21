#include <optimizer/optimizer.h>
#include <simulator/simulator.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "optimizer_node");
    ros::NodeHandle nh(""), nhpr("~");

    std::string urdf, srdf, jidmap;

    // create ModelInterface
    XBot::ConfigOptions cfg;
    if(nh.hasParam("robot_description_reduced") && nh.getParam("robot_description_reduced", urdf))
    {
        cfg.set_urdf(urdf);
    }
    else
    {
        throw std::runtime_error("robot_description_reduced parameter not set");
    }

    if(nh.hasParam("robot_description_semantic_reduced") && nh.getParam("robot_description_semantic_reduced", srdf))
    {
        cfg.set_srdf(srdf);
    }
    else
    {
        throw std::runtime_error("robot_description_semantic_reduced parameter not set");
    }

    if(nh.hasParam("robot_description_joint_id_map_reduced") && nh.getParam("robot_description_joint_id_map_reduced", jidmap))
    {
        cfg.set_jidmap(jidmap);
    }
    else
    {
        //success = false;
        if(!cfg.generate_jidmap())
            throw std::runtime_error("robot_description_joint_id_map_reduced parameter not set, failed to auto-generate jid_map");
    }

    std::string model_type;
    bool is_model_floating_base;

    cfg.set_parameter("model_type", nhpr.param<std::string>("model_type", "RBDL"));

    cfg.set_parameter("is_model_floating_base", nhpr.param<bool>("is_model_floating_base", true));

    cfg.set_parameter<std::string>("framework", "ROS");

    auto model = XBot::ModelInterface::getModel(cfg);
    Eigen::VectorXd qhome, qstart, qgoal;
    model->getRobotState("home", qhome);
    model->setJointPosition(qhome);
    model->update();

    // create trajectory;
    qstart = qhome;
    qgoal = qstart;
    qgoal(0) += 3;
    XBot::HyperGraph::Simulator simulator(50, qstart, qgoal, XBot::HyperGraph::Simulator::ScenarioType::ROBOTPOS);

    auto configurations = simulator.getConfigurations();
    std::vector<Eigen::VectorXd> q_vect;
    for (auto configuration : configurations)
        q_vect.push_back(configuration.q);

    XBot::HyperGraph::Optimizer optimizer(q_vect, model);

    ros::Rate r(30);
    while (ros::ok())
    {
        optimizer.run();
        r.sleep();
        ros::spinOnce();
    }
}
