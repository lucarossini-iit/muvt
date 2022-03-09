#include <teb_test/optimizer/optimizer.h>

#include <cartesio_planning/planner/cartesio_ompl_planner.h>
#include <cartesio_planning/validity_checker/validity_checker_context.h>
#include <cartesio_planning/interpolator/cartesian_trajectory_interpolation.h>
#include <cartesio_planning/utils/robot_viz.h>

std::vector<Eigen::VectorXd> readFromFileConfigs(std::string fileName){
    std::string filePrefix = "/home/luca/MultiDoF-superbuild/external/teb_test/config/";
    std::string filePath = filePrefix + fileName + ".txt";
    std::ifstream fileIn(filePath.c_str());
    std::string line;
    std::vector<Eigen::VectorXd> qList;

    int rate_index = 0;

    while(getline(fileIn, line))
    {
        if (rate_index % 10 == 0)
        {
            std::stringstream linestream(line);
            std::string value;
            Eigen::VectorXd q(18);
            int index = 0;

            while(linestream >> value)
            {
                q(index) = boost::lexical_cast<double>(value);
                index++;
            }
            qList.push_back(q);
        }
        rate_index++;
    }
    return qList;
}

XBot::ModelInterface::Ptr createReducedModel(ros::NodeHandle nh, ros::NodeHandle nhpr,
                                             std::string robot_description,
                                             std::string robot_description_semantic,
                                             std::string robot_description_joint_id_map)
{
    std::string urdf, srdf, jidmap;
    XBot::ConfigOptions cfg;

    std::cout << "generating ModelInterface using: " << std::endl;
    std::cout << "- " << robot_description << std::endl;
    std::cout << "- " << robot_description_semantic << std::endl;
    std::cout << "- " << robot_description_joint_id_map << std::endl;

    if (nh.hasParam(robot_description) && nh.getParam(robot_description, urdf))
    {
        cfg.set_urdf(urdf);
    }
    else
    {
        throw std::runtime_error(robot_description + " parameter not set!");
    }

    if (nh.hasParam(robot_description_semantic) && nh.getParam(robot_description_semantic, srdf))
    {
        cfg.set_srdf(srdf);
    }
    else
    {
        throw std::runtime_error(robot_description_semantic + " parameter not set!");
    }

    if (nh.hasParam(robot_description_joint_id_map) && nh.getParam(robot_description_joint_id_map, jidmap))
    {
        cfg.set_jidmap(jidmap);
    }
    else
    {
        if (!cfg.generate_jidmap())
        {
            throw std::runtime_error(robot_description_joint_id_map + " parameter not set, failed to auto-generate jidmap!");
        }
    }

    std::string model_type;
    bool is_model_floating_base;

    cfg.set_parameter("model_type", nhpr.param<std::string>("model_type", "RBDL"));
    cfg.set_parameter("is_model_floating_base", nhpr.param<bool>("is_model_floating_base", false));
    cfg.set_parameter<std::string>("framework", "ROS");

    auto model = XBot::ModelInterface::getModel(cfg);

    return model;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "optimizer_node");
    ros::NodeHandle nh(""), nhpr("~");

    auto model_rf = createReducedModel(nh, nhpr, "robot_description_rf", "robot_description_semantic_rf", "robot_description_joint_id_map_rf");
    auto model_lf = createReducedModel(nh, nhpr, "robot_description_lf", "robot_description_semantic_lf", "robot_description_joint_id_map_lf");
    auto model_rh = createReducedModel(nh, nhpr, "robot_description_rh", "robot_description_semantic_rh", "robot_description_joint_id_map_rh");
    auto model_lh = createReducedModel(nh, nhpr, "robot_description_lh", "robot_description_semantic_lh", "robot_description_joint_id_map_lh");

    std::vector<Eigen::VectorXd> qList = readFromFileConfigs("joint_position");
    
    XBot::HyperGraph::Optimizer opt_rf(qList, model_rf);
    XBot::HyperGraph::Optimizer opt_lf(qList, model_lf);
    XBot::HyperGraph::Optimizer opt_rh(qList, model_rh);
    XBot::HyperGraph::Optimizer opt_lh(qList, model_lh);

    ros::Rate r(100);
    
    while (ros::ok())
    {
        
    }

    return true;
}
