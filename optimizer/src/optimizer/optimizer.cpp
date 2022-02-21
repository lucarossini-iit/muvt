#include <optimizer/optimizer.h>

using namespace XBot::HyperGraph;

int old_trajectory_index;
std::string frame_id;

Optimizer::Optimizer():
_nh("optimizer"),
_nhpr("~"),
_number_obs(0),
_isJointCallbackDone(false)
{
    init_load_model();
    init_optimizer();
}

Optimizer::Optimizer(std::vector<Eigen::VectorXd> vertices):
_nh("optimizer"),
_nhpr("~"),
_vertices(vertices),
_number_obs(0),
_isJointCallbackDone(false)
{
    init_load_model();
    init_load_config();
    init_optimizer();
    init_vertices();
    init_load_edges();
}

Optimizer::Optimizer(std::vector<Eigen::VectorXd> vertices, XBot::ModelInterface::Ptr model):
_nh("optimizer"),
_nhpr("~"),
_vertices(vertices),
_number_obs(0),
_model(model),
_isJointCallbackDone(false)
{
    init_load_config();
    init_optimizer();
    init_vertices();
    init_load_edges();

    frame_id = "world";
    XBot::MatLogger2::Options opt;
    opt.default_buffer_size = 1e7;
    _logger = XBot::MatLogger2::MakeLogger("/tmp/optimizer_stats.mat", opt);
}

void Optimizer::object_callback(const teb_test::ObjectMessageStringConstPtr& msg)
{
    if (_obstacles.size() > 0)
        _obstacles.clear();

    if (_vertices.size() == 0)
        return;

    // take obstacles from topic
    for (auto object : msg->objects)
    {
        obstacle obs;
        tf::pointMsgToEigen(object.pose.position, obs.position);
        tf::quaternionMsgToEigen(object.pose.orientation, obs.orientation);
        obs.T.translation() = obs.position;
        obs.T.linear() = obs.orientation.toRotationMatrix();
        tf::vectorMsgToEigen(object.size, obs.size);
        obs.frame_id = object.header.frame_id;
        obs.id = object.header.seq;
        _obstacles.push_back(obs);
    }
    update_edges();
}

void Optimizer::update_local_vertices_callback(const std_msgs::Int16ConstPtr &msg)
{
    _active_vertices.clear();
    if (_vertices.size() == 0)
    {
        ROS_WARN("missing vertices!");
        return;
    }

    if (_vertices.size() < _num_local_vertices)
    {
        ROS_WARN("you are asking a number of local vertices greater than the number of vertices themselves...reducing num_local_vertices");
        _num_local_vertices = _vertices.size();
    }

    // find active vertices
    if(msg->data >= old_trajectory_index)
    {
        for (int i = 0; i < _num_local_vertices; i++)
        {
            if((msg->data + i) > _vertices.size()-1)
                _active_vertices.push_back(_vertices.size()-1 - i);
            else
                _active_vertices.push_back(msg->data + i);
        }
    }
    else
    {
        for (int i = 0; i < _num_local_vertices; i++)
        {
            if ((msg->data - i) < 0)
                _active_vertices.push_back(i);
            else
                _active_vertices.push_back(msg->data - i);
        }
    }
    auto vertices = _optimizer.vertices();

    // fix all
    for (auto vertex : vertices)
    {
        auto v = dynamic_cast<VertexRobotPos*>(vertex.second);
        v->setFixed(true);
    }

    // active the computed vertices
    for (auto index : _active_vertices)
    {
        auto vertex = dynamic_cast<VertexRobotPos*>(vertices[index]);
        vertex->setFixed(false);
    }

    old_trajectory_index = msg->data;
}

void Optimizer::update_kinetic_edge_reference_callback(const std_msgs::Float32ConstPtr &msg)
{
    auto vertices = _optimizer.vertices();
    for (auto vertex : vertices)
    {
        auto v = dynamic_cast<VertexRobotPos*>(vertex.second);
        for (auto edge : v->edges())
        {
            if(auto e = dynamic_cast<EdgeKinematic*>(edge))
            {
                if (e->getDistalLink() == "pelvis")
                {
                    _model->setJointPosition(v->estimate());
                    _model->update();
                    Eigen::Affine3d T;
                    _model->getPose("pelvis", T);
                    T.translation().z() = msg->data;
                    e->setReference(T);
                }
            }
        }
    }
}

void Optimizer::update_edges()
{
    auto vertices = _optimizer.vertices();
    for (auto vertex : vertices)
    {
        auto v = dynamic_cast<VertexRobotPos*>(vertex.second);
        double cum_err = 0;
        double error = 0;
        for (auto edge : v->edges())
        {
            if (dynamic_cast<EdgeCollision*>(edge) != nullptr)
            {
                auto e = dynamic_cast<EdgeCollision*>(edge);
                e->setObstacles(_obstacles);
            }

            if(dynamic_cast<EdgeTrajectoryVel*>(edge) != nullptr)
            {
                auto e = dynamic_cast<EdgeTrajectoryVel*>(edge);
                e->setRef(v->estimate());
            }
        }
    }
}

void Optimizer::run()
{
    update_edges();
    optimize();
}

void Optimizer::init_load_model()
{
    // ModelInterface
    auto cfg = XBot::ConfigOptionsFromParamServer();
    _model = XBot::ModelInterface::getModel(cfg);

    Eigen::VectorXd qhome;
    _model->getRobotState("home", qhome);
    _model->setJointPosition(qhome);
    _model->update();

    std::string world_frame_link;
    if(_nhpr.hasParam("world_frame_link"))
    {
        _nhpr.getParam("world_frame_link", world_frame_link);
        Eigen::Affine3d T;
        if(_model->getPose(world_frame_link,T))
        {
            ROS_INFO("Setting planner world frame in %s", world_frame_link.c_str());

            _model->setFloatingBasePose(T.inverse());
            _model->update();
        }
        else
            ROS_ERROR("world_frame_link %s does not exists, keeping original world!", world_frame_link.c_str());
    }
}

void Optimizer::init_load_config()
{
    if(!_nhpr.hasParam("optimizer_config"))
    {
        throw std::runtime_error("Mandatory private parameter 'optimizer_config' missing");
    }

    // load planner config file (yaml)
    std::string optimizer_config_string;
    _nhpr.getParam("optimizer_config", optimizer_config_string);

    _optimizer_config = YAML::Load(optimizer_config_string);

    YAML_PARSE_OPTION(_optimizer_config["optimizer"], iterations, int, 10);
    _iterations = iterations;
    YAML_PARSE_OPTION(_optimizer_config["optimizer"], num_local_vertices, int, 100);
    _num_local_vertices = num_local_vertices;
    old_trajectory_index = 0;

    for (int i = 0; i <= _num_local_vertices; i++)
        _active_vertices.push_back(i);

    _server = std::make_shared<interactive_markers::InteractiveMarkerServer>("optimizer");
    _create_obs_srv = _nh.advertiseService("create_obstacle", &Optimizer::create_obstacle_service, this);

    // advertise and subscribe to topics
    _obj_sub = _nh.subscribe("obstacles", 10, &Optimizer::object_callback, this);
    _trj_index_sub = _nh.subscribe("trajectory_index", 10, &Optimizer::update_local_vertices_callback, this);
    _edge_kin_sub = _nh.subscribe("kin_reference", 10, &Optimizer::update_kinetic_edge_reference_callback, this);

    _sol_pub = _nh.advertise<trajectory_msgs::JointTrajectory>("solution", 10, true);
    _ee_trj_pub = _nh.advertise<visualization_msgs::MarkerArray>("trjectory", 1, true);
    _vertices_pub = _nh.advertise<std_msgs::Int32MultiArray>("vertices", 10, true);
    _time_pub = _nh.advertise<std_msgs::Float32MultiArray>("time", 10, true);
    _err_pub = _nh.advertise<std_msgs::Float32>("error", 10, true);
    _init_time = ros::Time::now();
}

void Optimizer::init_optimizer()
{
    auto linearSolver = g2o::make_unique<LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>>();
    auto blockSolver = g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver));
    g2o::OptimizationAlgorithm *algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));

    _time_vector.resize(10);
    _time_vector = {0,0,0,0,0,0,0,0,0,0};

    _optimizer.setVerbose(false);
    _optimizer.setAlgorithm(algorithm);
}

void Optimizer::init_vertices()
{
    for (int i = 0; i < _vertices.size(); i++)
    {
        auto v = new VertexRobotPos();
        v->setDimension(_model->getJointNum());
        v->setEstimate(_vertices[i]);
        v->setId(i);
//        if (i == 0 || i == _vertices.size() - 1)
//            v->setFixed(true);
        _optimizer.addVertex(v);
    }
}

void Optimizer::init_load_edges()
{
    if(!_optimizer_config["constraints"])
    {
        ROS_WARN("No constraints were defined");
        return;
    }

    for(auto vc : _optimizer_config["constraints"])
    {
        auto vc_name = vc.as<std::string>();

        if (vc_name == "joint_limits")
        {
            for (int i = 0; i < _vertices.size(); i++)
            {
                // joint limits
                auto e_jl = new EdgeJointLimits(_model);
                Eigen::MatrixXd info_jl(_model->getJointNum(), _model->getJointNum());
                info_jl.setIdentity(); info_jl *= 0.1;
                e_jl->setInformation(info_jl);
                e_jl->vertices()[0] = _optimizer.vertex(i);
                e_jl->resize();
                _optimizer.addEdge(e_jl);
            }
        }
        else if (vc_name == "velocity_limits")
        {
            YAML_PARSE_OPTION(_optimizer_config[vc_name], weight, double, 1);
            if (_vertices.size() == 1)
            {
                auto e_vel = new EdgeRobotUnaryVel(_model);
                Eigen::MatrixXd info(_model->getJointNum(), _model->getJointNum());
                info.setIdentity();
                info *= weight;
                e_vel->setInformation(info);
                e_vel->vertices()[0] = _optimizer.vertex(0);
                e_vel->resize();
                auto v = dynamic_cast<VertexRobotPos*>(_optimizer.vertex(0));
                e_vel->setRef(v->estimate());
                _optimizer.addEdge(e_vel);
            }
            else
            {
                for (int i = 0; i < _vertices.size() - 1; i++)
                {
                    auto e_vel = new EdgeRobotVel(_model);
                    Eigen::MatrixXd info(_model->getJointNum(), _model->getJointNum());
                    info.setIdentity(); info *= weight;  info(0,0) = 10; info(1,1) = 1000; info(2,2) = 1000; info(3,3) = 1000; info(4,4) = 1000; info(5,5) = 1000;
                    e_vel->setInformation(info);
                    e_vel->vertices()[0] = _optimizer.vertex(i);
                    e_vel->vertices()[1] = _optimizer.vertex(i+1);
                    e_vel->resize();
                    _optimizer.addEdge(e_vel);
                }
            }
        }
        else if (vc_name == "nominal_trajectory")
        {
            YAML_PARSE_OPTION(_optimizer_config[vc_name], weight, double, 1);
            YAML_PARSE_OPTION(_optimizer_config[vc_name], end_effectors, std::vector<std::string>, {});
            for (int i = 0; i < _vertices.size(); i++)
            {
                auto e_t = new EdgeTask(_model);
//                Eigen::MatrixXd info_t(_model->getJointNum(), _model->getJointNum());
                Eigen::MatrixXd info_t(end_effectors.size() * 3, end_effectors.size() * 3);
                info_t.setIdentity();
                if (i == 0 || i == _vertices.size() - 1)
                    info_t *= 10000;
                else
                    info_t *= weight;
                e_t->setInformation(info_t);
                e_t->vertices()[0] = _optimizer.vertex(i);
                auto v = dynamic_cast<const VertexRobotPos*>(_optimizer.vertex(i));
                e_t->setEndEffectors(end_effectors);
                e_t->setReference(v->estimate());
                e_t->resize();
                _optimizer.addEdge(e_t);
            }
        }
        else if (vc_name == "collisions")
        {
            urdf::ModelSharedPtr collision_urdf = std::make_shared<urdf::Model>();
            if (collision_urdf->initParam("collision_urdf"))
            {
                _cld = std::make_shared<ComputeLinksDistance>(*_model, collision_urdf);

                // remove useless link pairs
                auto link_distances = _cld->getLinkDistances();
                std::list<LinkPairDistance::LinksPair> black_list;
                std::list<std::string> links {//"arm1_1", "arm1_2", "arm1_3", "arm1_4", "arm1_5", "arm1_6", "arm1_7", "ball1",
                                              //"arm2_1", "arm2_2", "arm2_3", "arm2_4", "arm2_5", "arm2_6", "arm2_7", "ball2",
                                              "hip1_1", "hip2_1", "knee_1", "ankle1_1", "ankle2_1", "wheel_1",
                                              "hip1_2", "hip2_2", "knee_2", "ankle1_2", "ankle2_2", "wheel_2",
                                              "hip1_3", "hip2_3", "knee_3", "ankle1_3", "ankle2_3", "wheel_3",
                                              "hip1_4", "hip2_4", "knee_4", "ankle1_4", "ankle2_4", "wheel_4",
                                              "pelvis", "torso_2"};
                _cld->setLinksVsEnvironment(links);
                for (auto link_distance : link_distances)
                {
                    if (std::find(links.begin(), links.end(), link_distance.getLinkNames().first) == links.end() ||
                        std::find(links.begin(), links.end(), link_distance.getLinkNames().second) == links.end())
                    {
                        black_list.push_back(link_distance.getLinkNames());
                    }
                }
                _cld->setCollisionBlackList(black_list);
                auto coll_links = _cld->getLinkDistances();
                for (auto link : coll_links)
                    std::cout << link.getLinkNames().first << "   " << link.getLinkNames().second << std::endl;
                std::cout << "created " << coll_links.size() << " link pairs" << std::endl;
            }

            else
                ROS_ERROR("unable to find collision_urdf");

            YAML_PARSE_OPTION(_optimizer_config[vc_name], max_pair_link, int, 1e2);
            YAML_PARSE_OPTION(_optimizer_config[vc_name], weight, double, 1);
            for (int i = 0; i < _vertices.size(); i++)
            {
                auto e_coll = new EdgeCollision(_model, _cld, max_pair_link);
                Eigen::MatrixXd info(max_pair_link, max_pair_link);
                info.setIdentity();
                info *= weight;
                e_coll->setInformation(info);
                e_coll->vertices()[0] = _optimizer.vertex(i);
                _optimizer.addEdge(e_coll);
            }
        }
        else if (vc_name == "trajectory_vel")
        {
            YAML_PARSE_OPTION(_optimizer_config["trajectory_vel"], weight, double, 1);
            for (int i = 0; i < _vertices.size(); i++)
            {
                auto e_trj_vel = new EdgeTrajectoryVel(_model);
                Eigen::MatrixXd info(_model->getJointNum(), _model->getJointNum());
                info.setIdentity();
                info *= weight;
                e_trj_vel->setInformation(info);
                e_trj_vel->vertices()[0] = _optimizer.vertex(i);
                e_trj_vel->resize();
                e_trj_vel->setRef(_vertices[i]);
                _optimizer.addEdge(e_trj_vel);
            }
        }
        else if (vc_name == "kinematic")
        {
            YAML_PARSE_OPTION(_optimizer_config[vc_name], distal_links, std::vector<std::string>, {});
            std::vector<int> default_indices {0, 1, 2, 3, 4, 5};

            for (auto link : distal_links)
            {
                YAML_PARSE_OPTION(_optimizer_config[vc_name][link], indices, std::vector<int>, default_indices);
                YAML_PARSE_OPTION(_optimizer_config[vc_name][link], weight, double, 1);
                for (int i = 0; i < _vertices.size(); i++)
                {
//                    if (weight.empty())
//                    {
//                        weight.resize(indices.size());
//                        for (auto& w : weight)
//                            w = 1;
//                    }

                    auto e_kin = new EdgeKinematic(_model);
//                    for (auto index : indices)
//                        std::cout << index << std::endl;
                    Eigen::MatrixXd info(indices.size(), indices.size());
                    info.setIdentity();
                    info *= weight;
                    e_kin->setIndices(indices);
                    e_kin->vertices()[0] = _optimizer.vertex(i);
                    e_kin->setInformation(info);
                    e_kin->setDistalLink(link);                
                    Eigen::Affine3d T;
                    _model->setJointPosition(_vertices[i]);
                    _model->update();
                    if(link != "com")
                        _model->getPose(link, T);
                    e_kin->setReference(T);                    
                    _optimizer.addEdge(e_kin);
                }
            }
        }       
        else if (vc_name == "postural")
        {
            YAML_PARSE_OPTION(_optimizer_config[vc_name], weight, double, 1);
            for (int i = 0; i < _vertices.size(); i++)
            {
                auto e_postural = new EdgePostural(_model);
                Eigen::MatrixXd info(_model->getJointNum(), _model->getJointNum());
                info.setIdentity();
                info *= weight; info(0,0) = 0; info(1,1) = 0; info(2,2) = 0; info(3,3) = 0; info(4,4) = 0; info(5,5) = 0;
                e_postural->setInformation(info);
                e_postural->vertices()[0] = _optimizer.vertex(i);
                e_postural->resize();
                e_postural->setReference(_vertices[i]);
                _optimizer.addEdge(e_postural);
            }
        }
        else
        {
            ROS_WARN("%s not found!", vc_name.c_str());
        }
    }
}

void Optimizer::setVertices(std::vector<Eigen::VectorXd> vertices)
{
    _vertices = vertices;
    init_vertices();

    if (_optimizer.edges().size() > 0)
        _optimizer.edges().clear();
    init_load_edges();
}

void Optimizer::optimize()
{
    XBot::JointNameMap jmap;
    _model->getJointPosition(jmap);
    visualization_msgs::MarkerArray ma;
    auto vertices = _optimizer.vertices();
    unsigned int counter_non_active = 0;
    unsigned int counter_active = 0;
    for (const auto vertex : vertices)
    {
        auto v = dynamic_cast<VertexRobotPos*>(vertex.second);
        if (v->fixed())
            counter_non_active++;
        else
            counter_active++;
    }
    std_msgs::Int32MultiArray multi_array;
    multi_array.data.push_back(counter_non_active);
    multi_array.data.push_back(counter_active);
    multi_array.layout.dim.resize(2);
    multi_array.layout.dim[0].label = "fixed";
    multi_array.layout.dim[1].label = "active";

    _vertices_pub.publish(multi_array);

    auto tic = std::chrono::high_resolution_clock::now();
    _optimizer.initializeOptimization(); 
    _optimizer.optimize(_iterations);
    auto toc = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> fsec = toc - tic;

    // average and publish
    std_msgs::Float32MultiArray time;
    auto t = ros::Time::now() - _init_time;
    time.data.push_back(fsec.count());
    time.data.push_back(t.toSec());
    _time_pub.publish(time);

    _logger->add("time", t.toSec());
    _logger->add("opt_time", fsec.count());

    // publish residual error
//    std_msgs::Float32 err_msg;
    double cum_err_coll = 0;
//    double cum_err_vel = 0;

//    err_msg.data = cum_err_coll;
//    _err_pub.publish(err_msg);
    if (!_active_vertices.empty())
    {
        for (int ind : {_active_vertices[0], _active_vertices.back()})
        {
            auto v = dynamic_cast<VertexRobotPos*>(_optimizer.vertex(ind));
            for (auto edge : v->edges())
            {
                if (auto e = dynamic_cast<EdgeKinematic*>(edge))
                {
                    e->computeError();
                    if(e->getDistalLink() != "com")
                    {
                        std::string distal_link = e->getDistalLink();
                        if (ind == _active_vertices[0])
                        {
                            _logger->add("kin_err_" + distal_link + "_z" + "_init", e->getError()(0));
                            _logger->add("kin_err_" + distal_link + "_r" + "_init", e->getError()(1));
                            _logger->add("kin_err_" + distal_link + "_p" + "_init", e->getError()(2));
                        }
                        else
                        {
                            _logger->add("kin_err_" + distal_link + "_z" + "_end", e->getError()(0));
                            _logger->add("kin_err_" + distal_link + "_r" + "_end", e->getError()(1));
                            _logger->add("kin_err_" + distal_link + "_p" + "_end", e->getError()(2));
                        }
                    }
                    else
                    {
                        std::string distal_link = e->getDistalLink();
                        if (ind == _active_vertices[0])
                        {
                            _logger->add("kin_err_" + distal_link + "_x" + "_init", e->getError()(0));
                            _logger->add("kin_err_" + distal_link + "_y" + "_init", e->getError()(1));
                        }
                        else
                        {
                            _logger->add("kin_err_" + distal_link + "_x" + "_end", e->getError()(0));
                            _logger->add("kin_err_" + distal_link + "_y" + "_end", e->getError()(1));
                        }
                    }
                }

                if (auto e = dynamic_cast<EdgeCollision*>(edge))
                {
                    double error;
                    e->computeError();
                    error = 0;
                    for (int i = 0; i < e->getError().size(); i++)
                        error += e->getError()(i) * e->getError()(i);
                    cum_err_coll += std::sqrt(error);
                    if (ind == *_active_vertices.begin())
                    {
                        _logger->add("coll_err_init", cum_err_coll);
                    }
                    else
                    {
                        _logger->add("coll_err_end", cum_err_coll);
                    }
                 }
            }
        }
    }


    // save and publish solution
    trajectory_msgs::JointTrajectory solution;
    solution.joint_names = _model->getEnabledJointNames();
    for (int i = 0; i < vertices.size(); i++)
    {
        auto v = dynamic_cast<VertexRobotPos*>(vertices[i]);
        if (v == nullptr)
            ROS_WARN("unable to cast vertex while saving solution!");
        std::vector<double> q(v->estimate().data(), v->estimate().data() + v->estimate().size());
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = q;
        solution.points.push_back(point);
        if (vertices.size() == 1)
        {
            solution.points.push_back(point);
        }

//        _model->setJointPosition(v->estimate());
//        _model->update();
//        Eigen::Affine3d T;
//        _model->getPose("ball1", T);
//        visualization_msgs::Marker m_left, m_right;
//        m_left.header.frame_id = "pelvis";
//        m_left.header.stamp = ros::Time::now();
//        m_left.id = v->id();
//        m_left.action = visualization_msgs::Marker::ADD;
//        m_left.type = visualization_msgs::Marker::SPHERE;
//        if (v->fixed())
//        {
//            m_left.color.r = 0; m_left.color.g = 1; m_left.color.b = 0; m_left.color.a = 1;
//        }
//        else
//        {
//            m_left.color.r = 0; m_left.color.g = 0; m_left.color.b = 1; m_left.color.a = 1;
//        }
//        m_left.scale.x = 0.01; m_left.scale.y = 0.01; m_left.scale.z = 0.01;
//        m_left.pose.position.x = T.translation()(0);
//        m_left.pose.position.y = T.translation()(1);
//        m_left.pose.position.z = T.translation()(2);
//        ma.markers.push_back(m_left);

//        _model->getPose("ball2", T);
//        m_right.header.frame_id = "pelvis";
//        m_right.header.stamp = ros::Time::now();
//        m_right.id = _vertices.size() + v->id();
//        m_right.action = visualization_msgs::Marker::ADD;
//        m_right.type = visualization_msgs::Marker::SPHERE;
//        if (v->fixed())
//        {
//            m_right.color.r = 0; m_right.color.g = 1; m_right.color.b = 0; m_right.color.a = 1;
//        }
//        else
//        {
//            m_right.color.r = 0; m_right.color.g = 0; m_right.color.b = 1; m_right.color.a = 1;
//        }
//        m_right.scale.x = 0.01; m_right.scale.y = 0.01; m_right.scale.z = 0.01;
//        m_right.pose.position.x = T.translation()(0);
//        m_right.pose.position.y = T.translation()(1);
//        m_right.pose.position.z = T.translation()(2);
//        ma.markers.push_back(m_right);

        _model->setJointPosition(v->estimate());
        _model->update();
        Eigen::Affine3d T;
        _model->getPose("wheel_1", T);
        visualization_msgs::Marker m_front_left, m_front_right, m_back_left, m_back_right;
        m_front_left.header.frame_id = "world";
        m_front_left.header.stamp = ros::Time::now();
        m_front_left.id = v->id();
        m_front_left.action = visualization_msgs::Marker::ADD;
        m_front_left.type = visualization_msgs::Marker::SPHERE;
        if (v->fixed())
        {
            m_front_left.color.r = 0; m_front_left.color.g = 1; m_front_left.color.b = 0; m_front_left.color.a = 1;
        }
        else
        {
            m_front_left.color.r = 0; m_front_left.color.g = 0; m_front_left.color.b = 1; m_front_left.color.a = 1;
        }
        m_front_left.scale.x = 0.01; m_front_left.scale.y = 0.01; m_front_left.scale.z = 0.01;
        m_front_left.pose.position.x = T.translation()(0);
        m_front_left.pose.position.y = T.translation()(1);
        m_front_left.pose.position.z = T.translation()(2);
        ma.markers.push_back(m_front_left);

        _model->getPose("wheel_2", T);
        m_front_right.header.frame_id = "world";
        m_front_right.header.stamp = ros::Time::now();
        m_front_right.id = _vertices.size() + v->id();
        m_front_right.action = visualization_msgs::Marker::ADD;
        m_front_right.type = visualization_msgs::Marker::SPHERE;
        if (v->fixed())
        {
            m_front_right.color.r = 0; m_front_right.color.g = 1; m_front_right.color.b = 0; m_front_right.color.a = 1;
        }
        else
        {
            m_front_right.color.r = 0; m_front_right.color.g = 0; m_front_right.color.b = 1; m_front_right.color.a = 1;
        }
        m_front_right.scale.x = 0.01; m_front_right.scale.y = 0.01; m_front_right.scale.z = 0.01;
        m_front_right.pose.position.x = T.translation()(0);
        m_front_right.pose.position.y = T.translation()(1);
        m_front_right.pose.position.z = T.translation()(2);
        ma.markers.push_back(m_front_right);

        _model->getPose("wheel_3", T);
        m_back_left.header.frame_id = "world";
        m_back_left.header.stamp = ros::Time::now();
        m_back_left.id = vertices.size()*2 + v->id();
        m_back_left.action = visualization_msgs::Marker::ADD;
        m_back_left.type = visualization_msgs::Marker::SPHERE;
        if (v->fixed())
        {
            m_back_left.color.r = 0; m_back_left.color.g = 1; m_back_left.color.b = 0; m_back_left.color.a = 1;
        }
        else
        {
            m_back_left.color.r = 0; m_back_left.color.g = 0; m_back_left.color.b = 1; m_back_left.color.a = 1;
        }
        m_back_left.scale.x = 0.01; m_back_left.scale.y = 0.01; m_back_left.scale.z = 0.01;
        m_back_left.pose.position.x = T.translation()(0);
        m_back_left.pose.position.y = T.translation()(1);
        m_back_left.pose.position.z = T.translation()(2);
        ma.markers.push_back(m_back_left);

        _model->getPose("wheel_4", T);
        m_back_right.header.frame_id = "world";
        m_back_right.header.stamp = ros::Time::now();
        m_back_right.id = _vertices.size()*3 + v->id();
        m_back_right.action = visualization_msgs::Marker::ADD;
        m_back_right.type = visualization_msgs::Marker::SPHERE;
        if (v->fixed())
        {
            m_back_right.color.r = 0; m_back_right.color.g = 1; m_back_right.color.b = 0; m_back_right.color.a = 1;
        }
        else
        {
            m_back_right.color.r = 0; m_back_right.color.g = 0; m_back_right.color.b = 1; m_back_right.color.a = 1;
        }
        m_back_right.scale.x = 0.01; m_back_right.scale.y = 0.01; m_back_right.scale.z = 0.01;
        m_back_right.pose.position.x = T.translation()(0);
        m_back_right.pose.position.y = T.translation()(1);
        m_back_right.pose.position.z = T.translation()(2);
        ma.markers.push_back(m_back_right);
    }

    _ee_trj_pub.publish(ma);
    _sol_pub.publish(solution);
}

void Optimizer::print()
{
    std::cout << "------ PRINTING OPTIMIZED VERTICES -------" << std::endl;
    auto vertices = _optimizer.vertices();

    for (auto vertex : vertices)
    {
       auto v = dynamic_cast<VertexRobotPos*>(vertex.second);

       std::cout << "id: " << v->id() << "\n q: " << v->estimate().transpose() << std::endl;
    }
}

bool Optimizer::create_obstacle_service (teb_test::SetObstacle::Request& req, teb_test::SetObstacle::Response& res)
{
    if (!req.add)
    {
        _server->clear();
        _server->applyChanges();
        _obstacles.clear();
        res.status = true;
        return res.status;
    }

    Eigen::Vector3d position;
    position << req.pose.position.x, req.pose.position.y, req.pose.position.z;
    obstacle obs;
    obs.position = position;
    obs.orientation.coeffs() << 0, 0, 0, 1;
    obs.size << 0.5, 0.5, 0.5;
    obs.id = _obstacles.size();
    _obstacles.push_back(obs);
    update_edges();

    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = frame_id;
    int_marker.header.stamp = ros::Time::now();
    int_marker.name = "obstacle_" + std::to_string(_obstacles.size());
    int_marker.description = "obstacle_" + std::to_string(_obstacles.size());

    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::CUBE;
    m.pose.position.x = 0;
    m.pose.position.y = 0;
    m.pose.position.z = 0;
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.pose.orientation.w = 1;
    m.scale.x = obs.size(0); m.scale.y = obs.size(1); m.scale.z = obs.size(2);
    m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0; m.color.a = 1.0;

    visualization_msgs::InteractiveMarkerControl obs_control;
    obs_control.always_visible = true;
    obs_control.markers.push_back(m);
    int_marker.controls.push_back(obs_control);
    int_marker.pose.position.x = req.pose.position.x;
    int_marker.pose.position.y = req.pose.position.y;
    int_marker.pose.position.z = req.pose.position.z;

    visualization_msgs::InteractiveMarkerControl move_control_x;
    move_control_x.name = "move_x";
    move_control_x.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(move_control_x);

    visualization_msgs::InteractiveMarkerControl move_control_y;
    move_control_y.name = "move_y";
    Eigen::Matrix3d R;
    R << cos(M_PI/2), -sin(M_PI/2), 0, sin(M_PI/2), cos(M_PI/2), 0, 0, 0, 1;
    Eigen::Quaternion<double> quat(R);
    tf::quaternionEigenToMsg(quat, move_control_y.orientation);
    move_control_y.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(move_control_y);

    visualization_msgs::InteractiveMarkerControl move_control_z;
    move_control_z.name = "move_z";
    R << cos(-M_PI/2), 0, sin(-M_PI/2), 0, 1, 0, -sin(-M_PI/2), 0, cos(-M_PI/2);
    Eigen::Quaternion<double> quat_z(R);
    tf::quaternionEigenToMsg(quat_z, move_control_z.orientation);
    move_control_z.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(move_control_z);

    _server->insert(int_marker);
    _server->setCallback(int_marker.name, boost::bind(&Optimizer::interactive_markers_feedback, this, _1));
    _server->applyChanges();

    res.status = true;
    return res.status;
}

void Optimizer::interactive_markers_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    auto c = feedback->marker_name.back();
    int index = c - '0';
    _obstacles[index-1].position << feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z;
    update_edges();
}

Optimizer::~Optimizer()
{
    _logger.reset();
}
