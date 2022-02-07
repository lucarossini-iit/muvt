#include <optimizer/optimizer.h>

using namespace XBot::HyperGraph;

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
                e->computeError();
//                if (e->getError().norm() > 1e-2)
//                    std::cout << "EdgeCollision related to vertex " << v->id() << " has error: " << e->getError().transpose() << std::endl;
                error = 0;
                for (int i = 0; i < e->getError().size(); i++)
                    error += e->getError()(i) * e->getError()(i);
                cum_err += std::sqrt(error);
            }
            else if (dynamic_cast<EdgeTask*>(edge) != nullptr)
            {
                auto e = dynamic_cast<EdgeTask*>(edge);
                e->computeError();
                error = 0;
//                if (e->getError().norm() > 1e-2)
//                    std::cout << "EdgeTask related to vertex " << v->id() << " has error: " << e->getError().transpose() << std::endl;
                for (int i = 0; i < e->getError().size(); i++)
                    error += e->getError()(i) * e->getError()(i);
                cum_err += std::sqrt(error);
            }
            else if (dynamic_cast<EdgeRobotVel*>(edge) != nullptr)
            {
                auto e = dynamic_cast<EdgeRobotVel*>(edge);
                e->computeError();
//                if (e->getError().norm() > 1e-2)
//                    std::cout << "EdgeRobotVel related to vertex " << v->id() << " has error: " << e->getError().transpose() << std::endl;
                error = 0;
                for (int i = 0; i < e->getError().size(); i++)
                    error += e->getError()(i) * e->getError()(i);
                cum_err += std::sqrt(error);
            }
            else if (dynamic_cast<EdgeTrajectoryVel*>(edge) != nullptr)
            {
                auto e = dynamic_cast<EdgeTrajectoryVel*>(edge);
                e->setRef(v->estimate());
            }
            if (vertices.size() == 1)
            {
                if (dynamic_cast<EdgeRobotUnaryVel*>(edge) != nullptr)
                {
                    auto e = dynamic_cast<EdgeRobotUnaryVel*>(edge);
                    e->setRef(v->estimate());
                    e->computeError();
                }
            }

            double thresh = 1e-2;
            if (/*cum_err > thresh && */ v->id() != 0 && v->id() != _vertices.size() - 1)
                v->setFixed(false);
            else
                v->setFixed(false);
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

    _server = std::make_shared<interactive_markers::InteractiveMarkerServer>("optimizer");
    _create_obs_srv = _nh.advertiseService("create_obstacle", &Optimizer::create_obstacle_service, this);

    // advertise and subscribe to topics
    _obj_sub = _nh.subscribe("obstacles", 10, &Optimizer::object_callback, this);

    _sol_pub = _nh.advertise<trajectory_msgs::JointTrajectory>("solution", 10, true);
    _ee_trj_pub = _nh.advertise<visualization_msgs::MarkerArray>("trjectory", 1, true);
    _vertices_pub = _nh.advertise<std_msgs::Int32MultiArray>("vertices", 10, this);
    _time_pub = _nh.advertise<std_msgs::Float32MultiArray>("time", 10, this);
    _err_pub = _nh.advertise<std_msgs::Float32>("error", 10, this);
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
        if (i == 0 || i == _vertices.size() - 1)
            v->setFixed(true);
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
            YAML_PARSE_OPTION(_optimizer_config["velocity_limits"], weight, double, 1);
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
                    info.setIdentity(); info *= 0.1;
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
            YAML_PARSE_OPTION(_optimizer_config["nominal_trajectory"], weight, double, 1);
            for (int i = 0; i < _vertices.size(); i++)
            {
                auto e_t = new EdgeTask();
                Eigen::MatrixXd info_t(_model->getJointNum(), _model->getJointNum());
                info_t.setIdentity();
                if (i == 0 || i == _vertices.size() - 1)
                    info_t *= 10000;
                else
                    info_t *= weight;
                e_t->setInformation(info_t);
                e_t->vertices()[0] = _optimizer.vertex(i);
                auto v = dynamic_cast<const VertexRobotPos*>(_optimizer.vertex(i));
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
                std::list<std::string> links {"arm1_1", "arm1_2", "arm1_3", "arm1_4", "arm1_5", "arm1_6", "arm1_7", "ball1"};
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
            }

            else
                ROS_ERROR("unable to find collision_urdf");

            YAML_PARSE_OPTION(_optimizer_config[vc_name], max_pair_link, int, 1e2);
            YAML_PARSE_OPTION(_optimizer_config["collisions"], weight, double, 1);
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
//    std::cout << fsec.count() << std::endl;

    // shift _time_vector elements
//    for (int i = 1; i < _time_vector.size(); i++)
//        _time_vector[i-1] = _time_vector[i];
//    _time_vector.back() = double(fsec.count());

    // average and publish
    std_msgs::Float32MultiArray time;
//    time.data.push_back(0);
//    for (auto i : _time_vector)
//        time.data[0] += i;

    auto t = ros::Time::now() - _init_time;
//    time.data[0] /= _time_vector.size();
//    time.data.push_back(t.toSec());
    time.data.push_back(fsec.count());
    time.data.push_back(t.toSec());
    _time_pub.publish(time);

    // publish residual error
    std_msgs::Float32 err_msg;
    double cum_err_coll = 0;
    double cum_err_vel = 0;
    for (auto vertex : vertices)
    {
        auto v = dynamic_cast<VertexRobotPos*>(vertex.second);

        double error = 0;
        for (auto edge : v->edges())
        {
            if (dynamic_cast<EdgeCollision*>(edge) != nullptr)
            {
                auto e = dynamic_cast<EdgeCollision*>(edge);
                e->setObstacles(_obstacles);
                e->computeError();
                error = 0;
                for (int i = 0; i < e->getError().size(); i++)
                    error += e->getError()(i) * e->getError()(i);
                cum_err_coll += std::sqrt(error);
            }
        }    
    }
    err_msg.data = cum_err_coll;
    _err_pub.publish(err_msg);

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

        _model->setJointPosition(v->estimate());
        _model->update();
        Eigen::Affine3d T;
        _model->getPose("arm1_7", T);
        visualization_msgs::Marker m;
        m.header.frame_id = "pelvis";
        m.header.stamp = ros::Time::now();
        m.id = v->id();
        m.action = visualization_msgs::Marker::ADD;
        m.type = visualization_msgs::Marker::SPHERE;
        m.color.r = 0; m.color.g = 1; m.color.b = 0; m.color.a = 1;
        m.scale.x = 0.01; m.scale.y = 0.01; m.scale.z = 0.01;
        m.pose.position.x = T.translation()(0);
        m.pose.position.y = T.translation()(1);
        m.pose.position.z = T.translation()(2);
        ma.markers.push_back(m);
    }

    _ee_trj_pub.publish(ma);
    _sol_pub.publish(solution);
}

void Optimizer::print()
{
    auto vertices = _optimizer.vertices();

    for (auto vertex : vertices)
    {
        auto v = dynamic_cast<VertexRobotPos*>(vertex.second);
//        std::cout << "------------------------" << std::endl;
//        std::cout << "VERTEX " << v->id() << std::endl;
//        std::cout << "q: " << v->estimate().transpose() << std::endl;
//        std::cout << "------------------------" << std::endl;

//        std::cout << "EDGES:" << std::endl;
        auto edges = v->edges();
        for (auto edge : edges)
        {
            if (dynamic_cast<EdgeCollision*>(edge) != nullptr)
            {
//                auto e = dynamic_cast<EdgeCollision*>(edge);
//                std::cout << "EdgeCollision: " << std::endl;
//                std::cout << "error: " << e->getError().transpose() << std::endl;
            }
            else if (dynamic_cast<EdgeRobotVel*>(edge) != nullptr)
            {
                auto e = dynamic_cast<EdgeRobotVel*>(edge);
                Eigen::VectorXd zero(_model->getJointNum());
                zero.setZero();
                if (e->getError() != zero)
                {
                    std::cout << "EdgeRobotVel: " << std::endl;
                    auto v0 = dynamic_cast<VertexRobotPos*>(e->vertex(0));
                    auto v1 = dynamic_cast<VertexRobotPos*>(e->vertex(1));
                    std::cout << "vertex0: " << v0->estimate() << std::endl;
                    std::cout << "vertex1: " << v1->estimate() << std::endl;
                    std::cout << "velocities: " << e->getVelocities().transpose() << std::endl;
                    std::cout << "error: " << e->getError().transpose() << std::endl;
                }
            }
            else if (dynamic_cast<EdgeTask*>(edge) != nullptr)
            {
//                auto e = dynamic_cast<EdgeTask*>(edge);
//                std::cout << "EdgeTask: " << std::endl;
//                std::cout << "error: " << e->getError().transpose() << std::endl;
            }
        }
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
    obs.size << 0.05, 0.05, 0.05;
    obs.id = _obstacles.size();
    _obstacles.push_back(obs);
    update_edges();

    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "pelvis";
    int_marker.header.stamp = ros::Time::now();
    int_marker.name = "obstacle_" + std::to_string(_obstacles.size());
    int_marker.description = "obstacle_" + std::to_string(_obstacles.size());

    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::SPHERE;
    m.pose.position.x = 0;
    m.pose.position.y = 0;
    m.pose.position.z = 0;
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.pose.orientation.w = 1;
    m.scale.x = obs.size(0); m.scale.y = obs.size(1); m.scale.z = obs.size(2);
    m.color.r = 1; m.color.g = 0; m.color.b = 0; m.color.a = 1;

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
