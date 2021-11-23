#include <optimizer/optimizer.h>

using namespace XBot::HyperGraph;

Optimizer::Optimizer():
_nh("optimizer"),
_nhpr("~"),
_number_obs(0)
{
    init_load_model();
    init_optimizer();
}

Optimizer::Optimizer(std::vector<Eigen::VectorXd> vertices):
_nh("optimizer"),
_nhpr("~"),
_vertices(vertices),
_number_obs(0)
{
    init_load_model();
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
                std::cout << "EdgeCollision found!" << std::endl;
                auto e = dynamic_cast<EdgeCollision*>(edge);
                e->setObstacles(_obstacles);
                e->computeError();
                error = 0;
                for (int i = 0; i < e->getError().size(); i++)
                    error += e->getError()(i) * e->getError()(i);
                cum_err += std::sqrt(error);
//                std::cout << "EdgeCollision error: " << std::sqrt(error) << std::endl;
            }
            else if (dynamic_cast<EdgeTask*>(edge) != nullptr)
            {
                std::cout << "EdgeTask found!" << std::endl;
                auto e = dynamic_cast<EdgeTask*>(edge);
                e->computeError();
                error = 0;
                for (int i = 0; i < e->getError().size(); i++)
                    error += e->getError()(i) * e->getError()(i);
                cum_err += std::sqrt(error);
                if (std::sqrt(error) > 1e-1)
                    std::cout << "EdgeTask error: " << std::sqrt(error) << std::endl;
            }

            double thresh = 1e-3;
            if (cum_err > thresh)
                v->setFixed(false);
            else
                v->setFixed(true);
        }
    }


}

void Optimizer::run()
{
    optimize();
}

void Optimizer::init_load_model()
{
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

    // advertise and subscribe to topics
    _obj_sub = _nh.subscribe("obstacles", 10, &Optimizer::object_callback, this);
    _sol_pub = _nh.advertise<trajectory_msgs::JointTrajectory>("solution", 10, true);
}

void Optimizer::init_optimizer()
{
    auto linearSolver = g2o::make_unique<LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>>();
    auto blockSolver = g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver));
    g2o::OptimizationAlgorithm *algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));

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
            if (_vertices.size() == 1)
            {
                auto e_vel = new EdgeRobotUnaryVel(_model);
                Eigen::MatrixXd info(_model->getJointNum(), _model->getJointNum());
                info.setIdentity(); info *= 0.1;
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
        else
        {
            ROS_WARN("%s not found!",vc_name);
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
    std::cout << "fixed: " << counter_non_active << std::endl;
    std::cout << "active: " << counter_active << std::endl;
    auto tic = std::chrono::high_resolution_clock::now();

    _optimizer.initializeOptimization();
    _optimizer.optimize(_iterations);

    auto toc = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> fsec = toc - tic;
    std::cout << "optimization done in " << fsec.count() << " seconds!" << std::endl;

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
            for (auto edge : _optimizer.edges())
            {
                if (dynamic_cast<EdgeRobotUnaryVel*>(edge) != nullptr)
                {
                    auto e = dynamic_cast<EdgeRobotUnaryVel*>(edge);
                    e->setRef(v->estimate());
                }
            }
        }
    }

    _sol_pub.publish(solution);
}
