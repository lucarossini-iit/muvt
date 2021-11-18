#include <optimizer/optimizer.h>

using namespace XBot::HyperGraph;

Optimizer::Optimizer():
_nh(""),
_nhpr("~"),
_number_obs(0)
{
    init_load_model();
    init_optimizer();
}

Optimizer::Optimizer(std::vector<Eigen::VectorXd> vertices):
_nh(""),
_nhpr("~"),
_vertices(vertices),
_number_obs(0)
{
    init_load_model();
    init_load_config();
    init_optimizer();
    init_vertices();

    _nh.subscribe("/obstacles", 10, &Optimizer::object_callback, this);
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

        obs.frame_id = object.header.frame_id;
        obs.id = object.header.seq;

        _obstacles.push_back(obs);
    }

    auto edges = _optimizer.edges();
    double cum_err = 0;

    for (auto e : edges)
    {
        if (dynamic_cast<EdgeCollision*>(e) != nullptr)
        {
            auto edge = dynamic_cast<EdgeCollision*>(e);
            edge->setObstacles(_obstacles);
            edge->computeError();

            double error = 0;
            for (int i = 0; i < edge->getError().size(); i++)
                error += pow(edge->getError()(i), 2);
            cum_err += std::sqrt(error);

            double thresh = 1e-3;
            if (cum_err > thresh)
            {
                auto v = dynamic_cast<VertexRobotPos*>(e->vertices()[0]);
                v->setFixed(false);
            }
            else
            {
                auto v = dynamic_cast<VertexRobotPos*>(e->vertices()[0]);
                v->setFixed(true);
            }
        }
    }
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
    _nhpr.getParam("planner_config", optimizer_config_string);

    _optimizer_config = YAML::Load(optimizer_config_string);
}

void Optimizer::init_optimizer()
{
    auto linearSolver = g2o::make_unique<LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();
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
        std::cout << "No constraints were defined" << std::endl;
        return;
    }

    // I think it is useless to init edges for collision avoidance since the sensor is not initialized yet

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
        else if (vc_name == "collisions")
        {
            urdf::ModelSharedPtr collision_urdf = std::make_shared<urdf::Model>();
            if (collision_urdf->initParam("collision_urdf"))
            {
                _cld = std::make_shared<ComputeLinksDistance>(*_model, collision_urdf);
            }
            else
                ROS_ERROR("unable to find collision_urdf");

            YAML_PARSE_OPTION(_optimizer_config[vc_name]["max_pair_link"], max_pair_link, int, 1e6);
            for (int i = 0; i < _vertices.size(); i++)
            {
                auto e_coll = new EdgeCollision(_model, _cld, max_pair_link);
                Eigen::MatrixXd info(max_pair_link, max_pair_link);
                info.setIdentity();
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
}
