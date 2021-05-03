#include <simulator/optimizer.h>

using namespace XBot::HyperGraph;
using namespace g2o;

Optimizer::Optimizer():
_nhpr("~"),
_nh()
{
    init_load_simulator();
    init_load_optimizer();
    load_vertices();
    
    _create_obs_srv = _nh.advertiseService("create_obstacle", &Optimizer::create_obstacle_service, this);
    _obs_pub = _nh.advertise<visualization_msgs::MarkerArray>("obstacles", 10, true);
    _trj_pub = _nh.advertise<visualization_msgs::MarkerArray>("trajectory", 10, true);
    
    tf::Transform t;
    t.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    t.setRotation(q);
    
    tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "map", "world"));
}

void Optimizer::run()
{
    solve(100);
    
    ros::spinOnce();
}

void Optimizer::solve (int max_iter) 
{
    visualization_msgs::MarkerArray ma, ma_obs;
    visualization_msgs::Marker m;
    
    _optimizer.optimize(max_iter);
    
    if (_trajectory.size() > 0 )
        _trajectory.clear();
    if (ma.markers.size() > 0)
        ma.markers.clear();
    if (ma_obs.markers.size() > 0)
        ma_obs.markers.clear();
    
    auto vertex_map = _optimizer.vertices();
    for (auto v : vertex_map)
    {
        auto vertex = static_cast<const VertexPointXYZ*>(v.second);
        _trajectory.push_back(Eigen::Vector3d(vertex->estimate()));
    }
    
    for (int i = 0; i < _trajectory.size(); i++)
    {
        m.header.frame_id = "world";
        m.header.stamp = ros::Time::now();
        
        m.action = visualization_msgs::Marker::ADD;
        m.type = visualization_msgs::Marker::CUBE;
        m.scale.x = 0.05; m.scale.y = 0.05; m.scale.z = 0.05;
        ma.markers.push_back(m);
    }
    
    for (int i = 0; i < _obstacles.size(); i++)
    {
        m.header.frame_id = "world";
        m.header.stamp = ros::Time::now();
        
        m.action = visualization_msgs::Marker::ADD;
        m.type = visualization_msgs::Marker::CUBE;
        m.scale.x = 0.2; m.scale.y = 0.2; m.scale.z = 0.2;
        ma_obs.markers.push_back(m);
    }
    
    _trj_pub.publish(ma);
    _obs_pub.publish(ma_obs);
}


void Optimizer::init_load_simulator()
{
    int n;
    double distance;
    if (!_nhpr.getParam("n", n))
        std::runtime_error("Missing mandatory parameter 'n'");
    if (!_nhpr.getParam("distance", distance))
        std::runtime_error("Missing mandatory parameter 'distance'");

    _simulator = std::make_shared<Simulator>(n, distance);
}

void Optimizer::init_load_optimizer() 
{
    auto linearSolver = g2o::make_unique<LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>>();
    auto blockSolver = g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver));
    g2o::OptimizationAlgorithm *algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
    
    _optimizer.setVerbose(true);
    _optimizer.setAlgorithm(algorithm);
}

void Optimizer::load_vertices() 
{
    std::cout << "adding vertices to optimizer..." << std::endl;
    PointGrid points = _simulator->getVertices();   
    for (int i = 0; i < points.size(); i++)
    {       
        auto v = new VertexPointXYZ;
        v->setEstimate(points[i].point);
        v->setId(i);
        if (i == 0 || i == points.size() - 1)
            v->setFixed(true);
        _optimizer.addVertex(v);
    }
    std::cout << "done!" << std::endl;
}

void Optimizer::load_edges() 
{
    _optimizer.clear();
    
    std::cout << "adding edges to optimizer..." << std::endl;
    PointGrid points = _simulator->getVertices();   
    Eigen::Vector3d obs;
    obs << 0.5, 0.1, 0.0;
    for (int i = 0; i < points.size(); i++)
    {
        auto e = new EdgeScalarXYZ;
        e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        e->vertices()[0] = _optimizer.vertex(i);
        e->setObstacle(obs);
        _optimizer.addEdge(e);
    }
    std::cout << "done!" << std::endl;

    std::cout << "adding binary edges to optimizer" << std::endl;
    for (int i = 0; i < points.size() - 1; i++)
    {
        auto e = new EdgeDistance;
        e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        e->vertices()[0] = _optimizer.vertex(i);
        e->vertices()[1] = _optimizer.vertex(i+1);
        _optimizer.addEdge(e);
    }
    
    _optimizer.initializeOptimization();
}

bool Optimizer::create_obstacle_service (teb_test::SetObstacle::Request& req, teb_test::SetObstacle::Response& res) 
{
    if (!req.add)
    {
        _obstacles.clear();
        res.status = true;
        return res.status;
    }
    
    Eigen::Vector3d position;
    position << req.pose.position.x, req.pose.position.y, req.pose.position.z;
    _obstacles.push_back(position);
    
    load_edges();
    
    res.status = true;
    return res.status;
}

