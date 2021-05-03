#include <simulator/optimizer.h>

using namespace XBot::HyperGraph;
using namespace g2o;

void interactive_markers_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    ROS_INFO_STREAM( feedback->marker_name << " is now at "
          << feedback->pose.position.x << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z );
}

Optimizer::Optimizer():
_nhpr("~"),
_nh()
{
    init_load_simulator();
    init_load_optimizer();
    load_vertices();
    load_edges();

    // interactive marker server for obstacles
    _server = std::make_shared<interactive_markers::InteractiveMarkerServer>("optimizer");
    
    _create_obs_srv = _nh.advertiseService("create_obstacle", &Optimizer::create_obstacle_service, this);
    _obs_pub = _nh.advertise<visualization_msgs::MarkerArray>("obstacles", 10, false);
    _trj_pub = _nh.advertise<visualization_msgs::MarkerArray>("trajectory", 10, false);
}

void Optimizer::run()
{
    tf::Transform t;
    t.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    t.setRotation(q);

    tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "map", "world"));

    publish();
    
    ros::spinOnce();
}

void Optimizer::publish()
{
    visualization_msgs::MarkerArray ma, ma_obs;
    visualization_msgs::Marker m;
    
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
        m.id = i;
        
        m.action = visualization_msgs::Marker::ADD;
        m.type = visualization_msgs::Marker::CUBE;
        m.pose.position.x = _trajectory[i](0);
        m.pose.position.y = _trajectory[i](1);
        m.pose.position.z = _trajectory[i](2);
        m.pose.orientation.x = 0;
        m.pose.orientation.y = 0;
        m.pose.orientation.z = 0;
        m.pose.orientation.w = 1;
        m.scale.x = 0.05; m.scale.y = 0.05; m.scale.z = 0.05;
        m.color.r = 0; m.color.g = 1; m.color.b = 0; m.color.a = 1;
        ma.markers.push_back(m);
    }
    
    for (int i = 0; i < _obstacles.size(); i++)
    {
        m.header.frame_id = "world";
        m.header.stamp = ros::Time::now();
        m.id = i;
        
        m.action = visualization_msgs::Marker::ADD;
        m.type = visualization_msgs::Marker::CUBE;
        m.pose.position.x = _obstacles[i](0);
        m.pose.position.y = _obstacles[i](1);
        m.pose.position.z = _obstacles[i](2);
        m.pose.orientation.x = 0;
        m.pose.orientation.y = 0;
        m.pose.orientation.z = 0;
        m.pose.orientation.w = 1;
        m.scale.x = 0.2; m.scale.y = 0.2; m.scale.z = 0.2;
        m.color.r = 1; m.color.g = 0; m.color.b = 0; m.color.a = 1;
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
    
    _optimizer.setVerbose(false);
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
    load_vertices();

    PointGrid points = _simulator->getVertices();
    
    std::cout << "adding edges to optimizer..." << std::endl;      
    for (int i = 0; i < _obstacles.size(); i++)
    {
        for(int j = 0; j < points.size(); j++)
        {
            auto e = new EdgeScalarXYZ;
            e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
            e->vertices()[0] = _optimizer.vertex(j);
            e->setObstacle(_obstacles[i]);
            _optimizer.addEdge(e);
        }
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
    std::cout << "done!" << std::endl;
    
    _optimizer.initializeOptimization();
    _optimizer.optimize(10);
    _optimizer.save(std::cout);
}

bool Optimizer::create_obstacle_service (teb_test::SetObstacle::Request& req, teb_test::SetObstacle::Response& res) 
{
    if (!req.add)
    {
        _server->clear();
        _server->applyChanges();
        _obstacles.clear();
        load_edges();
        res.status = true;
        return res.status;
    }
    
    Eigen::Vector3d position;
    position << req.pose.position.x, req.pose.position.y, req.pose.position.z;
    _obstacles.push_back(position);

    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "world";
    int_marker.header.stamp = ros::Time::now();
    int_marker.name = "obstacle_" + std::to_string(_obstacles.size());
    int_marker.description = "obstacle_" + std::to_string(_obstacles.size());

    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::CUBE;
    m.pose.position.x = req.pose.position.x;
    m.pose.position.y = req.pose.position.y;
    m.pose.position.z = req.pose.position.z;
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.pose.orientation.w = 1;
    m.scale.x = 0.2; m.scale.y = 0.2; m.scale.z = 0.2;
    m.color.r = 1; m.color.g = 0; m.color.b = 0; m.color.a = 1;

    visualization_msgs::InteractiveMarkerControl obs_control;
    obs_control.always_visible = true;
    obs_control.markers.push_back(m);
    int_marker.controls.push_back(obs_control);

    visualization_msgs::InteractiveMarkerControl move_control;
    move_control.name = "move_x";
    move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(move_control);

    visualization_msgs::InteractiveMarkerControl move_control_y;
    move_control_y.name = "move_y";
    Eigen::Matrix3d R;
    R << cos(M_PI/2), -sin(M_PI/2), 0, sin(M_PI/2), cos(M_PI/2), 0, 0, 0, 1;
    Eigen::Quaternion<double> quat(R);
    tf::quaternionEigenToMsg(quat, move_control_y.orientation);
    move_control_y.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(move_control_y);

    _server->insert(int_marker);
    _server->setCallback(int_marker.name, &interactive_markers_feedback);
    _server->applyChanges();

    load_edges();
    
    res.status = true;
    return res.status;
}


