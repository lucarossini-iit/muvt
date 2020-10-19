#include <environment/obstacle.h>

Obstacle::Obstacle(Eigen::Affine3d pose, std::string name):
    _name(name),
    _nh()
{
    _pub = _nh.advertise<visualization_msgs::Marker>(name + "_marker", 100, true);
    
    _m.header.frame_id = "world";
    _m.header.stamp = ros::Time::now();
    
    _m.pose.position.x = pose.translation().x();
    _m.pose.position.y = pose.translation().y();
    _m.pose.position.z = pose.translation().z();
    
    Eigen::Quaternion<double> q(pose.linear());
    _m.pose.orientation.x = q.coeffs()(0);
    _m.pose.orientation.y = q.coeffs()(1);
    _m.pose.orientation.z = q.coeffs()(2);
    _m.pose.orientation.w = q.coeffs()(3);
    
    _m.action = visualization_msgs::Marker::ADD;
    _m.type = visualization_msgs::Marker::POINTS;
}

void Obstacle::setName (std::__cxx11::string name) 
{
    _name = name;
}

std::__cxx11::string Obstacle::getName() const 
{
    return _name;
}

visualization_msgs::Marker Obstacle::getMarker() const 
{
    return _m;
}


