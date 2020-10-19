#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/node_handle.h>

#include <visualization_msgs/Marker.h>

#include <Eigen/Geometry>

class Obstacle
{
public:
    Obstacle(Eigen::Affine3d pose, std::string name);
    
    void setName(std::string name);
    std::string getName() const;
    
    visualization_msgs::Marker getMarker() const;

private:
    ros::NodeHandle _nh;
    ros::Publisher _pub;
    
    std::string _name;
    
    visualization_msgs::Marker _m;
};

