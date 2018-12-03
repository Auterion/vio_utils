#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

class PathBroadcaster {
  public:
    PathBroadcaster(ros::NodeHandle& nh);
    void setup();

  private:
    ros::NodeHandle& _nh;
    ros::Subscriber _gps_sub;
    ros::Subscriber _rovio_sub;
    ros::Publisher _gps_path_pub;
    ros::Publisher _rovio_path_pub;
    nav_msgs::Path _gps_path;
    nav_msgs::Path _rovio_path;

    void gpsCallback(const geometry_msgs::PoseStamped& msg);
    void vioCallback(const nav_msgs::Odometry& msg);
};

PathBroadcaster::PathBroadcaster(ros::NodeHandle& nh) : _nh(nh) {}

void PathBroadcaster::gpsCallback(const geometry_msgs::PoseStamped& msg) {
  _gps_path.header = msg.header;
  geometry_msgs::PoseStamped gps_pose = msg;

  _gps_path.poses.push_back(gps_pose);
  _gps_path_pub.publish(_gps_path);
}

void PathBroadcaster::vioCallback(const nav_msgs::Odometry& msg) {
  _rovio_path.header = msg.header;
  geometry_msgs::PoseStamped vio_pose;
  vio_pose.pose = msg.pose.pose;
  vio_pose.header = msg.header;

  _rovio_path.poses.push_back(vio_pose);
  _rovio_path_pub.publish(_rovio_path);
}

void PathBroadcaster::setup() {
  _gps_sub = _nh.subscribe("mavros/local_position/pose", 10, &PathBroadcaster::gpsCallback, this);
  _rovio_sub = _nh.subscribe("rovio/odometry", 10, &PathBroadcaster::vioCallback, this);  

  _gps_path_pub = _nh.advertise<nav_msgs::Path>("path_gps", 1000);
  _rovio_path_pub = _nh.advertise<nav_msgs::Path>("path_vio", 1000);
}   

int main(int argc, char** argv){
  ros::init(argc, argv, "path_broadcaster");

  ros::NodeHandle nh;
  
  PathBroadcaster *frame_aligner = new PathBroadcaster(nh);
  frame_aligner->setup();

  ros::spin();
  return 0;
};
