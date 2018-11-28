#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

class AlignFrames {
  public:
    AlignFrames(ros::NodeHandle& nh);
    void setup();

  private:
    ros::NodeHandle& _nh;
    ros::Subscriber _gps_sub;
    ros::Subscriber _vio_sub;
    bool _offset_initialized;
    double _vio_pose_covNorm;
    tf::Transform _vio_pose;
    tf::Transform _pose_offset;
    tf::TransformBroadcaster _br;

    void gpsCallback(const geometry_msgs::PoseStamped& msg);
    void vioCallback(const nav_msgs::Odometry& msg);
    double l2_norm(std::vector<double> const& v);
};

AlignFrames::AlignFrames(ros::NodeHandle& nh) : _nh(nh) {
  _offset_initialized = false;
  _vio_pose.setIdentity();
  _pose_offset.setIdentity();
}

void AlignFrames::gpsCallback(const geometry_msgs::PoseStamped& msg) {
  if(!_offset_initialized && _vio_pose_covNorm > 1.0) {
    tf::Transform gps_pose;
    tf::poseMsgToTF(msg.pose, gps_pose);
    _pose_offset = _vio_pose.inverse() * gps_pose;
    _offset_initialized = true;
    ROS_INFO("initialized GPS VIO offset TF");
  }

  _br.sendTransform(tf::StampedTransform(_pose_offset, msg.header.stamp, "world", "map"));
}

void AlignFrames::vioCallback(const nav_msgs::Odometry& msg) {
  tf::Transform vio_pose;
  tf::poseMsgToTF(msg.pose.pose, vio_pose);
  _vio_pose = vio_pose;

  auto cov = msg.pose.covariance;
  std::vector<double> pose_cov(std::begin(cov), std::end(cov));

  _vio_pose_covNorm = l2_norm(pose_cov);
}

void AlignFrames::setup() {
    _gps_sub = _nh.subscribe("mavros/local_position/pose", 10, &AlignFrames::gpsCallback, this);
    _vio_sub = _nh.subscribe("rovio/odometry", 10, &AlignFrames::vioCallback, this);  
}

double AlignFrames::l2_norm(std::vector<double> const& v) {
    double accum = 0.;
    for (int i = 0; i < v.size(); ++i) {
        accum += v[i] * v[i];
    }
    return sqrt(accum);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster");

  ros::NodeHandle nh;
  
  AlignFrames *frame_aligner = new AlignFrames(nh);
  frame_aligner->setup();

  ros::spin();
  return 0;
};
