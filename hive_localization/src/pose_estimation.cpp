#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/utils.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

auto createQuaternionMsgFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

struct Pose
{
 public:
  Pose() {}
  ~Pose() {}

  double getX() { return x_; }
  double getY() { return y_; }
  double getYaw() { return yaw_; }

  void setX(double x) { x_ = x; }
  void setY(double y) { y_ = y; }
  void setYaw(double yaw) { yaw_ = yaw; }

 private:
  double x_;
  double y_;
  double yaw_;
};

class RobotLocalization : public rclcpp::Node
{
 public:
  RobotLocalization(/* args */);
  ~RobotLocalization();
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr encoder_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr camera_odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void encoderOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void laserOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void cameraOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();

 private:
  /* data */
  Pose pose_from_encoder;
  Pose pose_from_camera;
  Pose pose_filtered;
};

RobotLocalization::RobotLocalization(/* args */) : Node("robot1_localization_node")
{
  encoder_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("robot1/odom_noise", 10,
    std::bind(&RobotLocalization::encoderOdomCallback, this, std::placeholders::_1));
  camera_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("robot1/camera_odom", 10,
    std::bind(&RobotLocalization::cameraOdomCallback, this, std::placeholders::_1));
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("robot1/filtered_odom", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&RobotLocalization::timerCallback, this));
}

RobotLocalization::~RobotLocalization()
{
}

void RobotLocalization::imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
  imu_msg->orientation;
}

void RobotLocalization::encoderOdomCallback(const nav_msgs::msg::Odometry::SharedPtr encoder_odom_msg)
{
  pose_from_encoder.setX(encoder_odom_msg->pose.pose.position.x);
  pose_from_encoder.setY(encoder_odom_msg->pose.pose.position.y);
  pose_from_encoder.setYaw(tf2::getYaw(encoder_odom_msg->pose.pose.orientation));
}

void RobotLocalization::cameraOdomCallback(const nav_msgs::msg::Odometry::SharedPtr camera_odom_msg)
{
  pose_from_camera.setX(camera_odom_msg->pose.pose.position.x);
  pose_from_camera.setY(camera_odom_msg->pose.pose.position.y);
  pose_from_camera.setYaw(tf2::getYaw(camera_odom_msg->pose.pose.orientation));
}

void RobotLocalization::timerCallback()
{
  /* TODO: 평균말고 다른 filtering 알고리즘 생각하기. EKF
   yaw는 imu만 쓸까 고민 중. 진짜 여기 어떻게 하지.
   파라미터 딸깍 한다 생각해서 아예 생각 안해놨는데
   */
  pose_filtered.setX((pose_from_encoder.getX() + pose_from_camera.getX()) / 2);
  pose_filtered.setY((pose_from_encoder.getY() + pose_from_camera.getY()) / 2);
  pose_filtered.setYaw((pose_from_encoder.getYaw() + pose_from_camera.getYaw()) / 2);

  // filtered odom publish
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = this->now();
  odom_msg.header.frame_id = "robot1_odom";
  odom_msg.child_frame_id = "robot1_base_link";
  odom_msg.pose.pose.position.x = pose_filtered.getX();
  odom_msg.pose.pose.position.y = pose_filtered.getY();
  odom_msg.pose.pose.position.z = 0;
  odom_msg.pose.pose.orientation = createQuaternionMsgFromYaw(pose_filtered.getYaw());
  odom_pub_->publish(odom_msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotLocalization>());
  rclcpp::shutdown();
  return 0;
}