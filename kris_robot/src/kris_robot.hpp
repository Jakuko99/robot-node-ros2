// #include <cstdio>
#include <string>
#include <sstream>
#include <fstream>
#include <wiringPi.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"

#define I2C_BUS "/dev/i2c-1"

#define MOTOR1_DIR_PIN 17
#define MOTOR1_STEP_PIN 27
#define MOTOR2_DIR_PIN 23
#define MOTOR2_STEP_PIN 24

#define BUTTON1_PIN 5
#define BUTTON2_PIN 6

#define ACC_ADDR 0x19
#define DISP_ADDR 0x3C
#define BME280_ADDR 0x76

class KRISRobot : public rclcpp::Node
{
public:
  KRISRobot(std::string);
  ~KRISRobot();
  void update_state();

private:
  void setup_gpio();
  void publish_scan();
  void publish_odometry();
  void publish_urdf();
  void publish_tf();
  void publish_joint_state();
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  float x;
  float y;
  float theta;
  float v_linear;
  float v_angular;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr urdf_pub;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr tf_pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
};