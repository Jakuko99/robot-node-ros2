// #include <cstdio>
#include <string>
#include <sstream>
#include <fstream>
#include <wiringPi.h>
#include <thread>
#include <chrono>
#include <cmath>
#include <mutex>
#include <atomic>
#include <csignal>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"

#define I2C_BUS "/dev/i2c-1"

#define MOTOR_LEFT_DIR_PIN 8
#define MOTOR_LEFT_STEP_PIN 7
#define MOTOR_RIGHT_DIR_PIN 23
#define MOTOR_RIGHT_STEP_PIN 24

#define BUTTON1_PIN 5
#define BUTTON2_PIN 6
#define PWM_OUTPUT 18

#define WHEEL_BASE 0.085     // Distance between wheels in meters
#define WHEEL_DIAMETER 0.066 // Wheel diameter in meters
#define STEPS_PER_REV 400    // Steps per revolution for the stepper motor

#define ACC_ADDR 0x19
#define DISP_ADDR 0x3C
#define BME280_ADDR 0x76

class StepperMotor
{
public:
  StepperMotor(int, int, int, double);
  void set_speed(double);
  ~StepperMotor();

private:
  void run_motor();

  int dir_pin;
  int step_pin;
  int steps_per_rev;
  double wheel_diameter_;
  double steps_per_meter_;
  double speed_hz_;
  bool running_;
  std::mutex mutex_;
  std::thread motor_thread_;
};

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

  std::string base_frame_id = "base_link";
  std::string odom_frame_id = "odom";

  std::shared_ptr<StepperMotor> left_motor;
  std::shared_ptr<StepperMotor> right_motor;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr urdf_pub;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr tf_pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
};