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
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"

#define I2C_BUS "/dev/i2c-1"

#define MOTOR_LEFT_DIR_PIN 17
#define MOTOR_LEFT_STEP_PIN 27
#define MOTOR_RIGHT_DIR_PIN 23
#define MOTOR_RIGHT_STEP_PIN 24

#define BUTTON1_PIN 5
#define BUTTON2_PIN 6
#define PWM_OUTPUT_PIN 18

#define WHEEL_BASE 0.085                                          // Distance between wheels in meters
#define WHEEL_DIAMETER 0.067                                      // Wheel diameter in meters
#define STEPS_PER_REV 2000                                        // Steps per revolution for the stepper motor
#define STEPS_PER_METER (STEPS_PER_REV / (M_PI * WHEEL_DIAMETER)) // Steps per meter
#define T_IMPULSE 10                                              // Stepper motor pulse ON time in milliseconds

#define ACC_ADDR 0x19
#define DISP_ADDR 0x3C
#define BME280_ADDR 0x76

class SoftwarePWM
{
public:
  SoftwarePWM(int pin, int frequency);
  void set_duty_cycle(int duty_cycle);
  ~SoftwarePWM();

private:
  void run_pwm();
  int pin_;
  int frequency_;
  int duty_cycle_;
  bool running_;
  std::thread pwm_thread_;
  std::mutex mutex_;
};

class StepperMotor
{
public:
  StepperMotor(int, int, int, double);
  void set_speed(double);
  long int get_impulse_count();
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

  volatile int direction;
  volatile long int impulse_cnt;
  volatile long int prev_impulse_cnt;
};

class KRISRobot : public rclcpp::Node
{
public:
  KRISRobot(std::string);
  ~KRISRobot();
  void update_state();

private:
  void setup_gpio();
  void publish_urdf();
  void publish_odometry();
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  float time_diff(const builtin_interfaces::msg::Time &start, const builtin_interfaces::msg::Time &end);

  float v_linear;
  float v_angular;

  float x_pos;
  float y_pos;
  float theta;

  std::string base_frame_id = "base_link";
  std::string odom_frame_id = "odom";
  std::string robot_namespace = "";

  std::shared_ptr<StepperMotor> left_motor;
  std::shared_ptr<StepperMotor> right_motor;
  std::shared_ptr<SoftwarePWM> software_pwm;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr urdf_pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  // builtin_interfaces::msg::Time last_tick;
};