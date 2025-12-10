#include "kris_robot.hpp"

using std::placeholders::_1;
// #define DEBUG

StepperMotor::StepperMotor(int step_pin, int dir_pin, int steps_per_rev = 200, double wheel_diameter = 0.065)
    : step_pin(step_pin),
      dir_pin(dir_pin),
      steps_per_rev(steps_per_rev),
      wheel_diameter_(wheel_diameter),
      speed_hz_(0.0),
      running_(false),
      impulse_cnt(0),
      prev_impulse_cnt(0),
      direction(0)
{
  steps_per_meter_ = steps_per_rev / (M_PI * wheel_diameter_);
  pinMode(step_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);

  // Start motor thread
  motor_thread_ = std::thread(&StepperMotor::run_motor, this);
  RCLCPP_INFO(rclcpp::get_logger("KRISRobot"), "StepperMotor initialized on step pin %d and dir pin %d", step_pin, dir_pin);
}

StepperMotor::~StepperMotor()
{
  running_ = false;
  if (motor_thread_.joinable())
  {
    motor_thread_.join();
  }
}

void StepperMotor::set_speed(double speed_m_s)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (speed_m_s < 0)
  {
    direction = 1;
    digitalWrite(dir_pin, HIGH);
  }
  else if (speed_m_s > 0)
  {
    direction = -1;
    digitalWrite(dir_pin, LOW);
  }
  else if (speed_m_s == 0)
  {
    direction = 0;
    running_ = false;
    speed_hz_ = 0;
    return;
  }

  speed_hz_ = std::abs(speed_m_s) * steps_per_meter_;
  if (speed_hz_ > 99.0)
  {
    RCLCPP_WARN(rclcpp::get_logger("KRISRobot"), "StepperMotor speed limited to 99 Hz from %f Hz", speed_hz_);
    speed_hz_ = 99.0; // Limit max speed to 100 Hz
  }
  running_ = (speed_hz_ > 0);
}

long int StepperMotor::get_impulse_count()
{
  std::lock_guard<std::mutex> lock(mutex_);
  long int delta = impulse_cnt - prev_impulse_cnt;
  prev_impulse_cnt = impulse_cnt;
  return delta;
}

void StepperMotor::run_motor()
{
  while (true)
  {
    double local_speed_hz;
    bool local_running;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      local_speed_hz = speed_hz_;
      local_running = running_;
    }

    if (local_running && local_speed_hz > 0)
    {
      double delay_s = 1.0 / local_speed_hz;
      impulse_cnt += direction;
      digitalWrite(step_pin, HIGH);
      std::this_thread::sleep_for(std::chrono::milliseconds(T_IMPULSE));
      digitalWrite(step_pin, LOW);

      std::this_thread::sleep_for(std::chrono::duration<double>(delay_s - (T_IMPULSE / 1000.0)));
    }
    else
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(T_IMPULSE));
    }
  }
}

KRISRobot::KRISRobot(std::string node_name) : rclcpp::Node(node_name),
                                              v_linear(0.0),
                                              v_angular(0.0),
                                              x_pos(0.0),
                                              y_pos(0.0),
                                              theta(0.0)
{
  this->setup_gpio();

  this->declare_parameter("robot_description", "");
  this->declare_parameter("base_frame_id", "base_link");
  this->declare_parameter("odom_frame_id", "odom");
  this->declare_parameter("robot_namespace", "");

  this->robot_namespace = this->get_parameter("robot_namespace").as_string();
  this->base_frame_id = this->get_parameter("base_frame_id").as_string();
  this->odom_frame_id = this->get_parameter("odom_frame_id").as_string();

  this->urdf_pub = this->create_publisher<std_msgs::msg::String>(robot_namespace + "/robot_description", rclcpp::QoS(RMW_QOS_POLICY_RELIABILITY_RELIABLE));
  this->odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(robot_namespace + "/odom", rclcpp::QoS(10));
  this->cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      robot_namespace + "/cmd_vel", rclcpp::QoS(10), std::bind(&KRISRobot::cmd_vel_callback, this, _1));
  this->tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  left_motor = std::make_shared<StepperMotor>(MOTOR_LEFT_STEP_PIN, MOTOR_LEFT_DIR_PIN, STEPS_PER_REV, WHEEL_DIAMETER);
  right_motor = std::make_shared<StepperMotor>(MOTOR_RIGHT_STEP_PIN, MOTOR_RIGHT_DIR_PIN, STEPS_PER_REV, WHEEL_DIAMETER);
  // last_tick = this->now();
  this->publish_transforms();

  RCLCPP_INFO(this->get_logger(), "KRIS Robot node initialized");
}

KRISRobot::~KRISRobot()
{
  left_motor->set_speed(0);
  right_motor->set_speed(0);

  // properly dealocate objects
  left_motor.reset();
  right_motor.reset();

  RCLCPP_INFO(this->get_logger(), "KRIS Robot node shutting down");
}

void KRISRobot::setup_gpio()
{
  if (wiringPiSetupGpio() == -1)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize wiringPi");
    rclcpp::shutdown();
    return;
  }

  pinMode(BUTTON1_PIN, INPUT);
  pinMode(BUTTON2_PIN, INPUT);
  RCLCPP_INFO(this->get_logger(), "GPIO setup complete");
}

void KRISRobot::publish_urdf()
{
  std::ifstream urdf_file(this->get_parameter("robot_description").as_string());
  if (!urdf_file.is_open())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to open URDF file");
    return;
  }
  std::stringstream buffer;
  buffer << urdf_file.rdbuf();
  urdf_file.close();

  auto message = std_msgs::msg::String();
  message.data = buffer.str();

  urdf_pub->publish(message);
}

void KRISRobot::publish_transforms()
{
  geometry_msgs::msg::TransformStamped tf_msg_lidar;
  tf_msg_lidar.header.stamp = this->now();
  tf_msg_lidar.header.frame_id = base_frame_id;
  tf_msg_lidar.child_frame_id = robot_namespace + "_laser_frame";
  tf_msg_lidar.transform.translation.x = 0.0;
  tf_msg_lidar.transform.translation.y = 0.0;
  tf_msg_lidar.transform.translation.z = LIDAR_HEIGHT;
  tf_msg_lidar.transform.rotation.x = 0.0;
  tf_msg_lidar.transform.rotation.y = 0.0;
  tf_msg_lidar.transform.rotation.z = 0.0;
  tf_msg_lidar.transform.rotation.w = 1.0;
  tf_broadcaster->sendTransform(tf_msg_lidar);

  geometry_msgs::msg::TransformStamped tf_msg_lwheel;
  tf_msg_lwheel.header.stamp = this->now();
  tf_msg_lwheel.header.frame_id = base_frame_id;
  tf_msg_lwheel.child_frame_id = robot_namespace + "_lwheel";
  tf_msg_lwheel.transform.translation.x = 0.0;
  tf_msg_lwheel.transform.translation.y = WHEEL_DISTANCE / 2.0;
  tf_msg_lwheel.transform.translation.z = 0.0;
  tf_msg_lwheel.transform.rotation.x = 0.0;
  tf_msg_lwheel.transform.rotation.y = 0.0;
  tf_msg_lwheel.transform.rotation.z = 0.0;
  tf_msg_lwheel.transform.rotation.w = 1.0;
  tf_broadcaster->sendTransform(tf_msg_lwheel);

  geometry_msgs::msg::TransformStamped tf_msg_rwheel;
  tf_msg_rwheel.header.stamp = this->now();
  tf_msg_rwheel.header.frame_id = base_frame_id;
  tf_msg_rwheel.child_frame_id = robot_namespace + "_rwheel";
  tf_msg_rwheel.transform.translation.x = 0.0;
  tf_msg_rwheel.transform.translation.y = -WHEEL_DISTANCE / 2.0;
  tf_msg_rwheel.transform.translation.z = 0.0;
  tf_msg_rwheel.transform.rotation.x = 0.0;
  tf_msg_rwheel.transform.rotation.y = 0.0;
  tf_msg_rwheel.transform.rotation.z = 0.0;
  tf_msg_rwheel.transform.rotation.w = 1.0;
  tf_broadcaster->sendTransform(tf_msg_rwheel);
}

float KRISRobot::time_diff(const builtin_interfaces::msg::Time &start, const builtin_interfaces::msg::Time &end)
{
  return (end.sec - start.sec) + (end.nanosec - start.nanosec) / 1e9;
}

void KRISRobot::publish_odometry()
{
  nav_msgs::msg::Odometry odom_msg;
  builtin_interfaces::msg::Time t = this->now();
  odom_msg.header.stamp = t;
  odom_msg.header.frame_id = odom_frame_id;
  odom_msg.child_frame_id = base_frame_id;
  odom_msg.pose.pose.position.z = 0.0;

  // float dt = time_diff(last_tick, t);
  // last_tick = t;

  long int delta_left_cnt = left_motor->get_impulse_count();
  long int delta_right_cnt = right_motor->get_impulse_count();

  double distance_left = (delta_left_cnt / STEPS_PER_METER);
  double distance_right = (delta_right_cnt / STEPS_PER_METER);

  double d_theta = (distance_left - distance_right) / (WHEEL_BASE);

  float dx, dy;

  if (fabs(d_theta) < FLT_EPSILON)
  {
    dx = (distance_left + distance_right) / 2.0;
    dy = (pow(distance_left, 2) - pow(distance_right, 2)) / (4 * WHEEL_BASE);
  }
  else
  {
    double R_turn = (distance_left + distance_right) / (2 * d_theta);
    dx = R_turn * sin(d_theta);
    dy = R_turn * (1 - cos(d_theta));
  }

  x_pos += dx * cos(theta) - dy * sin(theta);
  y_pos += dx * sin(theta) + dy * cos(theta);

  theta += d_theta;
  theta = fmod(theta, 2 * M_PI);

  odom_msg.pose.pose.position.x = x_pos;
  odom_msg.pose.pose.position.y = y_pos;
  odom_msg.pose.pose.orientation.z = sin(theta / 2.0);
  odom_msg.pose.pose.orientation.w = cos(theta / 2.0);
  odom_msg.twist.twist.linear.x = v_linear;
  odom_msg.twist.twist.angular.z = v_angular;
  odom_pub->publish(odom_msg);

  // Publish TF
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = t;
  tf_msg.header.frame_id = odom_frame_id;
  tf_msg.child_frame_id = base_frame_id;
  tf_msg.transform.translation.x = odom_msg.pose.pose.position.x;
  tf_msg.transform.translation.y = odom_msg.pose.pose.position.y;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation.z = odom_msg.pose.pose.orientation.z;
  tf_msg.transform.rotation.w = odom_msg.pose.pose.orientation.w;
  tf_broadcaster->sendTransform(tf_msg);
}

void KRISRobot::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
#ifdef DEBUG
  RCLCPP_INFO(this->get_logger(), "Recieved cmd_vel: linear.x = %f, angular.z = %f\n", msg->linear.x, msg->angular.z);
#endif
  this->v_linear = msg->linear.x;   // Set linear velocity
  this->v_angular = msg->angular.z; // Set angular velocity

  // Differential drive kinematics
  double v_left = (v_linear - v_angular * WHEEL_BASE / 2.0) / WHEEL_DIAMETER;
  double v_right = (v_linear + v_angular * WHEEL_BASE / 2.0) / WHEEL_DIAMETER;

  left_motor->set_speed(v_left);
  right_motor->set_speed(v_right);
}

void KRISRobot::update_state()
{
#ifdef DEBUG
  RCLCPP_INFO(this->get_logger(), "Updating node state");
#endif

  publish_urdf();
  publish_transforms();
  publish_odometry();
}