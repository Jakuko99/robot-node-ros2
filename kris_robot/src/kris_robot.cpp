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
  if (speed_m_s > 0)
  {
    direction = 1;
    digitalWrite(dir_pin, HIGH);
  }
  else if (speed_m_s < 0)
  {
    direction = -1;
    digitalWrite(dir_pin, LOW);
  }
  else if (speed_m_s == 0)
  {
    direction = 0;
    running_ = false;
    speed_hz_ = 0;
    impulse_cnt = 0;
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

  left_motor = std::make_shared<StepperMotor>(MOTOR_LEFT_STEP_PIN, MOTOR_LEFT_DIR_PIN, STEPS_PER_REV, WHEEL_DIAMETER);
  right_motor = std::make_shared<StepperMotor>(MOTOR_RIGHT_STEP_PIN, MOTOR_RIGHT_DIR_PIN, STEPS_PER_REV, WHEEL_DIAMETER);
  // last_tick = this->now();
  RCLCPP_INFO(this->get_logger(), "KRIS Robot node initialized");
}

KRISRobot::~KRISRobot()
{
  left_motor->set_speed(0);
  right_motor->set_speed(0);
  // analogWrite(PWM_OUTPUT_PIN, 0);
  softPwmWrite(PWM_OUTPUT_PIN, 0); // Set PWM to 0%
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
  pinMode(PWM_OUTPUT_PIN, OUTPUT);
  softPwmCreate(PWM_OUTPUT_PIN, 0, 100);
  softPwmWrite(PWM_OUTPUT_PIN, 80); // 80% duty cycle
  /*pwmSetMode(PWM_MODE_MS);
  pwmSetRange(1024);
  pwmSetClock(32); // 19.2MHz / 32 = 600kHz PWM base frequency

  analogWrite(PWM_OUTPUT_PIN, 512); // 80% duty cycle
  */
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

  if (fabs(distance_left - distance_right) < FLT_EPSILON)
  {
    x_pos += (distance_left + distance_right) / 2.0;
    y_pos += (pow(distance_left, 2) - pow(distance_right, 2)) / (4 * WHEEL_BASE);
  }
  else
  {
    double R_turn = (distance_left + distance_right) / (2 * theta);
    x_pos += R_turn * sin(theta);
    y_pos += R_turn * (1 - cos(theta));
  }

  theta += (distance_left - distance_right) / (WHEEL_BASE);
  theta = (theta != 0) ? fmod(theta, 2 * M_PI) : FLT_EPSILON; // Keep theta within [0, 2π]  

  odom_msg.pose.pose.position.x = x_pos;
  odom_msg.pose.pose.position.y = y_pos;
  odom_msg.pose.pose.orientation.z = sin(theta / 2.0);
  odom_msg.pose.pose.orientation.w = cos(theta / 2.0);
  odom_msg.twist.twist.linear.x = v_linear;
  odom_msg.twist.twist.angular.z = v_angular;
  odom_pub->publish(odom_msg);
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
  publish_odometry();
}