#include "kris_robot.hpp"

using std::placeholders::_1;
// #define DEBUG

StepperMotor::StepperMotor(int step_pin, int dir_pin, int steps_per_rev = 200, double wheel_diameter = 0.065)
    : step_pin(step_pin),
      dir_pin(dir_pin),
      steps_per_rev(steps_per_rev),
      wheel_diameter_(wheel_diameter),
      speed_hz_(0.0),
      running_(false)
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
  if (speed_m_s >= 0)
  {
    digitalWrite(dir_pin, HIGH);
  }
  else
  {
    digitalWrite(dir_pin, LOW);
  }
  
  speed_hz_ = std::abs(speed_m_s) * steps_per_meter_;
  RCLCPP_INFO(rclcpp::get_logger("KRISRobot"), "StepperMotor on step pin %d set to speed %.2f m/s (%.2f Hz)", step_pin, speed_m_s, speed_hz_);
  running_ = (speed_hz_ > 0);
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
                                              v_angular(0.0)
{
  this->setup_gpio();

  this->declare_parameter("robot_description", "");
  this->declare_parameter("base_frame_id", "base_link");
  this->declare_parameter("odom_frame_id", "odom");
  this->declare_parameter("robot_namespace", "");
  
  this->urdf_pub = this->create_publisher<std_msgs::msg::String>(this->get_parameter("robot_namespace").as_string() + "/robot_description", 10);
  this->cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      this->get_parameter("robot_namespace").as_string() + "/cmd_vel", rclcpp::QoS(10), std::bind(&KRISRobot::cmd_vel_callback, this, _1));

  RCLCPP_INFO(this->get_logger(), "KRIS Robot node initialized");

  this->base_frame_id = this->get_parameter("base_frame_id").as_string();
  this->odom_frame_id = this->get_parameter("odom_frame_id").as_string();

  left_motor = std::make_shared<StepperMotor>(MOTOR_LEFT_STEP_PIN, MOTOR_LEFT_DIR_PIN, STEPS_PER_REV, WHEEL_DIAMETER);
  right_motor = std::make_shared<StepperMotor>(MOTOR_RIGHT_STEP_PIN, MOTOR_RIGHT_DIR_PIN, STEPS_PER_REV, WHEEL_DIAMETER);
}

KRISRobot::~KRISRobot()
{
  left_motor->set_speed(0);
  right_motor->set_speed(0);
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
}