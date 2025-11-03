#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "kris_robot.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
	std::cout << "Initializing Robot node...\n";
	rclcpp::init(argc, argv);
	auto node = std::make_shared<KRISRobot>("kris_robot1");
	while (rclcpp::ok())
	{
		rclcpp::spin_some(node);
		node->update_state();
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}
	rclcpp::shutdown();

	return 0;
}