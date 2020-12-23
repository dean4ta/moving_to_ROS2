#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

class OdomHandler : public rclcpp::Node
{
public:
  OdomHandler()
    : Node("odom_handler")
  {
    std::cout << "in constructor\n";
    pose_pub_ = this->create_publisher<turtlesim::msg::Pose>("/pose", 10);
    this->get_parameter("sensor_data_location", sensor_data_location);
    processDataFile(sensor_data_location);
  }

private:
  void processDataFile(std::string filename)
  {
    std::cout << "in file processor\n";
    std::cout << filename << "\n";
    std::ifstream input(filename);
    for(std::string line; getline(input, line);)
    {
      RCLCPP_INFO(this->get_logger(), "Hello %s", line);
      std::cout << line << "\n";
    }
    std::cout << "leaving file processor\n";
  }

  rclcpp::Publisher<turtlesim::msg::Pose>::SharedPtr pose_pub_;
  std::string sensor_data_location;
  std::ifstream* infile;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  std::cout << "initiallized\n";
  rclcpp::spin(std::make_shared<OdomHandler>());
  rclcpp::shutdown();
  return 0;
}