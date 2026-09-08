// Load the companion YAML through Nav2, preserving image semantics and geometry.
#include <filesystem>
#include <memory>
#include <string>
#include <nav2_map_server/map_io.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

class OccGridBridge : public rclcpp::Node
{
public:
  OccGridBridge()
  : Node("occ_grid_bridge")
  {
    auto path = declare_parameter<std::string>("grid_path", "");
    frame_ = declare_parameter<std::string>("frame_id", "map");
    std::filesystem::path yaml(path);
    if (yaml.extension() == ".pgm") {yaml.replace_extension(".yaml");}
    path_ = yaml.string();
    pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/GRID_MAP", rclcpp::QoS(1).reliable().transient_local());
    timer_ = create_wall_timer(
      std::chrono::milliseconds(500), [this]() {
        if (path_.empty()) {return;}
        nav_msgs::msg::OccupancyGrid grid;
        if (nav2_map_server::loadMapFromYaml(path_, grid) != nav2_map_server::LOAD_MAP_SUCCESS) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Cannot load map YAML %s", path_.c_str());
          return;
        }
        grid.header.frame_id = frame_;
        grid.header.stamp = now();
        grid.info.map_load_time = now();
        pub_->publish(grid);
      });
  }

private:
  std::string path_, frame_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccGridBridge>());
  rclcpp::shutdown();
  return 0;
}
