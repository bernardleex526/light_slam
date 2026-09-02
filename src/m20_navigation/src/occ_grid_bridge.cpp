/**
 * @file occ_grid_bridge.cpp
 * @brief Bridges lightning-lm g2p5 occ_grid outputs to the native /GRID_MAP
 *        topic expected by the M20 navigation stack.
 *
 * The lightning-lm SLAM pipeline produces occ_grid.pgm on disk; this node
 * reads the currently active grid from the same directory (via a ROS topic
 * or file watch) and publishes it as nav_msgs/OccupancyGrid on /GRID_MAP for
 * the m20_navigation global planner.
 *
 * Usage:
 *   ros2 run m20_navigation occ_grid_bridge --ros-args \
 *     -p grid_path:=/var/opt/robot/data/maps/active/occ_grid.pgm \
 *     -p frame_id:=map
 */

#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/header.hpp>

namespace m20::bridge {

class OccGridBridge : public rclcpp::Node {
public:
  OccGridBridge() : Node("occ_grid_bridge") {
    grid_path_ = this->declare_parameter<std::string>("grid_path", "");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
    pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/GRID_MAP", 10);

    if (!grid_path_.empty()) {
      // File-based: reload on timer
      timer_ = this->create_wall_timer(
          std::chrono::milliseconds(500),
          [this]() { publishGrid(); });
      RCLCPP_INFO(get_logger(), "OccGridBridge watching %s -> /GRID_MAP [frame=%s]",
                  grid_path_.c_str(), frame_id_.c_str());
    } else {
      RCLCPP_WARN(get_logger(),
                  "No grid_path set. Set via --ros-args -p grid_path:=<path> [-p frame_id:=map]");
    }
  }

private:
  void publishGrid() {
    if (!loadPGM(grid_path_, data_, width_, height_, resolution_)) {
      return;  // file not ready yet, will retry
    }

    auto msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    msg->header.stamp = now();
    msg->header.frame_id = frame_id_;
    msg->info.map_load_time = now();
    msg->info.resolution = static_cast<float>(resolution_);
    msg->info.width = static_cast<uint32_t>(width_);
    msg->info.height = static_cast<uint32_t>(height_);
    msg->info.origin.position.x = -static_cast<double>(width_) * resolution_ * 0.5;
    msg->info.origin.position.y = -static_cast<double>(height_) * resolution_ * 0.5;
    msg->info.origin.orientation.w = 1.0;

    msg->data.resize(data_.size());
    for (size_t i = 0; i < data_.size(); ++i) {
      const int p = data_[i];
      if (p < 64) {
        msg->data[i] = 100;   // occupied
      } else if (p < 192) {
        msg->data[i] = 0;     // free
      } else {
        msg->data[i] = -1;    // unknown
      }
    }

    pub_->publish(std::move(msg));
  }

  /// Load a PGM (P5/P2) file into data vector.
  static bool loadPGM(const std::string& path,
                      std::vector<int>& data,
                      int& width, int& height, double& resolution) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs.is_open()) return false;

    std::string magic;
    std::getline(ifs, magic);
    if (magic != "P5" && magic != "P2") return false;

    // Skip comments and read width, height, maxval
    while (ifs.peek() == '#') ifs.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    ifs >> width >> height;
    while (ifs.peek() == '#') ifs.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    int maxval;
    ifs >> maxval;
    ifs.ignore();  // newline

    data.resize(static_cast<size_t>(width) * height);
    if (magic == "P5") {
      ifs.read(reinterpret_cast<char*>(data.data()),
               static_cast<std::streamsize>(data.size()));
    } else {
      for (auto& v : data) ifs >> v;
    }
    resolution = 0.2;  // default; ideally read from .yaml companion
    return true;
  }

  std::string grid_path_;
  std::string frame_id_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<int> data_;
  int width_{0};
  int height_{0};
  double resolution_{0.2};
};

}  // namespace m20::bridge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<m20::bridge::OccGridBridge>());
  rclcpp::shutdown();
  return 0;
}