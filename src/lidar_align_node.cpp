#include <cstdlib>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "lidar_align/aligner.h"
#include "lidar_align/loader.h"
#include "lidar_align/sensors.h"

using namespace lidar_align;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("lidar_align");

  Loader loader(Loader::getConfig(node));

  Lidar lidar;
  Odom odom;

  const std::string input_bag_path =
      node->declare_parameter<std::string>("input_bag_path", "");
  RCLCPP_INFO(node->get_logger(), "Loading pointcloud data...: %s",
              input_bag_path.c_str());
  if (input_bag_path.empty()) {
    RCLCPP_FATAL(node->get_logger(),
                 "Could not find input_bag_path parameter, exiting.");
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  if (!loader.loadPointcloudFromROSBag(input_bag_path, Scan::getConfig(node),
                                       &lidar)) {
    RCLCPP_FATAL(node->get_logger(),
                 "Error loading pointclouds from ROS 2 bag.");
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  const std::string odom_source =
      node->declare_parameter<std::string>("odom_source", "tf");
  RCLCPP_INFO(node->get_logger(), "Loading transformation data (source: %s)...",
              odom_source.c_str());

  bool odom_loaded = false;
  if (odom_source == "tf") {
    odom_loaded = loader.loadTformFromROSBag(input_bag_path, &odom);
  } else if (odom_source == "pose_stamped") {
    odom_loaded = loader.loadTformFromPoseStamped(input_bag_path, &odom);
  } else if (odom_source == "odometry") {
    odom_loaded = loader.loadTformFromOdometry(input_bag_path, &odom);
  } else if (odom_source == "csv") {
    const std::string input_csv_path =
        node->declare_parameter<std::string>("input_csv_path", "");
    if (input_csv_path.empty()) {
      RCLCPP_FATAL(node->get_logger(),
                   "odom_source is 'csv' but input_csv_path is empty, exiting.");
      rclcpp::shutdown();
      return EXIT_FAILURE;
    }
    odom_loaded = loader.loadTformFromMaplabCSV(input_csv_path, &odom);
  } else {
    RCLCPP_FATAL(node->get_logger(),
                 "Unknown odom_source '%s'. Must be 'tf', 'pose_stamped', 'odometry', or 'csv'.",
                 odom_source.c_str());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  if (!odom_loaded) {
    RCLCPP_FATAL(node->get_logger(),
                 "Error loading transforms from source '%s'.",
                 odom_source.c_str());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  if (lidar.getNumberOfScans() == 0) {
    RCLCPP_FATAL(node->get_logger(), "No lidar data loaded, exiting.");
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  RCLCPP_INFO(node->get_logger(), "Interpolating transformation data...");
  lidar.setOdomOdomTransforms(odom);

  Aligner aligner(Aligner::getConfig(node));
  aligner.lidarOdomTransform(&lidar, &odom);

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
