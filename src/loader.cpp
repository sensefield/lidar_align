#include "lidar_align/loader.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cstdint>
#include <limits>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "lidar_align/transform.h"

namespace lidar_align {
namespace {

rclcpp::Logger getLogger() { return rclcpp::get_logger("lidar_align.loader"); }

Timestamp stampToMicroseconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<Timestamp>(stamp.sec) * 1000000ll +
         static_cast<Timestamp>(stamp.nanosec / 1000u);
}

template <typename MessageT>
bool deserializeBagMessage(
    const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& bag_message,
    MessageT* message) {
  try {
    rclcpp::SerializedMessage serialized_message(*bag_message->serialized_data);
    rclcpp::Serialization<MessageT> serialization;
    serialization.deserialize_message(&serialized_message, message);
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(getLogger(), "Failed to deserialize topic '%s': %s",
                 bag_message->topic_name.c_str(), e.what());
    return false;
  }
}

}  // namespace

Loader::Loader(const Config& config) : config_(config) {}

Loader::Config Loader::getConfig(const std::shared_ptr<rclcpp::Node>& node) {
  Loader::Config config;
  config.use_n_scans =
      node->declare_parameter<int>("use_n_scans", config.use_n_scans);

  config.pointcloud_topic =
      node->declare_parameter<std::string>("pointcloud_topic", config.pointcloud_topic);

  config.tf_topic =
      node->declare_parameter<std::string>("tf_topic", config.tf_topic);
  config.tf_static_topic =
      node->declare_parameter<std::string>("tf_static_topic", config.tf_static_topic);
  config.tf_parent_frame =
      node->declare_parameter<std::string>("tf_parent_frame", config.tf_parent_frame);
  config.tf_child_frame =
      node->declare_parameter<std::string>("tf_child_frame", config.tf_child_frame);
  config.use_tf_static =
      node->declare_parameter<bool>("use_tf_static", config.use_tf_static);

  config.pose_topic =
      node->declare_parameter<std::string>("pose_topic", config.pose_topic);

  RCLCPP_INFO(getLogger(), "Loader configuration:");
  RCLCPP_INFO(getLogger(), "  use_n_scans: %d", config.use_n_scans);
  RCLCPP_INFO(getLogger(), "  pointcloud_topic: '%s'", config.pointcloud_topic.c_str());
  RCLCPP_INFO(getLogger(), "  tf_topic: '%s'", config.tf_topic.c_str());
  RCLCPP_INFO(getLogger(), "  tf_static_topic: '%s'", config.tf_static_topic.c_str());
  RCLCPP_INFO(getLogger(), "  tf_parent_frame: '%s'", config.tf_parent_frame.c_str());
  RCLCPP_INFO(getLogger(), "  tf_child_frame: '%s'", config.tf_child_frame.c_str());
  RCLCPP_INFO(getLogger(), "  use_tf_static: %s", config.use_tf_static ? "true" : "false");
  RCLCPP_INFO(getLogger(), "  pose_topic: '%s'", config.pose_topic.c_str());    

  return config;
}

void Loader::parsePointcloudMsg(const sensor_msgs::msg::PointCloud2& msg,
                                LoaderPointcloud* pointcloud) const {
  pointcloud->clear();
  pointcloud->reserve(static_cast<std::size_t>(msg.width) *
                      static_cast<std::size_t>(msg.height));

  bool has_x = false;
  bool has_y = false;
  bool has_z = false;
  bool has_intensity = false;
  bool has_time_offset_us = false;
  bool has_ring = false;
  bool has_reflectivity = false;

  for (const auto& field : msg.fields) {
    if (field.name == "x") has_x = true;
    else if (field.name == "y") has_y = true;
    else if (field.name == "z") has_z = true;
    else if (field.name == "intensity") has_intensity = true;
    else if (field.name == "time_offset_us") has_time_offset_us = true;
    else if (field.name == "ring") has_ring = true;
    else if (field.name == "reflectivity") has_reflectivity = true;
  }

  if (!(has_x && has_y && has_z)) {
    RCLCPP_WARN(getLogger(),
                "Skipping PointCloud2 because x/y/z fields are missing.");
    return;
  }

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

  std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> iter_intensity;
  std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<int32_t>> iter_time_offset;
  std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<uint16_t>> iter_reflectivity;
  std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<uint16_t>> iter_ring_u16;
  std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<uint8_t>> iter_ring_u8;

  if (has_intensity) {
    iter_intensity =
        std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(msg, "intensity");
  }
  if (has_time_offset_us) {
    iter_time_offset =
        std::make_unique<sensor_msgs::PointCloud2ConstIterator<int32_t>>(msg, "time_offset_us");
  }
  if (has_reflectivity) {
    iter_reflectivity =
        std::make_unique<sensor_msgs::PointCloud2ConstIterator<uint16_t>>(msg, "reflectivity");
  }
  if (has_ring) {
    bool ring_is_uint8 = false;
    for (const auto& field : msg.fields) {
      if (field.name == "ring") {
        ring_is_uint8 = (field.datatype == sensor_msgs::msg::PointField::UINT8);
        break;
      }
    }
    if (ring_is_uint8) {
      iter_ring_u8 =
          std::make_unique<sensor_msgs::PointCloud2ConstIterator<uint8_t>>(msg, "ring");
    } else {
      iter_ring_u16 =
          std::make_unique<sensor_msgs::PointCloud2ConstIterator<uint16_t>>(msg, "ring");
    }
  }

  const std::size_t point_count =
      static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height);

  for (std::size_t i = 0; i < point_count; ++i, ++iter_x, ++iter_y, ++iter_z) {
    PointAllFields point{};
    point.x = *iter_x;
    point.y = *iter_y;
    point.z = *iter_z;

    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      if (iter_intensity) ++(*iter_intensity);
      if (iter_time_offset) ++(*iter_time_offset);
      if (iter_reflectivity) ++(*iter_reflectivity);
      if (iter_ring_u8) ++(*iter_ring_u8);
      if (iter_ring_u16) ++(*iter_ring_u16);
      continue;
    }

    if (iter_intensity) {
      const float intensity = **iter_intensity;
      if (std::isfinite(intensity)) {
        point.intensity = static_cast<uint16_t>(
            std::max(0.0f, std::min(intensity,
                                    static_cast<float>(std::numeric_limits<uint16_t>::max()))));
      }
      ++(*iter_intensity);
    }

    if (iter_time_offset) {
      point.time_offset_us = **iter_time_offset;
      ++(*iter_time_offset);
    }

    if (iter_reflectivity) {
      point.reflectivity = **iter_reflectivity;
      ++(*iter_reflectivity);
    }

    if (iter_ring_u8) {
      point.ring = **iter_ring_u8;
      ++(*iter_ring_u8);
    }
    if (iter_ring_u16) {
      point.ring = static_cast<uint8_t>(
          std::min<uint16_t>(**iter_ring_u16, std::numeric_limits<uint8_t>::max()));
      ++(*iter_ring_u16);
    }

    pointcloud->push_back(point);
  }

  pointcloud->width = static_cast<std::uint32_t>(pointcloud->size());
  pointcloud->height = 1;
  pointcloud->is_dense = false;
  pointcloud->header.stamp =
      static_cast<std::uint64_t>(stampToMicroseconds(msg.header.stamp));
  pointcloud->header.frame_id = msg.header.frame_id;
}

bool Loader::loadPointcloudFromROSBag(const std::string& bag_path,
                                      const Scan::Config& scan_config,
                                      Lidar* lidar) const {
  rosbag2_cpp::Reader reader;
  try {
    reader.open(bag_path);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(getLogger(), "Opening rosbag2 input failed: %s", e.what());
    return false;
  }

  std::unordered_map<std::string, std::string> topic_types;
  for (const auto& topic : reader.get_all_topics_and_types()) {
    topic_types[topic.name] = topic.type;
  }

  size_t scan_num = 0;
  while (reader.has_next()) {
    auto bag_message = reader.read_next();
    const auto topic_it = topic_types.find(bag_message->topic_name);
    if (topic_it == topic_types.end() ||
        topic_it->second != "sensor_msgs/msg/PointCloud2") {
      continue;
    }

    if (!config_.pointcloud_topic.empty() &&
        bag_message->topic_name != config_.pointcloud_topic) {
      continue;
    }

    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    if (!deserializeBagMessage(bag_message, &pointcloud_msg)) {
      return false;
    }

    LoaderPointcloud pointcloud;
    parsePointcloudMsg(pointcloud_msg, &pointcloud);
    if (pointcloud.empty()) {
      continue;
    }

    std::cout << " Loading scan: \e[1m" << scan_num++
              << "\e[0m from rosbag2" << '\r' << std::flush;

    lidar->addPointcloud(pointcloud, scan_config);

    if (static_cast<int>(lidar->getNumberOfScans()) >= config_.use_n_scans) {
      break;
    }
  }
  std::cout << std::endl;

  if (lidar->getTotalPoints() == 0) {
    RCLCPP_ERROR(
        getLogger(),
        "No points were loaded. Check pointcloud_topic and PointCloud2 fields.");
    return false;
  }

  return true;
}

bool Loader::loadTformFromROSBag(const std::string& bag_path, Odom* odom) const {
  rosbag2_cpp::Reader reader;
  try {
    reader.open(bag_path);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(getLogger(), "Opening rosbag2 input failed: %s", e.what());
    return false;
  }

  std::unordered_map<std::string, std::string> topic_types;
  for (const auto& topic : reader.get_all_topics_and_types()) {
    topic_types[topic.name] = topic.type;
  }

  auto isDesiredTransform =
      [this](const geometry_msgs::msg::TransformStamped& transform_msg,
             const std::string& topic_name) -> bool {
    if (!config_.use_tf_static && topic_name == config_.tf_static_topic) {
      return false;
    }
    if (!config_.tf_parent_frame.empty() &&
        transform_msg.header.frame_id != config_.tf_parent_frame) {
      return false;
    }
    if (!config_.tf_child_frame.empty() &&
        transform_msg.child_frame_id != config_.tf_child_frame) {
      return false;
    }
    return true;
  };

  auto addTransformToOdom =
      [odom](const geometry_msgs::msg::TransformStamped& transform_msg) {
    const Timestamp stamp = stampToMicroseconds(transform_msg.header.stamp);

    const Transform T(
        Transform::Translation(
            static_cast<float>(transform_msg.transform.translation.x),
            static_cast<float>(transform_msg.transform.translation.y),
            static_cast<float>(transform_msg.transform.translation.z)),
        Transform::Rotation(
            static_cast<float>(transform_msg.transform.rotation.w),
            static_cast<float>(transform_msg.transform.rotation.x),
            static_cast<float>(transform_msg.transform.rotation.y),
            static_cast<float>(transform_msg.transform.rotation.z)));

    odom->addTransformData(stamp, T);
  };

  size_t tform_num = 0;
  while (reader.has_next()) {
    auto bag_message = reader.read_next();
    const auto topic_it = topic_types.find(bag_message->topic_name);
    if (topic_it == topic_types.end()) {
      continue;
    }

    const std::string& topic_name = bag_message->topic_name;
    const std::string& topic_type = topic_it->second;

    if (topic_type == "geometry_msgs/msg/TransformStamped") {
      geometry_msgs::msg::TransformStamped transform_msg;
      if (!deserializeBagMessage(bag_message, &transform_msg)) {
        return false;
      }
      if (!isDesiredTransform(transform_msg, topic_name)) {
        continue;
      }
      std::cout << " Loading transform: \e[1m" << tform_num++
                << "\e[0m from rosbag2" << '\r' << std::flush;
      addTransformToOdom(transform_msg);
      continue;
    }

    const bool is_tf_topic =
        (topic_name == config_.tf_topic || topic_name == config_.tf_static_topic);

    if (is_tf_topic && topic_type == "tf2_msgs/msg/TFMessage") {
      tf2_msgs::msg::TFMessage tf_msg;
      if (!deserializeBagMessage(bag_message, &tf_msg)) {
        return false;
      }

      for (const auto& transform_msg : tf_msg.transforms) {
        if (!isDesiredTransform(transform_msg, topic_name)) {
          continue;
        }
        std::cout << " Loading transform: \e[1m" << tform_num++
                  << "\e[0m from rosbag2" << '\r' << std::flush;
        addTransformToOdom(transform_msg);
      }
    }
  }
  std::cout << std::endl;

  if (odom->size() < 2) {
    RCLCPP_ERROR(
        getLogger(),
        "Fewer than two odometry transforms were found in the bag.");
    return false;
  }

  return true;
}

bool Loader::loadTformFromPoseStamped(const std::string& bag_path,
                                      Odom* odom) const {
  rosbag2_cpp::Reader reader;
  try {
    reader.open(bag_path);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(getLogger(), "Opening rosbag2 input failed: %s", e.what());
    return false;
  }

  std::unordered_map<std::string, std::string> topic_types;
  for (const auto& topic : reader.get_all_topics_and_types()) {
    topic_types[topic.name] = topic.type;
  }

  size_t tform_num = 0;
  while (reader.has_next()) {
    auto bag_message = reader.read_next();
    const auto topic_it = topic_types.find(bag_message->topic_name);
    if (topic_it == topic_types.end()) {
      continue;
    }

    if (!config_.pose_topic.empty() &&
        bag_message->topic_name != config_.pose_topic) {
      continue;
    }

    if (topic_it->second != "geometry_msgs/msg/PoseStamped") {
      continue;
    }

    geometry_msgs::msg::PoseStamped pose_msg;
    if (!deserializeBagMessage(bag_message, &pose_msg)) {
      return false;
    }

    const Timestamp stamp = stampToMicroseconds(pose_msg.header.stamp);

    const Transform T(
        Transform::Translation(
            static_cast<float>(pose_msg.pose.position.x),
            static_cast<float>(pose_msg.pose.position.y),
            static_cast<float>(pose_msg.pose.position.z)),
        Transform::Rotation(
            static_cast<float>(pose_msg.pose.orientation.w),
            static_cast<float>(pose_msg.pose.orientation.x),
            static_cast<float>(pose_msg.pose.orientation.y),
            static_cast<float>(pose_msg.pose.orientation.z)));

    std::cout << " Loading transform: \e[1m" << tform_num++
              << "\e[0m from rosbag2 (PoseStamped)" << '\r' << std::flush;

    odom->addTransformData(stamp, T);
  }
  std::cout << std::endl;

  if (odom->size() < 2) {
    RCLCPP_ERROR(
        getLogger(),
        "Fewer than two odometry transforms were found from PoseStamped messages.");
    return false;
  }

  return true;
}

bool Loader::loadTformFromMaplabCSV(const std::string& csv_path, Odom* odom) const {
  std::ifstream file(csv_path, std::ifstream::in);
  if (!file.is_open()) {
    RCLCPP_ERROR(getLogger(), "Could not open CSV file: %s", csv_path.c_str());
    return false;
  }

  size_t tform_num = 0;
  while (file.peek() != EOF) {
    std::cout << " Loading transform: \e[1m" << tform_num++
              << "\e[0m from csv file" << '\r' << std::flush;

    Timestamp stamp = 0;
    Transform T;

    if (getNextCSVTransform(file, &stamp, &T)) {
      odom->addTransformData(stamp, T);
    }
  }
  std::cout << std::endl;

  if (odom->size() < 2) {
    RCLCPP_ERROR(getLogger(),
                 "Fewer than two odometry transforms were loaded from the CSV file.");
    return false;
  }

  return true;
}

bool Loader::getNextCSVTransform(std::istream& str, Timestamp* stamp,
                                 Transform* T) {
  std::string line;
  std::getline(str, line);

  if (line.empty() || line[0] == '#') {
    return false;
  }

  std::stringstream line_stream(line);
  std::string cell;

  std::vector<std::string> data;
  while (std::getline(line_stream, cell, ',')) {
    data.push_back(cell);
  }

  if (data.size() < 9) {
    return false;
  }

  constexpr size_t TIME = 0;
  constexpr size_t X = 2;
  constexpr size_t Y = 3;
  constexpr size_t Z = 4;
  constexpr size_t RW = 5;
  constexpr size_t RX = 6;
  constexpr size_t RY = 7;
  constexpr size_t RZ = 8;

  *stamp = std::stoll(data[TIME]) / 1000ll;
  *T = Transform(
      Transform::Translation(static_cast<float>(std::stod(data[X])),
                             static_cast<float>(std::stod(data[Y])),
                             static_cast<float>(std::stod(data[Z]))),
      Transform::Rotation(static_cast<float>(std::stod(data[RW])),
                          static_cast<float>(std::stod(data[RX])),
                          static_cast<float>(std::stod(data[RY])),
                          static_cast<float>(std::stod(data[RZ]))));

  return true;
}

}  // namespace lidar_align
