// Feeds arise_slam from the lidar topics published by the robot's lower computer.
//
// arise_slam (sensor: livox) subscribes /lidar/scan as livox_ros_driver2/CustomMsg and /imu/data, both with
// RELIABLE QoS. The lower computer may publish the cloud as PointCloud2 (Livox xfer_format 0) or CustomMsg
// (xfer_format 1), under other names, and possibly BEST_EFFORT. This node subscribes best-effort (which
// matches either), converts PointCloud2 to CustomMsg if needed, and republishes reliably.
//
// Subscribes  inputCloudTopic (default /livox/lidar), PointCloud2 or CustomMsg per inputFormat
//             inputImuTopic   (default /livox/imu)
// Publishes   /lidar/scan  livox_ros_driver2/CustomMsg
//             /imu/data    sensor_msgs/Imu

#include <algorithm>
#include <cstring>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"

using livox_ros_driver2::msg::CustomMsg;
using livox_ros_driver2::msg::CustomPoint;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

class LivoxInputAdapter : public rclcpp::Node
{
public:
  LivoxInputAdapter() : Node("livoxInputAdapter")
  {
    const auto cloudTopic = declare_parameter<std::string>("inputCloudTopic", "/livox/lidar");
    const auto imuTopic = declare_parameter<std::string>("inputImuTopic", "/livox/imu");
    format_ = declare_parameter<std::string>("inputFormat", "pointcloud2");  // pointcloud2 | custom

    pubCloud_ = create_publisher<CustomMsg>("/lidar/scan", rclcpp::QoS(20).reliable());
    pubImu_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", rclcpp::QoS(200).reliable());

    const auto in = rclcpp::SensorDataQoS().keep_last(20);
    if (format_ == "custom") {
      subCustom_ = create_subscription<CustomMsg>(cloudTopic, in, [this](CustomMsg::UniquePtr msg) {
        countCloud();
        pubCloud_->publish(std::move(msg));
      });
    } else if (format_ == "pointcloud2") {
      subCloud_ = create_subscription<PointCloud2>(cloudTopic, in, [this](PointCloud2::ConstSharedPtr msg) {
        countCloud();
        convert(*msg);
      });
    } else {
      throw std::runtime_error("inputFormat must be 'pointcloud2' or 'custom', got '" + format_ + "'");
    }

    subImu_ = create_subscription<sensor_msgs::msg::Imu>(
        imuTopic, rclcpp::SensorDataQoS().keep_last(200),
        [this](sensor_msgs::msg::Imu::UniquePtr msg) {
          imuCount_++;
          pubImu_->publish(std::move(msg));
        });

    statusTimer_ = create_wall_timer(std::chrono::seconds(5), [this, cloudTopic, imuTopic]() {
      if (cloudCount_ == 0) RCLCPP_WARN(get_logger(), "No %s on %s in the last 5 s.", format_.c_str(), cloudTopic.c_str());
      if (imuCount_ == 0) RCLCPP_WARN(get_logger(), "No IMU on %s in the last 5 s.", imuTopic.c_str());
      cloudCount_ = 0;
      imuCount_ = 0;
    });

    RCLCPP_INFO(get_logger(), "%s (%s) -> /lidar/scan, %s -> /imu/data.", cloudTopic.c_str(), format_.c_str(),
                imuTopic.c_str());
  }

private:
  void countCloud() { cloudCount_++; }

  static const PointField* findField(const PointCloud2& msg, const std::string& name, uint8_t type)
  {
    for (const auto& f : msg.fields) {
      if (f.name == name && f.datatype == type) return &f;
    }
    return nullptr;
  }

  template <typename T>
  static T read(const uint8_t* p, const PointField* f)
  {
    T v;
    std::memcpy(&v, p + f->offset, sizeof(T));
    return v;
  }

  // Livox PointCloud2 (xfer_format 0): x y z intensity (float32), tag line (uint8), timestamp (float64, absolute ns).
  void convert(const PointCloud2& in)
  {
    const PointField* fx = findField(in, "x", PointField::FLOAT32);
    const PointField* fy = findField(in, "y", PointField::FLOAT32);
    const PointField* fz = findField(in, "z", PointField::FLOAT32);
    if (!fx || !fy || !fz || in.is_bigendian) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "PointCloud2 has no float32 x/y/z fields, dropping.");
      return;
    }
    const PointField* fi = findField(in, "intensity", PointField::FLOAT32);
    const PointField* ftag = findField(in, "tag", PointField::UINT8);
    const PointField* fline = findField(in, "line", PointField::UINT8);
    const PointField* ft = findField(in, "timestamp", PointField::FLOAT64);
    if (!ft && !warnedTime_) {
      RCLCPP_WARN(get_logger(), "PointCloud2 has no float64 'timestamp' field: per-point times set to 0 (no deskew).");
      warnedTime_ = true;
    }

    const size_t n = static_cast<size_t>(in.width) * in.height;
    if (in.data.size() < n * in.point_step) return;

    // Base time = earliest point time, as the Livox driver does for CustomMsg.
    uint64_t base = rclcpp::Time(in.header.stamp).nanoseconds();
    if (ft && n > 0) {
      double tmin = std::numeric_limits<double>::max();
      for (size_t i = 0; i < n; i++) tmin = std::min(tmin, read<double>(&in.data[i * in.point_step], ft));
      if (tmin > 0) base = static_cast<uint64_t>(tmin);
    }

    CustomMsg out;
    out.header = in.header;
    out.header.stamp = rclcpp::Time(static_cast<int64_t>(base));
    out.timebase = base;
    out.point_num = static_cast<uint32_t>(n);
    out.lidar_id = 0;
    out.points.resize(n);
    for (size_t i = 0; i < n; i++) {
      const uint8_t* p = &in.data[i * in.point_step];
      CustomPoint& q = out.points[i];
      q.x = read<float>(p, fx);
      q.y = read<float>(p, fy);
      q.z = read<float>(p, fz);
      q.reflectivity = fi ? static_cast<uint8_t>(std::clamp(read<float>(p, fi), 0.0f, 255.0f)) : 0;
      q.tag = ftag ? read<uint8_t>(p, ftag) : 0;
      q.line = fline ? read<uint8_t>(p, fline) : 0;
      q.offset_time = ft ? static_cast<uint32_t>(std::max(0.0, read<double>(p, ft) - static_cast<double>(base))) : 0;
    }
    pubCloud_->publish(std::move(out));
  }

  std::string format_;
  bool warnedTime_ = false;
  int cloudCount_ = 0, imuCount_ = 0;

  rclcpp::Publisher<CustomMsg>::SharedPtr pubCloud_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pubImu_;
  rclcpp::Subscription<CustomMsg>::SharedPtr subCustom_;
  rclcpp::Subscription<PointCloud2>::SharedPtr subCloud_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu_;
  rclcpp::TimerBase::SharedPtr statusTimer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxInputAdapter>());
  rclcpp::shutdown();
  return 0;
}
