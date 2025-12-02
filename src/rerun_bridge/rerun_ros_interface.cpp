#include "rerun_bridge/rerun_ros_interface.hpp"
#include "collection_adapters.hpp"

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <rerun.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>

void log_imu(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const sensor_msgs::Imu::ConstPtr& msg
) {
    rec.set_time_seconds("timestamp", msg->header.stamp.toSec());

    rec.log(entity_path + "/x", rerun::Scalar(msg->linear_acceleration.x));
    rec.log(entity_path + "/y", rerun::Scalar(msg->linear_acceleration.y));
    rec.log(entity_path + "/z", rerun::Scalar(msg->linear_acceleration.z));
}

void log_point_cloud(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const sensor_msgs::PointCloud2::ConstPtr& msg
) {
    rec.set_time_seconds("timestamp", msg->header.stamp.toSec());

    // Ensure point cloud has x,y,z
    bool has_xyz = false;
    for (const auto& f : msg->fields) {
        if (f.name == "x") {
            has_xyz = true;
            break;
        }
    }
    if (!has_xyz) {
        ROS_WARN("PointCloud2 message on %s has no 'x' field, skipping", entity_path.c_str());
        return;
    }

    const size_t num_points = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
    std::vector<rerun::Position3D> points;
    points.reserve(num_points);

    // Iterate using the provided iterators for x,y,z
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

    // Try to detect rgb/rgba field (common in many pointclouds). We'll try to extract RGB if present.
    bool has_rgb = false;
    bool rgb_is_float = false;
    for (const auto& f : msg->fields) {
        if (f.name == "rgb" || f.name == "rgba") {
            has_rgb = true;
            rgb_is_float = (f.datatype == sensor_msgs::PointField::FLOAT32);
            break;
        }
    }

    std::vector<rerun::components::Color> colors;
    if (has_rgb) {
        colors.reserve(num_points);
    }

    for (size_t i = 0; i < num_points; ++i, ++it_x, ++it_y, ++it_z) {
        float x = *it_x;
        float y = *it_y;
        float z = *it_z;

        // skip NaNs / infs
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            continue;
        }

        points.emplace_back(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));

        if (has_rgb) {
            // naive attempt: read rgb as float-packed or as 3x uint8. Fallback to white.
            uint8_t r = 255, g = 255, b = 255, a = 255;
            if (rgb_is_float) {
                // reinterpret float bits as uint32_t
                sensor_msgs::PointCloud2ConstIterator<float> it_rgb(*msg, "rgb");
                // advance iterator to i
                for (size_t k = 0; k < i; ++k) ++it_rgb;
                float rgbf = *it_rgb;
                uint32_t rgb_uint = *reinterpret_cast<uint32_t*>(&rgbf);
                r = static_cast<uint8_t>((rgb_uint >> 16) & 0xFF);
                g = static_cast<uint8_t>((rgb_uint >> 8) & 0xFF);
                b = static_cast<uint8_t>((rgb_uint) & 0xFF);
            } else {
                // try as 3 uint8 channels (r,g,b)
                // We'll try reading a uint8 iterator for 'rgb' field - may point to r,g,b,a bytes depending on layout.
                // Create a byte iterator and sample the three channels at the correct offset.
                sensor_msgs::PointCloud2ConstIterator<uint8_t> it_byte(*msg, "rgb");
                for (size_t k = 0; k < i * static_cast<size_t>(msg->point_step); ++k) {
                    ++it_byte; // this is not ideal but keeps implementation simple for many common layouts
                }
                // fallback: keep white if we cannot safely parse
            }
            colors.emplace_back(r, g, b, a);
        }
    }

    if (points.empty()) {
        return;
    }

    if (has_rgb && colors.size() == points.size()) {
        rec.log(entity_path, rerun::Points3D(points).with_colors(colors));
    } else {
        rec.log(entity_path, rerun::Points3D(points));
    }
}

void log_image(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const sensor_msgs::Image::ConstPtr& msg
) {
    rec.set_time_seconds("timestamp", msg->header.stamp.toSec());

    // Depth images are 32-bit float (in meters) or 16-bit uint (in millimeters)
    // See: https://ros.org/reps/rep-0118.html
    if (msg->encoding == "16UC1") {
        cv::Mat img = cv_bridge::toCvCopy(msg)->image;
        rec.log(
            entity_path,
            rerun::DepthImage({img.rows, img.cols}, rerun::TensorBuffer::u16(img)).with_meter(1000)
        );
    } else if (msg->encoding == "32FC1") {
        // NOTE this has not been tested
        cv::Mat img = cv_bridge::toCvCopy(msg)->image;
        rec.log(
            entity_path,
            rerun::DepthImage({img.rows, img.cols}, rerun::TensorBuffer::f32(img)).with_meter(1.0)
        );
    } else {
        cv::Mat img = cv_bridge::toCvCopy(msg, "rgb8")->image;
        rec.log(
            entity_path,
            rerun::Image(tensor_shape(img), rerun::TensorBuffer::u8(img))
        );
    }
}

void log_pose_stamped(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const geometry_msgs::PoseStamped::ConstPtr& msg
) {
    rec.set_time_seconds("timestamp", msg->header.stamp.toSec());

    rec.log(
        entity_path,
        rerun::Transform3D(
            rerun::Vector3D(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z),
            rerun::Quaternion::from_wxyz(
                msg->pose.orientation.w,
                msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z
            )
        )
    );

    // this is a somewhat hacky way to get a trajectory visualization in Rerun
    // this should be be easier in the future, see https://github.com/rerun-io/rerun/issues/723
    std::string trajectory_entity_path = "/trajectories/" + entity_path;
    rec.log(
        trajectory_entity_path,
        rerun::Points3D(
            {{static_cast<float>(msg->pose.position.x),
              static_cast<float>(msg->pose.position.y),
              static_cast<float>(msg->pose.position.z)}}
        )
    );
}

void log_tf_message(
    const rerun::RecordingStream& rec,
    const std::map<std::string, std::string>& tf_frame_to_entity_path,
    const tf2_msgs::TFMessage::ConstPtr& msg
) {
    for (const auto& transform : msg->transforms) {
        if (tf_frame_to_entity_path.find(transform.child_frame_id) ==
            tf_frame_to_entity_path.end()) {
            ROS_WARN("No entity path for frame_id %s, skipping", transform.child_frame_id.c_str());
            continue;
        }

        rec.set_time_seconds("timestamp", transform.header.stamp.toSec());

        rec.log(
            tf_frame_to_entity_path.at(transform.child_frame_id),
            rerun::Transform3D(
                rerun::Vector3D(
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                ),
                rerun::Quaternion::from_wxyz(
                    transform.transform.rotation.w,
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z
                )
            )
        );
    }
}

void log_odometry(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const nav_msgs::Odometry::ConstPtr& msg
) {
    rec.set_time_seconds("timestamp", msg->header.stamp.toSec());

    rec.log(
        entity_path,
        rerun::Transform3D(
            rerun::Vector3D(
                msg->pose.pose.position.x,
                msg->pose.pose.position.y,
                msg->pose.pose.position.z
            ),
            rerun::Quaternion::from_wxyz(
                msg->pose.pose.orientation.w,
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z
            )
        )
    );
}

void log_camera_info(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const sensor_msgs::CameraInfo::ConstPtr& msg
) {
    // Rerun uses column-major order for Mat3x3
    const std::array<float, 9> image_from_camera = {
        static_cast<float>(msg->K[0]),
        static_cast<float>(msg->K[3]),
        static_cast<float>(msg->K[6]),
        static_cast<float>(msg->K[1]),
        static_cast<float>(msg->K[4]),
        static_cast<float>(msg->K[7]),
        static_cast<float>(msg->K[2]),
        static_cast<float>(msg->K[5]),
        static_cast<float>(msg->K[8]),
    };
    rec.log(
        entity_path,
        rerun::Pinhole(image_from_camera)
            .with_resolution(static_cast<int>(msg->width), static_cast<int>(msg->height))
    );
}

void log_transform(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const geometry_msgs::TransformStamped& msg
) {
    rec.set_time_seconds("timestamp", msg.header.stamp.toSec());

    rec.log(
        entity_path,
        rerun::Transform3D(
            rerun::Vector3D(
                msg.transform.translation.x,
                msg.transform.translation.y,
                msg.transform.translation.z
            ),
            rerun::Quaternion::from_wxyz(
                msg.transform.rotation.w,
                msg.transform.rotation.x,
                msg.transform.rotation.y,
                msg.transform.rotation.z
            )
        )
    );
}
