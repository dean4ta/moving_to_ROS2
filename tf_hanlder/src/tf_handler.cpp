#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <turtlesim/msg/pose.hpp>

class TF2Publisher : public rclcpp::Node
{
public:
  TF2Publisher()
    : Node("tf2_publisher")
  {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/pose", 10,
            std::bind(&TF2Publisher::poseCallback, this, std::placeholders::_1));
    odom_published_ = false;
  }

private:
  void poseCallback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    rclcpp::Time now;

    geometry_msgs::msg::TransformStamped odom_tf;
    geometry_msgs::msg::TransformStamped base_link_tf;

    base_link_tf.transform.translation.x = msg->x;
    base_link_tf.transform.translation.y = msg->y;
    base_link_tf.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    base_link_tf.transform.rotation.x = q.x();
    base_link_tf.transform.rotation.y = q.y();
    base_link_tf.transform.rotation.z = q.z();
    base_link_tf.transform.rotation.w = q.w();

    base_link_tf.header.frame_id = "odom";
    base_link_tf.child_frame_id = "base_link";
    base_link_tf.header.stamp = now;
    tf_broadcaster_->sendTransform(base_link_tf);

    if (!odom_published_)
    {
      odom_published_ = true;
      odom_tf.transform.translation.x = 0.0;
      odom_tf.transform.translation.y = 0.0;
      odom_tf.transform.translation.z = 0.0;
      q.setRPY(0, 0, 0);
      odom_tf.transform.rotation.x = q.x();
      odom_tf.transform.rotation.y = q.y();
      odom_tf.transform.rotation.z = q.z();
      odom_tf.transform.rotation.w = q.w();

      odom_tf.header.frame_id = "map";
      odom_tf.child_frame_id = "odom";
      odom_tf.header.stamp = now;
      tf_broadcaster_->sendTransform(odom_tf);
    }
  }

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
  bool odom_published_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TF2Publisher>());
  rclcpp::shutdown();
  return 0;
}