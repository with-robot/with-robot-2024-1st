#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>


using std::placeholders::_1;


class UnityOdom:public rclcpp::Node
{
    private:
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_unity_odom;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

        rclcpp::Time last_time;
        geometry_msgs::msg::TransformStamped tf;

    public:
        UnityOdom() : Node("unity_odom") {
            tf.header.frame_id = "odom";
            
            tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            sub_unity_odom = this->create_subscription<geometry_msgs::msg::TwistStamped>("unity_tf", 10, std::bind(&UnityOdom::unity_odom_callback, this, _1));
        }
    
    private:
        void unity_odom_callback(const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg) {
            // delta time
            rclcpp::Time curr_time = msg->header.stamp;
            if (last_time.seconds() == 0) {
                last_time = curr_time;
                return;
            }
            // Euler to Quaternion
            tf2::Quaternion quaternion;
            quaternion.setRPY(msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z);

            // publish tf
            tf.header.stamp = curr_time;
            tf.child_frame_id = msg->header.frame_id;
            tf.transform.translation.x = msg->twist.linear.x;
            tf.transform.translation.y = msg->twist.linear.y;
            tf.transform.translation.z = msg->twist.linear.z;
            tf.transform.rotation.x = quaternion.x();
            tf.transform.rotation.y = quaternion.y();
            tf.transform.rotation.z = quaternion.z();
            tf.transform.rotation.w = quaternion.w();
            tf_broadcaster->sendTransform(tf);
        }
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<UnityOdom>());
	rclcpp::shutdown();
    return 0;
}