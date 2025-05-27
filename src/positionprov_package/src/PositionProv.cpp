#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <thread>

class MultiWaypointPublisher : public rclcpp::Node
{
public:
    MultiWaypointPublisher() : Node("multi_waypoint_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("target_waypoints", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            [this]() {
                std_msgs::msg::String msg;
                msg.data = "C1,red,E1,blue,A1,red";  // ensure uniqueness

                RCLCPP_INFO(this->get_logger(), "Publishing waypoints: %s", msg.data.c_str());
                publisher_->publish(msg);

                timer_->cancel();  // publish only once
            });
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_ = 1;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiWaypointPublisher>());
    rclcpp::shutdown();
    return 0;
}



