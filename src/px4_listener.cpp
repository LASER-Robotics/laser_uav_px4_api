#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <laser_msgs/msg/pose_with_heading.hpp>

using namespace std::chrono_literals;

class Px4Listener : public rclcpp::Node
{
public:
    Px4Listener() : Node("px4_listener")
    {

        this->declare_parameter<bool>("enable_publisher", true);

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        subscription_ = this->create_subscription<px4_msgs::msg::ManualControlSetpoint>(
            "/fmu/out/manual_control_setpoint",
            qos,
            std::bind(&Px4Listener::subscription_callback, this, std::placeholders::_1)
        );

        publisher_ = this->create_publisher<laser_msgs::msg::PoseWithHeading>("/px4_rc", 10);


        timer_ = this->create_wall_timer(
            500ms, std::bind(&Px4Listener::timer_callback, this));
    }

private:
    void subscription_callback(const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg)
    {
        x = msg->pitch;
        y = msg->roll;
        heading = msg->yaw;
        z = msg->throttle;

        mode_switch = msg->mode_switch;

    }

    void timer_callback()  
    {

        bool enable = this->get_parameter("enable_publisher").as_bool();

        RCLCPP_INFO(this->get_logger(), 
            "X: %.2f | Y: %.2f | Heading: %.2f | Z: %.2f | Mode Switch: %d",
            x, y, heading, z, (int)mode_switch);

        
        if (mode_switch == 1)  
        {
            auto msg = laser_msgs::msg::PoseWithHeading();
            msg.pose.position.x = x;
            msg.pose.position.y = y;
            msg.pose.position.z = z;
            msg.heading = heading;
            
            publisher_->publish(msg);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Switch desligado - não publicando");
        }
    }


    float x = 0.0f;
    float y = 0.0f;
    float heading = 0.0f;
    float z = 0.0f;
    uint8_t mode_switch = 0;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr subscription_;
    rclcpp::Publisher<laser_msgs::msg::PoseWithHeading>::SharedPtr publisher_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Px4Listener>());
    rclcpp::shutdown();
    return 0;
}