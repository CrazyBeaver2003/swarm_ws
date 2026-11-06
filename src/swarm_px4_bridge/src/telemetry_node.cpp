#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

using namespace std::chrono_literals;

class TelemetryNode : public rclcpp::Node
{
public:
    TelemetryNode() : Node("telemetry_node"){
        auto qos = rclcpp::SensorDataQoS();
        qos.keep_last(1);

        status_sub_=this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "fmu/out/vehicle_status_v1",
            qos,
            [this](px4_msgs::msg::VehicleStatus::ConstSharedPtr msg){
                status_ = msg;
                status_received_at_= Clock::now();
            }
        );

        position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "fmu/out/vehicle_local_position_v1",
            qos,
            [this](px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr msg)
            {
                position_ = msg;
                position_received_at_ = Clock::now();
            }
        );

        attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "fmu/out/vehicle_attitude",
            qos,
            [this](px4_msgs::msg::VehicleAttitude::ConstSharedPtr msg)
            {
                attitude_ = msg;
                attitude_received_at_ = Clock::now();
            }
        );
        timer_ = this->create_wall_timer(
            1s, 
            [this](){
                print_state();
            }
        );

        RCLCPP_INFO(get_logger(), "PX4 telemetry node started");

    }

private:
    using Clock = std::chrono::steady_clock;

    // Измеряем возраст получения по монотонным часам компьютера.
    // PX4 timestamp напрямую с этими часами сравнивать нельзя.

    template<typename MessagePtr>
    std::string stream_state(
        const MessagePtr& message,
        const Clock::time_point& received_at,
        const Clock::time_point& now) const 
    {
        if (!message){
            return "WAITING";
        }

        const double age = std::chrono::duration<double>(now - received_at).count();

        return age > 2.0 ? "STALE" : "FRESH";
    }

    void print_state()
    {
        const auto now = Clock::now();
        const auto status_state = stream_state(status_, status_received_at_, now);
        const auto position_state =  stream_state(position_, position_received_at_, now);
        const auto attitude_state = stream_state(attitude_, attitude_received_at_, now);
        
        if (status_ && status_state == "FRESH" && status_->failsafe) {
            RCLCPP_ERROR(get_logger(), "PX4 failsafe is active");
        }
        if (
            status_state != "FRESH" ||
            position_state != "FRESH" ||
            attitude_state != "FRESH"
        )
        {
            RCLCPP_WARN(
                get_logger(),
                "Telemetry: status=%s position=%s attitude=%s",
                status_state.c_str(),
                position_state.c_str(),
                attitude_state.c_str()
            );

            return;
        }

        const auto&  p = *position_;

        const bool position_valid = 
            p.xy_valid && 
            p.z_valid && 
            std::isfinite(p.x) && 
            std::isfinite(p.y) &&
            std::isfinite(p.z);
        
        const bool velocity_valid =
            p.v_xy_valid && p.v_z_valid &&
            std::isfinite(p.vx) &&
            std::isfinite(p.vy) &&
            std::isfinite(p.vz);

        const auto& q = attitude_->q;

        double norm_squared = 0.0;
        for (const auto component : q) {
            norm_squared += static_cast<double>(component) * component;
        }

        const bool attitude_valid =
            std::isfinite(norm_squared) &&
            std::abs(norm_squared - 1.0) < 0.05;

        if (!position_valid || !velocity_valid || !attitude_valid) {
        RCLCPP_WARN(
            get_logger(),
            "Invalid estimate: position=%s velocity=%s attitude=%s",
            position_valid ? "valid" : "invalid",
            velocity_valid ? "valid" : "invalid",
            attitude_valid ? "valid" : "invalid");

        return;
        }

        const double norm = std::sqrt(norm_squared);
        const double qw = q[0] / norm;
        const double qx = q[1] / norm;
        const double qy = q[2] / norm;
        const double qz = q[3] / norm;

        const double yaw_rad = std::atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)
        );

        constexpr double radians_to_degrees = 180.0 / 3.14159265358979323846;
        
        const bool armed = 
            status_->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
        
        RCLCPP_INFO(
            get_logger(),
            "sys=%u armed=%s nav_state=%u preflight=%s failsafe=%s | "
            "NED[m]=(%.1f, %.1f, %.1f) | "
            "V_NED[m/s]=(%.1f, %.1f, %.1f) | yaw=%.1f deg",
            static_cast<unsigned>(status_->system_id),
            armed ? "yes" : "no",
            static_cast<unsigned>(status_->nav_state),
            status_->pre_flight_checks_pass ? "pass" : "fail",
            status_->failsafe ? "yes" : "no",
            p.x, p.y, p.z,
            p.vx, p.vy, p.vz,
            yaw_rad * radians_to_degrees
        );
        
    }

    px4_msgs::msg::VehicleStatus::ConstSharedPtr status_;
    px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr position_;
    px4_msgs::msg::VehicleAttitude::ConstSharedPtr attitude_;

    Clock::time_point status_received_at_{};
    Clock::time_point position_received_at_{};
    Clock::time_point attitude_received_at_{};

    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TelemetryNode>());
    rclcpp::shutdown();
}
