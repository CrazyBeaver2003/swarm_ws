#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <GeographicLib/LocalCartesian.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <swarm_interfaces/msg/vehicle_state.hpp>

class TelemetryNode : public rclcpp::Node
{
    using Clock = std::chrono::steady_clock;
    using Status = px4_msgs::msg::VehicleStatus;
    using Global = px4_msgs::msg::VehicleGlobalPosition;
    using Local = px4_msgs::msg::VehicleLocalPosition;
    using Attitude = px4_msgs::msg::VehicleAttitude;
    using State = swarm_interfaces::msg::VehicleState;

public:
    explicit TelemetryNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("telemetry_node", options)
    {
        rcl_interfaces::msg::ParameterDescriptor fixed;
        fixed.read_only = true;
        const double nan = std::numeric_limits<double>::quiet_NaN();
        frame_id_ = declare_parameter<std::string>("frame_id", "", fixed);
        const double lat = declare_parameter("origin_lat_deg", nan, fixed);
        const double lon = declare_parameter("origin_lon_deg", nan, fixed);
        const double amsl = declare_parameter("origin_alt_amsl_m", nan, fixed);
        const double geoid = declare_parameter("origin_geoid_separation_m", 0.0, fixed);
        const double rate = declare_parameter("publish_rate_hz", 20.0, fixed);
        state_timeout_ = declare_parameter("state_timeout_s", 0.5, fixed);
        status_timeout_ = declare_parameter("status_timeout_s", 2.0, fixed);
        px4_namespace_ = declare_parameter<std::string>("px4_namespace", "", fixed);

        if (frame_id_.empty() || !validLatLon(lat, lon) ||
            !std::isfinite(amsl) || !std::isfinite(geoid) ||
            !std::isfinite(rate) || rate <= 0.0 || rate > 100.0 ||
            !std::isfinite(state_timeout_) || state_timeout_ <= 0.0 ||
            !std::isfinite(status_timeout_) || status_timeout_ <= 0.0)
        {
            throw std::invalid_argument("Invalid telemetry parameters; pass the reference YAML");
        }

        while (!px4_namespace_.empty() && px4_namespace_.back() == '/') {
            px4_namespace_.pop_back();
        }
        if (!px4_namespace_.empty() && px4_namespace_.front() != '/') {
            px4_namespace_.insert(0, "/");
        }
        origin_height_ = amsl + geoid;
        projection_.Reset(lat, lon, origin_height_);

        status_sub_ = subscribe<Status>("vehicle_status", status_, status_at_);
        global_sub_ = subscribe<Global>("vehicle_global_position", global_, global_at_);
        local_sub_ = subscribe<Local>("vehicle_local_position", local_, local_at_);
        attitude_sub_ = subscribe<Attitude>("vehicle_attitude", attitude_, attitude_at_);
        state_pub_ = create_publisher<State>("state", rclcpp::QoS(1).reliable());
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / rate)),
            [this]() { publishState(); });
        RCLCPP_INFO(get_logger(), "Publishing %s in frame %s",
                    state_pub_->get_topic_name(), frame_id_.c_str());
    }

private:
    static bool validLatLon(double lat, double lon)
    {
        return std::isfinite(lat) && std::isfinite(lon) &&
               std::abs(lat) <= 90.0 && std::abs(lon) <= 180.0;
    }

    template<typename Message>
    typename rclcpp::Subscription<Message>::SharedPtr subscribe(
        const std::string& name, std::shared_ptr<const Message>& latest,
        Clock::time_point& received_at)
    {
        std::string topic = px4_namespace_ + "/fmu/out/" + name;
        if (Message::MESSAGE_VERSION > 0) {
            topic += "_v" + std::to_string(Message::MESSAGE_VERSION);
        }
        auto qos = rclcpp::SensorDataQoS();
        qos.keep_last(1);
        return create_subscription<Message>(
            topic, qos,
            [&latest, &received_at](typename Message::ConstSharedPtr msg) {
                latest = std::move(msg);
                received_at = Clock::now();
            });
    }

    template<typename Message>
    static float age(const std::shared_ptr<const Message>& msg,
                     Clock::time_point received_at, Clock::time_point now)
    {
        return msg ? static_cast<float>(std::chrono::duration<double>(now - received_at).count())
                   : std::numeric_limits<float>::infinity();
    }

    void publishState()
    {
        const auto received_now = Clock::now();
        const float global_age = age(global_, global_at_, received_now);
        const float local_age = age(local_, local_at_, received_now);
        const float attitude_age = age(attitude_, attitude_at_, received_now);
        const double nan = std::numeric_limits<double>::quiet_NaN();

        State out;
        out.header.stamp = now();
        out.header.frame_id = frame_id_;
        out.position.x = out.position.y = out.position.z = nan;
        out.velocity.x = out.velocity.y = out.velocity.z = nan;
        out.orientation.x = out.orientation.y = out.orientation.z = out.orientation.w = nan;
        out.altitude_amsl_m = std::numeric_limits<float>::quiet_NaN();
        out.nav_state = out.nav_state_user_intention = State::NAV_STATE_UNKNOWN;
        out.position_age_s = out.altitude_age_s = global_age;
        out.velocity_age_s = std::max(local_age, global_age);
        out.attitude_age_s = std::max(attitude_age, global_age);
        out.status_age_s = age(status_, status_at_, received_now);
        out.vehicle_id = status_ ? status_->system_id : 0;

        out.status_valid = status_ && out.status_age_s <= status_timeout_ &&
            status_->system_id != 0 &&
            (status_->arming_state == Status::ARMING_STATE_ARMED ||
             status_->arming_state == Status::ARMING_STATE_DISARMED);
        if (out.status_valid) {
            out.armed = status_->arming_state == Status::ARMING_STATE_ARMED;
            out.failsafe = status_->failsafe;
            out.nav_state = status_->nav_state;
            out.nav_state_user_intention = status_->nav_state_user_intention;
        }

        out.altitude_valid = global_ && global_age <= state_timeout_ &&
            global_->alt_valid && std::isfinite(global_->alt);
        if (out.altitude_valid) {
            out.altitude_amsl_m = global_->alt;
        }

        const bool geographic_valid = global_ && global_age <= state_timeout_ &&
            global_->lat_lon_valid && validLatLon(global_->lat, global_->lon);
        if (geographic_valid) {
            double east, north, up;
            std::vector<double> rotation(9);
            const bool height_valid = out.altitude_valid && std::isfinite(global_->alt_ellipsoid);
            projection_.Forward(global_->lat, global_->lon,
                height_valid ? global_->alt_ellipsoid : origin_height_,
                east, north, up, rotation);
            out.position_valid = height_valid && std::isfinite(east) &&
                std::isfinite(north) && std::isfinite(up);
            if (out.position_valid) {
                out.position.x = east;
                out.position.y = north;
                out.position.z = up;
            }

            // GeographicLib returns current ENU -> common ENU.
            const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>
                common_from_current_enu(rotation.data());
            Eigen::Matrix3d enu_from_ned;
            enu_from_ned << 0, 1, 0, 1, 0, 0, 0, 0, -1;
            const Eigen::Matrix3d common_from_ned = common_from_current_enu * enu_from_ned;

            out.velocity_valid = local_ && out.velocity_age_s <= state_timeout_ &&
                local_->v_xy_valid && local_->v_z_valid &&
                std::isfinite(local_->vx) && std::isfinite(local_->vy) && std::isfinite(local_->vz);
            if (out.velocity_valid) {
                const Eigen::Vector3d v = common_from_ned *
                    Eigen::Vector3d(local_->vx, local_->vy, local_->vz);
                out.velocity.x = v.x();
                out.velocity.y = v.y();
                out.velocity.z = v.z();
            }

            if (attitude_ && out.attitude_age_s <= state_timeout_) {
                const auto& q = attitude_->q;
                Eigen::Quaterniond ned_from_frd(q[0], q[1], q[2], q[3]);
                const double norm_squared = ned_from_frd.squaredNorm();
                if (std::isfinite(norm_squared) && std::abs(norm_squared - 1.0) < 0.05) {
                    ned_from_frd.normalize();
                    // FLU -> FRD is a 180-degree rotation about body X.
                    Eigen::Quaterniond common_from_flu =
                        Eigen::Quaterniond(common_from_ned) * ned_from_frd *
                        Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0);
                    common_from_flu.normalize();
                    out.orientation.x = common_from_flu.x();
                    out.orientation.y = common_from_flu.y();
                    out.orientation.z = common_from_flu.z();
                    out.orientation.w = common_from_flu.w();
                    out.attitude_valid = true;
                }
            }
        }

        state_pub_->publish(out);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "id=%u valid[p/v/q/s]=%d/%d/%d/%d ENU=(%.1f, %.1f, %.1f) nav=%u failsafe=%d",
            static_cast<unsigned>(out.vehicle_id), out.position_valid, out.velocity_valid,
            out.attitude_valid, out.status_valid, out.position.x, out.position.y, out.position.z,
            static_cast<unsigned>(out.nav_state), out.failsafe);
    }

    std::string frame_id_, px4_namespace_;
    double state_timeout_{0.5}, status_timeout_{2.0}, origin_height_{0.0};
    GeographicLib::LocalCartesian projection_;
    Status::ConstSharedPtr status_;
    Global::ConstSharedPtr global_;
    Local::ConstSharedPtr local_;
    Attitude::ConstSharedPtr attitude_;
    Clock::time_point status_at_{}, global_at_{}, local_at_{}, attitude_at_{};
    rclcpp::Subscription<Status>::SharedPtr status_sub_;
    rclcpp::Subscription<Global>::SharedPtr global_sub_;
    rclcpp::Subscription<Local>::SharedPtr local_sub_;
    rclcpp::Subscription<Attitude>::SharedPtr attitude_sub_;
    rclcpp::Publisher<State>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    int result = 0;
    try {
        rclcpp::spin(std::make_shared<TelemetryNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("telemetry_node"), "%s", e.what());
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
