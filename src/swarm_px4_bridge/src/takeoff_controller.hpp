#pragma once

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <px4_ros2/odometry/global_position.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/utils/message_version.hpp>
#include <px4_ros2/utils/subscription.hpp>

class TakeoffController {
    using Status = px4_msgs::msg::VehicleStatus;
    using Land = px4_msgs::msg::VehicleLandDetected;
    using Command = px4_msgs::msg::VehicleCommand;
    using Ack = px4_msgs::msg::VehicleCommandAck;
    using Trigger = std_srvs::srv::Trigger;
    using Clock = std::chrono::steady_clock;

    enum class Phase {
        Idle,
        TakeoffMode,
        Arming,
        Climbing
    };

public:
    TakeoffController(
        rclcpp::Node& node,
        const std::string& flight_namespace,
        std::shared_ptr<px4_ros2::Subscription<Status>> status,
        std::shared_ptr<px4_ros2::Subscription<Land>> land,
        std::shared_ptr<px4_ros2::OdometryGlobalPosition> global,
        std::shared_ptr<px4_ros2::OdometryLocalPosition> local,
        rclcpp::Publisher<Command>::SharedPtr publisher)
        : node_(node),
          status_(status),
          land_(land),
          global_(global),
          local_(local),
          publisher_(publisher)
    {
        height_ = node_.declare_parameter<double>(
            "takeoff_height_m", 70.0);

        timeout_ = node_.declare_parameter<double>(
            "takeoff_timeout_s", 300.0);

        if (!std::isfinite(height_) ||
            height_ < 20.0 || height_ > 250.0 ||
            !std::isfinite(timeout_) ||
            timeout_ < 30.0 || timeout_ > 600.0) {
            throw std::invalid_argument(
                "Invalid takeoff height or timeout");
        }

        ack_sub_ = node_.create_subscription<Ack>(
            "fmu/out/vehicle_command_ack" +
                px4_ros2::getMessageNameVersion<Ack>(),
            rclcpp::SensorDataQoS().keep_last(20),
            [this](Ack::ConstSharedPtr ack) {
                onAck(*ack);
            });

        service_ = node_.create_service<Trigger>(
            flight_namespace + "/takeoff",
            [this](
                Trigger::Request::SharedPtr,
                Trigger::Response::SharedPtr response) {
                start(*response);
            });

        timer_ = node_.create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                tick();
            });
    }

    bool busy() const
    {
        return phase_ != Phase::Idle;
    }

    void cancel()
    {
        if (busy()) {
            phase_ = Phase::Idle;
            RCLCPP_INFO(
                node_.get_logger(),
                "Automatic takeoff cancelled");
        }
    }

private:
    bool statusFresh() const
    {
        return status_->lastValid(std::chrono::seconds(1)) &&
               node_.now() >= status_->lastTime();
    }

    bool dataFresh() const
    {
        return statusFresh() &&
               land_->lastValid(std::chrono::seconds(2)) &&
               node_.now() >= land_->lastTime() &&
               global_->positionValid() &&
               local_->positionXYValid() &&
               std::isfinite(global_->position().z());
    }

    Command command(uint32_t id) const
    {
        Command out{};
        out.command = id;
        out.target_system = system_id_;
        out.target_component = component_id_;
        out.source_system = 255;
        out.source_component = 191;
        out.from_external = true;
        return out;
    }

    void send(const Command& out, Phase phase)
    {
        phase_ = phase;
        phase_started_ = Clock::now();

        ack_received_ = false;
        expected_command_ = out.command;
        min_ack_timestamp_ = status_->last().timestamp;

        publisher_->publish(out);
    }

    void start(Trigger::Response& response)
    {
        response.success = false;

        if (busy()) {
            response.message = "Takeoff already running";
            return;
        }

        if (!dataFresh() ||
            publisher_->get_subscription_count() == 0) {
            response.message =
                "Fresh navigation, land status and PX4 command link required";
            return;
        }

        const auto& s = status_->last();

        if (s.system_id == 0 ||
            s.component_id == 0 ||
            s.failsafe ||
            s.arming_state != Status::ARMING_STATE_DISARMED ||
            s.nav_state != Status::NAVIGATION_STATE_AUTO_LOITER ||
            s.nav_state_user_intention != Status::NAVIGATION_STATE_AUTO_LOITER ||
            !s.pre_flight_checks_pass ||
            !land_->last().landed ||
            !land_->last().at_rest) {
            response.message =
                "Need a ready, disarmed aircraft at rest in Hold";
            return;
        }

        system_id_ = s.system_id;
        component_id_ = s.component_id;

        target_altitude_ = static_cast<float>(
            global_->position().z() + height_);

        auto out = command(Command::VEHICLE_CMD_NAV_TAKEOFF);

        out.param4 = std::numeric_limits<float>::quiet_NaN();
        out.param5 = std::numeric_limits<double>::quiet_NaN();
        out.param6 = std::numeric_limits<double>::quiet_NaN();
        out.param7 = target_altitude_;

        started_ = last_good_data_ = Clock::now();

        send(out, Phase::TakeoffMode);

        response.success = true;
        response.message =
            "Takeoff sequence started; completion is reported in the log";

        RCLCPP_INFO(
            node_.get_logger(),
            "Takeoff requested: target %.1f m AMSL",
            target_altitude_);
    }

    void fail(const std::string& reason)
    {
        phase_ = Phase::Idle;

        RCLCPP_ERROR(
            node_.get_logger(),
            "Automatic takeoff failed: %s",
            reason.c_str());

        // Запрашиваем Hold только если самолёт ещё в Takeoff.
        // Другой режим оператора или failsafe не перебиваем.
        if (statusFresh() &&
            !status_->last().failsafe &&
            status_->last().system_id == system_id_ &&
            status_->last().component_id == component_id_ &&
            status_->last().nav_state ==
                Status::NAVIGATION_STATE_AUTO_TAKEOFF &&
            status_->last().nav_state_user_intention ==
                Status::NAVIGATION_STATE_AUTO_TAKEOFF) {
            auto out = command(Command::VEHICLE_CMD_SET_NAV_STATE);
            out.param1 = Status::NAVIGATION_STATE_AUTO_LOITER;
            publisher_->publish(out);
        }
    }

    void onAck(const Ack& ack)
    {
        if ((phase_ != Phase::TakeoffMode &&
             phase_ != Phase::Arming) ||
            ack.command != expected_command_ ||
            ack.target_system != 255 ||
            ack.target_component != 191 ||
            ack.timestamp < min_ack_timestamp_) {
            return;
        }

        if (ack.result == Ack::VEHICLE_CMD_RESULT_ACCEPTED) {
            ack_received_ = true;
        } else if (ack.result != Ack::VEHICLE_CMD_RESULT_IN_PROGRESS) {
            fail(
                "PX4 rejected command " +
                std::to_string(ack.command) +
                ", result=" + std::to_string(ack.result));
        }
    }

    void tick()
    {
        if (!busy()) {
            return;
        }

        const auto now = Clock::now();

        if (std::chrono::duration<double>(
                now - started_).count() > timeout_ ||
            (phase_ != Phase::Climbing &&
             now - phase_started_ > std::chrono::seconds(5))) {
            fail("timeout");
            return;
        }

        if (!dataFresh()) {
            if (now - last_good_data_ > std::chrono::seconds(2)) {
                fail("navigation or status did not recover");
            }
            return;
        }

        last_good_data_ = now;

        const auto& s = status_->last();

        const bool takeoff =
            s.nav_state == Status::NAVIGATION_STATE_AUTO_TAKEOFF &&
            s.nav_state_user_intention ==
                Status::NAVIGATION_STATE_AUTO_TAKEOFF;

        const bool hold =
            s.nav_state == Status::NAVIGATION_STATE_AUTO_LOITER &&
            s.nav_state_user_intention ==
                Status::NAVIGATION_STATE_AUTO_LOITER;

        if (s.system_id != system_id_ ||
            s.component_id != component_id_ ||
            s.failsafe ||
            (!takeoff && !hold)) {
            fail("vehicle ID, failsafe or flight mode changed");
            return;
        }

        if (phase_ == Phase::TakeoffMode) {
            if (s.arming_state != Status::ARMING_STATE_DISARMED ||
                !land_->last().landed ||
                !land_->last().at_rest) {
                fail("aircraft state changed before arming");
                return;
            }

            if (ack_received_ && takeoff) {
                if (!s.pre_flight_checks_pass) {
                    fail("preflight checks failed in Takeoff mode");
                    return;
                }

                auto out = command(
                    Command::VEHICLE_CMD_COMPONENT_ARM_DISARM);

                out.param1 = 1.0f;
                out.param2 = 0.0f;  // Обычное армирование.

                send(out, Phase::Arming);

                RCLCPP_INFO(
                    node_.get_logger(),
                    "Takeoff mode confirmed; arming requested");
            }
        } else if (phase_ == Phase::Arming) {
            if (!takeoff) {
                fail("Takeoff mode left while arming");
                return;
            }

            if (ack_received_ &&
                s.arming_state == Status::ARMING_STATE_ARMED) {
                phase_ = Phase::Climbing;

                RCLCPP_INFO(
                    node_.get_logger(),
                    "Arming confirmed; waiting for climb");
            }
        } else {
            if (s.arming_state != Status::ARMING_STATE_ARMED) {
                fail("aircraft disarmed during takeoff");
                return;
            }

            if (hold) {
                if (land_->last().landed) {
                    fail("Hold entered while still on the ground");
                    return;
                }

                if (std::abs(
                        global_->position().z() -
                        target_altitude_) > 5.0) {
                    return;
                }

                phase_ = Phase::Idle;

                RCLCPP_INFO(
                    node_.get_logger(),
                    "Takeoff complete: airborne at %.1f m AMSL in Hold",
                    global_->position().z());
            }
        }
    }

    rclcpp::Node& node_;

    std::shared_ptr<px4_ros2::Subscription<Status>> status_;
    std::shared_ptr<px4_ros2::Subscription<Land>> land_;
    std::shared_ptr<px4_ros2::OdometryGlobalPosition> global_;
    std::shared_ptr<px4_ros2::OdometryLocalPosition> local_;

    rclcpp::Publisher<Command>::SharedPtr publisher_;
    rclcpp::Subscription<Ack>::SharedPtr ack_sub_;
    rclcpp::Service<Trigger>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;

    Phase phase_{Phase::Idle};

    double height_{70.0};
    double timeout_{300.0};
    float target_altitude_{};

    bool ack_received_{false};

    uint8_t system_id_{};
    uint8_t component_id_{};
    uint32_t expected_command_{};
    uint64_t min_ack_timestamp_{};

    Clock::time_point started_{};
    Clock::time_point phase_started_{};
    Clock::time_point last_good_data_{};
};