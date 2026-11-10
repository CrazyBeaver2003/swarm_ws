#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <stdexcept>


#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <swarm_interfaces/msg/fixed_wing_command.hpp>

#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/control/setpoint_types/fixedwing/lateral_longitudinal.hpp>
#include <px4_ros2/odometry/global_position.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/utils/message_version.hpp>
#include <px4_ros2/utils/subscription.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>

#include "takeoff_controller.hpp"

class SwarmControlMode : public px4_ros2::ModeBase{
public:
    explicit SwarmControlMode(rclcpp::Node& node) : ModeBase(node, Settings{"Swarm Control"}.preventArming(true))
    {
        global_position_= std::make_shared<px4_ros2::OdometryGlobalPosition>(*this);
        local_position_= std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
        setpoint_ = std::make_shared<px4_ros2::FwLateralLongitudinalSetpointType>(*this);
        vehicle_status_ = std::make_shared<px4_ros2::Subscription<VehicleStatus>>(
            *this,
            "fmu/out/vehicle_status" + px4_ros2::getMessageNameVersion<VehicleStatus>());
        
        auto qos = rclcpp::QoS(1);
        qos.reliable().durability_volatile();

        command_sub_ = node.create_subscription<Command>(
            "~/command",
            qos,
            [this](Command::ConstSharedPtr msg){
                onCommand(*msg);
            }
        );

        reset_fault_service_ = node.create_service<ResetFault>(
            "~/reset_fault",
            [this](ResetFault::Request::SharedPtr, ResetFault::Response::SharedPtr response) {
                onResetFault(*response);
            });

        const auto flight_namespace = node.declare_parameter<std::string>(
            "flight_namespace", "/plane_1/flight");

        if (flight_namespace.empty() ||
            flight_namespace.front() != '/' ||
            flight_namespace.back() == '/') {
            throw std::invalid_argument(
                "flight_namespace must be absolute, without a trailing slash");
        }

        land_detected_ = std::make_shared<
            px4_ros2::Subscription<VehicleLandDetected>>(
                *this,
                "fmu/out/vehicle_land_detected" +
                    px4_ros2::getMessageNameVersion<VehicleLandDetected>());

        mode_command_pub_ = node.create_publisher<VehicleCommand>(
            "fmu/in/vehicle_command" +
                px4_ros2::getMessageNameVersion<VehicleCommand>(),
            10);

        takeoff_ = std::make_unique<TakeoffController>(
            node,
            flight_namespace,
            vehicle_status_,
            land_detected_,
            global_position_,
            local_position_,
            mode_command_pub_);

        hold_service_ = node.create_service<FlightRequest>(
            flight_namespace + "/hold",
            [this](
                FlightRequest::Request::SharedPtr,
                FlightRequest::Response::SharedPtr response) {
                takeoff_->cancel();

                requestMode(
                    VehicleStatus::NAVIGATION_STATE_AUTO_LOITER,
                    *response);
            });

        activate_service_ = node.create_service<FlightRequest>(
            flight_namespace + "/activate_swarm",
            [this](
                FlightRequest::Request::SharedPtr,
                FlightRequest::Response::SharedPtr response) {
                requestActivation(*response);
            });
    }


    void checkArmingAndRunConditions(
        px4_ros2::HealthAndArmingCheckReporter& reporter) override
    {
        checkCommandTimeout();
        if (!dataReady() || (isActive() && !targets_valid_)) {
            reporter.armingCheckFailureExt(
                px4_ros2::events::ID("swarm_control_invalid_odometry"),
                px4_ros2::events::Log::Error,
                "Swarm control needs valid position and velocity"
            );
        }
        if (command_timed_out_) {
            reporter.armingCheckFailureExt(
                px4_ros2::events::ID("swarm_control_command_timeout"),
                px4_ros2::events::Log::Error,
                "Swarm control command timeout");
        }
    }
    void onActivate() override
    {
        mode_active_=true;
        targets_valid_ = false;
        last_command_stamp_ns_ = 0;

        activation_stamp_ns_ = node().now().nanoseconds();

        // Две секунды на получение первой команды.
        command_deadline_ = Clock::now() + std::chrono::seconds(2);

        // Повторная проверка непосредственно при активации
        if (!dataReady()) {
        RCLCPP_ERROR(
            node().get_logger(),
            "Cannot activate: odometry unavailable");
        return;
        }

        // Высота над средним уровнем моря, метры AMSL.
        altitude_amsl_sp_ = static_cast<float>(global_position_->position().z());
        
        // Направление движения относительно земли.
        const auto velocity = local_position_->velocityNed();
        course_sp_ = std::atan2(velocity.y(), velocity.x());

        airspeed_sp_=15.0f;
    
        targets_valid_ = true;
        
        RCLCPP_INFO(
            node().get_logger(),
            "Swarm Control active, waiting for commands");
    }

    void onDeactivate() override
    {
        mode_active_=false;
        targets_valid_ = false;


        RCLCPP_INFO(
            node().get_logger(),
            "Swarm Control deactivated");
    }

    void updateSetpoint(float) override
    {
        checkCommandTimeout();
        if (targets_valid_) {
            setpoint_->updateWithAltitude(
                altitude_amsl_sp_,
                course_sp_,
                airspeed_sp_);
        }
    }
private:
    using Command = swarm_interfaces::msg::FixedWingCommand;
    using Clock = std::chrono::steady_clock;
    using ResetFault = std_srvs::srv::Trigger;
    using VehicleStatus = px4_msgs::msg::VehicleStatus;
    using VehicleCommand = px4_msgs::msg::VehicleCommand;
    using VehicleLandDetected = px4_msgs::msg::VehicleLandDetected;
    using FlightRequest = std_srvs::srv::Trigger;

    static constexpr double kTimeoutS = 2.0;
    static constexpr float kPi = 3.14159265358979323846f;

    // Соответствуют настройкам Cessna в проверенном ULog.
    static constexpr float kMinAirspeed = 10.0f;
    static constexpr float kMaxAirspeed = 20.0f;

    bool flightStatusFresh()
    {
        return vehicle_status_->lastValid(std::chrono::seconds(1)) &&
               node().now() >= vehicle_status_->lastTime();
    }

    void requestMode(
        uint8_t desired_mode,
        FlightRequest::Response& response)
    {
        response.success = false;

        if (!flightStatusFresh()) {
            response.message = "PX4 status is unavailable or stale";
            return;
        }

        const auto& status = vehicle_status_->last();

        if (status.system_id == 0 || status.component_id == 0) {
            response.message = "PX4 system/component ID is invalid";
            return;
        }

        if (mode_command_pub_->get_subscription_count() == 0) {
            response.message = "PX4 command subscriber is unavailable";
            return;
        }

        VehicleCommand command{};
        command.command = VehicleCommand::VEHICLE_CMD_SET_NAV_STATE;
        command.param1 = static_cast<float>(desired_mode);

        command.target_system = status.system_id;
        command.target_component = status.component_id;

        command.source_system = 255;
        command.source_component = 191;
        command.from_external = true;

        // В нашей версии DDS-мост PX4 заменяет timestamp=0
        // временем приёма сообщения внутри PX4.
        mode_command_pub_->publish(command);

        response.success = true;
        response.message =
            "Mode request published; verify actual PX4 nav_state=" +
            std::to_string(static_cast<unsigned>(desired_mode));

        RCLCPP_INFO(
            node().get_logger(),
            "%s",
            response.message.c_str());
    }

    void requestActivation(FlightRequest::Response& response)
    {
        response.success = false;
        if (takeoff_->busy()) {
            response.message = "Wait for takeoff completion";
            return;
        }
        checkCommandTimeout();

        if (!flightStatusFresh()) {
            response.message = "PX4 status is unavailable or stale";
            return;
        }

        const auto& status = vehicle_status_->last();

        if (status.arming_state != VehicleStatus::ARMING_STATE_ARMED ||
            status.failsafe) {
            response.message =
                "Need an armed aircraft without an active failsafe";
            return;
        }

        if (!land_detected_->lastValid(std::chrono::seconds(2)) ||
            node().now() < land_detected_->lastTime() ||
            land_detected_->last().landed) {
            response.message =
                "Need fresh confirmation that the aircraft is airborne";
            return;
        }

        if (!dataReady()) {
            response.message =
                "Need valid position and horizontal velocity above 1 m/s";
            return;
        }

        if (command_timed_out_) {
            response.message =
                "Clear the command timeout with reset_fault first";
            return;
        }

        if (!commandSourceFresh()) {
            response.message =
                "Start publishing fresh commands before activation";
            return;
        }

        if (id() == px4_ros2::ModeBase::kModeIDInvalid) {
            response.message = "Swarm Control is not registered in PX4";
            return;
        }

        requestMode(id(), response);
    }

    bool dataReady() const
    {
        if (
            !global_position_->positionValid() ||
            !local_position_->velocityXYValid()
        )
        {
            return false;
        }

        const auto velocity = local_position_->velocityNed();

        return std::isfinite(global_position_->position().z()) &&
            std::isfinite(velocity.x()) &&
            std::isfinite(velocity.y()) &&
            std::hypot(velocity.x(), velocity.y()) > 1.0f;
    }
    void checkCommandTimeout(){
        if (
            mode_active_ &&
            targets_valid_ &&
            !command_timed_out_ &&
            Clock::now() >= command_deadline_)
        {
            command_timed_out_=true;
             RCLCPP_ERROR(
                node().get_logger(),
                "Command timeout; reporting failure to PX4");
        }
    }

    void onCommand(const Command& msg){
        checkCommandTimeout();

        // Невалидные команды не продлевают тайм-аут.
        if (
            !std::isfinite(msg.course_rad) ||
            std::abs(msg.course_rad) > kPi ||
            !std::isfinite(msg.altitude_amsl_m) ||
            !std::isfinite(msg.equivalent_airspeed_m_s) ||
            msg.equivalent_airspeed_m_s < kMinAirspeed ||
            msg.equivalent_airspeed_m_s > kMaxAirspeed ||
            msg.stamp.sec < 0 ||
            msg.stamp.nanosec >= 1000000000u)
        {
            RCLCPP_WARN_THROTTLE(
                node().get_logger(),
                *node().get_clock(),
                2000,
                "Rejected command: invalid values or ranges");
            return;
        }

        const int64_t stamp_ns =
            static_cast<int64_t>(msg.stamp.sec) * 1000000000LL +
            msg.stamp.nanosec;

        const double age_s =
            (node().now().nanoseconds() - stamp_ns) * 1e-9;

        if (
            stamp_ns <= 0 ||
            stamp_ns <= last_observed_command_stamp_ns_ ||
            age_s < -0.1 ||
            age_s >= kTimeoutS)
        {
            RCLCPP_WARN_THROTTLE(
                node().get_logger(),
                *node().get_clock(),
                2000,
                "Rejected command: stale, repeated or invalid timestamp");
            return;
        }

        // Наблюдаем исправный источник даже в Hold и после тайм-аута.
        // Это не меняет задания самолёту и не снимает ошибку автоматически.
        last_observed_command_stamp_ns_ = stamp_ns;
        observed_command_deadline_ =
            Clock::now() + std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(kTimeoutS - std::max(age_s, 0.0)));

        if (!mode_active_ || !targets_valid_ || command_timed_out_) {
            return;
        }

        // Команда, созданная до активации, не становится заданием нового сеанса.
        if (stamp_ns < activation_stamp_ns_ || stamp_ns <= last_command_stamp_ns_) {
            return;
        }

        course_sp_=msg.course_rad;
        altitude_amsl_sp_ = msg.altitude_amsl_m;
        airspeed_sp_ = msg.equivalent_airspeed_m_s;

        last_command_stamp_ns_ = stamp_ns;
        // Учитываем возраст команды при получении.
        // Дальше отсчитываем срок по монотонным часам.
        command_deadline_ = observed_command_deadline_;
        RCLCPP_INFO_THROTTLE(
            node().get_logger(),
            *node().get_clock(),
            1000,
            "Command: course=%.3f rad altitude=%.1f m AMSL EAS=%.1f m/s",
            course_sp_,
            altitude_amsl_sp_,
            airspeed_sp_);

    }

    bool commandSourceFresh()
    {
        if (last_observed_command_stamp_ns_ <= 0 ||
            Clock::now() >= observed_command_deadline_)
        {
            return false;
        }

        const double age_s =
            (node().now().nanoseconds() - last_observed_command_stamp_ns_) * 1e-9;
        return age_s >= -0.1 && age_s < kTimeoutS;
    }

    void onResetFault(ResetFault::Response& response)
    {
        response.success = false;

        if (!vehicle_status_->lastValid()) {
            response.message = "PX4 vehicle status is unavailable or stale";
            return;
        }

        const auto& status = vehicle_status_->last();
        if (mode_active_ || isActive() ||
            status.nav_state != VehicleStatus::NAVIGATION_STATE_AUTO_LOITER ||
            status.nav_state_user_intention != VehicleStatus::NAVIGATION_STATE_AUTO_LOITER ||
            status.failsafe)
        {
            response.message = "Select Hold explicitly and wait for Swarm Control to deactivate";
            return;
        }

        if (!dataReady()) {
            response.message = "Valid position and horizontal velocity above 1 m/s are required";
            return;
        }

        if (!commandSourceFresh()) {
            response.message = "Resume fresh valid commands before resetting the fault";
            return;
        }

        command_timed_out_ = false;
        response.success = true;
        response.message = "Fault cleared; select Swarm Control manually while commands continue";
        RCLCPP_INFO(node().get_logger(), "%s", response.message.c_str());
    }

    std::shared_ptr<
        px4_ros2::OdometryGlobalPosition> global_position_;

    std::shared_ptr<
        px4_ros2::OdometryLocalPosition> local_position_;

    std::shared_ptr<
        px4_ros2::FwLateralLongitudinalSetpointType> setpoint_;

    std::shared_ptr<px4_ros2::Subscription<VehicleStatus>> vehicle_status_;
    rclcpp::Subscription<Command>::SharedPtr command_sub_;
    rclcpp::Service<ResetFault>::SharedPtr reset_fault_service_;
    
    std::shared_ptr<px4_ros2::Subscription<VehicleLandDetected>> land_detected_;

    rclcpp::Publisher<VehicleCommand>::SharedPtr mode_command_pub_;

    rclcpp::Service<FlightRequest>::SharedPtr hold_service_;
    rclcpp::Service<FlightRequest>::SharedPtr activate_service_;

    float altitude_amsl_sp_ = 0.0f;
    float course_sp_ = 0.0f;
    float airspeed_sp_{15.0f};

    bool targets_valid_ = false;
    bool mode_active_{false};
    bool command_timed_out_{false};

    int64_t activation_stamp_ns_{0};
    int64_t last_command_stamp_ns_{0};
    int64_t last_observed_command_stamp_ns_{0};

    Clock::time_point command_deadline_{};
    Clock::time_point observed_command_deadline_{};
    std::unique_ptr<TakeoffController> takeoff_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try{
        auto node = std::make_shared<px4_ros2::NodeWithMode<SwarmControlMode>>("swarm_control");
        rclcpp::spin(node);
    }
    catch (const std::exception& e){
        RCLCPP_FATAL(
            rclcpp::get_logger("swarm_control"),
            "%s",
            e.what()
        );
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
