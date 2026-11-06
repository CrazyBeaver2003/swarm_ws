#include <cmath>
#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <swarm_interfaces/msg/fixed_wing_command.hpp>

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/control/setpoint_types/fixedwing/lateral_longitudinal.hpp>
#include <px4_ros2/odometry/global_position.hpp>
#include <px4_ros2/odometry/local_position.hpp>

class SwarmControlMode : public px4_ros2::ModeBase{
public:
    explicit SwarmControlMode(rclcpp::Node& node) : ModeBase(node, Settings{"Swarm Control"}.preventArming(true))
    {
        global_position_= std::make_shared<px4_ros2::OdometryGlobalPosition>(*this);
        local_position_= std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
        setpoint_ = std::make_shared<px4_ros2::FwLateralLongitudinalSetpointType>(*this);
        
        auto qos = rclcpp::QoS(1);
        qos.reliable().durability_volatile();

        command_sub_ = node.create_subscription<Command>(
            "~/command",
            qos,
            [this](Command::ConstSharedPtr msg){
                onCommand(*msg);
            }
        );
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

    static constexpr double kTimeoutS = 2.0;
    static constexpr float kPi = 3.14159265358979323846f;

    // Соответствуют настройкам Cessna в проверенном ULog.
    static constexpr float kMinAirspeed = 10.0f;
    static constexpr float kMaxAirspeed = 20.0f;

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

        if (!mode_active_ || !targets_valid_ || command_timed_out_) {
            return;
        }
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
            stamp_ns < activation_stamp_ns_ ||
            stamp_ns <= last_command_stamp_ns_ ||
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
        course_sp_=msg.course_rad;
        altitude_amsl_sp_ = msg.altitude_amsl_m;
        airspeed_sp_ = msg.equivalent_airspeed_m_s;

        last_command_stamp_ns_ = stamp_ns;
        // Учитываем возраст команды при получении.
        // Дальше отсчитываем срок по монотонным часам.
        command_deadline_=
            Clock::now() + std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(kTimeoutS - std::max(age_s, 0.0)));
        RCLCPP_INFO_THROTTLE(
            node().get_logger(),
            *node().get_clock(),
            1000,
            "Command: course=%.3f rad altitude=%.1f m AMSL EAS=%.1f m/s",
            course_sp_,
            altitude_amsl_sp_,
            airspeed_sp_);

    }
    std::shared_ptr<
        px4_ros2::OdometryGlobalPosition> global_position_;

    std::shared_ptr<
        px4_ros2::OdometryLocalPosition> local_position_;

    std::shared_ptr<
        px4_ros2::FwLateralLongitudinalSetpointType> setpoint_;

    rclcpp::Subscription<Command>::SharedPtr command_sub_;

    float altitude_amsl_sp_ = 0.0f;
    float course_sp_ = 0.0f;
    float airspeed_sp_{15.0f};

    bool targets_valid_ = false;
    bool mode_active_{false};
    bool command_timed_out_{false};

    int64_t activation_stamp_ns_{0};
    int64_t last_command_stamp_ns_{0};

    Clock::time_point command_deadline_{};
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