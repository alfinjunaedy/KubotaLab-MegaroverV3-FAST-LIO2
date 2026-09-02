#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>

using namespace std;
using namespace std::chrono_literals;

bool mission_complete = false;

class RoverController : public rclcpp::Node {
public:
    RoverController() : Node("rover_controller") {
        linear_speed_ = this->declare_parameter<double>("linear_speed", 0.1);   // not used
        angular_speed_ = this->declare_parameter<double>("angular_speed", 0.5); // not used

        param_cb_handle_ = this->add_on_set_parameters_callback(std::bind(&RoverController::on_param_change, this, std::placeholders::_1));
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/rover_twist", 10);
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.best_effort();
        odo_sub_ = create_subscription<geometry_msgs::msg::Twist>("/rover_odo", qos, std::bind(&RoverController::odom_callback, this, std::placeholders::_1));
        sensor_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>("/rover_sensor", qos, std::bind(&RoverController::sensor_callback, this, std::placeholders::_1));

        // x+=forward, y+=left, z+=up, quaternion xyzw, NWU frame.
        fastlio2_pose_sub_ = create_subscription<geometry_msgs::msg::Pose>("/fastlio2_pose", qos, std::bind(&RoverController::fastlio2_pose_callback, this, std::placeholders::_1));

        timer_ = create_wall_timer(100ms, std::bind(&RoverController::update_odometry, this));
        last_time_ = now();
        last_fastlio2_time_ = now();
        RCLCPP_INFO(get_logger(), "Rover Controller Started (linear_speed=%.2f m/s, angular_speed=%.2f rad/s)", linear_speed_, angular_speed_);

        std::thread([this]() {
            std::this_thread::sleep_for(2s);

            // Planner ****************************************************************************************

            const bool bypass = true;
            const float set_delay = 0.1; 
            int i = 0;

            //for (i = 0; i < 10; i++) { // loop until dead
            while (!mission_complete) {

                // Goal pose (x, y, yaw) in the START frame:
                // the robot starts at (0, 0, 0) - x+=forward, y+=left, yaw CCW (rad).
                // This is CLOSED LOOP on absolute pose: if the robot is pushed
                // sideways, it re-computes the error and converges back onto the
                // goal, so go_to_pose(1, 0, 0) really ends at x=1, y=0, yaw=0.

                if (!bypass) {
                    go_to_pose(1.1, 0, 0, 0.5); delay_seconds(set_delay);
                    go_to_pose(2.1, 0, 0, 0.5); delay_seconds(set_delay);
                    go_to_pose(3.1, 0, 0, 0.5); delay_seconds(set_delay);
                    go_to_pose(4.2, 0, 0, 0.5); delay_seconds(set_delay);

                    go_to_pose(4.9, -0.7, -0.7854, 0.5); delay_seconds(set_delay);
                    go_to_pose(5.6, -1.4, -0.7854, 0.5); delay_seconds(set_delay);

                    go_to_pose(5.6, -1.4, -2.3562, 0.5);
                    go_to_pose(5.6, -1.4, -3.1416, 0.5); delay_seconds(set_delay);

                    go_to_pose(4.1, -1.4, -3.1416, 0.5); delay_seconds(set_delay);
                    go_to_pose(3.1, -1.4, -3.1416, 0.5); delay_seconds(set_delay);
                    go_to_pose(2.1, -1.4, -3.1416, 0.5); delay_seconds(set_delay);
                    go_to_pose(1.1, -1.4, -3.1416, 0.5); delay_seconds(set_delay);
                    go_to_pose(0.1, -1.4, -3.1416, 0.5); delay_seconds(set_delay);

                    go_to_pose(0.1, -1.4, -1.5708, 0.5); delay_seconds(set_delay);

                    go_to_pose(0.1, -2.3, -1.5708, 0.5); delay_seconds(set_delay);
                    go_to_pose(0.1, -3.3, -1.5708, 0.5); delay_seconds(set_delay);
                    go_to_pose(0.1, -4.4, -1.5708, 0.5); delay_seconds(set_delay);

                    go_to_pose(0.1, -4.4, -0.7854, 0.5);
                    go_to_pose(0.1, -4.4, 0, 0.5); delay_seconds(set_delay);

                    go_to_pose(1.1, -4.4, 0, 0.5); delay_seconds(set_delay);
                    go_to_pose(2.1, -4.4, 0, 0.5); delay_seconds(set_delay);
                    go_to_pose(3.1, -4.4, 0, 0.5); delay_seconds(set_delay);
                    go_to_pose(4.1, -4.4, 0, 0.5); delay_seconds(set_delay);
                    go_to_pose(5.1, -4.4, 0, 0.5); delay_seconds(set_delay);
                    go_to_pose(6.2, -4.4, 0, 0.5); delay_seconds(set_delay);

                    go_to_pose(6.2, -4.4, 0.7854, 0.5);
                    go_to_pose(6.2, -4.4, 1.5708, 0.5); delay_seconds(set_delay);

                    go_to_pose(6.2, -2.8, 1.5708, 0.5); delay_seconds(set_delay);

                    go_to_pose(6.2, -2.8, 2.3562, 0.5);
                    go_to_pose(6.2, -2.8, 3.1416, 0.5); delay_seconds(set_delay);

                    go_to_pose(5.1, -2.8, 3.1416, 0.5); delay_seconds(set_delay);
                    go_to_pose(4.1, -2.8, 3.1416, 0.5); delay_seconds(set_delay);

                    go_to_pose(4.1, -2.8, -2.6180, 0.5);
                    go_to_pose(4.1, -2.8, -2.3562, 0.5); delay_seconds(set_delay);

                    go_to_pose(3.4, -3.5, -2.3562, 0.5); delay_seconds(set_delay);
                    go_to_pose(2.6, -4.4, -2.3562, 0.5); delay_seconds(set_delay);

                    go_to_pose(2.6, -4.4, -2.6180, 0.5);
                    go_to_pose(2.6, -4.4, -3.1416, 0.5); delay_seconds(set_delay);

                    go_to_pose(1.1, -4.4, -3.1416, 0.5); delay_seconds(set_delay);
                    go_to_pose(0.1, -4.4, -3.1416, 0.5); delay_seconds(set_delay);
                    go_to_pose(-0.9, -4.4, -3.1416, 0.5); delay_seconds(set_delay);
                    go_to_pose(-1.9, -4.4, -3.1416, 0.5); delay_seconds(set_delay);
                    go_to_pose(-2.9, -4.4, -3.1416, 0.5); delay_seconds(set_delay);
                    go_to_pose(-3.9, -4.4, -3.1416, 0.5); delay_seconds(set_delay);

                    go_to_pose(-3.9, -4.4, -2.3562, 0.5);
                    go_to_pose(-3.9, -4.4, -1.5708, 0.5); delay_seconds(set_delay);

                    go_to_pose(-3.9, -4.9, -1.5708, 0.5); delay_seconds(set_delay);

                    go_to_pose(-3.9, -4.9, -0.7854, 0.5);
                    go_to_pose(-3.9, -4.9, 0, 0.5); 
                    go_to_pose(-3.9, -4.9, 0.7854, 0.5); 
                    go_to_pose(-3.9, -4.9, 1.5708, 0.5); delay_seconds(set_delay);

                    go_to_pose(-3.9, -4.4, 1.5708, 0.5); delay_seconds(set_delay);

                    go_to_pose(-3.9, -4.4, 0.7854, 0.5);
                    go_to_pose(-3.9, -4.4, 0, 0.5); delay_seconds(set_delay);

                    go_to_pose(-2.9, -4.4, 0, 0.5); delay_seconds(set_delay);
                    go_to_pose(-1.9, -4.4, 0, 0.5); delay_seconds(set_delay);
                    go_to_pose(-0.9, -4.4, 0, 0.5); delay_seconds(set_delay);
                    go_to_pose(0.15, -4.4, 0, 0.5); delay_seconds(set_delay);

                    go_to_pose(0.15, -4.4, 0.7854, 0.5);
                    go_to_pose(0.15, -4.4, 1.5708, 0.5); delay_seconds(set_delay);

                    go_to_pose(0.15, -3.4, 1.5708, 0.5); delay_seconds(set_delay);
                    go_to_pose(0.15, -2.4, 1.5708, 0.5); delay_seconds(set_delay);
                    go_to_pose(0.15, -1.4, 1.5708, 0.5); delay_seconds(set_delay);
                    go_to_pose(0.15, 0.075, 1.5708, 0.5); delay_seconds(set_delay);

                    go_to_pose(0.15, 0.075, 0.7854, 0.5);
                    go_to_pose(0.15, 0.075, 0, 0.5); delay_seconds(set_delay);
                }
                else {
                    go_to_pose(4.2, 0, 0, 0.5); delay_seconds(set_delay);

                    go_to_pose(5.6, -1.4, -0.7854, 0.5); delay_seconds(set_delay);

                    go_to_pose(5.6, -1.4, -3.1416, 0.5); delay_seconds(set_delay);

                    go_to_pose(0.1, -1.4, -3.1416, 0.5); delay_seconds(set_delay);

                    go_to_pose(0.1, -1.4, -1.5708, 0.5); delay_seconds(set_delay);

                    go_to_pose(0.1, -4.4, -1.5708, 0.5); delay_seconds(set_delay);

                    go_to_pose(0.1, -4.4, 0, 0.5); delay_seconds(set_delay);

                    go_to_pose(6.2, -4.4, 0, 0.5); delay_seconds(set_delay);

                    go_to_pose(6.2, -4.4, 1.5708, 0.5); delay_seconds(set_delay);

                    go_to_pose(6.2, -2.8, 1.5708, 0.5); delay_seconds(set_delay);

                    go_to_pose(6.2, -2.8, 3.1416, 0.5); delay_seconds(set_delay);

                    go_to_pose(4.1, -2.8, 3.1416, 0.5); delay_seconds(set_delay);

                    go_to_pose(4.1, -2.8, -2.3562, 0.5); delay_seconds(set_delay);

                    go_to_pose(2.6, -4.4, -2.3562, 0.5); delay_seconds(set_delay);

                    go_to_pose(2.6, -4.4, -3.1416, 0.5); delay_seconds(set_delay);

                    go_to_pose(-3.9, -4.4, -3.1416, 0.5); delay_seconds(set_delay);

                    go_to_pose(-3.9, -4.4, -1.5708, 0.5); delay_seconds(set_delay);

                    go_to_pose(-3.9, -4.9, -1.5708, 0.5); delay_seconds(set_delay);

                    go_to_pose(-3.9, -4.9, 0, 0.5);
                    go_to_pose(-3.9, -4.9, 1.5708, 0.5); delay_seconds(set_delay);

                    go_to_pose(-3.9, -4.4, 1.5708, 0.5); delay_seconds(set_delay);

                    go_to_pose(-3.9, -4.4, 0, 0.5); delay_seconds(set_delay);

                    go_to_pose(0.15, -4.4, 0, 0.5); delay_seconds(set_delay);

                    go_to_pose(0.15, -4.4, 1.5708, 0.5); delay_seconds(set_delay);

                    go_to_pose(0.15, 0.075, 1.5708, 0.5); delay_seconds(set_delay);

                    go_to_pose(0.15, 0.075, 0, 0.5); delay_seconds(set_delay);
                }
           
                i++;
                RCLCPP_INFO(get_logger(), "\n\n\nLoop: %d bool:%d\n\n\n", i,mission_complete);
            }

            // ************************************************************************************************

            stop_robot();

        }).detach();
    }

private:
    struct Pose2D {
        double x;
        double y;
        double yaw;
    };

    rcl_interfaces::msg::SetParametersResult on_param_change(const std::vector<rclcpp::Parameter> & params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for(const auto & p : params) {
            if(p.get_name() == "linear_speed") {
                double v = p.as_double();
                if(v <= 0.0) {
                    result.successful = false;
                    result.reason = "linear_speed must be > 0";
                    continue;
                }
                linear_speed_ = v;
                RCLCPP_INFO(get_logger(), "linear_speed updated to %.2f m/s", linear_speed_);
            }
            else if(p.get_name() == "angular_speed") {
                double v = p.as_double();
                if(v <= 0.0) {
                    result.successful = false;
                    result.reason = "angular_speed must be > 0";
                    continue;
                }
                angular_speed_ = v;
                RCLCPP_INFO(get_logger(), "angular_speed updated to %.2f rad/s", angular_speed_);
            }
        }
        return result;
    }

    void odom_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(pose_mutex_);
        linear_vel_ = msg->linear.x;
        angular_vel_ = msg->angular.z;
    }

    void fastlio2_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        double raw_yaw = yaw_from_quaternion(msg->orientation);
        std::lock_guard<std::mutex> lk(pose_mutex_);
        // First pose defines the START frame: the robot begins at (0, 0, 0).
        // Goals given to go_to_pose() are expressed in this frame.
        if(!origin_set_) {
            origin_x_ = msg->position.x;
            origin_y_ = msg->position.y;
            origin_yaw_ = raw_yaw;
            origin_set_ = true;
            x_ = 0.0;
            y_ = 0.0;
            yaw_ = 0.0;
            fastlio2_initialized_ = true;
            RCLCPP_INFO(get_logger(),
                "Start frame captured: SLAM(%.2f, %.2f, %.1f deg) -> local(0, 0, 0 deg). Goals are relative to this start pose.",
                origin_x_, origin_y_, origin_yaw_ * 180.0 / M_PI);
        }
        // Transform SLAM pose into the start frame (rotate + translate).
        double dx = msg->position.x - origin_x_;
        double dy = msg->position.y - origin_y_;
        fastlio2_x_ =  cos(origin_yaw_) * dx + sin(origin_yaw_) * dy;
        fastlio2_y_ = -sin(origin_yaw_) * dx + cos(origin_yaw_) * dy;
        fastlio2_yaw_ = normalize_angle(raw_yaw - origin_yaw_);
        last_fastlio2_time_ = now();
        fastlio2_fresh_ = true;
    }

    void sensor_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        if(msg->data.size() >= 2) {
            bumper_ = msg->data[0];
            battery_ = msg->data[1];
        }
    }

    void update_odometry() {
        std::lock_guard<std::mutex> lk(pose_mutex_);
        auto current_time = now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;
        double x_pred = x_ + linear_vel_ * cos(yaw_) * dt;
        double y_pred = y_ + linear_vel_ * sin(yaw_) * dt;
        double yaw_pred = yaw_ + angular_vel_ * dt;
        bool slam_valid = fastlio2_initialized_ && (current_time - last_fastlio2_time_).seconds() < fastlio2_timeout_;
        if(slam_valid && fastlio2_fresh_) {
            x_ = fastlio2_weight_ * fastlio2_x_ + odom_weight_ * x_pred;
            y_ = fastlio2_weight_ * fastlio2_y_ + odom_weight_ * y_pred;
            double sin_blend = fastlio2_weight_ * sin(fastlio2_yaw_) + odom_weight_ * sin(yaw_pred);
            double cos_blend = fastlio2_weight_ * cos(fastlio2_yaw_) + odom_weight_ * cos(yaw_pred);
            yaw_ = atan2(sin_blend, cos_blend);
            fastlio2_fresh_ = false;
        }
        else {
            x_ = x_pred;
            y_ = y_pred;
            yaw_ = yaw_pred;
            if(fastlio2_initialized_ && !slam_valid) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Pose stale (>%.1fs) - running on odometry only",
                    fastlio2_timeout_
                );
            }
        }
    }

    Pose2D get_pose() {
        std::lock_guard<std::mutex> lk(pose_mutex_);
        return {x_, y_, yaw_};
    }

    bool slam_ready() {
        std::lock_guard<std::mutex> lk(pose_mutex_);
        return fastlio2_initialized_ && (now() - last_fastlio2_time_).seconds() < fastlio2_timeout_;
    }

    static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q) {
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return atan2(siny_cosp, cosy_cosp);
    }

    static double normalize_angle(double angle) {
        while(angle > M_PI) angle -= 2.0 * M_PI;
        while(angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    static double clamp_omega(double w, double w_limit) {
        if(w >  w_limit) return  w_limit;
        if(w < -w_limit) return -w_limit;
        return w;
    }

    void send_velocity(double linear, double angular) {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = linear;
        cmd.angular.z = angular;
        cmd_pub_->publish(cmd);
    }

    void stop_robot() {
        send_velocity(0.0, 0.0);
        RCLCPP_INFO(get_logger(), "STOP bat0:%.2f bat1:%.2f diff:%.2f",
        (float)batt_start / 1000.0f,
        (float)batt_min / 1000.0f,
        (float)(batt_start-batt_min) / 1000.0f);
    }

    void delay_seconds(double seconds) {
        RCLCPP_INFO(get_logger(), "DELAY %.2f s", seconds);
        auto start = now();
        while(rclcpp::ok() && (now() - start).seconds() < seconds) {
            if(check_safety()) return;
            send_velocity(0.0, 0.0); // hold still
            rclcpp::sleep_for(50ms);
        }
    }

    // Closed-loop absolute pose controller.
    // (x_t, y_t, yaw_t) is the goal in the START frame (robot began at 0,0,0).
    // Phases: ALIGN_TO_TARGET (spin to face the heading reference) -> DRIVE
    // (steer onto the goal point; position error re-computed every cycle so
    // lateral disturbances are corrected. The heading reference is the bearing
    // to the goal when far away, and continuously morphs into the goal heading
    // yaw_t over the last slow_dist meters - "terminal heading law" below - so
    // the robot arrives already facing yaw_t when the difference is small)
    // -> TURN_TO_YAW (only rotates if a larger heading error remains on
    // arrival, e.g. goal heading far from the approach direction).
    void go_to_pose(double x_t, double y_t, double yaw_t, double speed = 0.2) {
        // Absolute goals need SLAM: wait for a fresh /fastlio2_pose first.
        double waited = 0.0;
        while(rclcpp::ok() && !slam_ready()) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for /fastlio2_pose ...");
            rclcpp::sleep_for(100ms);
            waited += 0.1;
            if(waited >= 10.0) {
                RCLCPP_ERROR(get_logger(), "GO TO aborted: no /fastlio2_pose after 10 s");
                stop_robot();
                return;
            }
        }
        if(!rclcpp::ok()) return;

        speed = fabs(speed);
        if(speed < 0.02) speed = 0.02;
        else if(speed > 0.5) speed = 0.5;
        yaw_t = normalize_angle(yaw_t);

        const double tol_xy    = 0.05;  // m,   position tolerance
        const double align_tol = 0.15;  // rad, heading-to-target tolerance (~8.6 deg)
        const double yaw_tol   = 0.10;  // rad, final yaw tolerance (~5.7 deg)
        const double kp_w      = 2.0;   // heading P gain
        const double w_max     = 1.5;   // rad/s cap while driving
        const double w_spin    = 0.6;   // rad/s cap while spinning in place
        const double w_min     = 0.25;  // rad/s floor so spins actually move
        const double slow_dist = 0.5;   // m, start decelerating below this
        // Terminal heading law: ref = yaw_t + atan2(k * e_lat, max(e_lon, lon_min))
        // in goal-aligned coordinates, with k ramping 1 -> lat_gain as the distance
        // goes ramp_far -> ramp_near. Far away (k=1, e_lon>lon_min) this IS plain
        // bearing tracking; near the goal the denominator saturates so the reference
        // stays bounded instead of swinging +-90 deg, and on the line through the
        // goal it equals yaw_t exactly - small heading differences are absorbed
        // while driving, so no separate spin is needed at the end.
        const double lat_gain  = 1;//0.5;//2.5;   // cross-track gain reached near the goal
        const double lon_min   = 0.15;//0.15;  // m, saturation distance of the law
        const double ramp_far  = 1.0;   // m, start raising the gain here
        const double ramp_near = 0.5; //0.25;  // m, full gain here

        RCLCPP_INFO(get_logger(), "GO TO x=%.2f y=%.2f yaw=%.1f deg (speed=%.2f m/s)",
                    x_t, y_t, yaw_t * 180.0 / M_PI, speed);

        enum Phase { ALIGN_TO_TARGET, DRIVE, TURN_TO_YAW, DONE };
        Phase phase = ALIGN_TO_TARGET;
        int cnt = 0;

        while(rclcpp::ok()) {
            // same safety method as the other primitives
            if (cnt < 5) {
                cnt ++;
                if(check_safety()) return;
            }
            else { if(check_safety(true)) return; }

            Pose2D p = get_pose();
            double ex = x_t - p.x;
            double ey = y_t - p.y;
            double dist = sqrt(ex * ex + ey * ey);

            // Unified heading reference (see "Terminal heading law" above).
            // e_lon/e_lat: goal offset along/across the final heading yaw_t.
            // - e_lon > 0 (goal ahead along yaw_t):
            //     ref = yaw_t + atan2(k * e_lat, max(e_lon, lon_min)), k: 1 -> lat_gain.
            //     Far away (k=1, e_lon>lon_min) this equals the plain bearing to the
            //     goal, so the long-range behavior is unchanged. Near the goal the
            //     denominator saturates: the reference stays bounded instead of
            //     swinging +-90 deg, and on the line through the goal it equals yaw_t
            //     exactly, so the heading is absorbed while still driving.
            // - e_lon <= 0 (overshot the goal plane): plain bearing, so the robot
            //     turns back toward the point (rare, e.g. a big late push).
            double ux = cos(yaw_t);
            double uy = sin(yaw_t);
            double e_lon = ex * ux + ey * uy;
            double e_lat = ey * ux - ex * uy;
            double heading_ref;
            if(e_lon > 0.0) {
                double s = (ramp_far - dist) / (ramp_far - ramp_near);
                if(s < 0.0) s = 0.0;
                else if(s > 1.0) s = 1.0;
                double k = 1.0 + (lat_gain - 1.0) * s * s * (3.0 - 2.0 * s);
                double lon = (e_lon > lon_min) ? e_lon : lon_min;
                heading_ref = yaw_t + atan2(k * e_lat, lon);
            }
            else {
                heading_ref = atan2(ey, ex);
            }

            if(phase == ALIGN_TO_TARGET) {
                if(dist < tol_xy) { phase = TURN_TO_YAW; continue; } // already on the point
                double err = normalize_angle(heading_ref - p.yaw);
                if(fabs(err) < align_tol) { phase = DRIVE; continue; }
                double w = kp_w * err;
                if(fabs(w) < w_min) w = (err >= 0.0 ? w_min : -w_min);
                send_velocity(0.0, clamp_omega(w, w_spin));
                rclcpp::sleep_for(20ms);
            }
            else if(phase == DRIVE) {
                if(dist < tol_xy) { phase = TURN_TO_YAW; continue; }
                double err = normalize_angle(heading_ref - p.yaw);
                if(fabs(err) > 4.0 * align_tol) { phase = ALIGN_TO_TARGET; continue; } // big heading error: re-aim first
                double v_cmd = speed;
                if(dist < slow_dist) v_cmd = speed * dist / slow_dist; // decelerate near goal
                v_cmd *= cos(err);                                     // slow down while turning
                if(v_cmd < 0.05) v_cmd = 0.05;
                send_velocity(v_cmd, clamp_omega(kp_w * err, w_max));
                rclcpp::sleep_for(50ms);
            }
            else if(phase == TURN_TO_YAW) {
                double err = normalize_angle(yaw_t - p.yaw);
                if(fabs(err) < yaw_tol) { phase = DONE; break; } // goal reached
                double w = kp_w * err;
                if(fabs(w) < w_min) w = (err >= 0.0 ? w_min : -w_min);
                send_velocity(0.0, clamp_omega(w, w_spin));
                rclcpp::sleep_for(20ms);
            }
            else {
                break; // DONE (defensive: never spin idly)
            }
        }
        stop_robot();

        Pose2D f = get_pose();
        RCLCPP_INFO(get_logger(), "GO TO done: final=(%.2f, %.2f, %.1f deg), err=(%.3f m, %.3f m, %.1f deg)",
                    f.x, f.y, f.yaw * 180.0 / M_PI,
                    x_t - f.x, y_t - f.y, normalize_angle(yaw_t - f.yaw) * 180.0 / M_PI);
    }

    void move_forward(double meters, double speed=0.1) {
        speed = fabs(speed);
        if (speed<0.01) speed=0.01; else if (speed>0.5) speed=0.5;
        Pose2D start = get_pose();
        RCLCPP_INFO(
            get_logger(),
            "MOVE FORWARD %.2f m (speed=%.2f m/s)",
            meters,
            speed
        );
        const double yaw_kp = 1;
        const double max_omega = 3.14;
        const double start_yaw = start.yaw;
        int cnt = 0;
        while(rclcpp::ok()) {
            if (cnt < 5) {
                cnt ++;
                if(check_safety()) return;
            }
            else { if(check_safety(true)) return; }
            Pose2D p = get_pose();
            double dx = p.x - start.x;
            double dy = p.y - start.y;
            double dist = sqrt(dx*dx + dy*dy);
            if(dist >= meters-0.050) break;
            const double yaw_err = normalize_angle(start_yaw - p.yaw);
            double omega = yaw_kp * yaw_err;
            if (omega >  max_omega) omega =  max_omega;
            if (omega < -max_omega) omega = -max_omega;
            send_velocity(speed, omega);
            rclcpp::sleep_for(50ms);
        }
        stop_robot();
    }

    void move_backward(double meters, double speed=0.1) {
        speed = fabs(speed);
        if (speed<0.01) speed=0.01; else if (speed>0.5) speed=0.5;
        Pose2D start = get_pose();
        RCLCPP_INFO(
            get_logger(),
            "MOVE BACKWARD %.2f m (speed=%.2f m/s)",
            meters,
            speed
        );
        const double yaw_kp = 1;
        const double max_omega = 3.14;
        const double start_yaw = start.yaw;
        int cnt = 0;
        while(rclcpp::ok()) {
            if (cnt < 5) {
                cnt ++;
                if(check_safety()) return;
            }
            else { if(check_safety(true)) return; }
            Pose2D p = get_pose();
            double dx = p.x - start.x;
            double dy = p.y - start.y;
            double dist = sqrt(dx*dx + dy*dy);
            if(dist >= meters-0.050) break;
            const double yaw_err = normalize_angle(start_yaw - p.yaw);
            double omega = yaw_kp * yaw_err;
            if (omega >  max_omega) omega =  max_omega;
            if (omega < -max_omega) omega = -max_omega;
            send_velocity(-speed, omega);
            rclcpp::sleep_for(50ms);
        }
        stop_robot();
    }

    void rotate_ccw(double deg, double speed=0.5) {
        deg = fabs(deg);
        speed = fabs(speed);
        if (speed<0.01) speed=0.01; else if (speed>1.58) speed=1.58;
        double target = normalize_angle(get_pose().yaw + deg * M_PI / 180.0);
        RCLCPP_INFO(get_logger(),"ROTATE CCW %.1f deg",deg);
        int cnt = 0;
        while(rclcpp::ok()) {
            if (cnt < 5) {
                cnt ++;
                if(check_safety()) return;
            }
            else { if(check_safety(true)) return; }
            double err = normalize_angle(target - get_pose().yaw);
            if(std::fabs(err) < 0.15) break;
            send_velocity(0.0,speed);
            rclcpp::sleep_for(20ms);
        }
        stop_robot();
    }

    void rotate_cw(double deg, double speed=0.5) {
        deg = fabs(deg);
        speed = fabs(speed);
        if (speed<0.01) speed=0.01; else if (speed>1.58) speed=1.58;
        double target = normalize_angle(get_pose().yaw - deg * M_PI / 180.0);
        RCLCPP_INFO(get_logger(),"ROTATE CW %.1f deg",deg);
        int cnt = 0;
        while(rclcpp::ok()) {
            if (cnt < 5) {
                cnt ++;
                if(check_safety()) return;
            }
            else { if(check_safety(true)) return; }
            double err = normalize_angle(target - get_pose().yaw);
            if(std::fabs(err) < 0.15) break;
            send_velocity(0.0,-speed);
            rclcpp::sleep_for(20ms);
        }
        stop_robot();
    }

    bool check_safety(bool onload = false) {
        int bat = battery_.load();
        if (onload) {
            if (batt_start == -1) { batt_start = bat; }
            if (bat >= 12000 && bat < batt_min) { batt_min = bat; }
        }
        if(bumper_.load() != 0) {
            RCLCPP_WARN(get_logger(), "BUMPER HIT");
            stop_robot();
            mission_complete = true;
            return true;
        }
        if(bat < 22000) {
            RCLCPP_WARN(get_logger(), "LOW BATTERY");
            stop_robot();
            mission_complete = true;
            return true;
        }
        return false;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr odo_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sensor_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr fastlio2_pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    double linear_speed_ = 0.1;
    double angular_speed_ = 0.5;
    double linear_vel_ = 0.0;
    double angular_vel_ = 0.0;

    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;

    double fastlio2_x_ = 0.0;
    double fastlio2_y_ = 0.0;
    double fastlio2_yaw_ = 0.0;

    // Start frame: first /fastlio2_pose is treated as (0, 0, 0).
    // Goals for go_to_pose() are given in this frame.
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    double origin_yaw_ = 0.0;
    bool origin_set_ = false;
    bool fastlio2_initialized_ = false;
    bool fastlio2_fresh_ = false;
    rclcpp::Time last_fastlio2_time_;

    // Complementary filter weights
    const double fastlio2_weight_ = 1.0; //0.75 0.5 0.0
    const double odom_weight_ = 1.0 - fastlio2_weight_;
    const double fastlio2_timeout_ = 1.0; // seconds

    std::atomic<int> bumper_{0};
    std::atomic<int> battery_{0};
    int batt_start = -1;
    int batt_min = 999999;
    rclcpp::Time last_time_;
    std::mutex pose_mutex_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoverController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
