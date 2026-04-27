#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <array>
#include <fstream>
#include <string>
#include <vector>

// ── Per-lap aggregate statistics ──────────────────────────────────────────────
struct LapStats {
    int    lap_number{0};
    double lap_time_s{0.0};

    // race line deviation (metric 3)
    double mean_raceline_dev_m{-1.0};
    double p95_raceline_dev_m{-1.0};

    // wall clearance (metric 4)
    double min_wall_clearance_m{-1.0};
    double p5_wall_clearance_m{-1.0};
    double mean_wall_clearance_m{-1.0};

    // velocity utilization (metric 6)
    double velocity_utilization{0.0};   // fraction of steps above threshold

    // smoothness (metric 2b)
    double mean_jerk_m_s3{0.0};         // |da/dt|
    double mean_yaw_accel_rad_s2{0.0};  // |d(yaw_rate)/dt|

    // lateral acceleration (metric 5)
    double mean_lat_accel_m_s2{0.0};
    double peak_lat_accel_m_s2{0.0};

    // control effort (metric 8)
    double ctrl_effort_steer{0.0};      // mean |steering_cmd| per step
    double ctrl_effort_speed{0.0};      // mean |speed_cmd| per step

    // steering reversals
    int steer_reversals{0};

    // accelerating / braking / coasting fractions
    double accel_frac{0.0};
    double brake_frac{0.0};
    double coast_frac{0.0};

    // sector splits (metric 1b)
    std::vector<double> sector_times_s;
    std::vector<double> sector_mean_dev_m;  // mean deviation per sector
};

// ─────────────────────────────────────────────────────────────────────────────

class MetricsNode : public rclcpp::Node
{
public:
    explicit MetricsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
    ~MetricsNode();

private:
    // ── parameters ────────────────────────────────────────────────────────────
    bool        use_hardware_;
    std::string output_dir_;
    double      max_speed_;
    double      vel_util_threshold_frac_;
    std::string race_line_path_;
    double      lap_cooldown_s_;
    int         num_sectors_;    double      stall_speed_threshold_;
    double      stall_duration_s_;
    double      scan_freshness_s_;
    // ── state machine ─────────────────────────────────────────────────────────
    enum class State { WAITING, RACING, STOPPED };
    State state_{State::WAITING};

    // ── start / finish line ───────────────────────────────────────────────────
    bool   start_set_{false};
    double start_x_{0.0}, start_y_{0.0};
    // normal to the finish line = car's heading direction at /initialpose
    double finish_nx_{1.0}, finish_ny_{0.0};
    double prev_signed_dist_{0.0};
    rclcpp::Time last_lap_stamp_;

    // ── vehicle state (updated every pose message) ────────────────────────────
    double cur_x_{0.0}, cur_y_{0.0}, cur_yaw_{0.0};
    double cur_speed_{0.0}, cur_yaw_rate_{0.0};

    // derivatives for jerk / yaw-accel
    double prev_speed_{0.0}, prev_yaw_rate_{0.0}, prev_accel_{0.0};
    rclcpp::Time prev_pose_stamp_;
    bool first_pose_{true};

    // hardware-mode: PF gives pose; odom gives velocity
    double hw_speed_{0.0}, hw_yaw_rate_{0.0};

    // ── commanded inputs (from /drive) ────────────────────────────────────────
    double cmd_steering_{0.0}, cmd_speed_{0.0};

    // ── lap / sector tracking ─────────────────────────────────────────────────
    int              lap_count_{0};
    rclcpp::Time     lap_start_;
    int              cur_sector_{0};
    std::vector<rclcpp::Time> sector_starts_;

    // ── per-lap accumulators ──────────────────────────────────────────────────
    std::vector<double> lap_deviations_;
    std::vector<double> lap_clearances_;
    std::vector<double> lap_jerks_;
    std::vector<double> lap_yaw_accels_;
    std::vector<double> lap_lat_accels_;
    double peak_lat_accel_{0.0};
    double lap_ctrl_steer_{0.0};
    double lap_ctrl_speed_{0.0};
    int    vel_util_count_{0};
    int    step_count_{0};
    int    accel_steps_{0};
    int    brake_steps_{0};
    int    lap_steer_reversals_{0};

    // ── race line ─────────────────────────────────────────────────────────────
    std::vector<std::array<double, 2>> race_line_;
    // indices into race_line_ that mark sector boundaries (num_sectors - 1 entries)
    std::vector<size_t> sector_boundary_indices_;    // per-sector deviation buckets for computing sector-level statistics
    std::vector<std::vector<double>> sector_dev_buckets_;

    // ── scan / clearance tracking ─────────────────────────────────────────────
    double last_scan_min_{std::numeric_limits<double>::max()};
    rclcpp::Time last_scan_stamp_;
    bool last_scan_valid_{false};

    // ── stall detection ───────────────────────────────────────────────────────
    bool in_stall_{false};
    rclcpp::Time stall_start_;
    // ── completed lap records ─────────────────────────────────────────────────
    std::vector<LapStats> laps_;

    // ── log file handles ──────────────────────────────────────────────────────
    std::ofstream step_csv_;   // per-timestep log
    std::ofstream lap_csv_;    // per-lap summary log
    std::string run_output_dir_; // per-run subdirectory inside `output_dir_`

    // ── ROS subscriptions ─────────────────────────────────────────────────────
    // sim
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    // hardware
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pf_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr hw_odom_sub_;
    // shared
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;

    // ── ROS callbacks ─────────────────────────────────────────────────────────
    void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pfPoseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void hwOdomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
    void initPoseCb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void driveCb(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
    void stopCb(const std_msgs::msg::Bool::SharedPtr msg);

    // ── internal logic ────────────────────────────────────────────────────────
    void processNewPose(double x, double y, double yaw,
                        double speed, double yaw_rate,
                        const rclcpp::Time & stamp);

    void checkLapCrossing(const rclcpp::Time & stamp);
    void checkSectorCrossing(double x, double y, const rclcpp::Time & stamp);
    void finalizeLap(const rclcpp::Time & stamp);
    void resetLapAccumulators();

    // ── math helpers ──────────────────────────────────────────────────────────
    double racelineDeviation(double x, double y) const;
    double segmentPerpDistance(double px, double py,
                               const std::array<double, 2> & a,
                               const std::array<double, 2> & b) const;
    static double percentile(std::vector<double> v, double p);
    static double yawFromQuat(double qx, double qy, double qz, double qw);

    // ── I/O helpers ───────────────────────────────────────────────────────────
    void loadRaceLine();
    void openLogFiles();
    void openRunLogFiles(const rclcpp::Time & start_stamp);
    void writeStepRow(const rclcpp::Time & stamp, double dev, double min_scan,
                      double jerk, double yaw_accel, double lat_accel);
    void writeLapRow(const LapStats & s);
    void writeSummary();
};