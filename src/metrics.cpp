#include "metrics.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <ctime>
#include <stdexcept>

namespace fs = std::filesystem;

namespace
{

std::string humanReadableRunDirName()
{
    const auto now = std::chrono::system_clock::now();
    const auto now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm{};
    localtime_r(&now_time, &local_tm);

    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

    std::ostringstream oss;
    oss << std::put_time(&local_tm, "%Y-%m-%d_%H-%M-%S")
        << '.' << std::setw(3) << std::setfill('0') << milliseconds.count();
    return oss.str();
}

}  // namespace

// ═════════════════════════════════════════════════════════════════════════════
//  Constructor / Destructor
// ═════════════════════════════════════════════════════════════════════════════

MetricsNode::MetricsNode(const rclcpp::NodeOptions & options)
: Node("metrics_node", options)
{
    // ── declare & fetch parameters ────────────────────────────────────────────
    declare_parameter("use_hardware",             false);
    declare_parameter("output_dir",               "/tmp/f1tenth_metrics");
    declare_parameter("max_speed",                6.0);
    declare_parameter("vel_util_threshold_frac",  0.75);
    declare_parameter("race_line_path",           "");
    declare_parameter("lap_cooldown_s",           3.0);
    declare_parameter("num_sectors",              3);
    declare_parameter("stall_speed_threshold_m_s", 0.1);
    declare_parameter("stall_duration_s",          3.0);
    declare_parameter("scan_freshness_s",          0.05);

    use_hardware_             = get_parameter("use_hardware").as_bool();
    output_dir_               = get_parameter("output_dir").as_string();
    max_speed_                = get_parameter("max_speed").as_double();
    vel_util_threshold_frac_  = get_parameter("vel_util_threshold_frac").as_double();
    race_line_path_           = get_parameter("race_line_path").as_string();
    lap_cooldown_s_           = get_parameter("lap_cooldown_s").as_double();
    num_sectors_              = get_parameter("num_sectors").as_int();
    stall_speed_threshold_    = get_parameter("stall_speed_threshold_m_s").as_double();
    stall_duration_s_         = get_parameter("stall_duration_s").as_double();
    scan_freshness_s_         = get_parameter("scan_freshness_s").as_double();

    // ── optional race line ────────────────────────────────────────────────────
    if (!race_line_path_.empty()) {
        loadRaceLine();
    }

    // ── open CSV log files ────────────────────────────────────────────────────
    openLogFiles();

    // ── subscriptions ─────────────────────────────────────────────────────────
    using std::placeholders::_1;

    // always present
    init_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 1,
        std::bind(&MetricsNode::initPoseCb, this, _1));

    stop_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/metric_stop", 1,
        std::bind(&MetricsNode::stopCb, this, _1));

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&MetricsNode::scanCb, this, _1));

    drive_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "/drive", 10,
        std::bind(&MetricsNode::driveCb, this, _1));

    if (use_hardware_) {
        // PF gives localised pose; odom gives wheel-encoder velocity
        pf_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/pf/viz/inferred_pose", 50,
            std::bind(&MetricsNode::pfPoseCb, this, _1));

        hw_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 50,
            std::bind(&MetricsNode::hwOdomCb, this, _1));
    } else {
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 50,
            std::bind(&MetricsNode::odomCb, this, _1));
    }

    RCLCPP_INFO(get_logger(), "MetricsNode ready  [mode: %s]  [output: %s]",
                use_hardware_ ? "HARDWARE" : "SIM", output_dir_.c_str());
    RCLCPP_INFO(get_logger(), "Waiting for /initialpose to define start/finish line...");
}

MetricsNode::~MetricsNode()
{
    if (state_ == State::RACING) {
        RCLCPP_WARN(get_logger(),
            "Node destroyed mid-lap — partial lap not recorded. Writing summary of completed laps.");
    }
    writeSummary();
    if (step_csv_.is_open()) step_csv_.close();
    if (lap_csv_.is_open())  lap_csv_.close();
}

// ═════════════════════════════════════════════════════════════════════════════
//  ROS Callbacks
// ═════════════════════════════════════════════════════════════════════════════

void MetricsNode::odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    const auto & pos  = msg->pose.pose.position;
    const auto & quat = msg->pose.pose.orientation;
    const auto & twist = msg->twist.twist;

    double yaw   = yawFromQuat(quat.x, quat.y, quat.z, quat.w);
    double speed = std::hypot(twist.linear.x, twist.linear.y);

    processNewPose(pos.x, pos.y, yaw, speed, twist.angular.z,
                   rclcpp::Time(msg->header.stamp));
}

void MetricsNode::pfPoseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    const auto & pos  = msg->pose.position;
    const auto & quat = msg->pose.orientation;

    double yaw = yawFromQuat(quat.x, quat.y, quat.z, quat.w);

    // velocity comes from hwOdomCb which runs independently
    processNewPose(pos.x, pos.y, yaw, hw_speed_, hw_yaw_rate_,
                   rclcpp::Time(msg->header.stamp));
}

void MetricsNode::hwOdomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Only used in hardware mode to extract velocity from VESC odometry
    const auto & twist = msg->twist.twist;
    hw_speed_    = std::hypot(twist.linear.x, twist.linear.y);
    hw_yaw_rate_ = twist.angular.z;
}

void MetricsNode::initPoseCb(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    if (state_ == State::STOPPED) return;

    const auto & pos  = msg->pose.pose.position;
    const auto & quat = msg->pose.pose.orientation;

    start_x_ = pos.x;
    start_y_ = pos.y;

    double yaw = yawFromQuat(quat.x, quat.y, quat.z, quat.w);

    // The finish line is perpendicular to the car's heading.
    // Its outward normal IS the car's heading vector, so the signed distance
    // of any point P from the line is: dot((P - start), heading).
    // A lap crossing occurs when this value goes from negative → positive.
    finish_nx_ = std::cos(yaw);
    finish_ny_ = std::sin(yaw);

    start_set_         = true;
    first_pose_        = true;
    prev_signed_dist_  = 0.0;
    state_             = State::WAITING;

    RCLCPP_INFO(get_logger(),
        "Start/finish line set at (%.3f, %.3f)  heading %.3f rad",
        start_x_, start_y_, yaw);
    RCLCPP_INFO(get_logger(), "Waiting for first pose to begin recording...");
}

void MetricsNode::scanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (state_ != State::RACING) return;

    float min_r = std::numeric_limits<float>::max();
    for (const float r : msg->ranges) {
        if (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max) {
            min_r = std::min(min_r, r);
        }
    }

    if (min_r < std::numeric_limits<float>::max()) {
        const double d = static_cast<double>(min_r);
        // Store timestamped reading for per-step freshness check
        last_scan_min_   = d;
        last_scan_stamp_ = rclcpp::Time(msg->header.stamp);
        last_scan_valid_ = true;
        // Also accumulate for lap-level clearance stats
        lap_clearances_.push_back(d);
    }
}

void MetricsNode::driveCb(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
    const double new_steer = msg->drive.steering_angle;
    const double new_speed = msg->drive.speed;

    if (state_ == State::RACING) {
        // Steering reversal: sign flip between two nonzero commands
        if (cmd_steering_ != 0.0 && new_steer != 0.0 &&
            std::signbit(new_steer) != std::signbit(cmd_steering_)) {
            ++lap_steer_reversals_;
        }
        lap_ctrl_steer_ += std::abs(new_steer);
        lap_ctrl_speed_ += std::abs(new_speed);
    }

    cmd_steering_ = new_steer;
    cmd_speed_    = new_speed;
}

void MetricsNode::stopCb(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data && state_ != State::STOPPED) {
        RCLCPP_INFO(get_logger(),
            "Stop signal received — %d lap(s) recorded. Writing summary.", lap_count_);
        state_ = State::STOPPED;
        writeSummary();
        if (step_csv_.is_open()) step_csv_.close();
        if (lap_csv_.is_open())  lap_csv_.close();
        laps_.clear();
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  Core Processing
// ═════════════════════════════════════════════════════════════════════════════

void MetricsNode::processNewPose(double x, double y, double yaw,
                                  double speed, double yaw_rate,
                                  const rclcpp::Time & stamp)
{
    if (!start_set_ || state_ == State::STOPPED) return;

    cur_x_        = x;
    cur_y_        = y;
    cur_yaw_      = yaw;
    cur_speed_    = speed;
    cur_yaw_rate_ = yaw_rate;

    // ── WAITING: first pose after /initialpose — transition to RACING ─────────
    if (state_ == State::WAITING) {
        // Don't start recording until the vehicle is actually moving.
        // Some sources may publish an initial pose while the vehicle is still
        // stationary; require a non-zero speed to begin a lap.
        const double start_speed_thresh = 1e-3;
        if (speed <= start_speed_thresh) {
            return;
        }

        state_      = State::RACING;
        lap_start_  = stamp;
        last_lap_stamp_ = stamp;

        // create a unique per-run output directory and open CSV files there
        openRunLogFiles(lap_start_);

        cur_sector_ = 0;
        sector_starts_.clear();
        sector_starts_.push_back(stamp);

        // seed signed distance so the very first departure doesn't fake-trigger
        double dx = cur_x_ - start_x_;
        double dy = cur_y_ - start_y_;
        prev_signed_dist_ = dx * finish_nx_ + dy * finish_ny_;

        prev_pose_stamp_  = stamp;
        prev_speed_       = speed;
        prev_yaw_rate_    = yaw_rate;
        prev_accel_       = 0.0;
        first_pose_       = false;

        resetLapAccumulators();
        RCLCPP_INFO(get_logger(), "Racing started — collecting metrics.");
        return;
    }

    // ── RACING: compute incremental metrics ───────────────────────────────────
    const double dt = (stamp - prev_pose_stamp_).seconds();
    if (dt <= 0.0) return;   // duplicate or out-of-order message

    // ── stall detection — abort lap if near-zero speed persists too long ──────
    if (speed < stall_speed_threshold_) {
        if (!in_stall_) {
            in_stall_    = true;
            stall_start_ = stamp;
        } else if ((stamp - stall_start_).seconds() > stall_duration_s_) {
            RCLCPP_WARN(get_logger(),
                "Vehicle stalled for %.1f s — abandoning lap and waiting for a new /initialpose.",
                (stamp - stall_start_).seconds());
            in_stall_   = false;
            start_set_  = false;
            first_pose_ = true;
            state_      = State::WAITING;
            writeSummary();
            if (step_csv_.is_open()) step_csv_.close();
            if (lap_csv_.is_open())  lap_csv_.close();
            laps_.clear();
            resetLapAccumulators();
            return;
        }
    } else {
        in_stall_ = false;
    }

    // jerk (m/s³) and yaw acceleration (rad/s²)
    const double accel     = (speed - prev_speed_) / dt;
    const double jerk      = (accel - prev_accel_) / dt;
    const double yaw_accel = (yaw_rate - prev_yaw_rate_) / dt;

    lap_jerks_.push_back(std::abs(jerk));
    lap_yaw_accels_.push_back(std::abs(yaw_accel));

    // lateral acceleration (m/s²) — centripetal: v · ω
    const double lat_accel = std::abs(speed * yaw_rate);
    lap_lat_accels_.push_back(lat_accel);
    if (lat_accel > peak_lat_accel_) peak_lat_accel_ = lat_accel;

    // accelerating / braking / coasting (±0.1 m/s² dead zone to suppress noise)
    if      (accel >  0.1) ++accel_steps_;
    else if (accel < -0.1) ++brake_steps_;

    // velocity utilization
    if (speed >= vel_util_threshold_frac_ * max_speed_) ++vel_util_count_;
    ++step_count_;

    // race line deviation
    const double dev = racelineDeviation(x, y);
    if (dev >= 0.0) {
        lap_deviations_.push_back(dev);
        // Accumulate into current sector bucket (clamped to valid range)
        if (!sector_dev_buckets_.empty()) {
            const size_t bucket = static_cast<size_t>(
                std::min(cur_sector_, static_cast<int>(sector_dev_buckets_.size()) - 1));
            sector_dev_buckets_[bucket].push_back(dev);
        }
    }

    // sector crossing
    checkSectorCrossing(x, y, stamp);

    // step log: use the most recent scan only if it arrived within scan_freshness_s_
    double step_scan = -1.0;
    if (last_scan_valid_) {
        const double scan_age = (stamp - last_scan_stamp_).seconds();
        if (scan_age >= 0.0 && scan_age <= scan_freshness_s_) {
            step_scan = last_scan_min_;
        }
    }
    writeStepRow(stamp, dev, step_scan, jerk, yaw_accel, lat_accel);

    // lap crossing
    checkLapCrossing(stamp);

    // update previous state
    prev_pose_stamp_ = stamp;
    prev_speed_      = speed;
    prev_yaw_rate_   = yaw_rate;
    prev_accel_      = accel;
}

// ─────────────────────────────────────────────────────────────────────────────

void MetricsNode::checkLapCrossing(const rclcpp::Time & stamp)
{
    const double dx = cur_x_ - start_x_;
    const double dy = cur_y_ - start_y_;
    const double signed_dist = dx * finish_nx_ + dy * finish_ny_;

    // Crossing: signed distance goes from negative to positive (moving in
    // the same direction as the car's heading at the start/finish line).
    const bool crossed = (prev_signed_dist_ < 0.0 && signed_dist >= 0.0);
    const double since_last = (stamp - last_lap_stamp_).seconds();

    if (crossed && since_last > lap_cooldown_s_) {
        finalizeLap(stamp);
    }

    prev_signed_dist_ = signed_dist;
}

void MetricsNode::checkSectorCrossing(double x, double y, const rclcpp::Time & stamp)
{
    if (sector_boundary_indices_.empty()) return;
    if (cur_sector_ >= static_cast<int>(sector_boundary_indices_.size())) return;

    const size_t idx = sector_boundary_indices_[static_cast<size_t>(cur_sector_)];
    const double bx  = race_line_[idx][0];
    const double by  = race_line_[idx][1];
    const double dist = std::hypot(x - bx, y - by);

    if (dist < 1.0) {  // 1 m proximity threshold
        RCLCPP_INFO(get_logger(), "  Sector %d complete (%.3f s since lap start)",
            cur_sector_ + 1, (stamp - lap_start_).seconds());
        sector_starts_.push_back(stamp);
        ++cur_sector_;
    }
}

void MetricsNode::finalizeLap(const rclcpp::Time & stamp)
{
    ++lap_count_;

    LapStats s;
    s.lap_number  = lap_count_;
    s.lap_time_s  = (stamp - lap_start_).seconds();

    // sector splits
    sector_starts_.push_back(stamp);  // lap-end is final sector boundary
    for (size_t i = 1; i < sector_starts_.size(); ++i) {
        s.sector_times_s.push_back(
            (sector_starts_[i] - sector_starts_[i - 1]).seconds());
    }

    // race line deviation
    if (!lap_deviations_.empty()) {
        const double sum = std::accumulate(lap_deviations_.begin(),
                                           lap_deviations_.end(), 0.0);
        s.mean_raceline_dev_m = sum / static_cast<double>(lap_deviations_.size());
        s.p95_raceline_dev_m  = percentile(lap_deviations_, 95.0);
    }

    // wall clearance
    if (!lap_clearances_.empty()) {
        s.min_wall_clearance_m  = *std::min_element(lap_clearances_.begin(),
                                                      lap_clearances_.end());
        s.p5_wall_clearance_m   = percentile(lap_clearances_, 5.0);
        const double sum = std::accumulate(lap_clearances_.begin(),
                                           lap_clearances_.end(), 0.0);
        s.mean_wall_clearance_m = sum / static_cast<double>(lap_clearances_.size());
    }

    // velocity utilization
    s.velocity_utilization = (step_count_ > 0)
        ? static_cast<double>(vel_util_count_) / static_cast<double>(step_count_) : 0.0;

    // smoothness
    auto mean_vec = [](const std::vector<double> & v) -> double {
        if (v.empty()) return 0.0;
        return std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
    };
    s.mean_jerk_m_s3        = mean_vec(lap_jerks_);
    s.mean_yaw_accel_rad_s2 = mean_vec(lap_yaw_accels_);

    // lateral acceleration
    s.mean_lat_accel_m_s2 = mean_vec(lap_lat_accels_);
    s.peak_lat_accel_m_s2 = peak_lat_accel_;

    // steering reversals
    s.steer_reversals = lap_steer_reversals_;

    // accelerating / braking / coasting fractions
    if (step_count_ > 0) {
        const double n = static_cast<double>(step_count_);
        s.accel_frac = static_cast<double>(accel_steps_) / n;
        s.brake_frac = static_cast<double>(brake_steps_) / n;
        s.coast_frac = static_cast<double>(step_count_ - accel_steps_ - brake_steps_) / n;
    }

    // sector mean deviation
    s.sector_mean_dev_m.clear();
    for (const auto & bucket : sector_dev_buckets_) {
        if (!bucket.empty()) {
            const double sum = std::accumulate(bucket.begin(), bucket.end(), 0.0);
            s.sector_mean_dev_m.push_back(sum / static_cast<double>(bucket.size()));
        } else {
            s.sector_mean_dev_m.push_back(-1.0);
        }
    }

    // control effort (per-step average so lap length doesn't skew comparison)
    s.ctrl_effort_steer = (step_count_ > 0)
        ? lap_ctrl_steer_ / static_cast<double>(step_count_) : 0.0;
    s.ctrl_effort_speed = (step_count_ > 0)
        ? lap_ctrl_speed_ / static_cast<double>(step_count_) : 0.0;

    laps_.push_back(s);
    writeLapRow(s);

    RCLCPP_INFO(get_logger(),
        "── Lap %d complete ──  time: %.3f s | mean dev: %.3f m | min clearance: %.3f m",
        s.lap_number, s.lap_time_s, s.mean_raceline_dev_m, s.min_wall_clearance_m);

    // reset for next lap
    lap_start_      = stamp;
    last_lap_stamp_ = stamp;
    cur_sector_     = 0;
    sector_starts_.clear();
    sector_starts_.push_back(stamp);
    resetLapAccumulators();
}

void MetricsNode::resetLapAccumulators()
{
    lap_deviations_.clear();
    lap_clearances_.clear();
    lap_jerks_.clear();
    lap_yaw_accels_.clear();
    lap_lat_accels_.clear();
    lap_ctrl_steer_      = 0.0;
    lap_ctrl_speed_      = 0.0;
    vel_util_count_      = 0;
    step_count_          = 0;
    accel_steps_         = 0;
    brake_steps_         = 0;
    peak_lat_accel_      = 0.0;
    lap_steer_reversals_ = 0;
    last_scan_valid_     = false;
    sector_dev_buckets_.assign(static_cast<size_t>(std::max(num_sectors_, 1)), {});
}

// ═════════════════════════════════════════════════════════════════════════════
//  Math Helpers
// ═════════════════════════════════════════════════════════════════════════════

double MetricsNode::racelineDeviation(double x, double y) const
{
    if (race_line_.size() < 2) return -1.0;

    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < race_line_.size(); ++i) {
        const size_t j = (i + 1) % race_line_.size();  // wrap for closed loop
        const double d = segmentPerpDistance(x, y, race_line_[i], race_line_[j]);
        min_dist = std::min(min_dist, d);
    }
    return min_dist;
}

double MetricsNode::segmentPerpDistance(double px, double py,
                                         const std::array<double, 2> & a,
                                         const std::array<double, 2> & b) const
{
    const double ax  = b[0] - a[0];
    const double ay  = b[1] - a[1];
    const double len2 = ax * ax + ay * ay;

    if (len2 < 1e-9) {
        return std::hypot(px - a[0], py - a[1]);
    }

    // project P onto the segment, clamp to [0,1]
    const double t  = std::clamp(((px - a[0]) * ax + (py - a[1]) * ay) / len2, 0.0, 1.0);
    const double cx = a[0] + t * ax;
    const double cy = a[1] + t * ay;
    return std::hypot(px - cx, py - cy);
}

double MetricsNode::percentile(std::vector<double> v, double p)
{
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    const double idx  = (p / 100.0) * static_cast<double>(v.size() - 1);
    const size_t lo   = static_cast<size_t>(std::floor(idx));
    const size_t hi   = std::min(lo + 1, v.size() - 1);
    const double frac = idx - static_cast<double>(lo);
    return v[lo] * (1.0 - frac) + v[hi] * frac;
}

double MetricsNode::yawFromQuat(double qx, double qy, double qz, double qw)
{
    return std::atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz));
}

// ═════════════════════════════════════════════════════════════════════════════
//  I/O
// ═════════════════════════════════════════════════════════════════════════════

void MetricsNode::loadRaceLine()
{
    std::ifstream f(race_line_path_);
    if (!f.is_open()) {
        RCLCPP_WARN(get_logger(),
            "Could not open race line file: %s — deviation metric disabled.",
            race_line_path_.c_str());
        return;
    }

    std::string line;
    int line_num = 0;
    while (std::getline(f, line)) {
        ++line_num;
        if (line.empty() || line[0] == '#') continue;  // skip comments/blank
        std::istringstream ss(line);
        std::string xs, ys;
        if (std::getline(ss, xs, ',') && std::getline(ss, ys, ',')) {
            try {
                race_line_.push_back({std::stod(xs), std::stod(ys)});
            } catch (const std::exception & e) {
                RCLCPP_WARN(get_logger(),
                    "Skipping malformed race line entry at line %d: %s", line_num, e.what());
            }
        }
    }

    if (race_line_.size() < 2) {
        RCLCPP_WARN(get_logger(), "Race line has <2 valid points — deviation disabled.");
        race_line_.clear();
        return;
    }

    RCLCPP_INFO(get_logger(), "Loaded race line: %zu waypoints.", race_line_.size());

    // compute sector boundary indices — evenly spaced along the race line
    if (num_sectors_ > 1) {
        sector_boundary_indices_.clear();
        const size_t n = race_line_.size();
        for (int s = 1; s < num_sectors_; ++s) {
            sector_boundary_indices_.push_back((static_cast<size_t>(s) * n)
                                                / static_cast<size_t>(num_sectors_));
        }
        RCLCPP_INFO(get_logger(), "Sector boundaries at race line indices:");
        for (size_t idx : sector_boundary_indices_) {
            RCLCPP_INFO(get_logger(), "  %zu  (%.2f, %.2f)",
                idx, race_line_[idx][0], race_line_[idx][1]);
        }
    }
}

void MetricsNode::openLogFiles()
{
    // Ensure base output directory exists. Per-run CSV files are opened
    // when a race actually starts to create a unique subdirectory per run.
    fs::create_directories(output_dir_);
    RCLCPP_INFO(get_logger(), "Base output dir ready: %s", output_dir_.c_str());
}

void MetricsNode::openRunLogFiles(const rclcpp::Time & start_stamp)
{
    (void)start_stamp;

    // close existing files if any
    if (step_csv_.is_open()) step_csv_.close();
    if (lap_csv_.is_open())  lap_csv_.close();

    run_output_dir_ = output_dir_ + "/" + humanReadableRunDirName();
    fs::create_directories(run_output_dir_);

    step_csv_.open(run_output_dir_ + "/step_log.csv");
    lap_csv_.open(run_output_dir_ + "/lap_log.csv");

    if (!step_csv_.is_open() || !lap_csv_.is_open()) {
        RCLCPP_ERROR(get_logger(),
            "Failed to open log files in '%s' — check path and permissions.",
            run_output_dir_.c_str());
        return;
    }

    // ── step log header ───────────────────────────────────────────────────────
    step_csv_
        << "timestamp_s,"
        << "lap,"
        << "x_m,y_m,yaw_rad,"
        << "speed_m_s,yaw_rate_rad_s,"
        << "jerk_m_s3,yaw_accel_rad_s2,"
        << "lat_accel_m_s2,"
        << "raceline_dev_m,"
        << "min_scan_m,"
        << "cmd_steering_rad,cmd_speed_m_s\n";

    // ── lap log header ────────────────────────────────────────────────────────
    lap_csv_
        << "lap,"
        << "lap_time_s,"
        << "mean_raceline_dev_m,p95_raceline_dev_m,"
        << "min_wall_clearance_m,p5_wall_clearance_m,mean_wall_clearance_m,"
        << "vel_utilization,"
        << "mean_jerk_m_s3,mean_yaw_accel_rad_s2,"
        << "mean_lat_accel_m_s2,peak_lat_accel_m_s2,"
        << "steer_reversals,"
        << "accel_frac,brake_frac,coast_frac,"
        << "ctrl_effort_steer,ctrl_effort_speed\n";

    RCLCPP_INFO(get_logger(), "Logging to: %s", run_output_dir_.c_str());
}

void MetricsNode::writeStepRow(const rclcpp::Time & stamp, double dev, double min_scan,
                                double jerk, double yaw_accel, double lat_accel)
{
    if (!step_csv_.is_open()) return;
    step_csv_ << std::fixed << std::setprecision(6)
              << stamp.seconds()   << ","
              << lap_count_        << ","
              << cur_x_            << "," << cur_y_ << "," << cur_yaw_ << ","
              << cur_speed_        << "," << cur_yaw_rate_ << ","
              << jerk              << "," << yaw_accel << ","
              << lat_accel         << ","
              << dev               << ","
              << min_scan          << ","
              << cmd_steering_     << "," << cmd_speed_ << "\n";
}

void MetricsNode::writeLapRow(const LapStats & s)
{
    if (!lap_csv_.is_open()) return;
    lap_csv_ << std::fixed << std::setprecision(6)
             << s.lap_number              << ","
             << s.lap_time_s              << ","
             << s.mean_raceline_dev_m     << ","
             << s.p95_raceline_dev_m      << ","
             << s.min_wall_clearance_m    << ","
             << s.p5_wall_clearance_m     << ","
             << s.mean_wall_clearance_m   << ","
             << s.velocity_utilization    << ","
             << s.mean_jerk_m_s3          << ","
             << s.mean_yaw_accel_rad_s2   << ","
             << s.mean_lat_accel_m_s2     << ","
             << s.peak_lat_accel_m_s2     << ","
             << s.steer_reversals         << ","
             << s.accel_frac              << ","
             << s.brake_frac              << ","
             << s.coast_frac              << ","
             << s.ctrl_effort_steer       << ","
             << s.ctrl_effort_speed       << "\n";
    lap_csv_.flush();  // flush after each lap so data is safe on hard stop
}

void MetricsNode::writeSummary()
{
    const std::string summary_dir = run_output_dir_.empty() ? output_dir_ : run_output_dir_;
    std::ofstream f(summary_dir + "/summary.txt");
    if (!f.is_open()) {
        RCLCPP_ERROR(get_logger(), "Could not write summary to %s", summary_dir.c_str());
        return;
    }

    if (laps_.empty()) {
        RCLCPP_WARN(get_logger(), "No completed laps to summarise.");
        f << "══════════════════════════════════════════════════════\n"
          << "  F1TENTH Metrics Summary\n"
          << "══════════════════════════════════════════════════════\n\n"
          << "Total laps completed : 0\n\n"
          << "No completed laps were recorded for this run.\n";
        RCLCPP_INFO(get_logger(), "Summary written to %s/summary.txt", summary_dir.c_str());
        return;
    }

    // ── lap time statistics ───────────────────────────────────────────────────
    std::vector<double> times;
    for (const auto & l : laps_) times.push_back(l.lap_time_s);

    const double best  = *std::min_element(times.begin(), times.end());
    const double avg   = std::accumulate(times.begin(), times.end(), 0.0)
                         / static_cast<double>(times.size());
    double var = 0.0;
    for (const double t : times) var += (t - avg) * (t - avg);
    const double stddev = std::sqrt(var / static_cast<double>(times.size()));

    // ── write ─────────────────────────────────────────────────────────────────
    f << "══════════════════════════════════════════════════════\n"
      << "  F1TENTH Metrics Summary\n"
      << "══════════════════════════════════════════════════════\n\n"
      << "Total laps completed : " << laps_.size() << "\n\n"
      << "Lap Times (s)\n"
      << "  Best    : " << std::fixed << std::setprecision(3) << best   << "\n"
      << "  Average : " << avg    << "\n"
      << "  Std Dev : " << stddev << "  ← consistency (lower = more repeatable)\n\n";

    // ── per-lap table ─────────────────────────────────────────────────────────
    const int W = 14;
    f << std::left
      << std::setw(5)  << "Lap"
      << std::setw(W)  << "Time(s)"
      << std::setw(W)  << "MeanDev(m)"
      << std::setw(W)  << "P95Dev(m)"
      << std::setw(W)  << "MinClr(m)"
      << std::setw(W)  << "VelUtil"
      << std::setw(W)  << "MeanJerk"
      << std::setw(W)  << "YawAccel"
      << std::setw(W)  << "LatAccel"
      << std::setw(10) << "SteerRev"
      << std::setw(11) << "Accel%"
      << std::setw(11) << "Brake%"
      << "SectorSplits(s)\n";
    f << std::string(5 + W * 8 + 10 + 11 + 11 + 20, '-') << "\n";

    for (const auto & l : laps_) {
        f << std::left  << std::setw(5)  << l.lap_number
          << std::fixed << std::setprecision(3)
          << std::setw(W) << l.lap_time_s
          << std::setw(W) << l.mean_raceline_dev_m
          << std::setw(W) << l.p95_raceline_dev_m
          << std::setw(W) << l.min_wall_clearance_m
          << std::setprecision(2)
          << std::setw(W) << l.velocity_utilization
          << std::setprecision(3)
          << std::setw(W) << l.mean_jerk_m_s3
          << std::setw(W) << l.mean_yaw_accel_rad_s2
          << std::setw(W) << l.mean_lat_accel_m_s2
          << std::setw(10) << l.steer_reversals
          << std::setprecision(1)
          << std::setw(10) << (l.accel_frac * 100.0)
          << std::setw(10) << (l.brake_frac  * 100.0);

        // sector splits inline
        for (size_t i = 0; i < l.sector_times_s.size(); ++i) {
            f << (i == 0 ? "  " : " | ")
              << std::fixed << std::setprecision(3) << l.sector_times_s[i];
        }
        f << "\n";
    }

    // ── sector mean deviation detail ──────────────────────────────────────────
    const bool has_sector_dev = std::any_of(laps_.begin(), laps_.end(),
        [](const LapStats & l){ return !l.sector_mean_dev_m.empty(); });

    if (has_sector_dev) {
        f << "\nSector Mean Raceline Deviation (m)  [-1 = no data]\n";
        f << std::string(50, '-') << "\n";
        for (const auto & l : laps_) {
            f << "  Lap " << l.lap_number << ": ";
            for (size_t i = 0; i < l.sector_mean_dev_m.size(); ++i) {
                if (i > 0) f << " | ";
                f << "S" << (i + 1) << " ";
                if (l.sector_mean_dev_m[i] < 0.0)
                    f << " n/a ";
                else
                    f << std::fixed << std::setprecision(3) << l.sector_mean_dev_m[i];
            }
            f << "\n";
        }
    }

    RCLCPP_INFO(get_logger(), "Summary written to %s/summary.txt", summary_dir.c_str());
}

// ═════════════════════════════════════════════════════════════════════════════
//  main
// ═════════════════════════════════════════════════════════════════════════════

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MetricsNode>());
    rclcpp::shutdown();
    return 0;
}