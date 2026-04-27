"""
metrics.launch.py

Usage examples
--------------
# Simulation (defaults)
ros2 launch f1tenth_race_metrics metrics.launch.py

# Simulation with a race line and custom output dir
ros2 launch f1tenth_race_metrics metrics.launch.py \
    output_dir:=/home/user/runs/algo_a \
    race_line_path:=/home/user/maps/levine_raceline.csv

# Hardware car — uses hw_params.yaml as base config
ros2 launch f1tenth_race_metrics metrics.launch.py \
    use_hardware:=true \
    output_dir:=/home/nvidia/metrics/run_01 \
    race_line_path:=/home/nvidia/maps/levine_raceline.csv
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _select_config(context, *args, **kwargs):
    """Pick the appropriate base YAML depending on use_hardware."""
    pkg_share = FindPackageShare("f1tenth_race_metrics").perform(context)
    use_hw = LaunchConfiguration("use_hardware").perform(context).lower()
    cfg = "hw_params.yaml" if use_hw == "true" else "sim_params.yaml"
    config_path = f"{pkg_share}/config/{cfg}"

    # Keep YAML defaults unless a non-empty CLI race_line_path is provided.
    race_line_path_cli = LaunchConfiguration("race_line_path").perform(context).strip()
    param_overrides = {
        "use_hardware":            LaunchConfiguration("use_hardware"),
        "output_dir":              LaunchConfiguration("output_dir"),
        "max_speed":               LaunchConfiguration("max_speed"),
        "vel_util_threshold_frac": LaunchConfiguration("vel_util_threshold_frac"),
        "lap_cooldown_s":          LaunchConfiguration("lap_cooldown_s"),
        "num_sectors":             LaunchConfiguration("num_sectors"),
    }
    if race_line_path_cli:
        param_overrides["race_line_path"] = race_line_path_cli

    node = Node(
        package="f1tenth_race_metrics",
        executable="f1tenth_race_metrics",
        name="metrics_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            # 1. Base YAML (mode-specific defaults)
            config_path,
            # 2. CLI overrides — take precedence over YAML values
            param_overrides,
        ],
    )
    return [node]


def generate_launch_description():
    return LaunchDescription([
        # ── mode ──────────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            "use_hardware",
            default_value="false",
            description="true → use /pf/viz/inferred_pose + hardware odom; "
                        "false → use /ego_racecar/odom (sim)",
        ),

        # ── I/O ───────────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            "output_dir",
            default_value="/workspaces/roboracer_ws/metrics_output",
            description="Directory where step_log.csv, lap_log.csv, and summary.txt are written.",
        ),
        DeclareLaunchArgument(
            "race_line_path",
            default_value="",
            description="Path to race line CSV (one 'x,y' pair per row, '#' for comments). "
                        "Leave empty to disable deviation metric.",
        ),

        # ── tuning — can be overridden without editing the YAML ───────────────
        DeclareLaunchArgument(
            "max_speed",
            default_value="6.0",
            description="Car's maximum speed in m/s (used for velocity utilisation metric).",
        ),
        DeclareLaunchArgument(
            "vel_util_threshold_frac",
            default_value="0.75",
            description="Fraction of max_speed above which a timestep counts as 'utilised'.",
        ),
        DeclareLaunchArgument(
            "lap_cooldown_s",
            default_value="3.0",
            description="Minimum seconds between two lap-crossing events (prevents double-count).",
        ),
        DeclareLaunchArgument(
            "num_sectors",
            default_value="3",
            description="Number of sectors to split each lap into (requires a race line CSV).",
        ),

        # ── node (config selected at launch time) ─────────────────────────────
        OpaqueFunction(function=_select_config),
    ])