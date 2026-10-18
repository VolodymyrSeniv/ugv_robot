"""Derive UGV Nav2 params from the stock nav2_params.yaml (no lidar, rtabmap /map, no AMCL)."""
import sys
import yaml

src, dst = sys.argv[1:3]
d = yaml.safe_load(open(src))
for k in ("amcl", "map_server", "map_saver", "robot_state_publisher"):
    d.pop(k, None)

d["bt_navigator"]["ros__parameters"]["odom_topic"] = "/odometry/filtered"

c = d["controller_server"]["ros__parameters"]
c["controller_frequency"] = 15.0
c["failure_tolerance"] = 1.0
c["progress_checker"]["required_movement_radius"] = 0.25
c["progress_checker"]["movement_time_allowance"] = 20.0
c["general_goal_checker"].update(xy_goal_tolerance=0.25, yaw_goal_tolerance=3.14)
c["controller_plugins"] = ["FollowPath"]
c["FollowPath"] = {
    "plugin": "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    "desired_linear_vel": 0.4,
    "lookahead_dist": 0.7, "min_lookahead_dist": 0.4, "max_lookahead_dist": 1.0,
    "use_velocity_scaled_lookahead_dist": False,
    "transform_tolerance": 0.2,
    "use_rotate_to_heading": True,
    "rotate_to_heading_angular_vel": 0.4,       # slow turns keep VSLAM tracking
    "rotate_to_heading_min_angle": 0.6,
    "max_angular_accel": 1.0,
    "allow_reversing": False,
    "use_collision_detection": False,
    "use_regulated_linear_velocity_scaling": True,
    "regulated_linear_scaling_min_radius": 0.8,
    "regulated_linear_scaling_min_speed": 0.15,
    "use_cost_regulated_linear_velocity_scaling": False,
}

common = {
    "robot_radius": 0.45,
    "resolution": 0.05,
    "track_unknown_space": True,
    "always_send_full_costmap": True,
}
lc = d["local_costmap"]["local_costmap"]["ros__parameters"]
lc.update(common, global_frame="odom", width=4, height=4,
          plugins=["static_layer", "inflation_layer"])
lc.pop("voxel_layer", None)
lc["static_layer"] = {"plugin": "nav2_costmap_2d::StaticLayer",
                      "map_subscribe_transient_local": True,
                      "subscribe_to_updates": True}
lc["inflation_layer"].update(inflation_radius=0.6, cost_scaling_factor=3.0)

gc = d["global_costmap"]["global_costmap"]["ros__parameters"]
gc.update(common, plugins=["static_layer", "inflation_layer"])
gc.pop("obstacle_layer", None)
gc["static_layer"] = {"plugin": "nav2_costmap_2d::StaticLayer",
                      "map_subscribe_transient_local": True,
                      "subscribe_to_updates": True}
gc["inflation_layer"].update(inflation_radius=0.6, cost_scaling_factor=3.0)

p = d["planner_server"]["ros__parameters"]["GridBased"]
p["allow_unknown"] = True
p["tolerance"] = 0.5

vs = d["velocity_smoother"]["ros__parameters"]
vs.update(max_velocity=[0.5, 0.0, 0.5], min_velocity=[-0.2, 0.0, -0.5],
          max_accel=[0.5, 0.0, 1.0], max_decel=[-0.5, 0.0, -1.0],
          odom_topic="/odometry/filtered", feedback="OPEN_LOOP")

yaml.safe_dump(d, open(dst, "w"), sort_keys=False)
print("written", dst)
