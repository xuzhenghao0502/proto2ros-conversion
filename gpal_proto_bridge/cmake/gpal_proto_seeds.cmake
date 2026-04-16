# Proto roots for EkaRT DB schema types + camera; protoc --include_imports expands the rest.
# Keeping this list tight avoids compiling orphan .proto files that collide on short ROS message names.
set(GPAL_PROTO_SEEDS
  header.proto
  geometry.proto
  hal/camera_frame.proto
  hal/chassis_info.proto
  hal/imu.proto
  hal/diag.proto
  hal/tsync_status.proto
  calib/camera_calib_param.proto
  calib/lidar_calib_param.proto
  calib/pose_calib_param.proto
  calib/radar_calib_param.proto
  calib/sensor_calibs.proto
  control/control_debug.proto
  control/control_result.proto
  planning/decision_result.proto
  planning/planning_result.proto
  planning/planning_visualization.proto
  planning/task_response.proto
  application/hmi_command.proto
  application/remote_control.proto
  application/task_command.proto
  application/task_manager_visualization.proto
  map_server/hpa_local_map.proto
  map_server/hpa_route_points.proto
  map_server/hpa_routing_info.proto
  map_server/map_region_info.proto
  map_server/navigation_sd.proto
  localization/fusion_localization_debug_info.proto
  localization/point_cloud_map_info.proto
  localization/radar_cloud_info.proto
  localization/vehicle_pose.proto
  perception/model_obstacle.proto
  perception/model_slot.proto
  perception/perception_obstacle.proto
  perception/perception_parking_slot.proto
  perception/visual_traffic_light.proto
  # road_cognition/env_road_cognition.proto — omitted: message short names collide with
  # gpal.proto.perception.* in proto2ros (single ROS package). Convert that topic in a follow-up
  # package or after renaming. db2bag skips unknown descriptors gracefully.
  system/system_resource_stats.proto
  vehicle_config/vehicle_parameters.proto
)
