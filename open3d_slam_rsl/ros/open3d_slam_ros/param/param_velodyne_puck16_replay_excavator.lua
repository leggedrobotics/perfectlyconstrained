include "default/default_parameters.lua"


params = deepcopy(DEFAULT_PARAMETERS)

--ScanToScan ODOMETRY
params.odometry.scan_processing.voxel_size = 0.05
params.odometry.scan_processing.downsampling_ratio = 1.0

--Advanced Options.
params.odometry.use_odometry_topic_instead_of_scan_to_scan = true --Uses Odometry topic instead of Scan2Scan registration.
params.odometry.use_IMU_for_attitude_initialization = false

--MAPPER_LOCALIZER
params.mapper_localizer.is_use_map_initialization = false
params.mapper_localizer.republish_the_preloaded_map = false
params.mapper_localizer.is_merge_scans_into_map = false
params.mapper_localizer.is_build_dense_map = true
params.mapper_localizer.is_attempt_loop_closures = false
params.mapper_localizer.is_print_timing_information = false
params.mapper_localizer.map_merge_delay_in_seconds = 10.0

params.mapper_localizer.is_carving_enabled = false
params.mapper_localizer.scan_to_map_registration.scan_processing.voxel_size = 0.1
params.mapper_localizer.scan_to_map_registration.scan_processing.downsampling_ratio = 1.0
params.mapper_localizer.scan_to_map_registration.scan_processing.scan_cropping.cropping_radius_max = 100.0
params.mapper_localizer.scan_to_map_registration.scan_processing.scan_cropping.cropping_radius_min = 1.0
params.mapper_localizer.scan_to_map_registration.icp.max_correspondence_dist = 2.0 --not used
params.mapper_localizer.scan_to_map_registration.icp.knn = 15 --not used
params.mapper_localizer.scan_to_map_registration.icp.max_distance_knn = 0.5 --not used
params.mapper_localizer.scan_to_map_registration.icp.reference_cloud_seting_period = 1.0 --sec

--MAP_INITIALIZER
params.map_initializer.pcd_file_package = "open3d_slam_ros"
params.map_initializer.pcd_file_path = "/data/ETH_LEE_H_with_terrace.pcd"
params.map_initializer.is_initialize_interactively = false
params.map_initializer.init_pose.x = 0.0
params.map_initializer.init_pose.y = 2.0
params.map_initializer.init_pose.z = 0.0
params.map_initializer.init_pose.roll = 0.0
params.map_initializer.init_pose.pitch = 0.0
params.map_initializer.init_pose.yaw = 120.0

--SUBMAP
params.submap.submap_size = 40.0 --meters
params.submap.adjacency_based_revisiting_min_fitness = 0.01
params.submap.min_seconds_between_feature_computation = 5.0
params.submap.submaps_num_scan_overlap = 200
params.submap.max_num_points = 7500000

--MAP_BUILDER
params.map_builder.map_voxel_size = 0.2
params.map_builder.scan_cropping.cropping_radius_max = 100.0
params.map_builder.scan_cropping.cropping_radius_min = 1.0
params.map_builder.space_carving.carve_space_every_n_scans = 10

--DENSE_MAP_BUILDER
params.dense_map_builder.map_voxel_size = 0.05
params.dense_map_builder.scan_cropping.cropping_radius_max = 105.0
params.dense_map_builder.space_carving.carve_space_every_n_scans = 10
params.dense_map_builder.space_carving.truncation_distance = 0.1

--SAVING
params.saving.save_map = true
params.saving.save_submaps = false
params.saving.save_dense_submaps = true


return params