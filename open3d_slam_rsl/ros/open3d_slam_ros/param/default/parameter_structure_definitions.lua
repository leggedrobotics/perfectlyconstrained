-- need this to deep copy all the tables
function deepcopy(orig, copies)
    copies = copies or {}
    local orig_type = type(orig)
    local copy
    if orig_type == 'table' then
        if copies[orig] then
            copy = copies[orig]
        else
            copy = {}
            copies[orig] = copy
            for orig_key, orig_value in next, orig, nil do
                copy[deepcopy(orig_key, copies)] = deepcopy(orig_value, copies)
            end
            setmetatable(copy, deepcopy(getmetatable(orig), copies))
        end
    else -- number, string, boolean, etc
        copy = orig
    end
    return copy
end



SAVING_PARAMETERS = {
  save_at_mission_end = true,
  save_map = true,
  save_submaps = false,
  save_dense_submaps = true,
}

MOTION_COMPENSATION_PARAMETERS = {
  is_undistort_scan = false,
  is_spinning_clockwise = true,
  scan_duration = 0.1,
  num_poses_vel_estimation = 3,
}

VISUALIZATION_PARAMETERS = {
  assembled_map_voxel_size = 0.05,
  submaps_voxel_size = 0.05,
  visualize_every_n_msec = 250.0,
}

GLOBAL_OPTIMIZATION_PARAMETERS = {
  edge_prune_threshold = 0.2,
  loop_closure_preference = 2.0,
  max_correspondence_distance = 1000.0,
  reference_node = 0,
}

SCAN_CROPPING_PARAMETERS = {
  cropping_radius_max= 100.0,
  cropping_radius_min= 2.0,
  min_z= -50.0,
  max_z= 50.0,
  cropper_type= "MinMaxRadius", -- options are Cylinder, MaxRadius, MinRadius, MinMaxRadius
}

SCAN_PROCESSING_PARAMETERS = {
  voxel_size = 0.01,
  downsampling_ratio = 1.0,
  point_cloud_buffer_size = 1, -- the scan processing buffer size. 1 means no buffering.
  scan_cropping = deepcopy(SCAN_CROPPING_PARAMETERS),
}

ICP_PARAMETERS = {
  max_correspondence_dist= 1.0,
  knn= 20,
  max_distance_knn= 1.0,
  max_n_iter= 50,
  reference_cloud_seting_period= 1.0,
}

SCAN_MATCHING_PARAMETERS = {
  icp = deepcopy(ICP_PARAMETERS),
  cloud_registration_type = "GeneralizedIcp", -- options GeneralizedIcp, PointToPointIcp, PointToPlaneIcp
}

ODOMETRY_PARAMETERS = {
  use_odometry_topic_instead_of_scan_to_scan = true,
  use_IMU_for_attitude_initialization = false,
  is_publish_odometry_msgs = true,
  odometry_buffer_size = 1, -- the scan2scan odometry buffer size. 1 means no buffering.
  scan_matching = deepcopy(SCAN_MATCHING_PARAMETERS),
  scan_processing = deepcopy(SCAN_PROCESSING_PARAMETERS),
}

SUBMAP_PARAMETERS = {
  submap_size = 20, -- meters
  min_num_range_data = 10,
  adjacency_based_revisiting_min_fitness = 0.5,
  min_seconds_between_feature_computation = 5.0,
  submaps_num_scan_overlap = 10,
}

SPACE_CARVING_PARAMETERS = {
  voxel_size= 0.2,
  max_raytracing_length = 20.0,
  truncation_distance = 0.3,
  carve_space_every_n_scans= 10.0,
}

MAP_BUILDER_PARAMETERS = {
  map_voxel_size = 0.05, --meters
  scan_cropping = deepcopy(SCAN_CROPPING_PARAMETERS),
  space_carving = deepcopy(SPACE_CARVING_PARAMETERS),
}

SCAN_TO_MAP_REGISTRATION_PARAMETERS = {
  min_refinement_fitness = 0.7,
  scan_to_map_refinement_type = "GeneralizedIcp", -- options GeneralizedIcp, PointToPointIcp, PointToPlaneIcp
  icp = deepcopy(ICP_PARAMETERS),
  scan_processing = deepcopy(SCAN_PROCESSING_PARAMETERS),
}

MAPPER_LOCALIZER_PARAMETERS = {
  is_print_timing_information = true,
  is_carving_enabled = false,
  is_build_dense_map = false,
  is_attempt_loop_closures = false,
  is_use_map_initialization = false,
  republish_the_preloaded_map = true,
  map_merge_delay_in_seconds = 10.0,
  is_merge_scans_into_map = false,
  dump_submaps_to_file_before_after_lc = false,
  is_refine_odometry_constraints_between_submaps = false,
  min_movement_between_mapping_steps = 0.0,
  ignore_minimum_refinement_fitness = false,
  mapping_buffer_size = 1, -- the scan2map odometry buffer size. 1 means no buffering.
  scan_to_map_registration = deepcopy(SCAN_TO_MAP_REGISTRATION_PARAMETERS),
}

POSE = {
 x = 0.0,
 y = 0.0,
 z = 0.0,
 roll = 0.0, --roll, pitch ,yaw in degrees!!!!!
 pitch = 0.0,
 yaw = 0.0,
}

MAP_INITIALIZER_PARAMETERS = {
  is_initialize_interactively = false,
  frame_id = "map_o3d",
  pcd_file_path = "",
  pcd_file_package = "",
  init_pose = POSE,
}

LOOP_CLOSURE_CONSISTENCY_CHECK_PARAMETERS = {
  max_drift_roll = 30.0, --deg
  max_drift_pitch = 30.0, --deg
  max_drift_yaw = 30.0, --deg
  max_drift_x = 80.0, --meters
  max_drift_y = 80.0, --meters
  max_drift_z = 40.0, --meters
}

PLACE_RECOGNITION_PARAMETERS = {
  feature_map_normal_estimation_radius = 2.0,
  feature_voxel_size = 0.5,
  feature_radius = 2.5,
  feature_knn = 100,
  feature_normal_knn = 20,
  ransac_num_iter = 10000000,
  ransac_probability = 0.999,
  ransac_model_size = 3,
  ransac_max_correspondence_dist = 0.75,
  ransac_correspondence_checker_distance = 0.8,
  ransac_correspondence_checker_edge_length = 0.6,
  ransac_min_corresondence_set_size = 25,
  max_icp_correspondence_distance = 0.3,
  min_icp_refinement_fitness = 0.7, -- the more aliasing, the higher this should be
  dump_aligned_place_recognitions_to_file = false , --useful for debugging
  min_submaps_between_loop_closures = 2,
  loop_closure_search_radius = 20.0,
  consistency_check = deepcopy(LOOP_CLOSURE_CONSISTENCY_CHECK_PARAMETERS),
}

