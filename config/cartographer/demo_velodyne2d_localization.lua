-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "demo_velodyne2d.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}
TRAJECTORY_BUILDER_2D.max_z= 0.8 --1
TRAJECTORY_BUILDER_2D.min_z= 0.0 ---0.1
TRAJECTORY_BUILDER_2D.min_range=0.4
TRAJECTORY_BUILDER_2D.max_range=12.0  
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.
--TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false

POSE_GRAPH.optimize_every_n_nodes = 1
POSE_GRAPH.constraint_builder.max_constraint_distance = 25
MAP_BUILDER.num_background_threads =8

-- the parameters useful 0.2 0.3
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
POSE_GRAPH.global_sampling_ratio = 1e-2
POSE_GRAPH.constraint_builder.min_score=0.45
POSE_GRAPH.constraint_builder.global_localization_min_score=0.6
global_constraint_search_after_n_seconds = 10.

return options
