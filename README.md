# Degeneracy Mitigation in LiDAR SLAM
Official implementations from the paper "Informed, Constrained, Aligned: A Field Analysis on Degeneracy-aware Point Cloud Registration in the Wild" [arxiv link](https://arxiv.org/abs/2408.11809).

This repository contains the implementation of all the methods compared and analyized in this work, including the degeneracy detection method of X-ICP.

## Components
The work investigates different degeneracy mitigation methods for the problem of degenerate point cloud registration. To achieve this, this work relies on many open-sourced libraries. These libraries / packages are subject to their licenses. 

- This work builds on the open-source version of [Open3D SLAM](https://github.com/leggedrobotics/open3d_slam).
- This work uses libpointmatcher point cloud registration library in the backend. The locally available (dont need to clone) libpointmatcher library is based on this [version](https://github.com/ANYbotics/libpointmatcher).
- The locally available (dont need to clone) `pointmatcher_ros` package is based on [this version](https://github.com/ANYbotics/pointmatcher-ros).
- For NL-Reg. and NL-Solver methods, this work depends on [Ceres](https://github.com/ceres-solver/ceres-solver). Version 2.1.0
- For the Ineq. Con. method, this work relies on an old version of QPmad quadratic problem library [link](https://github.com/leggedrobotics/qpmad).
- To replicate the result of the work of Petracek et al. (RMS), the authors [forked](https://github.com/leggedrobotics/RMS) the [original work](https://github.com/ctu-mrs/RMS). Please also install the dependencies of this repository.


## Dependencies
1. Ubuntu 20.04 (expecting PCL 1.10)
2. ROS Noetic
3. CMake > 3.18 (tested 3.25)
```bash
cmake --version
```
**If NOT** install the cmake from https://cmake.org/download download tar.
Unpack tar and
``` bash
cd cmake-<version>
./configure
make -j<num_of_processes>
sudo make install
```
4. Open3D, automatically installed by `open3d_catkin` package (this take 6-7gb of space).
5. libnabo (dependency of libpointmatcher) [link](http://github.com/anybotics/libnabo)
6. install dependencies for `open3d_slam` (`sudo apt install libgoogle-glog-dev libglfw3 libglfw3-dev liblua5.2-dev`)
7. message_logger [repository link](https://github.com/ANYbotics/message_logger)
8. QPmad [fork](https://github.com/leggedrobotics/qpmad). + `catkin build qpmad`
9. Ceres (tested with 2.1.0). [Ceres](https://github.com/ceres-solver/ceres-solver)
10. RMS [fork](https://github.com/leggedrobotics/RMS) and its dependencies.

## Build
To build this repository, you need to:

Clone the repository to:
``` bash
cd catkin_ws/src
git clone git@github.com:leggedrobotics/perfectlyconstrained.git.
cd ..
catkin init
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build open3d_slam_ros
```
Step by step building could have debugging installation process. 

1. `catkin build libpointmatcher`
2. `catkin build pointmatcher_ros`
3. `catkin build open3d_slam`
4. `catkin build open3d_slam_ros`

## Running / Result replicating
Here, we describe the process for 1 dataset but this process apply to all datasets.

1. Get the `.bag` file for the [ANYmal forest experiment](https://drive.google.com/drive/folders/1Y8w1Twdv2db-PMsx9uKMwGQ48yDisjx6)
``` bash
roslaunch open3d_slam_ros replay_forest.launch rosbag_filepath:=/path/to/your_file.bag
```
2. An RVIZ window pop-up and replay the rosbag as fast as possible.
3. The results of the test will be saved to the pre-defined folder in the launch file.

Running Ulmberg Tunnel experiment 
``` bash
roslaunch open3d_slam_ros replay_tunnel.launch rosbag_filepath:=/path/to/your_file.bag
```

Running HEAP Excavation experiment 
``` bash
roslaunch open3d_slam_ros replay_excavator.launch rosbag_filepath:=/path/to/your_file.bag
```

Running ANYmal simulation experiment 
``` bash
roslaunch open3d_slam_ros replay_sim.launch rosbag_filepath:=/path/to/your_file.bag
```

## Changing the method

All the degeneracy point cloud registration method parameters are located in (open3d_slam_rsl/ros/open3d_slam_ros/param/icp.yaml). Unfortunately, the parameters are not nicely separated however, below you can find the parameter sets that exactly correspond to the methods described in the work.

<details>
<summary>P2Plane</summary>

```
readingDataPointsFilters:
# No specific filters.

referenceDataPointsFilters:
# No specific filters.

matcher:
  KDTreeMatcher:
    knn: 1
    maxDist: 0.5
    epsilon: 0.01

outlierFilters:
  - TrimmedDistOutlierFilter:
     ratio: 0.95
  - SurfaceNormalOutlierFilter:
     maxAngle: 1.57

# This enables L-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" degeneracyAwareness method
regularization:
  Disabled: # "Enabled" or "Disabled.
    regularizationWeight: 440.0

# Enables the RMS filtering by Petracek et al.
enableRMSfiltering:
  Disabled: # "Enabled" or "Disabled
    rmsLambda: 0.0036 # Set to 0.0042 for Ouster based experiments.

# This enables L-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" degeneracyAwareness method
ceresDegeneracyAnalysis:
  CeresDegeneracyState:
    isActive: 0 # "Enabled -> 1" or "Disabled -> 0. This is the NL-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" for correct detection.
    usePointToPointCost: 0
    usePointToPlaneCost: 1
    useBoundConstraints: 1 # This is the transition from NL-Reg. to NL-Solver. 1: NL-Reg. 0: NL-Solver.
    useSixDofRegularization: 0
    useThreeDofRegularization: 1
    regularizationWeight: 675.0 # The weight of NL-Reg.

degeneracyAwareness:
  None:

transformationCheckers:
  - DifferentialTransformationChecker:
      minDiffRotErr: 0.001
      minDiffTransErr: 0.01
      smoothLength: 3
  - CounterTransformationChecker:
      maxIterationCount: 30

inspector:
  NullInspector

logger:
  NullLogger

errorMinimizer:
  PointToPlaneErrorMinimizer

# Leave it enabled, this enforces the XICP detection.
forceXICPdetection:
  Enabled: # "Enabled" or "Disabled

degeneracyDebug:
  Enabled: # "Enabled" or "Disabled.

printingDegeneracy:
  Enabled: # "Enabled" or "Disabled.
```

</details>

<details>
<summary>Eq. Con.</summary>

```
readingDataPointsFilters:
# No specific filters.

referenceDataPointsFilters:
# No specific filters.

matcher:
  KDTreeMatcher:
    knn: 1
    maxDist: 0.5
    epsilon: 0.01

outlierFilters:
  - TrimmedDistOutlierFilter:
     ratio: 0.95
  - SurfaceNormalOutlierFilter:
     maxAngle: 1.57

# This enables L-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" degeneracyAwareness method
regularization:
  Disabled: # "Enabled" or "Disabled.
    regularizationWeight: 440.0

# Enables the RMS filtering by Petracek et al.
enableRMSfiltering:
  Disabled: # "Enabled" or "Disabled
    rmsLambda: 0.0036 # Set to 0.0042 for Ouster based experiments.

# This enables L-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" degeneracyAwareness method
ceresDegeneracyAnalysis:
  CeresDegeneracyState:
    isActive: 0 # "Enabled -> 1" or "Disabled -> 0. This is the NL-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" for correct detection.
    usePointToPointCost: 0
    usePointToPlaneCost: 1
    useBoundConstraints: 1 # This is the transition from NL-Reg. to NL-Solver. 1: NL-Reg. 0: NL-Solver.
    useSixDofRegularization: 0
    useThreeDofRegularization: 1
    regularizationWeight: 675.0 # The weight of NL-Reg.

degeneracyAwareness:
  OptimizedEqualityConstraints:
    enoughInformationThreshold: 300
    insufficientInformationThreshold: 150
    point2NormalMinimalAlignmentAngleThreshold: 80
    point2NormalStrongAlignmentAngleThreshold: 45

transformationCheckers:
  - DifferentialTransformationChecker:
      minDiffRotErr: 0.001
      minDiffTransErr: 0.01
      smoothLength: 3
  - CounterTransformationChecker:
      maxIterationCount: 30

inspector:
  NullInspector

logger:
  NullLogger

errorMinimizer:
  PointToPlaneErrorMinimizer

# Leave it enabled, this enforces the XICP detection.
forceXICPdetection:
  Enabled: # "Enabled" or "Disabled

degeneracyDebug:
  Enabled: # "Enabled" or "Disabled.

printingDegeneracy:
  Enabled: # "Enabled" or "Disabled.
```

</details>

<details>
<summary>Ineq. Con.</summary>

```
readingDataPointsFilters:
# No specific filters.

referenceDataPointsFilters:
# No specific filters.

matcher:
  KDTreeMatcher:
    knn: 1
    maxDist: 0.5
    epsilon: 0.01

outlierFilters:
  - TrimmedDistOutlierFilter:
     ratio: 0.95
  - SurfaceNormalOutlierFilter:
     maxAngle: 1.57

# This enables L-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" degeneracyAwareness method
regularization:
  Disabled: # "Enabled" or "Disabled.
    regularizationWeight: 440.0

# Enables the RMS filtering by Petracek et al.
enableRMSfiltering:
  Disabled: # "Enabled" or "Disabled
    rmsLambda: 0.0036 # Set to 0.0042 for Ouster based experiments.

# This enables L-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" degeneracyAwareness method
ceresDegeneracyAnalysis:
  CeresDegeneracyState:
    isActive: 0 # "Enabled -> 1" or "Disabled -> 0. This is the NL-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" for correct detection.
    usePointToPointCost: 0
    usePointToPlaneCost: 1
    useBoundConstraints: 1 # This is the transition from NL-Reg. to NL-Solver. 1: NL-Reg. 0: NL-Solver.
    useSixDofRegularization: 0
    useThreeDofRegularization: 1
    regularizationWeight: 675.0 # The weight of NL-Reg.

degeneracyAwareness:
  InequalityConstraints:
   highInformationThreshold: 300
   enoughInformationThreshold: 300
   insufficientInformationThreshold: 150
   point2NormalMinimalAlignmentAngleThreshold: 80
   point2NormalStrongAlignmentAngleThreshold: 45
   inequalityboundmultiplier: 0.0014

transformationCheckers:
  - DifferentialTransformationChecker:
      minDiffRotErr: 0.001
      minDiffTransErr: 0.01
      smoothLength: 3
  - CounterTransformationChecker:
      maxIterationCount: 30

inspector:
  NullInspector

logger:
  NullLogger

errorMinimizer:
  PointToPlaneErrorMinimizer

# Leave it enabled, this enforces the XICP detection.
forceXICPdetection:
  Enabled: # "Enabled" or "Disabled

degeneracyDebug:
  Enabled: # "Enabled" or "Disabled.

printingDegeneracy:
  Enabled: # "Enabled" or "Disabled.
```

</details>

<details>
<summary>L-Reg.</summary>

```
readingDataPointsFilters:
# No specific filters.

referenceDataPointsFilters:
# No specific filters.

matcher:
  KDTreeMatcher:
    knn: 1
    maxDist: 0.5
    epsilon: 0.01

outlierFilters:
  - TrimmedDistOutlierFilter:
     ratio: 0.95
  - SurfaceNormalOutlierFilter:
     maxAngle: 1.57

# This enables L-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" degeneracyAwareness method
regularization:
  Enabled: # "Enabled" or "Disabled.
    regularizationWeight: 440.0

# Enables the RMS filtering by Petracek et al.
enableRMSfiltering:
  Disabled: # "Enabled" or "Disabled
    rmsLambda: 0.0036 # Set to 0.0042 for Ouster based experiments.

# This enables L-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" degeneracyAwareness method
ceresDegeneracyAnalysis:
  CeresDegeneracyState:
    isActive: 0 # "Enabled -> 1" or "Disabled -> 0. This is the NL-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" for correct detection.
    usePointToPointCost: 0
    usePointToPlaneCost: 1
    useBoundConstraints: 1 # This is the transition from NL-Reg. to NL-Solver. 1: NL-Reg. 0: NL-Solver.
    useSixDofRegularization: 0
    useThreeDofRegularization: 1
    regularizationWeight: 675.0 # The weight of NL-Reg.

degeneracyAwareness:
  OptimizedEqualityConstraints:
    enoughInformationThreshold: 300
    insufficientInformationThreshold: 150
    point2NormalMinimalAlignmentAngleThreshold: 80
    point2NormalStrongAlignmentAngleThreshold: 45

transformationCheckers:
  - DifferentialTransformationChecker:
      minDiffRotErr: 0.001
      minDiffTransErr: 0.01
      smoothLength: 3
  - CounterTransformationChecker:
      maxIterationCount: 30

inspector:
  NullInspector

logger:
  NullLogger

errorMinimizer:
  PointToPlaneErrorMinimizer

# Leave it enabled, this enforces the XICP detection.
forceXICPdetection:
  Enabled: # "Enabled" or "Disabled

degeneracyDebug:
  Enabled: # "Enabled" or "Disabled.

printingDegeneracy:
  Enabled: # "Enabled" or "Disabled.
```

</details>

<details>
<summary>NL-Reg.</summary>

```
readingDataPointsFilters:
# No specific filters.

referenceDataPointsFilters:
# No specific filters.

matcher:
  KDTreeMatcher:
    knn: 1
    maxDist: 0.5
    epsilon: 0.01

outlierFilters:
  - TrimmedDistOutlierFilter:
     ratio: 0.95
  - SurfaceNormalOutlierFilter:
     maxAngle: 1.57

# This enables L-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" degeneracyAwareness method
regularization:
  Disabled: # "Enabled" or "Disabled.
    regularizationWeight: 440.0

# Enables the RMS filtering by Petracek et al.
enableRMSfiltering:
  Disabled: # "Enabled" or "Disabled
    rmsLambda: 0.0036 # Set to 0.0042 for Ouster based experiments.

# This enables L-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" degeneracyAwareness method
ceresDegeneracyAnalysis:
  CeresDegeneracyState:
    isActive: 1 # "Enabled -> 1" or "Disabled -> 0. This is the NL-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" for correct detection.
    usePointToPointCost: 0
    usePointToPlaneCost: 1
    useBoundConstraints: 1 # This is the transition from NL-Reg. to NL-Solver. 1: NL-Reg. 0: NL-Solver.
    useSixDofRegularization: 0
    useThreeDofRegularization: 1
    regularizationWeight: 675.0 # The weight of NL-Reg.

degeneracyAwareness:
  OptimizedEqualityConstraints:
    enoughInformationThreshold: 300
    insufficientInformationThreshold: 150
    point2NormalMinimalAlignmentAngleThreshold: 80
    point2NormalStrongAlignmentAngleThreshold: 45

transformationCheckers:
  - DifferentialTransformationChecker:
      minDiffRotErr: 0.001
      minDiffTransErr: 0.01
      smoothLength: 3
  - CounterTransformationChecker:
      maxIterationCount: 30

inspector:
  NullInspector

logger:
  NullLogger

errorMinimizer:
  PointToPlaneErrorMinimizer

# Leave it enabled, this enforces the XICP detection.
forceXICPdetection:
  Enabled: # "Enabled" or "Disabled

degeneracyDebug:
  Enabled: # "Enabled" or "Disabled.

printingDegeneracy:
  Enabled: # "Enabled" or "Disabled.
```

</details>

<details>
<summary>Cauchy</summary>

```
readingDataPointsFilters:
# No specific filters.

referenceDataPointsFilters:
# No specific filters.

matcher:
  KDTreeMatcher:
    knn: 1
    maxDist: 0.5
    epsilon: 0.01

outlierFilters:
  - RobustOutlierFilter: # Cauchy method.
      robustFct: cauchy
      scaleEstimator: mad
      tuning: 1.0
  - TrimmedDistOutlierFilter:
     ratio: 0.95
  - SurfaceNormalOutlierFilter:
     maxAngle: 1.57

# This enables L-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" degeneracyAwareness method
regularization:
  Disabled: # "Enabled" or "Disabled.
    regularizationWeight: 440.0

# Enables the RMS filtering by Petracek et al.
enableRMSfiltering:
  Disabled: # "Enabled" or "Disabled
    rmsLambda: 0.0036 # Set to 0.0042 for Ouster based experiments.

# This enables L-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" degeneracyAwareness method
ceresDegeneracyAnalysis:
  CeresDegeneracyState:
    isActive: 0 # "Enabled -> 1" or "Disabled -> 0. This is the NL-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" for correct detection.
    usePointToPointCost: 0
    usePointToPlaneCost: 1
    useBoundConstraints: 1 # This is the transition from NL-Reg. to NL-Solver. 1: NL-Reg. 0: NL-Solver.
    useSixDofRegularization: 0
    useThreeDofRegularization: 1
    regularizationWeight: 675.0 # The weight of NL-Reg.

degeneracyAwareness:
  None:

transformationCheckers:
  - DifferentialTransformationChecker:
      minDiffRotErr: 0.001
      minDiffTransErr: 0.01
      smoothLength: 3
  - CounterTransformationChecker:
      maxIterationCount: 30

inspector:
  NullInspector

logger:
  NullLogger

errorMinimizer:
  PointToPlaneErrorMinimizer

# Leave it enabled, this enforces the XICP detection.
forceXICPdetection:
  Enabled: # "Enabled" or "Disabled

degeneracyDebug:
  Enabled: # "Enabled" or "Disabled.

printingDegeneracy:
  Enabled: # "Enabled" or "Disabled.
```

</details>

<details>
<summary>TSVD</summary>

```
readingDataPointsFilters:
# No specific filters.

referenceDataPointsFilters:
# No specific filters.

matcher:
  KDTreeMatcher:
    knn: 1
    maxDist: 0.5
    epsilon: 0.01

outlierFilters:
  - TrimmedDistOutlierFilter:
     ratio: 0.95
  - SurfaceNormalOutlierFilter:
     maxAngle: 1.57

# This enables L-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" degeneracyAwareness method
regularization:
  Disabled: # "Enabled" or "Disabled.
    regularizationWeight: 440.0

# Enables the RMS filtering by Petracek et al.
enableRMSfiltering:
  Disabled: # "Enabled" or "Disabled
    rmsLambda: 0.0036 # Set to 0.0042 for Ouster based experiments.

# This enables L-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" degeneracyAwareness method
ceresDegeneracyAnalysis:
  CeresDegeneracyState:
    isActive: 0 # "Enabled -> 1" or "Disabled -> 0. This is the NL-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" for correct detection.
    usePointToPointCost: 0
    usePointToPlaneCost: 1
    useBoundConstraints: 1 # This is the transition from NL-Reg. to NL-Solver. 1: NL-Reg. 0: NL-Solver.
    useSixDofRegularization: 0
    useThreeDofRegularization: 1
    regularizationWeight: 675.0 # The weight of NL-Reg.

degeneracyAwareness:
  SolutionRemapping:
    threshold: 100
    use2019: 0
    useTruncatedSVD: 1 # Enables TSVD
    skipRegistration: 0
    #
    # Hacky solution to use X-ICP degeneracy detection for Zhang et al., and TSVD.
    enoughInformationThreshold: 300
    insufficientInformationThreshold: 150
    point2NormalMinimalAlignmentAngleThreshold: 80
    point2NormalStrongAlignmentAngleThreshold: 45

transformationCheckers:
  - DifferentialTransformationChecker:
      minDiffRotErr: 0.001
      minDiffTransErr: 0.01
      smoothLength: 3
  - CounterTransformationChecker:
      maxIterationCount: 30

inspector:
  NullInspector

logger:
  NullLogger

errorMinimizer:
  PointToPlaneErrorMinimizer

# Leave it enabled, this enforces the XICP detection.
forceXICPdetection:
  Enabled: # "Enabled" or "Disabled

degeneracyDebug:
  Enabled: # "Enabled" or "Disabled.

printingDegeneracy:
  Enabled: # "Enabled" or "Disabled.
```

</details>

<details>
<summary>Zhang et al.</summary>

```
readingDataPointsFilters:
# No specific filters.

referenceDataPointsFilters:
# No specific filters.

matcher:
  KDTreeMatcher:
    knn: 1
    maxDist: 0.5
    epsilon: 0.01

outlierFilters:
  - TrimmedDistOutlierFilter:
     ratio: 0.95
  - SurfaceNormalOutlierFilter:
     maxAngle: 1.57

# This enables L-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" degeneracyAwareness method
regularization:
  Disabled: # "Enabled" or "Disabled.
    regularizationWeight: 440.0

# Enables the RMS filtering by Petracek et al.
enableRMSfiltering:
  Disabled: # "Enabled" or "Disabled
    rmsLambda: 0.0036 # Set to 0.0042 for Ouster based experiments.

# This enables L-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" degeneracyAwareness method
ceresDegeneracyAnalysis:
  CeresDegeneracyState:
    isActive: 0 # "Enabled -> 1" or "Disabled -> 0. This is the NL-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" for correct detection.
    usePointToPointCost: 0
    usePointToPlaneCost: 1
    useBoundConstraints: 1 # This is the transition from NL-Reg. to NL-Solver. 1: NL-Reg. 0: NL-Solver.
    useSixDofRegularization: 0
    useThreeDofRegularization: 1
    regularizationWeight: 675.0 # The weight of NL-Reg.

degeneracyAwareness:
  SolutionRemapping:
    threshold: 100
    use2019: 0
    useTruncatedSVD: 0 # Enables TSVD
    skipRegistration: 0
    #
    # Hacky solution to use X-ICP degeneracy detection for Zhang et al., and TSVD.
    enoughInformationThreshold: 300
    insufficientInformationThreshold: 150
    point2NormalMinimalAlignmentAngleThreshold: 80
    point2NormalStrongAlignmentAngleThreshold: 45

transformationCheckers:
  - DifferentialTransformationChecker:
      minDiffRotErr: 0.001
      minDiffTransErr: 0.01
      smoothLength: 3
  - CounterTransformationChecker:
      maxIterationCount: 30

inspector:
  NullInspector

logger:
  NullLogger

errorMinimizer:
  PointToPlaneErrorMinimizer

# Leave it enabled, this enforces the XICP detection.
forceXICPdetection:
  Enabled: # "Enabled" or "Disabled

degeneracyDebug:
  Enabled: # "Enabled" or "Disabled.

printingDegeneracy:
  Enabled: # "Enabled" or "Disabled.
```

</details>

<details>
<summary>RMS(Petracek et al.)</summary>

```
readingDataPointsFilters:
# No specific filters.

referenceDataPointsFilters:
# No specific filters.

matcher:
  KDTreeMatcher:
    knn: 1
    maxDist: 0.5
    epsilon: 0.01

outlierFilters:
  - TrimmedDistOutlierFilter:
     ratio: 0.95
  - SurfaceNormalOutlierFilter:
     maxAngle: 1.57

# This enables L-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" degeneracyAwareness method
regularization:
  Disabled: # "Enabled" or "Disabled.
    regularizationWeight: 440.0

# Enables the RMS filtering by Petracek et al.
enableRMSfiltering:
  Enabled: # "Enabled" or "Disabled
    rmsLambda: 0.0036 # Set to 0.0042 for Ouster based experiments.

# This enables L-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" degeneracyAwareness method
ceresDegeneracyAnalysis:
  CeresDegeneracyState:
    isActive: 0 # "Enabled -> 1" or "Disabled -> 0. This is the NL-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" for correct detection.
    usePointToPointCost: 0
    usePointToPlaneCost: 1
    useBoundConstraints: 1 # This is the transition from NL-Reg. to NL-Solver. 1: NL-Reg. 0: NL-Solver.
    useSixDofRegularization: 0
    useThreeDofRegularization: 1
    regularizationWeight: 675.0 # The weight of NL-Reg.

degeneracyAwareness:
  None:

transformationCheckers:
  - DifferentialTransformationChecker:
      minDiffRotErr: 0.001
      minDiffTransErr: 0.01
      smoothLength: 3
  - CounterTransformationChecker:
      maxIterationCount: 30

inspector:
  NullInspector

logger:
  NullLogger

errorMinimizer:
  PointToPlaneErrorMinimizer

# Leave it enabled, this enforces the XICP detection.
forceXICPdetection:
  Enabled: # "Enabled" or "Disabled

degeneracyDebug:
  Enabled: # "Enabled" or "Disabled.

printingDegeneracy:
  Enabled: # "Enabled" or "Disabled.
```

</details>

<details>
<summary>NL-Solver</summary>

```
readingDataPointsFilters:
# No specific filters.

referenceDataPointsFilters:
# No specific filters.

matcher:
  KDTreeMatcher:
    knn: 1
    maxDist: 0.5
    epsilon: 0.01

outlierFilters:
  - TrimmedDistOutlierFilter:
     ratio: 0.95
  - SurfaceNormalOutlierFilter:
     maxAngle: 1.57

# This enables L-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" degeneracyAwareness method
regularization:
  Disabled: # "Enabled" or "Disabled.
    regularizationWeight: 440.0

# Enables the RMS filtering by Petracek et al.
enableRMSfiltering:
  Disabled: # "Enabled" or "Disabled
    rmsLambda: 0.0036 # Set to 0.0042 for Ouster based experiments.

# This enables L-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" degeneracyAwareness method
ceresDegeneracyAnalysis:
  CeresDegeneracyState:
    isActive: 1 # "Enabled -> 1" or "Disabled -> 0. This is the NL-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" for correct detection.
    usePointToPointCost: 0
    usePointToPlaneCost: 1
    useBoundConstraints: 1 # This is the transition from NL-Reg. to NL-Solver. 1: NL-Reg. 0: NL-Solver.
    useSixDofRegularization: 0
    useThreeDofRegularization: 0
    regularizationWeight: 675.0 # The weight of NL-Reg.

degeneracyAwareness:
  OptimizedEqualityConstraints:
    enoughInformationThreshold: 300
    insufficientInformationThreshold: 150
    point2NormalMinimalAlignmentAngleThreshold: 80
    point2NormalStrongAlignmentAngleThreshold: 45

transformationCheckers:
  - DifferentialTransformationChecker:
      minDiffRotErr: 0.001
      minDiffTransErr: 0.01
      smoothLength: 3
  - CounterTransformationChecker:
      maxIterationCount: 30

inspector:
  NullInspector

logger:
  NullLogger

errorMinimizer:
  PointToPlaneErrorMinimizer

# Leave it enabled, this enforces the XICP detection.
forceXICPdetection:
  Enabled: # "Enabled" or "Disabled

degeneracyDebug:
  Enabled: # "Enabled" or "Disabled.

printingDegeneracy:
  Enabled: # "Enabled" or "Disabled.
```

</details>

<details>
<summary>Gelfand et al.</summary>

```
readingDataPointsFilters:
  - CovarianceSamplingDataPointsFilter: # Gelfand et al.
      nbSample: 50000000 # In the filter, ratio based max number is applied

referenceDataPointsFilters:
# No specific filters.

matcher:
  KDTreeMatcher:
    knn: 1
    maxDist: 0.5
    epsilon: 0.01

outlierFilters:
  - TrimmedDistOutlierFilter:
     ratio: 0.95
  - SurfaceNormalOutlierFilter:
     maxAngle: 1.57

# This enables L-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" degeneracyAwareness method
regularization:
  Disabled: # "Enabled" or "Disabled.
    regularizationWeight: 440.0

# Enables the RMS filtering by Petracek et al.
enableRMSfiltering:
  Disabled: # "Enabled" or "Disabled
    rmsLambda: 0.0036 # Set to 0.0042 for Ouster based experiments.

# This enables L-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" degeneracyAwareness method
ceresDegeneracyAnalysis:
  CeresDegeneracyState:
    isActive: 0 # "Enabled -> 1" or "Disabled -> 0. This is the NL-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" for correct detection.
    usePointToPointCost: 0
    usePointToPlaneCost: 1
    useBoundConstraints: 1 # This is the transition from NL-Reg. to NL-Solver. 1: NL-Reg. 0: NL-Solver.
    useSixDofRegularization: 0
    useThreeDofRegularization: 1
    regularizationWeight: 675.0 # The weight of NL-Reg.

degeneracyAwareness:
  None:

transformationCheckers:
  - DifferentialTransformationChecker:
      minDiffRotErr: 0.001
      minDiffTransErr: 0.01
      smoothLength: 3
  - CounterTransformationChecker:
      maxIterationCount: 30

inspector:
  NullInspector

logger:
  NullLogger

errorMinimizer:
  PointToPlaneErrorMinimizer

# Leave it enabled, this enforces the XICP detection.
forceXICPdetection:
  Enabled: # "Enabled" or "Disabled

degeneracyDebug:
  Enabled: # "Enabled" or "Disabled.

printingDegeneracy:
  Enabled: # "Enabled" or "Disabled.
```

</details>

<details>
<summary>Prior Only</summary>

```
readingDataPointsFilters:
# No specific filters.

referenceDataPointsFilters:
# No specific filters.

matcher:
  KDTreeMatcher:
    knn: 1
    maxDist: 0.5
    epsilon: 0.01

outlierFilters:
  - TrimmedDistOutlierFilter:
     ratio: 0.95
  - SurfaceNormalOutlierFilter:
     maxAngle: 1.57

# This enables L-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" degeneracyAwareness method
regularization:
  Disabled: # "Enabled" or "Disabled.
    regularizationWeight: 440.0

# Enables the RMS filtering by Petracek et al.
enableRMSfiltering:
  Disabled: # "Enabled" or "Disabled
    rmsLambda: 0.0036 # Set to 0.0042 for Ouster based experiments.

# This enables L-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" degeneracyAwareness method
ceresDegeneracyAnalysis:
  CeresDegeneracyState:
    isActive: 0 # "Enabled -> 1" or "Disabled -> 0. This is the NL-Reg. Method. Needs to be run together with "OptimizedEqualityConstraints:" for correct detection.
    usePointToPointCost: 0
    usePointToPlaneCost: 1
    useBoundConstraints: 1 # This is the transition from NL-Reg. to NL-Solver. 1: NL-Reg. 0: NL-Solver.
    useSixDofRegularization: 0
    useThreeDofRegularization: 1
    regularizationWeight: 675.0 # The weight of NL-Reg.

degeneracyAwareness:
  SolutionRemapping:
    threshold: 100
    use2019: 0
    useTruncatedSVD: 0 # Enables TSVD
    skipRegistration: 1
    #
    # Hacky solution to use X-ICP degeneracy detection for Zhang et al., and TSVD.
    enoughInformationThreshold: 300
    insufficientInformationThreshold: 150
    point2NormalMinimalAlignmentAngleThreshold: 80
    point2NormalStrongAlignmentAngleThreshold: 45

transformationCheckers:
  - DifferentialTransformationChecker:
      minDiffRotErr: 0.001
      minDiffTransErr: 0.01
      smoothLength: 3
  - CounterTransformationChecker:
      maxIterationCount: 30

inspector:
  NullInspector

logger:
  NullLogger

errorMinimizer:
  PointToPlaneErrorMinimizer

# Leave it enabled, this enforces the XICP detection.
forceXICPdetection:
  Enabled: # "Enabled" or "Disabled

degeneracyDebug:
  Enabled: # "Enabled" or "Disabled.

printingDegeneracy:
  Enabled: # "Enabled" or "Disabled.
```

</details>

## Acknowledgement
We would like to thank all the researchers who made their work open-source to the community, which allowed us to compose this work.

## License
All the 3rd party libraries and repositories are subject to the license of their own.

## Common Issues

```
CMake Error at /usr/local/share/cmake-3.25/Modules/CMakeFindDependencyMacro.cmake:47 (find_package):
  find_package called with invalid argument ".."
  /usr/local/lib/cmake/Ceres/CeresConfig.cmake:182 (find_dependency)
  CatkinBuild.cmake:27 (find_package)
  CMakeLists.txt:81 (include)
```

On certain Ceres / Cmake versions this error prevents correct build. In that case, just go to the errorous file (/usr/local/lib/cmake/Ceres/CeresConfig.cmake) and remove `..` which is out of date. `find_dependency(CXSparse .. )` -> `find_dependency(CXSparse)`
