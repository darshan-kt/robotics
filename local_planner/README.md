# LOCAL PLANNER

- Package Description
        -- Local Planner is a custom made algorithm which works on Augmented Trajectory Transforms method.
           Initial reference trajectory is fed to the algorithm over which the trajectory segments are augmented and best trajectory is selected based on costs.

- Sub-packages(if any) 
        -- Nil
- Version description
        --V1.0
          Basic code to test the actuations with the given waypoints
- RQT graph of nodes


---



### Nodes

- **local_planner**: 
  - The node is located at 'src/' and code files are 'activate_local_planner.cpp','main.cpp', 'dubins.cpp'
  - This node takes input from following topics:
        -- '/ndt_pose'             - Localization data
        -- '/currentState'         - What state to be in is fed from Behavior Planner eg.KeepLane,LanneChangeLeft
        -- '/local_map'            - Binary grid for obstacle validation
        -- '/speed'                - Velocity feedback of the vehicle

  - Topics from the node
        -- '/localTrajectory'      - Trajectory output in local frame
        -- '/globalTrajectory'     - Trajectory output in map frame
        -- '/curves'               - Curve Trajectories for lane-shifts



---



### Launch Files

- Launch File Name: NIL


---



### Inputs 

*This section contains the Input data to the package/sub-packages*

- Data
    -- Localization
    -- CurrentState , the state for which the trajectory has to be generated
    -- Velocity Feedback
    -- Global Waypoints



### Outputs 

*This section contains the output data from the package/sub-packages*

- Data
    -- Local Trajectory in local frame
    -- Global Trajectory in map frame
    -- Curves for lane shift
---



### Parameters for GUI

*In the following table,*
*first row: Headings*
*second row: general description*
*third row: example*
*fourth row: example*

| Variable                 | Description                      | Code Location                                                | Data-type         | Variable Value                | Range                      |
| ------------------------ | -------------------------------- | ------------------------------------------------------------ | ----------------- | ----------------------------- | -------------------------- |
| `variableName` from code | value that variable stores       | [Location of file in which variable is defined]`directory/filename.ext` | Variable DataType | Initial variable value in GUI | Value of varibale in range |
| `model_path`             | Object detection model path      | `scripts/object_detection.py`                                | String            | `models/YOLO.h5`              | N.A.                       |
| learning_rate            | learning rate for model training | `scripts/model_training.py`                                  | Float32           | `0.001`                       | `0.0001 to 0.1`            |





---



### Topics

*In the following table,*
*first row: Headings*
*second row: general description*
*third row: example*

| Topic Name                  | Description                        | Data Type         |
| --------------------------- | -----------------------            | ----------------- |
| `/localTrajectory`          | Trajectory output in local frame   | nav_msgs/Path     |
| `/globalTrajectory`         | Trajectory output in map frame     | nav_msgs/Path     |
| `/curves`                   | Curve Trajectories for lane-shifts | nav_msgs/Path     | 



### Version Description

- Version 1.0
  - New:
    - Initial Code has been developed



---



### Package Status

| Node Name     | Coding | Simulator Testing | On-vehicle testing/ parameter tuning | Deployed on vehicle |
| ---------     | ------ | ----------------- | ------------------------------------ | ------------------- |
| local_planner | ✔️      | ✔️                 |❌                                     | ❌                   |



---



Following package follows [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) and [PEP8 style guide](https://www.python.org/dev/peps/pep-0008/)



---



### Authors

- VIMAL RAJ AYYAPPAN (vimal.raj@fluxauto.xyz)




---

Updated on Jan 20, 2020 by AuthorName