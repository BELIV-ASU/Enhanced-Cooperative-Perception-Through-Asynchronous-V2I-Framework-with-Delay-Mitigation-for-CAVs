# Enhanced Cooperative Perception Through Asynchronous Vehicle to Infrastructure Framework with Delay Mitigation for Connected and Automated Vehicles

## Usage
1. Autoware installation - [github repo](https://github.com/autowarefoundation/autoware/tree/release/2024.04)  
Note: Our framework is tested with Autoware version 2024.04

2. Download and install the custom created ROS2 packages for V2I  
    ```
    # Change directory to your Autoware base folder
    cd autoware 

    # Clone the github repository
    git clone https://github.com/BELIV-ASU/Enhanced-Cooperative-Perception-Through-Asynchronous-V2I-Framework-with-Delay-Mitigation-for-CAVs.git V2I_repo

    # Import all the repos in CV2I.repos using vcs
    vcs import src < V2I_repo/CV2I.repos --recursive

    # Install dependencies
    rosdep install --from-paths src -y --ignore-src

    # Build the newly imported custom packages
    colcon build --packages-select infrastructure_objects_frame_transformation \
    infrastructure_objects_delay_compensation \
    euclidean_object_merger \
    sim_c_v2i_launch\
    autoware_launch\
    map_based_prediction
    ```

## Run the C-V2I Framework
- ROS2 bag files - [click here](https://www.dropbox.com/home/Nithish%20Saravanan/Nithish%20Thesis)  
+ CARLA Town10 lanelet and pointcloud map - [click here](https://www.dropbox.com/home/Nithish%20Saravanan/Nithish%20Thesis/Town10)  
Place in the map in the following folder:  
/home/**Your USERNAME**/autoware_map/Town10


1. Extract the ROS2 bag and run scenario 1 ROS2 bag file
    ```
    ros2 bag play your_ros2_bag_name.db3
    ```
2. Then launch the Autoware using the following command. Not all the modules in Autoware are required for this C-V2I framework. Only specific subsystems like
vehicle, system and map are launched and the rest of the subsystems are set to false in the autoware launch file to prevent them from being launched.
    ```
    cd autoware

    source install/setup.bash

    ros2 launch autoware_launch modified_autoware.launch.xml map_path:=your_map_path
    ```
3. Launch the C-V2I framework (**Remember to source autoware workspace**)
    ```
    cd autoware

    source install/setup.bash

    ros2 launch sim_c_v2i_launch sim_c_v2i.launch.xml
    ```

## Run Perception Framework also without C-V2I
1. Extract the ROS2 bag and run scenario 1 ROS2 bag file
    ```
    ros2 bag play your_ros2_bag_name.db3
    ```
2. Launching the LiDAR Centerpoint algorithm
    ```
    ros2 launch lidar_centerpoint lidar_centerpoint.launch.xml
    input/pointcloud:=/sensing/lidar/concatenated/rate_downsampled/pointcloud
    output/objects:=/perception/object_recognition/detection/centerpoint/objects
    ```

3. Launch Multi-Object Tracking
    ```
    ros2 launch multi_object_tracker multi_object_tracker.launch.xml
    input:=/perception/object_recognition/detection/centerpoint/objects
    output:=/perception/object_recognition/tracking/centerpoint/objects
    ```

4. Launch Map based prediction
    ```
    ros2 launch map_based_prediction map_based_prediction.launch.xml
    input_topic:=/perception/object_recognition/tracking/centerpoint/objects
    output_topic:=/perception/object_recognition/prediction/onboard_objects
    ```