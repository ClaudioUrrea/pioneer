Perception & Navigation System for Semantic Priority Navigation in
Mining Robots

This repository contains the code, configurations, and resources
supporting the research paper \"SemanticPriority Navigation for
EnergyAware Mining Robots in ROS\" (to be published). It implements a
prioritized semantic navigation framework for Husky robots in mining
environments using ROS and Gazebo simulations, focusing on perception
(CNN detection, LiDAR segmentation, semantic fusion) and navigation
(costmaps, local planners, experiment execution).

Prerequisites: Install ROS Noetic Follow the official installation
guide: ROS Noetic Installation on Ubuntu.

System Requirements: Ubuntu 20.04 (recommended for ROS Noetic) Python
3.x with required libraries (see requirements.txt) ROS Noetic
desktop-full installation

Setup Instructions: 1. Create a Catkin Workspace 1.1. Create the
Workspace Directory mkdir -p \~/catkin_ws/src cd \~/catkin_ws/src 1.2.
Initialize the Catkin Workspace catkin_init_workspace 1.3. Build the
Workspace cd \~/catkin_ws catkin_make 1.4. Source the Workspace source
\~/catkin_ws/devel/setup.bash

2\. Install the Simulation Package sudo apt-get install
ros-noetic-husky-simulator

3\. Download Clearpath Gazebo Worlds: Clone the repository into your ROS
workspace: cd \~/catkin_ws/src git clone
https://github.com/clearpathrobotics/cpr_gazebo.git

4\. Export Sensor Parameters Enable 3D LIDAR (Velodyne VLP-16) and
RealSense camera: \# 1. LIDAR 3D (Velodyne VLP-16) export
HUSKY_LASER_3D_ENABLED=1 export HUSKY_LASER_3D_XYZ=\"0.0 0.0 0.0\"
export HUSKY_LASER_3D_RPY=\"0.0 0.0 0.0\" \# 2. CAMERA RealSense export
HUSKY_URDF_EXTRAS=\$HOME/Desktop/realsense.urdf.xacro

5\. Copy Project Directories Copy the husky_navigation and
husky_perception directories to your workspace: cp -r husky_navigation
husky_perception \~/catkin_ws/src/ cd \~/catkin_ws catkin_make

6\. Launch the Simulation Environment Deploy the robot in the Gazebo
world: roslaunch cpr_inspection_gazebo inspection_world.launch
platform:=husky

7\. Launch the CNN Detection Node rosrun husky_perception
cnn_detector.py

8\. Launch the Obstacle Segmentation Node rosrun husky_perception
obstacle_segmentation.py \_visualize:=false

9\. Launch the Navigation and Fusion Nodes roslaunch husky_navigation
navigation.launch

10\. Adjust Parameters Live Install and run the reconfigure tool: sudo
apt-get install ros-noetic-rqt-reconfigure rosrun rqt_reconfigure
rqt_reconfigure - From the GUI, select /semantic_fusion_node. - Adjust
semantic_fusion_node parameters (e.g., thresholds, rays per priority)
without restarting the node.

Running Experiments Once the system is running, execute experiments
using the experiment_runner.py script with the following format: rosrun
husky_navigation experiment_runner.py \[ON/OFF\] \[obstacle_name\]
\[test_number\] - ON: Enables the perception system. - OFF: Disables the
perception system. - obstacle_name: Name of the obstacle to avoid (e.g.,
person, backpack, chair). - test_number: Test number (e.g., 01, 05).

Example: rosrun husky_navigation experiment_runner.py OFF person 05 This
runs test 5 with the perception system off, avoiding a person obstacle.
Each experiment generates a .bag file in \~/experiment_bags for later
analysis.

Analyzing Data Extract Data from Rosbags Run the analysis script to
process .bag files: python3 analyze_experiments.py \~/experiment_bags
\--salida \~/resultados_analisis This generates an Excel file
(experimental_results.xlsx) in the specified output directory for
further analysis.

Generate Graphs Run the script to create summary figures (e.g., Figures
10, 11, 12 from the paper): python3 generate_graphs_results.py Output
figures are saved in the figures folder.

Run Statistical Analysis Execute the following scripts to generate
tables from the paper:

For Table 2 (assumption checks): python3 checking_assumptions_anova.py

For Table 4 (ANOVA results): python3 anova.py

For Tables 5 and 6 (Tukey HSD results): python3 Tukey.py

Repository Structure /husky_perception/cfg/: Dynamic reconfiguration
files (e.g., SemanticFusionConfig.cfg.py). /husky_perception/scripts/:
Perception nodes (e.g., cnn_detector.py, obstacle_segmentation.py,
lidar_cnn_semantic_integration.py). /husky_navigation/config/:
Navigation configuration files (e.g., costmap_common_params.yaml,
global_costmap_params.yaml, teb_local_planner_params.yaml).
/husky_navigation/launch/: Launch files (e.g., navigation.launch).
/husky_navigation/scripts/: Navigation support scripts (e.g.,
clear_costmap_node.py, experiment_runner.py). /analysis/: Data analysis
scripts (e.g., analyze_experiments.py, anova.py,
checking_assumptions_anova.py, generate_graphs_results.py, Tukey.py).
/figures/: Generated plots (e.g., TotalEffort_summary_barplot.png,
trajectory_run\_\*.png). /data/: Experimental data (e.g.,
experimental_results.xlsx). /: Root files (this README.md,
realsense.urdf.xacro).

Data Availability The synthetic data presented in this study are
available on FigShare\[\](https://doi.org/10.6084/m9.figshare.29852780)
in CSV format, with an optional Parquet version. Selected scripts and
figures are available in this repository.

License This project is licensed under the MIT License - see the LICENSE
file for details.

Acknowledgments This work was supported by the Faculty of Engineering of
the University of Santiago of Chile, Chile.

Contact For questions, please contact Claudio Urrea at
claudio.urrea@usach.cl.
