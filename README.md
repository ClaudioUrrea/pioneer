Perception \& Navigation System for Semantic Priority Navigation in
Mining Robots

This repository contains the code, configurations, and resources
supporting the research paper "SemanticPriority Navigation for
EnergyAware Mining Robots in ROS" (to be published). It implements a
prioritized semantic navigation framework for Husky robots in mining
environments using ROS and Gazebo simulations, focusing on perception
(CNN detection, LiDAR segmentation, semantic fusion) and navigation
(costmaps, local planners, experiment execution).

Prerequisites: Install ROS Noetic Follow the official installation
guide: ROS Noetic Installation on Ubuntu.

System Requirements: Ubuntu 20.04 (recommended for ROS Noetic) Python
3.x with required libraries (see requirements.txt) ROS Noetic
desktop-full installation

Setup Instructions: 

1\. Create a Catkin Workspace 1.1. Create the
Workspace Directory mkdir -p ~/catkin\_ws/src cd ~/catkin\_ws/src 1.2.
Initialize the Catkin Workspace catkin\_init\_workspace 1.3. Build the
Workspace cd ~/catkin\_ws catkin\_make 1.4. Source the Workspace source
~/catkin\_ws/devel/setup.bash

2\. Install the Simulation Package sudo apt-get install
ros-noetic-husky-simulator

3\. Download Clearpath Gazebo Worlds: Clone the repository into your ROS
workspace: cd ~/catkin\_ws/src git clone
https://github.com/clearpathrobotics/cpr\_gazebo.git

4\. Export Sensor Parameters Enable 3D LIDAR (Velodyne VLP-16) and
RealSense camera: # 1. LIDAR 3D (Velodyne VLP-16) export
HUSKY\_LASER\_3D\_ENABLED=1 export HUSKY\_LASER\_3D\_XYZ="0.0 0.0 0.0"
export HUSKY\_LASER\_3D\_RPY="0.0 0.0 0.0" # 2. CAMERA RealSense export
HUSKY\_URDF\_EXTRAS=$HOME/Desktop/realsense.urdf.xacro

5\. Copy Project Directories Copy the husky\_navigation and
husky\_perception directories to your workspace: cp -r husky\_navigation
husky\_perception ~/catkin\_ws/src/ cd ~/catkin\_ws catkin\_make

6\. Launch the Simulation Environment Deploy the robot in the Gazebo
world: roslaunch cpr\_inspection\_gazebo inspection\_world.launch
platform:=husky

7\. Launch the CNN Detection Node rosrun husky\_perception
cnn\_detector.py

8\. Launch the Obstacle Segmentation Node rosrun husky\_perception
obstacle\_segmentation.py \_visualize:=false

9\. Launch the Navigation and Fusion Nodes roslaunch husky\_navigation
navigation.launch

10\. Adjust Parameters Live Install and run the reconfigure tool: sudo
apt-get install ros-noetic-rqt-reconfigure rosrun rqt\_reconfigure
rqt\_reconfigure - From the GUI, select /semantic\_fusion\_node. - Adjust
semantic\_fusion\_node parameters (e.g., thresholds, rays per priority)
without restarting the node.

Running Experiments Once the system is running, execute experiments
using the experiment\_runner.py script with the following format: rosrun
husky\_navigation experiment\_runner.py \[ON/OFF] \[obstacle\_name]
\[test\_number] - ON: Enables the perception system. - OFF: Disables the
perception system. - obstacle\_name: Name of the obstacle to avoid (e.g.,
person, backpack, chair). - test\_number: Test number (e.g., 01, 05).

Example: rosrun husky\_navigation experiment\_runner.py OFF person 05 This
runs test 5 with the perception system off, avoiding a person obstacle.
Each experiment generates a .bag file in ~/experiment\_bags for later
analysis.

Analyzing Data Extract Data from Rosbags Run the analysis script to
process .bag files: python3 analyze\_experiments.py ~/experiment\_bags
--salida ~/resultados\_analisis This generates an Excel file
(experimental\_results.xlsx) in the specified output directory for
further analysis.

Generate Graphs Run the script to create summary figures (e.g., Figures
10, 11, 12 from the paper): python3 generate\_graphs\_results.py Output
figures are saved in the figures folder.

Run Statistical Analysis Execute the following scripts to generate
tables from the paper:

For Table 2 (assumption checks): python3 checking\_assumptions\_anova.py

For Table 4 (ANOVA results): python3 anova.py

For Tables 5 and 6 (Tukey HSD results): python3 Tukey.py

Repository Structure /husky\_perception/cfg/: Dynamic reconfiguration
files (e.g., SemanticFusionConfig.cfg.py). /husky\_perception/scripts/:
Perception nodes (e.g., cnn\_detector.py, obstacle\_segmentation.py,
lidar\_cnn\_semantic\_integration.py). /husky\_navigation/config/:
Navigation configuration files (e.g., costmap\_common\_params.yaml,
global\_costmap\_params.yaml, teb\_local\_planner\_params.yaml).
/husky\_navigation/launch/: Launch files (e.g., navigation.launch).
/husky\_navigation/scripts/: Navigation support scripts (e.g.,
clear\_costmap\_node.py, experiment\_runner.py). /analysis/: Data analysis
scripts (e.g., analyze\_experiments.py, anova.py,
checking\_assumptions\_anova.py, generate\_graphs\_results.py, Tukey.py).
/figures/: Generated plots (e.g., TotalEffort\_summary\_barplot.png,
trajectory\_run\_\*.png). /data/: Experimental data (e.g.,
experimental\_results.xlsx). /: Root files (this README.md,
realsense.urdf.xacro).

Data Availability The synthetic data presented in this study are
available on FigShare\[](https://doi.org/10.6084/m9.figshare.29852780)
in CSV format, with an optional Parquet version. Selected scripts and
figures are available in this repository.

License This project is licensed under the MIT License - see the LICENSE
file for details.

Acknowledgments This work was supported by the Faculty of Engineering of
the University of Santiago of Chile, Chile.

Contact For questions, please contact Claudio Urrea at
claudio.urrea@usach.cl.

