#!/usr/bin/env python3
import rospy
import subprocess
import os
import sys
import time
import random
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionResult

class ExperimentRunner:
    def __init__(self, condition, obstacle_class, run_number):
        rospy.init_node("experiment_runner", anonymous=True)

        self.condition = condition.upper()
        self.obstacle_class = obstacle_class.lower()
        self.run_number = int(run_number)
        self.goal_reached = False
        self.waiting_for_goal = False

        self.bag_dir = os.path.expanduser("~/experiment_bags")
        os.makedirs(self.bag_dir, exist_ok=True)

        self.initial_pose = PoseWithCovarianceStamped()
        self.initial_pose.header.frame_id = "map"
        self.initial_pose.pose.pose.position.x = 0.0 + random.uniform(-0.05, 0.05)
        #self.initial_pose.pose.pose.position.x = 0.0
        self.initial_pose.pose.pose.position.y = 0.0
        self.initial_pose.pose.pose.orientation.w = 1.0
        self.initial_pose.pose.covariance = [0.0] * 36

        self.pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1, latch=True)

        rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.result_callback)

        self.topics_common = [
            "/odometry/filtered",
            "/tf",
            "/detections",
            "/move_base/status",
            "/husky_velocity_controller/cmd_vel"
        ]
        self.topics_baseline = ["/raw_scan"]
        self.topics_semantic = ["/raw_scan", "/semantic_scan"]

    def result_callback(self, msg):
        if not self.waiting_for_goal:
            return
        if msg.status.status == 3:
            self.goal_reached = True
            rospy.loginfo("✅ Goal alcanzado oficialmente (move_base/result)")

    def publish_initial_pose(self):
        rospy.loginfo("📍 Publicando posición inicial del robot...")
        for _ in range(5):
            self.initial_pose.header.stamp = rospy.Time.now()
            self.pose_pub.publish(self.initial_pose)
            rospy.sleep(0.2)

    def send_goal(self):
        rospy.loginfo("🎯 Publicando goal en /move_base_simple/goal...")

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = 8.1
        goal.pose.position.y = -0.093
        goal.pose.orientation.w = 1.0

        rospy.sleep(1.0)
        self.goal_pub.publish(goal)
        rospy.loginfo("🚩 Goal publicado.")

    def wait_until_goal_reached(self, timeout=90):
        rospy.loginfo("⏳ Esperando a que se alcance el goal...")
        self.waiting_for_goal = True
        start_time = rospy.Time.now()
        rate = rospy.Rate(2)

        while not rospy.is_shutdown():
            if self.goal_reached:
                self.waiting_for_goal = False
                return True
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logwarn("⏰ Timeout esperando al goal.")
                self.waiting_for_goal = False
                return False
            rate.sleep()

    def run_trial(self):
        rospy.loginfo(f"🚀 Iniciando prueba {self.run_number} - Condición: {self.condition} - Obstáculo: {self.obstacle_class}")
        self.publish_initial_pose()

        topics = self.topics_common + (self.topics_semantic if self.condition == "ON" else self.topics_baseline)
        bag_name = f"{self.condition}_{self.obstacle_class}_run_{self.run_number:02d}.bag"
        bag_path = os.path.join(self.bag_dir, bag_name)

        rospy.loginfo(f"📦 Grabando rosbag en: {bag_path}")
        rosbag_proc = subprocess.Popen(["rosbag", "record", "-O", bag_path] + topics)
        rospy.sleep(2.0)

        self.goal_reached = False
        self.send_goal()
        self.wait_until_goal_reached(timeout=120)
        rospy.loginfo("⌛ Esperando unos segundos adicionales para registrar odometría final...")
        rospy.sleep(3.0)  # <--- IMPORTANTE
        rosbag_proc.terminate()
        rosbag_proc.wait()
        rospy.loginfo("✅ Prueba finalizada y rosbag guardado.")

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Uso: rosrun husky_navigation experiment_runner.py [ON|OFF] [CLASE_OBJETO] [numero_de_prueba]")
        print("Ejemplo: rosrun husky_navigation experiment_runner.py ON persona 1")
        sys.exit(1)

    try:
        runner = ExperimentRunner(sys.argv[1], sys.argv[2], sys.argv[3])
        runner.run_trial()
    except rospy.ROSInterruptException:
        pass

