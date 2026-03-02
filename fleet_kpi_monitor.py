#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from fleet_interfaces.msg import TransportJob, RobotStatus

import pandas as pd
import matplotlib.pyplot as plt
import csv
import os
import time
from datetime import datetime


class FleetKPIMonitor(Node):

    def __init__(self):
        super().__init__('fleet_kpi_monitor')

        # -------------------------------
        # Data Storage
        # -------------------------------
        self.job_data = {}
        self.robot_busy_time = {}
        self.robot_task_count = {}
        self.sim_start_time = time.time()
        
        self.completed_jobs = []

        # Create timestamped filename (once per run)
        self.filename = f"fleet_kpi_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

        # 🔁 Autosave every 30 seconds
        self.autosave_timer = self.create_timer(30.0, self.save_csv)

        self.get_logger().info("📊 Fleet KPI Monitor Started")
        self.get_logger().info("⏳ Autosave every 30 seconds enabled")

        # -------------------------------
        # Subscribers
        # -------------------------------
        self.assignment_sub = self.create_subscription(
            TransportJob,
            '/fleet/assignments',
            self.assignment_callback,
            10
        )

        self.status_sub = self.create_subscription(
            RobotStatus,
            '/fleet/robot_status',
            self.status_callback,
            10
        )

        self.get_logger().info("📊 Fleet KPI Monitor Started")

    # ---------------------------------------------------
    # JOB ASSIGNED
    # ---------------------------------------------------
    
    def assignment_callback(self, msg):

        job_id = msg.job_id

        if job_id not in self.job_data:
            self.job_data[job_id] = {}

        self.job_data[job_id]["created"] = self.ros_time_to_sec(msg.created_time)
        self.job_data[job_id]["assigned"] = time.time()
        self.job_data[job_id]["robot"] = msg.assigned_robot

        if msg.assigned_robot not in self.robot_task_count:
            self.robot_task_count[msg.assigned_robot] = 0
            self.robot_busy_time[msg.assigned_robot] = 0

    # ---------------------------------------------------
    # ROBOT STATUS UPDATE
    # ---------------------------------------------------
    
    def save_csv(self):
        if len(self.completed_jobs) == 0:
            self.get_logger().warning("⚠ No completed jobs yet. Skipping CSV save.")
            return

        df = pd.DataFrame(self.completed_jobs)
        df.to_csv(self.filename, index=False)

        self.get_logger().info(f"💾 KPI data saved to {self.filename}")
    
    def status_callback(self, msg):

        robot = msg.robot_name
        state = msg.state
        job_id = msg.current_job_id

        if state == "BUSY":
            self.robot_busy_time[robot] -= time.time()

        elif state == "IDLE":
            self.robot_busy_time[robot] += time.time()
            self.robot_task_count[robot] += 1

            if job_id in self.job_data:
                self.job_data[job_id]["completed"] = time.time()

                self.compute_job_metrics(job_id)

    # ---------------------------------------------------
    # COMPUTE JOB METRICS
    # ---------------------------------------------------
    def compute_job_metrics(self, job_id):

        job = self.job_data[job_id]

        created = job.get("created", 0)
        assigned = job.get("assigned", 0)
        completed = job.get("completed", 0)

        waiting_time = assigned - created
        execution_time = completed - assigned
        total_time = completed - created

        job["waiting_time"] = waiting_time
        job["execution_time"] = execution_time
        job["total_time"] = total_time

        self.completed_jobs.append({
            "job_id": job_id,
            "robot": job.get("robot", ""),
            "waiting_time": waiting_time,
            "execution_time": execution_time,
            "total_time": total_time,
            "timestamp": completed
        })

        self.get_logger().info(
            f"📦 Job {job_id} | Waiting: {waiting_time:.2f}s | "
            f"Execution: {execution_time:.2f}s | Total: {total_time:.2f}s"
        )

    # ---------------------------------------------------
    # UTILITY
    # ---------------------------------------------------
    def ros_time_to_sec(self, ros_time):
        return ros_time.sec + ros_time.nanosec * 1e-9

    # ---------------------------------------------------
    # ON SHUTDOWN → SAVE & PLOT
    # ---------------------------------------------------
    def destroy_node(self):

        self.get_logger().info("📊 Generating KPI Report...")

        df = pd.DataFrame.from_dict(self.job_data, orient="index")

        # Save CSV
        csv_path = os.path.join(os.getcwd(), "fleet_kpi_log.csv")
        df.to_csv(csv_path)

        print("\n================ KPI SUMMARY ================")
        print(df.describe())

        sim_time = time.time() - self.sim_start_time
        total_jobs = len(df)

        if total_jobs > 0:
            avg_total_time = df["total_time"].mean()
            avg_waiting_time = df["waiting_time"].mean()
            throughput = total_jobs / sim_time

            print(f"Simulation Time: {sim_time:.2f}s")
            print(f"Total Jobs: {total_jobs}")
            print(f"Average Task Time: {avg_total_time:.2f}s")
            print(f"Average Waiting Time: {avg_waiting_time:.2f}s")
            print(f"Throughput: {throughput:.4f} jobs/sec")

        # -------------------------------
        # Robot Utilization
        # -------------------------------
        print("\nRobot Utilization:")
        for robot, busy_time in self.robot_busy_time.items():
            utilization = (busy_time / sim_time) * 100
            print(f"{robot}: {utilization:.2f}%")

        # -------------------------------
        # Plot 1: Total Task Time
        # -------------------------------
        if total_jobs > 0:
            plt.figure()
            plt.plot(df["total_time"].values)
            plt.title("Task Total Time per Job")
            plt.xlabel("Job Index")
            plt.ylabel("Time (s)")
            plt.show()

            # Plot 2: Waiting Time
            plt.figure()
            plt.plot(df["waiting_time"].values)
            plt.title("Waiting Time per Job")
            plt.xlabel("Job Index")
            plt.ylabel("Time (s)")
            plt.show()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = FleetKPIMonitor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("🛑 Ctrl+C detected. Shutting down KPI Monitor...")

    finally:
        # 🔥 THIS IS THE IMPORTANT PART
        node.save_csv()
        node.print_summary()

        node.destroy_node()
        rclpy.shutdown()
