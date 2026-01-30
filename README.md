#ROS-Based 2D SLAM and Indoor Mapping Using Rawseeds Laser Data

# CPE 521 – SLAM Final Project

This repository contains the final project for **CPE 521: Autonomous Mobile Robotic Systems**.  
The objective of this project is to demonstrate a **Simultaneous Localization and Mapping (SLAM)** algorithm using real-world sensor data within the ROS framework.

-----------------------------------------------------

## SLAM Algorithm
- **Algorithm Used:** Particle Filter–based SLAM (GMapping)
- GMapping uses a Rao-Blackwellized particle filter to estimate the robot trajectory while incrementally building an occupancy grid map.
- The algorithm processes laser scan data and odometry information to generate a 2D map of the environment.

-----------------------------------------------------

## Dataset
- **Source:** Rawseeds Project  
- **Dataset:** Bicocca_2009-02-25b (Laser SLAM benchmark)
- The dataset contains real sensor logs including laser scans and odometry recorded in an indoor environment.

> **Note:** Due to GitHub file size limitations, the raw dataset and ROS bag files are not included in this repository.  
> The dataset was converted locally into ROS bag format for SLAM evaluation.

-----------------------------------------------------

## Implementation Details
- Raw CSV sensor logs were converted into ROS bag files using a custom Python script.
- The ROS bag was replayed using simulated time.
- The GMapping node subscribed to the laser scan topic to perform online SLAM.
- RViz was used to visualize laser scans, robot trajectory, TF frames, and the generated occupancy grid map.

-----------------------------------------------------

## Visualization and Results
- The SLAM process was visualized in real time using RViz.
- An occupancy grid map of the environment was successfully generated.
- Map quality improved progressively as more sensor data was processed.

Screenshots of the mapping process and final results are included in the final report.

## SLAM Results

The figure below shows the occupancy grid map generated using the GMapping SLAM algorithm
from the Rawseeds Bicocca dataset.

![SLAM Map Result](results/tuned_front.png)

**
-----------------------------------------------------

## Tools and Software
- ROS (Noetic)
- GMapping
- RViz
- Python (for data conversion)

-----------------------------------------------------

## Files Included
- `convert_csv_to_bag.py` – Python script used to convert CSV sensor logs into ROS bag format  
- `Final_Report.pdf` – Complete project report including methodology, results, and discussion

-----------------------------------------------------

## Author
Mohammad Althaf Syed  
