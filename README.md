# Unmanned Ground Vehicle (UGV) Project by Kolaydi Team

Welcome to the repository of the Kolaydi Team's UGV project, designed for the METU EEE Capstone project fair. This project aims to develop a cost-effective unmanned ground vehicle capable of 3D indoor mapping and autonomous navigation, solely utilizing a 2D LiDAR sensor.

## Table of Contents

- [Introduction](#introduction)
- [Team Members](#team-members)
- [Project Overview](#project-overview)
- [Setup and Installation](#setup-and-installation)
- [Usage Guide](#usage-guide)
- [Additional Resources](#additional-resources)

## Introduction

This repository contains all the necessary files and instructions to set up and deploy our autonomous UGV system. The UGV is designed to perform tasks in indoor environments, such as mapping and autonomous navigation, while avoiding both static and dynamic obstacles.

## Team Members

![Kolaydi Team Logo](logo_2.png)

Our team consists of five dedicated Electrical-Electronics Engineering students from Middle East Technical University (METU):

- M. Taha Bekar
- Enfal Cabuk
- Yunus T. Erzurumlu
- Burcu Şakır
- Duru Uyar

## Project Overview

The project focuses on the following functionalities:

- **3D Mapping**: Generating a three-dimensional map of an environment using a 2D LiDAR sensor.
- **Manual Movement**: Human-guided movement during the mapping phase.
- **Region Labeling**: Manual labeling of different areas within the mapped environment.
- **Autonomous Navigation**: Ability to autonomously navigate to a designated target region within the map.
- **Obstacle Avoidance**: Identification and avoidance of both static objects and dynamic obstacles, including hazardous negative obstacles like stairways.

## Setup and Installation

To get started with the project, follow these installation steps:

1. Create a new directory for the project:
   ```bash
   mkdir -p ~/git/kolaydi
   cd ~/git/kolaydi
   ```
2. Clone the repository:

   ```bash
   git clone https://github.com/yunusstalha/kolaydi.git
   ```
3. Build the project using Catkin:
   ```bash
   catkin build # or catkin_make
   source devel/setup.bash
   ```
4. Ensure all terminals are set up correctly:

   ```bash
   source devel/setup.bash
   ```

## Usage Guide

1. Launch the simulation environment:
   ```bash
   roslaunch ugv_sim ugv_gazebo.launch
   ```
2. Open the visualization tool:
   ```bash
   rviz
   ```
   Load the configuration from the ugv_rviz.rviz file through the "File" menu.
4. For 3D mapping, run the following:
   ```bash
   roslaunch ugv_sim mapping.launch
   ```

5. To control the LiDAR rotation for 3D data acquisition:
   ```bash
   rqt
   ```
   Navigate to Plugins -> Topics -> Message Publisher, add the topic servo_controller/command, and set the data to 0.1 * sin(0.5 * i).

## Additional Resources
For more detailed information on system components and troubleshooting, please refer to the project wiki or contact any team member for support.

Enjoy exploring and developing with our autonomous UGV system!







