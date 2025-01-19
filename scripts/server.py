#!/usr/bin/env python3

import rospy
from rover_control_system.srv import checker, checkerResponse
import random
import math

def obstacle_position():
    # Hardcoded obstacle position (3.0, 3.0)
    x = random.uniform(1.0,10.0)
    y = random.uniform(1.0,10.0)
    return x, y

def laser(req):
    # Compute laser position based on the destination and range
    theta = math.atan(req.y / req.x)
    range_max = random.uniform(4.0,5.0)
    laser_distance = (range_max - req.obstacle_data.range_min)
    y_laser = (math.sin(theta)) * laser_distance
    x_laser = (math.cos(theta)) * laser_distance
    return x_laser, y_laser

def handle_response(req):
    # Initial laser scan and obstacle position
    laserscan = laser(req)
    laser_pos = laser(req)
    obstacle = obstacle_position()
    current = (0, 0)  # Starting position
    destination = (req.x, req.y)  # Destination from request
    epsilon = 1e-6  # Small tolerance for floating-point comparisons

    while current[0] < destination[0] and current[1] < destination[1]:
        # Log the current, laser position, and obstacle
        rospy.loginfo(f"Current: {current}, Laser Position: {laser_pos}, Obstacle: {obstacle}")

        # Check if obstacle is in the path using intermediate laser positions
        for i in range(1, 11):  # Subdivide the laser scan into 10 steps
            intermediate = (
                current[0] + i * laserscan[0] / 10,
                current[1] + i * laserscan[1] / 10
            )
            # Check if obstacle is between the current and intermediate laser positions
            if (current[0] <= obstacle[0] <= intermediate[0] and
                current[1] <= obstacle[1] <= intermediate[1] and
                abs((obstacle[0] - current[0]) * (intermediate[1] - current[1]) -
                    (obstacle[1] - current[1]) * (intermediate[0] - current[0])) < epsilon):
                rospy.loginfo("Obstacle detected in intermediate step!")
                return checkerResponse(
                    task_completed=False,
                    obstacle_position=list(obstacle),
                    destination=list(destination)
                )

        # Update the current position to laser position and move to next laser position
        current = laser_pos
        laser_pos = (current[0] + laserscan[0], current[1] + laserscan[1])

    # If no obstacle was found, path is clear
    rospy.loginfo("Path is clear!")
    return checkerResponse(
        task_completed=True,
        obstacle_position=list(obstacle),
        destination=list(destination)
    )

if __name__ == "__main__":
    rospy.init_node("server")
    rospy.loginfo("Server node has been initialized")
    server = rospy.Service("check_distination", checker, handle_response)
    rospy.spin()
