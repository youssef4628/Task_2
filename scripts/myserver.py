#!/usr/bin/env python3

import rospy
from rover_control_system.srv import checker, checkerResponse
import random
import math


def laser_inc_calc(req, range_max):
    distance = range_max - req.obstacle_data.range_min
    angle = math.atan(req.y / req.x)
    x_inc = distance * math.cos(angle)
    y_inc = distance * math.sin(angle)
    
    # Round after calculation to maintain precision
    return (round(x_inc, 5), round(y_inc, 5))


def obstacle_position():
    x_cor = random.uniform(1.00,10.00)
    y_cor = random.uniform(1.00,10.00)
    
    # Round after calculation to maintain precision
    return (round(x_cor, 5), round(y_cor, 5))


def compare_slopes(slope1, slope2, tolerance=1e-5):
    # Compare two slopes with a tolerance
    return abs(slope1 - slope2) < tolerance


def handle_response(req):
    current = (0.0, 0.0)
    destination = (req.x, req.y)
    obstacle = obstacle_position()
    laser_inc = laser_inc_calc(req, range_max=random.uniform(5.00, 6.00))
    laser_pos = (current[0] + laser_inc[0], current[1] + laser_inc[1])

    while current[0] <= destination[0] and current[1] <= destination[1]:
        print(current)

        # Calculate slopes before rounding
        obstacle_slope = obstacle[1] / obstacle[0] if obstacle[0] != 0 else float('inf')
        laser_slope = laser_pos[1] / laser_pos[0] if laser_pos[0] != 0 else float('inf')

        # Compare slopes using the tolerance function
        if (current[0] < obstacle[0] <= laser_pos[0] and 
            current[1] < obstacle[1] <= laser_pos[1] and 
            compare_slopes(obstacle_slope, laser_slope)):
            if destination[0] < obstacle[0]:
                rospy.loginfo("obstacle in the range but I can reach the destination")
                print("\n\n\n\n")
                return checkerResponse(task_completed=True, obstacle_position=list(obstacle), destination=list(destination))
            else:
                rospy.loginfo("obstacle in the range so I can (not) reach the destination")
                print("\n\n\n\n")
                return checkerResponse(task_completed=False, obstacle_position=list(obstacle), destination=list(destination))
        else:
            if destination[0] <= laser_pos[0] and destination[1] <= laser_pos[1]:
                rospy.loginfo("clear, and I reached the destination")
                print("\n\n\n\n")
                return checkerResponse(task_completed=True, obstacle_position=list(obstacle), destination=list(destination))
            else:
                rospy.loginfo("clear, but I'm still trying ")
                current = laser_pos
                laser_pos = (current[0] + laser_inc[0], current[1] + laser_inc[1])

    rospy.loginfo("Could not reach the destination")
    return checkerResponse(
        task_completed=False, 
        obstacle_position=list(obstacle), 
        destination=list(destination)
    )


if __name__ == "__main__":
    rospy.init_node("server")
    rospy.loginfo("Server node has been initialized")
    server = rospy.Service("check_distination", checker, handle_response)
    rospy.spin()
