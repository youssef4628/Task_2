#!/usr/bin/env python3 
import rospy
from rover_control_system.srv import checker, checkerRequest
import random
import matplotlib.pyplot as plt


if __name__=="__main__":
    rospy.init_node("Client_interface")
    service=rospy.ServiceProxy("check_distination",checker)
    service.wait_for_service()
    request=checkerRequest()
    request.x=random.uniform(5.0,10.0)
    request.y=random.uniform(5.0,10.0)
    request.obstacle_data.range_min=random.uniform(0.10,0.15)
    response=service(request)
    print(response.task_completed)
    
    obstacle_pos = response.obstacle_position  # List [x, y] for obstacle
    destination_pos = response.destination  # List [x, y] for destination

    # Print response for debugging
    print(f"Obstacle Position: {obstacle_pos}")
    print(f"Destination Position: {destination_pos}")

    # Visualization with Matplotlib
    plt.figure(figsize=(8, 6))
    plt.grid(True)

    # Plot the rover's starting point
    plt.plot(0, 0, "go", label="Rover Start (0,0)", markersize=10)

    # Plot the obstacle
    plt.plot(obstacle_pos[0], obstacle_pos[1], "rx", label="Obstacle", markersize=12)

    # Plot the destination
    plt.plot(destination_pos[0], destination_pos[1], "bo", label="Destination", markersize=10)

    # Annotate the positions
    plt.text(0, 0, " Start", fontsize=10)
    plt.text(obstacle_pos[0], obstacle_pos[1], f" Obstacle ({obstacle_pos[0]:.2f}, {obstacle_pos[1]:.2f})", fontsize=10)
    plt.text(destination_pos[0], destination_pos[1], f" Destination ({destination_pos[0]:.2f}, {destination_pos[1]:.2f})", fontsize=10)

    # Configure the plot
    plt.title("Rover Navigation Visualization")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.legend()
    plt.show()
    
    
    
    