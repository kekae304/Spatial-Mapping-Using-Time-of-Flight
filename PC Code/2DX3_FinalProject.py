import serial
import math
import numpy as np
import open3d as o3d
import time 

s = serial.Serial('COM4', 115200, timeout=30)

s.reset_output_buffer()   # Clear the output buffer

s.reset_input_buffer()    # Clear the input buffer

points = []  # Initalize a list to store the XYZ coordinates

z_val = 0  # Initialize Z value

step = 1000  # Z increment for each scan set

num_scans = 3  # Number of scans

num_points_per_scan = 32  # Points per 360-degree scan

angle_increment = 11.25  # Angle increment in degrees to achieve 32 points -> 360 / 32 = 11.25 deg

print("Scanning initiated...") # Print statement

for _ in range(num_scans): # Iterate through each scan

    for i in range(num_points_per_scan): # Iterate through each point in the scan

        distance_str = s.readline().decode().strip() # Read the distance from serial port

        print(f"Measurement {i}: {distance_str} mm") # Print statement 

        # Validation Process: 

        try:

            distance = int(distance_str)

        except ValueError:

            print(f"Invalid distance value: {distance_str}")

            continue  # Skip non-numeric data

        angle = math.radians(i * angle_increment) # Find Angle

        x = distance * math.cos(angle) # Find X 

        y = distance * math.sin(angle) # Find Y 

        points.append([x, y, z_val]) # Store the XYZ coordinates

    z_val += step  # Increment Z for the next scan set

    time.sleep(1)  # Pause for a second to allow for processing

s.close() # Close the serial

pcd = o3d.geometry.PointCloud() # Convert XYZ points to the Open3D point cloud

pcd.points = o3d.utility.Vector3dVector(np.array(points))

lines = [] # Creating a mesh-like structure

for k in range(num_scans):

    base = k * num_points_per_scan

    for i in range(num_points_per_scan):

        lines.append([base + i, base + (i + 1) % num_points_per_scan]) # Connecting points in the same layer 

        if k < num_scans - 1:

            lines.append([base + i, base + i + num_points_per_scan]) # Connceting the points to the next layer 

line_set = o3d.geometry.LineSet( # Creating a line set for visualization

    points=o3d.utility.Vector3dVector(np.array(points)),

    lines=o3d.utility.Vector2iVector(np.array(lines))

)

o3d.visualization.draw_geometries([pcd, line_set]) # Visualize the 3D map 