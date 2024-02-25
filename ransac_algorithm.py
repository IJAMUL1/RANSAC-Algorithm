import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import math
import random
import numpy as np

def plane_equation_from_points(point1, point2, point3):
    # Calculate vectors on the plane
    v1 = np.array(point2) - np.array(point1)
    v2 = np.array(point3) - np.array(point1)
    
    # Calculate the cross product to get the normal vector
    normal_vector = np.cross(v1, v2)

    # Check if the cross product is a zero vector (collinear points)
    if np.all(normal_vector == 0):
        return "Points are collinear; no unique plane exists."

    # Find the constant term 'd' in the plane equation
    d = -np.dot(normal_vector, np.array(point1))
    

    # Construct the plane equation
    a, b, c = normal_vector
         #f"{a:.4f}x + {b:.4f}y + {c:.4f}z = {d:.4f}"

    return a,b,c,d

def distance(A, B, C, D, x0, y0, z0):
  numerator = abs(A * x0 + B * y0 + C * z0 + D)
  denominator = math.sqrt(A**2 + B**2 + C**2)
  return numerator / denominator

def fit_plane_adaptive(pcd, sample_size, p):
    N = float('inf')
    sample_count = 0
    distance_threshold = 0.025
    best_plane = None
    total_inliers = 0
    total_inlier_data_index = []
    
    while N > sample_count:
        # Choose a sample
        sampled_points = np.asarray(pcd.points)
        random_indices = np.random.choice(len(sampled_points), 3, replace=False)
        random_points = sampled_points[random_indices] 
        a, b, c, d = plane_equation_from_points(random_points[0], random_points[1], random_points[2])
        inliers, inlier_data_index = is_inlier(a, b, c, d, sampled_points, distance_threshold)
                
        if inliers > total_inliers:
            total_inliers = inliers
            total_inlier_data_index = inlier_data_index
            best_plane = [a, b, c, d]
          
        total_points = sampled_points.shape[0]
                        
        # Calculate e
        e = 1 - inliers / total_points
      
        # Update N using the formula with p = 0.99
        N = np.ceil(math.log(1 - p) / math.log(1 - (1 - e) ** sample_size)).astype(int)
        
        # Increment sample_count
        print(sample_count)
        sample_count += 1
    
    # Convert inlier indices to indices in the original point cloud
    inlier_indices_original = np.array(list(range(len(pcd.points))))[total_inlier_data_index]

    # Color the inliers red in the original point cloud
    colors = np.asarray(pcd.colors)
    colors[inlier_indices_original] = [1.0, 0.0, 0.0]
    pcd.colors = o3d.utility.Vector3dVector(colors)
    # Visualize the point cloud with colored inliers
    o3d.visualization.draw_geometries([pcd],
                                  zoom=1,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])


def is_inlier(a, b, c, d, sampled_points, distance_threshold):
    # Count the number of points within the threshold
    inlier_count = 0
    inlier_indices = []

    for index, point in enumerate(sampled_points):
        if distance(a, b, c, d, point[0], point[1], point[2]) <= distance_threshold:
            inlier_count += 1
            inlier_indices.append(index)

    return inlier_count, inlier_indices




# Read the demo point cloud
pcd_point_cloud = o3d.data.PCDPointCloud()
pcd = o3d.io.read_point_cloud(pcd_point_cloud.path)
sample_size = 3
p = 0.99

# # Visualize the original point cloud
# o3d.visualization.draw_geometries([pcd],
#                                   zoom=1,
#                                   front=[0.4257, -0.2125, -0.8795],
#                                   lookat=[2.6172, 2.0475, 1.532],
#                                   up=[-0.0694, -0.9768, 0.2024])

best_plane, inlier_data_index = fit_plane_adaptive(pcd, sample_size, p)

