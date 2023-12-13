import open3d as o3d
import numpy as np

def grid_x(): 
    points = [] 
    lines = []

    for n in range( -100, 100, 5 ): 

        points.append( [-100, n, 0] )
        points.append( [100, n, 0] )

        i = len(points)
        lines.append( [i-1,i-2])

        points.append( [n,-100, 0] )
        points.append( [n,100, 0] )

        i = len(points)
        lines.append( [i-1,i-2])
    
    return points, lines 



# Create a point cloud with two points representing the endpoints of the line
points = np.array([[-10.0, 0.0, 0.0],  # Endpoint 1
                   [10.0, 0.0, 0.0]])  # Endpoint 2
lines = [[0, 1]]  # Line connecting points 0 and 1
points, lines = grid_x()
print( lines )

point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(points)

line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector(points)
line_set.lines = o3d.utility.Vector2iVector(lines)

# Visualize the point cloud
o3d.visualization.draw_geometries([point_cloud, line_set])