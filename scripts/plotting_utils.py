import open3d as o3d
import numpy as np

def create_camera_visualization_mesh(wTc, color=[1, 0, 0], scale=1.0):
    # Camera frustum vertices in camera coordinate system
    frustum_vertices = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 1.0, 1.0],
        [-1.0, 1.0, 1.0],
        [-1.0, -1.0, 1.0],
        [1.0, -1.0, 1.0],
    ])

    # Apply scale to the frustum vertices
    frustum_vertices *= scale

    # Apply extrinsic matrix to transform to world coordinates
    frustum_vertices_world = (
        wTc[:3, :3] @ frustum_vertices.T).T + wTc[:3, 3]

    # Create a TriangleMesh for the camera frustum
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(frustum_vertices_world)
    mesh.triangles = o3d.utility.Vector3iVector(
        [[0, 1, 2], [0, 2, 3], [0, 3, 4], [0, 4, 1]])

    # Set the color of the camera frustum
    # color = [1.0, 0.0, 0.0]  # Red color
    mesh.paint_uniform_color(color)
    # return mesh

    line_length = scale*2.
    line_points = np.array([
        [0.0, 0.0, 0.0],
        [0.0, 0.0, line_length]
    ])
    # Apply extrinsic matrix to transform to world coordinates
    line_points_world = (
        wTc[:3, :3] @ line_points.T).T + wTc[:3, 3]

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(line_points_world)
    line_set.lines = o3d.utility.Vector2iVector([[0, 1]])
    # line_set.colors = o3d.utility.Vector3dVector([[0, 0, 1]])
    line_set.colors = o3d.utility.Vector3dVector([color])
    line_set.lines = o3d.utility.Vector2iVector([[0, 1]])
    # line_set.line_colors = o3d.utility.Vector3dVector([color])
    # line_set.line_width = 0.1 # line_thickness

    # combined_geometry = o3d.geometry.Geometry()
    # combined_geometry += mesh
    # combined_geometry += line_set

    return [mesh, line_set]

