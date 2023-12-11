import open3d as o3d
import numpy as np

from common_colors import color_map_normalized
from plotting_utils import create_camera_visualization_mesh

from data_utils import load_mat, comb, file_to_str



BASE="../build/data"

wTc = load_mat( file_to_str( BASE+"/wTc_3.txt" ), '\n', ' ' )
pts = load_mat( file_to_str( BASE+"/pts3d_3.txt" ), '\n', ' ' )

print( pts )
norms = np.linalg.norm(pts, axis=1)
pts = pts[norms <= 100]




all_geoms = []
all_geoms.append(
    o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0))


all_geoms += create_camera_visualization_mesh(
    np.eye(4), color=color_map_normalized['gray'],  scale=0.1)
all_geoms += create_camera_visualization_mesh(
    wTc, color=color_map_normalized['green'],  scale=0.1)

#pts = np.random.rand(100, 3)
single_color = [1.0, 0.0, 0.0]
point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(pts[:,0:3]  )
point_cloud.colors = o3d.utility.Vector3dVector(np.tile(single_color, (pts.shape[0], 1)))
all_geoms.append( point_cloud )

o3d.visualization.draw_geometries(
    all_geoms, window_name="Camera Visualization")
