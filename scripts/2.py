import open3d as o3d
import numpy as np

from common_colors import color_map_normalized
from plotting_utils import create_camera_visualization_mesh, add_pose, add_pts

from data_utils import load_mat, comb, file_to_str


BASE = "../build/data2"
# BASE="../data/SFMedu/result/selected_solution"

wTc1 = load_mat(file_to_str(BASE+"/wTc1.txt"), '\n', ' ')
wTc2 = load_mat(file_to_str(BASE+"/wTc2.txt"), '\n', ' ')
pts = load_mat(file_to_str(BASE+"/pts3d_w.txt"), '\n', ' ')

#print( pts )
norms = np.linalg.norm(pts, axis=1)
pts = pts[norms <= 10]


all_geoms = []
all_geoms.append(
    o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0))

if True: 
    add_pose(np.eye(4, 4), all_geoms, 'gray')
    add_pose(wTc2, all_geoms, 'teal')
    add_pts(pts, all_geoms, 'teal')

if False: 
    add_pose(np.eye(4, 4), all_geoms, 'gray')
    for colr, i in zip(['red', 'green', 'blue', 'cyan'], range(0, 4)):
        sol_wTc2 = load_mat(file_to_str(
            BASE+"/sol{}_wTc2.txt".format(i)), '\n', ' ')
        sol_pts = load_mat(file_to_str(
            BASE+"/sol{}_pts3d_w.txt".format(i)), '\n', ' ')

        norms = np.linalg.norm(sol_pts, axis=1)
        sol_pts = sol_pts[norms <= 10]
        
        print( colr )
        print( BASE+"/sol{}_wTc2.txt".format(i) )
        add_pose(sol_wTc2, all_geoms, colr)
        add_pts(sol_pts, all_geoms, colr)


o3d.visualization.draw_geometries(
    all_geoms, window_name="Camera Visualization")
