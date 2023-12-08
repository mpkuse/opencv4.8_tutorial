import open3d as o3d
import numpy as np

from common_colors import color_map_normalized


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


def load_mat(R_txt):
    xrows = R_txt.split(";")

    R = []
    for r in xrows:
        nums = r.split(",")
        R_row = []
        for n in nums:
            n = float(n.strip())
            # print( n )
            R_row.append(n)
        R.append(R_row)
        # print( "--")

    return np.array(R)


def comb(R, t):
    TT = np.eye(4)
    TT[0:3, 0:3] = R
    TT[0:3, 3] = t
    return TT


if __name__ == "__main__":
    # using unnormalized image cordinates
    R1_txt = """0.9844201323069794, 0.01208259142742983, 0.1754166870429003;
 0.01081240876956615, -0.9999079632787907, 0.008194924542646911;
 0.175499558191207, -0.006170571777207297, -0.9844601714232187
    """

    R2_txt = """0.9777399970787368, 0.003199736768202681, -0.209795757338145;
    -0.005052487138173522, 0.9999528245576609, -0.008295844313513287;
    0.2097593156124185, 0.0091711691604545, 0.977710038390749
    """

    t_txt = """   0.98126;
    0.00288049;
    0.192665
    """

## when f was computed using normalized image co-ordinates
R1_txt_n = """
 0.9737043123310533,  0.2278154689098958, -0.0001557988285700;
-0.2278154717910460,  0.9737043241210523, -0.0000007666568953;
 0.0001515273367715,  0.0000362398807602,  0.9999999878630687
 """

R2_txt_n = """
-0.9737035182230082, -0.2278144355776288,  0.0014288255954713;
 0.2278162795944501, -0.9737032729677136,  0.0012957483187613;
 0.0010960619869212,  0.0015871844280590,  0.9999981397451257
 """
t_txt_n = """
0.0006365137169831;
0.0006474911702176;
0.9999995878026514"""


# ## when F was computed with x-mean/sigma. ## normalize_for_f_matrix_computation=true
R1_txt_n="""
-0.9777403356386232, -0.0032001412650330,  0.2097941733250869;
 0.0050529855336521, -0.9999528181758461,  0.0082963099857998;
-0.2097577254893596, -0.0091717238329342, -0.9777103743335565""" 
R2_txt_n="""
-0.9844177358105566, -0.0120903008759865, -0.1754296042472469;
-0.0108200891239780,  0.9999078770648051, -0.0081953068950741;
-0.1755125268833160,  0.0061694415049821,  0.9844578664922894"""
t_txt_n="""
-0.9812592698392634;
-0.0028840815327131;
-0.1926705152020633"""

## normalize_for_f_matrix_computation=false
# R1_txt_n="""
#  0.9844201323069798,  0.0120825914274298,  0.1754166870429005;
#  0.0108124087695662, -0.9999079632787913,  0.0081949245426468;
#  0.1754995581912071, -0.0061705717772071, -0.9844601714232191""" 
# R2_txt_n="""
#  0.9777399970787372,  0.0031997367682027, -0.2097957573381452;
# -0.0050524871381735,  0.9999528245576613, -0.0082958443135131;
#  0.2097593156124186,  0.0091711691604543,  0.9777100383907493"""
# t_txt_n:"""
# 0.9812603940061498;
# 0.0028804901724480;
# 0.1926648435269460"""


R1 = load_mat(R1_txt)
R2 = load_mat(R2_txt)
t = load_mat(t_txt).flatten()

R1_n = load_mat(R1_txt_n)
R2_n = load_mat(R2_txt_n)
t_n = load_mat(t_txt_n).flatten()

# Create a camera pose (replace with your actual pose)
# extrinsic_matrix = np.eye(4)
# extrinsic_matrix[0, 3] = 10

all_geoms = []
all_geoms.append(
    o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0))

all_geoms += create_camera_visualization_mesh(
    np.eye(4), color=color_map_normalized['gray'],  scale=0.1)
# all_geoms += create_camera_visualization_mesh(
#     comb(R1, t), color=color_map_normalized['red'],  scale=0.1)
# all_geoms += create_camera_visualization_mesh(
#     comb(R1, -t), color=color_map_normalized['green'], scale=0.1)
# all_geoms += create_camera_visualization_mesh(
#     comb(R2, t), color=color_map_normalized['blue'], scale=0.1)
# all_geoms += create_camera_visualization_mesh(
#     comb(R2, -t), color=color_map_normalized['cyan'], scale=0.1)


all_geoms += create_camera_visualization_mesh(
    comb(R1, t), color=color_map_normalized['pink'],  scale=0.1)
all_geoms += create_camera_visualization_mesh(
    comb(R1, -t), color=color_map_normalized['pink'], scale=0.1)
all_geoms += create_camera_visualization_mesh(
    comb(R2, t), color=color_map_normalized['pink'], scale=0.1)
all_geoms += create_camera_visualization_mesh(
    comb(R2, -t), color=color_map_normalized['pink'], scale=0.1)

all_geoms += create_camera_visualization_mesh(
    comb(R1_n, t_n), color=color_map_normalized['cyan'],  scale=0.1)
all_geoms += create_camera_visualization_mesh(
    comb(R1_n, -t_n), color=color_map_normalized['cyan'], scale=0.1)
all_geoms += create_camera_visualization_mesh(
    comb(R2_n, t_n), color=color_map_normalized['cyan'], scale=0.1)
all_geoms += create_camera_visualization_mesh(
    comb(R2_n, -t_n), color=color_map_normalized['cyan'], scale=0.1)


o3d.visualization.draw_geometries(
    all_geoms, window_name="Camera Visualization")
