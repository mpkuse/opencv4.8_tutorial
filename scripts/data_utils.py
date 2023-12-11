import numpy as np

def file_to_str( file_path ):
    # Open the file and read its contents into a list of strings
    with open(file_path, 'r') as file:
        #file_content_lines = file.readlines()
        file_content_lines = [line.strip() for line in file.readlines() if line.strip()]
        return '\n'.join(file_content_lines)
    
    return None 

def load_mat(R_txt, row_sep=';', col_sep=','):
    xrows = R_txt.split(row_sep)

    R = []
    for r in xrows:
        #nums = r.split(col_sep)
        nums = [value for value in r.split(col_sep) if value]

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

def quaternion_to_rotation_matrix(quaternion):
    w, x, y, z = quaternion
    rotation_matrix = np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
        [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
        [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2]
    ])
    return rotation_matrix