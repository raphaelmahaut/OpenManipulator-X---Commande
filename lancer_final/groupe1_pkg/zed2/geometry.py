import numpy as np
DEBUG = True


# +---------------------------------------+
# | ELEMENTARY TRANSFORMATION 4D MATRICES |
# +---------------------------------------+

def translation_matrix(a, b, c) :
    """
    Parameters:
        - a: float, abscissa translation to be applied
        - b: float, ordinate translation to be applied
        - c: float, applicate translation to be applied
    
    Returns:
        - matrix: ndarray float[][], 4 by 4 translation matrix
    """
    matrix = np.array([
        [1, 0, 0, a],
        [0, 1, 0, b],
        [0, 0, 1, c],
        [0, 0, 0, 1]
    ])
    return matrix

def translation_matrix_vec(vector) :
    """
    Parameters:
        - vector: array float[], 3D translation to be applied
    
    Returns:
        - matrix: ndarray float[][], 4 by 4 translation matrix
    """
    a, b, c = vector[0], vector[1], vector[2]
    matrix = np.array([
        [1, 0, 0, a],
        [0, 1, 0, b],
        [0, 0, 1, c],
        [0, 0, 0, 1]
    ])
    return matrix

def rotation_matrix_x_angle(angle) :
    """
    Parameters:
        - angle: float, angle of rotation around x to be applied
    
    Returns:
        - matrix: ndarray float[][], 4 by 4 rotation matrix (around x-axis)
    """
    matrix = np.array([
        [1, 0, 0, 0],
        [0, np.cos(angle), -np.sin(angle), 0],
        [0, np.sin(angle), np.cos(angle), 0],
        [0, 0, 0, 1]
    ])
    return matrix

def rotation_matrix_x(cos, sin) :
    """
    Parameters:
        - cos: float, cosine of angle of rotation around x to be applied
        - sin: float, sine of angle of rotation around x to be applied
    
    Returns:
        - matrix: ndarray float[][], 4 by 4 rotation matrix (around x-axis)
    """
    matrix = np.array([
        [1, 0, 0, 0],
        [0, cos, -sin, 0],
        [0, sin, cos, 0],
        [0, 0, 0, 1]
    ])
    return matrix

def rotation_matrix_y_angle(angle) :
    """
    Parameters:
        - angle: float, angle of rotation around y to be applied
    
    Returns:
        - matrix: ndarray float[][], 4 by 4 rotation matrix (around y-axis)
    """
    matrix = np.array([
        [np.cos(angle), 0, np.sin(angle), 0],
        [0, 1, 0, 0],
        [-np.sin(angle), 0, np.cos(angle), 0],
        [0, 0, 0, 1]
    ])
    return matrix

def rotation_matrix_y(cos, sin) :
    """
    Parameters:
        - cos: float, cosine of angle of rotation around y to be applied
        - sin: float, sine of angle of rotation around y to be applied
    
    Returns:
        - matrix: ndarray float[][], 4 by 4 rotation matrix (around y-axis)
    """
    matrix = np.array([
        [cos, 0, sin, 0],
        [0, 1, 0, 0],
        [-sin, 0, cos, 0],
        [0, 0, 0, 1]
    ])
    return matrix

def rotation_matrix_z_angle(angle) :
    """
    Parameters:
        - angle: float, angle of rotation around z to be applied
    
    Returns:
        - matrix: ndarray float[][], 4 by 4 rotation matrix (around z-axis)
    """
    matrix = np.array([
        [np.cos(angle), -np.sin(angle), 0, 0],
        [np.sin(angle), np.cos(angle), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    return matrix

def rotation_matrix_z(cos, sin) :
    """
    Parameters:
        - cos: float, cosine of angle of rotation around z to be applied
        - sin: float, sine of angle of rotation around z to be applied
    
    Returns:
        - matrix: ndarray float[][], 4 by 4 rotation matrix (around z-axis)
    """
    matrix = np.array([
        [cos, -sin, 0, 0],
        [sin, cos, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    return matrix



# +--------------+
# | FRAME CHANGE |
# +--------------+

def norm_3D(v) :
    """
    Parameters:
        - v: array float[], vector of length >= 3
    
    Returns:
        - float, euclidean 3D norm of vector v

    Computes and returns euclidean 3D norm of given vector. Meant to be used with vectors of
    length 3 or vectors of lengths 4 with fourth component equal to 1. If vector of length > 3 
    is given, only the first 3 components are used to compute the norm.
    """
    vv = np.array([v[0], v[1], v[2]])
    return np.sum(vv * vv)**(1/2)



def scalar_product_3D(v1, v2) :
    """
    Parameters:
        - v1: array float[], vector of length >= 3
        - v2: array float[], vector of length >= 3
    
    Returns:
        - float, 3D scalar product of vectors v1 and v2

    Computes and returns 3D scalar product of given vectors. Meant to be used with vectors of
    length 3 or vectors of lengths 4 with fourth component equal to 1. If vector of length > 3 
    is given, only the first 3 components are used to compute the scalar product.
    """
    vv1 = np.array([v1[0], v1[1], v1[2]])
    vv2 = np.array([v2[0], v2[1], v2[2]])
    return np.sum(vv1*vv2)



def vector_product_3D(v1, v2) :
    """
    Parameters:
        - v1: array float[], vector of length >= 3
        - v2: array float[], vector of length >= 3
    
    Returns:
        - vv3: array float[], 3D vector product of v1 and v2, of length 3

    Computes and returns 3D vector product of given vectors. Meant to be used with vectors of
    length 3 or vectors of lengths 4 with fourth component equal to 1. If vector of length > 3 
    is given, only the first 3 components are used to compute the vector product.
    """
    vv1 = np.array([v1[0], v1[1], v1[2]])
    vv2 = np.array([v2[0], v2[1], v2[2]])
    vv3 = np.cross(vv1, vv2)
    return vv3



def frame_change_rotation_matrices(x, y, z) :
    """
    Parameters:
        - x: array float[], vector x-axis of frame to be used to compute frame change matrices
        - y: array float[], vector y-axis of frame to be used to compute frame change matrices
        - z: array float[], vector z-axis of frame to be used to compute frame change matrices

    Returns:
        - mat_other2standard_r: ndarray float[][], rotation matrix used to change from given frame
        to standard frame
        - mat_standard2other_r: ndarray[][], rotation matrix used to change from standard frame 
        to given frame

    Creates and returns the frame change rotation matrices 
    allowing to go from given frame (x, y, z) to the standard
    frame (i, j, k) = ((1, 0, 0), (0, 1, 0), (0, 0, 1)).
    
    [WARNING]: only use these on vectors. They are not the full frame change matrices that
    can be applied to points.
    """
    xx = np.array([x[0], x[1], x[2], 1])
    yy = np.array([y[0], y[1], y[2], 1])
    zz = np.array([z[0], z[1], z[2], 1])


    # [STEP 1] Rotate vector x into (i, j) plane
    # Let x_jk = [0, x[1], x[2]] be the projection of x into (j, k) plane
    x_jk_norm = norm_3D([0, x[1], x[2]])
    
    # To get x to be in the (i, j) plane, it is enough to get x_jk to be aligned with j.
    # To get x_jk to be aligned with j, we need to rotate it around the i axis
    # with a negative angle alpha such that:
    cos_alpha = x[1]/x_jk_norm
    sin_alpha = - x[2]/x_jk_norm # Minus because negative

    #if DEBUG :
    #    print("[DEBUG] cos(alpha)² + sin(alpha)² =", cos_alpha**2 + sin_alpha**2)

    # Hence, applying the following rotation to vector x should result in a
    # vector in the (i, j) plane :
    mx1 = rotation_matrix_x(cos_alpha, sin_alpha)
    x_p = np.around(np.dot(mx1, xx), 3)

    if DEBUG :
        # (Notice that after the rotation, x_p = [x[0], x_kj_norm, 0, 1])
        # print("[DEBUG] mx1 =\n", mx1)
        print("[DEBUG] x_p =", x_p, "(Third component should be zero)")
        print()


    # [STEP 2] Rotate vector x_p to be aligned with i
    # We need to rotate it around k with a certain negative angle beta such that:
    x_p_norm = norm_3D(x_p)
    cos_beta = x_p[0]/x_p_norm
    sin_beta = - x_p[1]/x_p_norm # Minus because negative

    #if DEBUG :
    #    print("[DEBUG] cos(beta)² + sin(beta)² =", cos_beta**2 + sin_beta**2)

    # Hence, applying the following rotation to vector x_p should result in a
    # vector aligned with i (+ going same way):
    mz2 = rotation_matrix_z(cos_beta, sin_beta)
    x_pp = np.around(np.dot(mz2, x_p), 3)

    if DEBUG :
        # (Notice that after the rotation, x_pp = [x_p_norm, 0, 0, 1]
        # where x_p_norm = 1 if x is a unit vector)
        #print("[DEBUG] mz2 =\n", mz2)
        print("[DEBUG] x_pp = x_ppp =", x_pp, "(Second and third component should be zero)")
    

    # [STEP 3] Rotate vector y in the same way as x to get y_pp, and rotate y_pp to be 
    # aligned with j
    y_pp = np.around(np.dot(mz2, np.dot(mx1, yy)), 3)

    # We need to rotate it around i with a certain negative angle gamma such that:
    y_pp_norm = norm_3D(y_pp)
    cos_gamma = y_pp[1]/y_pp_norm
    sin_gamma = - y_pp[2]/y_pp_norm # Minus because negative

    #if DEBUG :
    #    print("[DEBUG] cos(gamma)² + sin(gamma)² =", cos_gamma**2 + sin_gamma**2)

    # Hence, applying the following rotation to vector y_pp should result in a
    # vector aligned with j:
    mx3 = rotation_matrix_x(cos_gamma, sin_gamma)
    y_ppp = np.around(np.dot(mx3, y_pp), 3)

    if DEBUG :
        #print("[DEBUG] mx3 =\n", mx3)
        print("[DEBUG] y_ppp =", y_ppp, "(First and third component should be zero)")
    
    # Finally, apply the same rotations to vector z. Notice that applying mx3 to x_pp has no effect, 
    # hence x_ppp = x_pp.
    z_ppp = np.around(np.dot(mx3, np.dot(mz2, np.dot(mx1, zz))), 3)

    if DEBUG :
        print("[DEBUG] z_ppp =", z_ppp, "(First and second component should be zero)")
        print()
    
    # And here goes our matrix to rotate the given frame (x, y, z) into the standard frame (i, j, k)
    mat_other2standard_r = np.dot(mx3, np.dot(mz2, mx1))

    ### Lastly, here is the reciprocal matrix
    mx1_p = rotation_matrix_x(cos_alpha, -sin_alpha)
    mz2_p = rotation_matrix_z(cos_beta, -sin_beta)
    mx3_p = rotation_matrix_x(cos_gamma, -sin_gamma)
    mat_standard2other_r = np.dot(mx1_p, np.dot(mz2_p, mx3_p))

    # Last check : 
    if DEBUG :
        eye4 = np.dot(mat_other2standard_r, mat_standard2other_r)
        print("[DEBUG] Product of reciprocal rotation matrices :\n", eye4)
        print()

    return mat_other2standard_r, mat_standard2other_r



def frame_change_matrices(origin, x, y, z) :
    """
    Parameters:
        - origin: array float[], 3D coordinates of origin of frame to be used to compute frame change matrices
        - x: array float[], vector x-axis of frame to be used to compute frame change matrices
        - y: array float[], vector y-axis of frame to be used to compute frame change matrices
        - z: array float[], vector z-axis of frame to be used to compute frame change matrices

    Returns:
        - mat_other2standard: ndarray float[][], frame change matrix used to change from given frame
        to standard frame
        - mat_standard2other: ndarray[][], frame change matrix used to change from standard frame 
        to given frame

    Creates and returns the frame change matrices 
    allowing to go from given frame (x, y, z) of given origin to the standard
    frame (i, j, k) = ((1, 0, 0), (0, 1, 0), (0, 0, 1)) of origin (0, 0, 0).
    """
    o_i = origin[0]
    o_j = origin[1]
    o_k = origin[2]

    # Handle translation
    # other --> standard : translation of minus the coordinates of origin of other frame
    # standard --> other : translation of the coordinates of origin of other frame
    mat_other2standard_t = translation_matrix(-o_i, -o_j, -o_k)
    mat_standard2other_t = translation_matrix(o_i, o_j, o_k)

    # Handle rotation
    mat_other2standard_r, mat_standard2other_r = frame_change_rotation_matrices(x, y, z)

    # Build complete frame change matrices
    # other --> standard : apply translation first, then rotation
    # standard --> other : apply rotation first, then translation
    mat_other2standard = np.dot(mat_other2standard_r, mat_other2standard_t)
    mat_standard2other = np.dot(mat_standard2other_t, mat_standard2other_r)

    if DEBUG :
        eye4 = np.dot(mat_other2standard, mat_standard2other)
        print("[DEBUG] Product of reciprocal frame change matrices :\n", eye4)
        print()

    return mat_other2standard, mat_standard2other



def test_frame_change(origin, x, y, z) :
    """
    Parameters:
        - origin: array float[], 3D coordinates of origin of frame to be used for testing
        - x: array float[], vector x-axis of frame to be used for testing
        - y: array float[], vector y-axis of frame to be used for testing
        - z: array float[], vector z-axis of frame to be used for testing

    Recovers the frame change matrices used to go from the given frame with given origin to
    the standard frame (i, j, k) = ((1, 0, 0), (0, 1, 0), (0, 0, 1)) of origin (0, 0, 0) and
    the other way around, and displays a few tests with them.
    """
    mat_o2s_r, mat_s2o_r = frame_change_rotation_matrices(x, y, z)
    mat_o2s, mat_s2o = frame_change_matrices(origin, x, y, z)

    oo = np.array([0, 0, 0, 1])
    ii = np.array([1, 0, 0, 1])
    jj = np.array([0, 1, 0, 1])
    kk = np.array([0, 0, 1, 1])

    oorigin = np.array([origin[0], origin[1], origin[2], 1])
    xx = np.array([x[0], x[1], x[2], 1])
    yy = np.array([y[0], y[1], y[2], 1])
    zz = np.array([z[0], z[1], z[2], 1])


    # Test frame change other --> standard :
    oo2 = np.dot(mat_o2s, oorigin)
    ii2 = np.dot(mat_o2s_r, xx)
    jj2 = np.dot(mat_o2s_r, yy)
    kk2 = np.dot(mat_o2s_r, zz)

    print("[TEST] Testing frame change other --> standard...")
    print("       Frame yielded by applying mat_o2s to other frame :")
    print("       i =", ii2)
    print("       j =", jj2)
    print("       k =", kk2)
    print("       origin =", oo2)
    print()


    # Test frame change standard --> other :
    oorigin2 = np.dot(mat_s2o, oo)
    xx2 = np.dot(mat_s2o_r, ii)
    yy2 = np.dot(mat_s2o_r, jj)
    zz2 = np.dot(mat_s2o_r, kk)
    
    print("[TEST] Testing frame change standard --> other...")
    print("       Original other frame :")
    print("       x =", xx)
    print("       y =", yy)
    print("       z =", zz)
    print("       origin =", oorigin)
    print("       Frame yielded by applying mat_s2o to standard frame :")
    print("       x =", xx2)
    print("       y =", yy2)
    print("       z =", zz2)
    print("       origin =", oorigin2)
    print()





if __name__ == "__main__" :
    origin = [1, 1, 1]
    x = [0, -1, 0]
    y = [-1, 0, 0]
    z = [0, 0, -1]
    test_frame_change(origin, x, y, z)
