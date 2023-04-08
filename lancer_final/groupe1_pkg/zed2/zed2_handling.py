# +---------+
# | IMPORTS |
# +---------+

import pyzed.sl as sl



# +----------------+
# | ZED2 FUNCTIONS |
# +----------------+

def open_zed2() :
    """
    Returns:
        - zed: sl.Camera(), open instance of sl.Camera()

    Creates and returns an open instance of sl.Camera() with the proper initialization
    parameters in order to use the Zed2 SDK.
    """

    # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_fps = 30  # Set fps at 30
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Use PERFORMANCE depth mode
    init_params.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP # x axis goes to the right, y axis goes through the left eye

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("[ERROR] Failed to open camera. Exiting...")
        exit(1)

    # Create and set RuntimeParameters after opening the camera
    runtime_parameters = sl.RuntimeParameters()
    runtime_parameters.sensing_mode = sl.SENSING_MODE.STANDARD  # Use STANDARD sensing mode
    # Setting the depth confidence parameters
    runtime_parameters.confidence_threshold = 100
    runtime_parameters.textureness_confidence_threshold = 100
    
    return zed



def close_zed2(zed) :
    """
    Parameters:
        - zed: sl.Camera(), open instance of sl.Camera() (to be closed)
    
    Closes given open instance of sl.Camera().
    """

    zed.close()



def left_eye_slMat(zed) :
    """
    Parameters:
        - zed: sl.Camera(), open instance of sl.Camera()
    
    Returns:
        - image: sl.Mat(), picture taken with the left eye of the camera
    
    Takes a picture with the left eye of the Zed2 camera associated to the given open instance
    of sl.Camera(), and returns it in the sl.Mat() format.
    """

    # Initialize variables
    image = sl.Mat()
    runtime_parameters = sl.RuntimeParameters()

    # Attempt to grab picture
    err = zed.grab(runtime_parameters)
    got_picture = (err == sl.ERROR_CODE.SUCCESS)

    nb_tries = 0
    while not(got_picture) and nb_tries < 5 :
        err = zed.grab(runtime_parameters)
        got_picture = (err == sl.ERROR_CODE.SUCCESS)
        nb_tries += 1

    if not(got_picture):
        print("[ERROR] Couldn't grab picture")

    # Retrieve left eye image into the "image" variable
    zed.retrieve_image(image, sl.VIEW.LEFT)

    return image



def convert_slMat(slMat) :
    """
    Parameters:
        - slMat: sl.Mat(), matrix instance of sl.Mat() 

    Returns:
        - mat: MxNx3 array, slMat converted into a regular image matrix

    Converts given sl.Mat()-format image into a regular python array-format image (colors: BGR).
    """

    return slMat.get_data()[:,:,:3]



def left_eye_retrieve(zed) :
    """
    Parameters:
        - zed: sl.Camera(), open instance of sl.Camera()
    
    Returns:
        - image: sl.Mat(), picture taken with the left eye of the camera 
        - depth: sl.Mat(), depth map, aligned on the left eye image 
        - point_cloud: sl.Mat(), colored point cloud, aligned on the left image

    With the Zed2 camera assiciated to the given open instance of sl.Camera(), takes a picture
    with the left eye, captures the depth map relative to the left eye, and builds the colored 3D-point
    cloud associated to the latter. Then, returns all 3.
    """

    # Initialize variables
    image = sl.Mat()
    depth = sl.Mat()
    point_cloud = sl.Mat()
    runtime_parameters = sl.RuntimeParameters()

    # Attempt to grab picture
    err = zed.grab(runtime_parameters)
    got_picture = (err == sl.ERROR_CODE.SUCCESS)

    nb_tries = 0
    while not(got_picture) and nb_tries < 5 :
        err = zed.grab(runtime_parameters)
        got_picture = (err == sl.ERROR_CODE.SUCCESS)
        nb_tries += 1

    if not(got_picture):
        print("[ERROR] Failed to grab picture.")

    # Retrieve left image
    zed.retrieve_image(image, sl.VIEW.LEFT)
    # Retrieve depth map. Depth is aligned on the left image
    zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
    # Retrieve colored point cloud. Point cloud is aligned on the left image.
    zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

    return image, depth, point_cloud



def read_point_cloud_pos(point_cloud, line, column) :
    """
    Parameters:
        - point_cloud: sl.Mat(), colored point cloud recovered from a Zed2 camera
        - line: int, matrix line of the point
        - column: int, matrix column of the point
    
    Returns:
        - x: float, abscissa of the point in the camera frame
        - y: float, ordinate of the point in the camera frame
        - z: float, applicate of the point in the camera frame
    
    Uses given point cloud to read the (x,y,z) position in the camera frame of a point 
    located at (line, column) on the left eye image.
    """

    # Recover position data from point cloud:
    err, point_cloud_value = point_cloud.get_value(column, line)
    x, y, z = point_cloud_value[0], point_cloud_value[1], point_cloud_value[2]

    return x, y, z