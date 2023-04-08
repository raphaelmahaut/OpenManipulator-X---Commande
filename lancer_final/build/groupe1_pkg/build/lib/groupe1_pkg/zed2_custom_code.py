# +---------+
# | IMPORTS |
# +---------+

import pyzed.sl as sl
import cv2 # pip install opencv-python
import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, acos, asin
import torchvision.transforms as T
import torchvision

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from rclpy.qos import QoSProfile


class Objectif(Node):

    def __init__(self):
        super().__init__('objectif')
        self.publisher_ = self.create_publisher(Vector3, 'gob', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.x = 0.5
        self.y = 0.
        self.z = 0.8

    def timer_callback(self):
        msg = Vector3()
        msg.x = self.x
        msg.y = self.y
        msg.z = self.z
        self.publisher_.publish(msg)
        self.get_logger().info("demande de lancement: objectif x = {}, y = {}, z = {}".format(self.x, self.y, self.z))



# +----------------+
# | ZED2 FUNCTIONS |
# +----------------+

def open_zed2() :
    """
    Returns:
        - zed: sl.Camera(), open instance of sl.Camera()
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
    Arguments:
        - zed: sl.Camera(), open instance of sl.Camera() (to be closed)
    """

    zed.close()



def left_eye_slMat(zed) :
    """
    Arguments:
        - zed: sl.Camera(), open instance of sl.Camera()
    
    Returns:
        - image: sl.Mat(), picture taken with the left eye of the camera
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
    Arguments:
        - slMat: sl.Mat(), matrix instance of sl.Mat() 

    Returns:
        - mat: MxNx3 array, slMat converted into a regular image matrix
    """

    return slMat.get_data()[:,:,:3]



def left_eye_retrieve(zed) :
    """
    Arguments:
        - zed: sl.Camera(), open instance of sl.Camera()
    
    Returns:
        - image: sl.Mat(), picture taken with the left eye of the camera 
        - depth: sl.Mat(), depth map, aligned on the left eye image 
        - point_cloud: sl.Mat(), colored point cloud, aligned on the left image
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



def real_time_window(zed) :
    flip = False
    while True :
        image, depth, point_cloud = left_eye_retrieve(zed)
        image_cv2 = convert_slMat(image)
        depth_cv2 = depth.get_data()

        if flip :
            cv2.imshow("Frame", cv2.flip(image_cv2,1))
            cv2.imshow("Depth", cv2.flip(depth_cv2,1))
        else : 
            cv2.imshow("Frame", image_cv2)
            cv2.imshow("Depth", depth_cv2)
        a = cv2.waitKey(200)
        if a == 27: #ESC key
            break   
    plt.imshow(depth_cv2)
    #plt.show()
    plt.clf()




def left_eye_world_frame_point_cloud(zed) :
    """
    Arguments:
        - zed: sl.Camera(), open instance of sl.Camera()
    
    Returns:
        - point_cloud: sl.Mat(), colored point cloud, aligned on the left image, 
        in world frame coordinates
    """

    # Initialize variables
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

    # Retrieve colored point cloud in robot frame
    zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA, sl.REFERENCE_FRAME.WORLD)

    return point_cloud



def read_point_cloud_pos(point_cloud, line, column) :
    """
    Reads the position in the camera frame of a point at (line, column).

    Arguments:
        - point_cloud: sl.Mat(), colored point cloud recovered from a Zed2 camera
        - line : int, matrix line of the point
        - column : int, matrix column of the point
    
    Returns:
        - x: float, abscissa of the point in the camera frame
        - y: float, ordinate of the point in the camera frame
        - z: float, applicate of the point in the camera frame
    """

    # Recover position data from point cloud:
    err, point_cloud_value = point_cloud.get_value(column, line)
    x, y, z = point_cloud_value[0], point_cloud_value[1], point_cloud_value[2]

    return x, y, z



def set_world_frame(x, y, z, ox, oy, oz, ow) :
    """
    Sets the world frame at position x, y, z in the camera frame with given rotations.

    Arguments:
        - x: float, abscissa of the world frame origin in the camera frame
        - y: float, ordinate of the world frame origin in the camera frame
        - z: float, applicate of the world frame origin in the camera frame
        - ox: float, orientation quaternion abscissa (x_0*sin(theta/2))
        - oy: float, orientation quaternion ordinate (y_0*sin(theta/2))
        - oz: float, orientation quaternion applicate (z_0*sin(theta/2))
        - ow: float, orientation quaternion fourth coordinate (cos(theta/2))
            - Both orientation quaternion and (x_0, y_0, z_0) rotation axis vector must have unit norm!
            - Rotation axis vector (x_0, y_0, z_0) is represented in the camera frame
    """

    initial_position = sl.Transform()
    tracking_parameters = sl.PositionalTrackingParameters()
    
    # Set the world frame at (x, y, z) in camera frame, 
    # i.e. set camera frame at (-x, -y, -z) in world frame.
    initial_translation = sl.Translation()
    initial_translation.init_vector(-x, -y, -z)
    initial_position.set_translation(initial_translation)

    # Set world frame orientation at (ox, oy, oz, ow) in camera frame, 
    # i.e. set camera frame orientation at (-ox, -oy, -oz, +ow) in world frame (change theta to -theta)
    initial_orientation = sl.Orientation()
    initial_orientation.init_vector(-ox, -oy, -oz, ow)
    initial_position.set_orientation(initial_orientation)

    # Update tracking parameters
    tracking_parameters.set_initial_world_transform(initial_position)



# +----------------+
# | MATH FUNCTIONS |
# +----------------+

def quaternion_product(q1, q2) :
    """
    Note that the quaternion product of q1 x q2 is equivalent to applying q2 first and then q1.

    Arguments:
        - q1: float[4], tuple of (ox1, oy1, oz1, ow1)
        - q2: float[4], tuple of (ox2, oy2, oz2, ow2)
    
    Returns:
        - q: float[4], tuple of (ox, oy, oz, ow), product of q1 times q2

    Sources :
        - https://en.wikipedia.org/wiki/Quaternion 
        - https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
    """

    ox1, oy1, oz1, ow1 = q1
    ox2, oy2, oz2, ow2 = q2

    ow = ow1*ow2 - ox1*ox2 - oy1*oy2 - oz1*oz2
    ox = ow1*ox2 + ow2*ox1 + oy1*oz2 - oy2*oz1
    oy = ow1*oy2 - ox1*oz2 + oy1*ow2 + oz1*ox2
    oz = ow1*oz2 + ox1*oy2 - oy1*ox2 + oz1*ow2

    #print("[DEBUG] Quaternion product check : norm =", ox**2 + oy**2 + oz**2 + ow**2)

    return (ox, oy, oz, ow)



def apply_quaternion_rotation(v, q) :
    """
    Arguments:
        - v: float[3], tuple of (x, y, z)
        - q: float[4], tuple of (ox, oy, oz, ow)
    
    Returns:
        - v2: float[3], tuple of (x2, y2, z2), vector resulting from applying q to v
    """

    x, y, z = v
    ox, oy, oz, ow = q
    v_quaternion = (x, y, z, 0)
    q_inverse = (-ox, -oy, -oz, ow)

    qvq = quaternion_product(quaternion_product(q, v_quaternion), q_inverse)
    x2 = qvq[0]
    y2 = qvq[1]
    z2 = qvq[2]

    return np.array([x2, y2, z2])



def scalar_product(v1, v2) :
    """
    Arguments:
        - v1: float[3], tuple of (x1, y1, z1)
        - v2: float[3], tuple of (x2, y2, z2)
    
    Returns:
        - sp: float, scalar product of v1 dot v2
    """

    x1, y1, z1 = v1
    x2, y2, z2 = v2

    return x1*x2 + y1*y2 + z1*z2



def vector_product(v1, v2) :
    """
    Arguments:
        - v1: float[3], tuple of (x1, y1, z1)
        - v2: float[3], tuple of (x2, y2, z2)
    
    Returns:
        - v: float[3], vector product of v1 times v2
    """

    x1, y1, z1 = v1
    x2, y2, z2 = v2

    x = y1*z2 - z1*y2
    y = z1*x2 - x1*z2
    z = x1*y2 - y1*x2

    return np.array([x, y, z])



# +------------------+
# | OPENCV FUNCTIONS |
# +------------------+

def do_nothing(a) :
    # A function that does nothing, used as the mandatory callback function for cv2 trackbars
    pass



def color_filter(img, color, color_min, other_color_max) :
    # Remember that cv2 images are BGR and not RGB
    blue = np.copy(img[:, :, 0])
    green = np.copy(img[:, :, 1])
    red = np.copy(img[:, :, 2])

    if color == "blue" :
        return np.where(((blue >= color_min) & (red < other_color_max) & (green < other_color_max)), blue, 0)
    elif color == "green" :
        return np.where(((green >= color_min) & (red < other_color_max) & (blue < other_color_max)), green, 0)
    else : # color == "red"
        return np.where(((red >= color_min) & (blue < other_color_max) & (green < other_color_max)), red, 0)



def find_dots(img, color_min, other_color_max) :
    points = []
    lines = len(img)
    cols = len(img[0])
    lines_nbs = np.array([[i for j in range(cols)] for i in range(lines)])
    cols_nbs = lines_nbs = np.array([[j for j in range(cols)] for i in range(lines)])

    for color in ["red", "green", "blue"] :
        colornp = color_filter(img, color, color_min, other_color_max)
        weight_sum = np.sum(np.sum(colornp))

        if weight_sum > 0 :
            line_times_weight_sum = np.sum(np.sum(colornp * lines_nbs))
            col_times_weight_sum = np.sum(np.sum(colornp * cols_nbs))
            centroid_line = int(line_times_weight_sum/weight_sum)
            centroid_col = int(col_times_weight_sum/weight_sum)
            points = points + [(centroid_line, centroid_col)]

    return points



def find_red_dot(img, color_min, other_color_max) :
    points = []
    lines = len(img)
    cols = len(img[0])
    lines_nbs = np.array([[i for j in range(cols)] for i in range(lines)])
    cols_nbs = lines_nbs = np.array([[j for j in range(cols)] for i in range(lines)])

    for color in ["red"] :
        colornp = color_filter(img, color, color_min, other_color_max)
        weight_sum = np.sum(np.sum(colornp))

        if weight_sum > 0 :
            line_times_weight_sum = np.sum(np.sum(colornp * lines_nbs))
            col_times_weight_sum = np.sum(np.sum(colornp * cols_nbs))
            centroid_line = int(line_times_weight_sum/weight_sum)
            centroid_col = int(col_times_weight_sum/weight_sum)
            points = points + [(centroid_line, centroid_col)]

    return points



# +-------------------------+
# | CUP DETECTION FUNCTIONS |
# +-------------------------+

def get_prediction(img, threshold=0.6):
    model=torchvision.models.detection.fasterrcnn_resnet50_fpn(pretrained=True)
    model.eval()

    COCO_INSTANCE_CATEGORY_NAMES= ['__background__', 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
        'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'N/A', 'stop sign',
        'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
        'elephant', 'bear', 'zebra', 'giraffe', 'N/A', 'backpack', 'umbrella', 'N/A', 'N/A',
        'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
        'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
        'bottle', 'N/A', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl',
        'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
        'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'N/A', 'dining table',
        'N/A', 'N/A', 'toilet', 'N/A', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
        'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'N/A', 'book',
        'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush']
    transform=T.Compose([T.ToTensor()])
    img=transform(img)
    pred=model([img])
    cup_boxes=[]
    pred_class=[COCO_INSTANCE_CATEGORY_NAMES[i] for i in list(pred[0]['labels'].numpy())]
    pred_boxes=[[(i[0],i[1]),(i[2],i[3])] for i in list(pred[0]['boxes'].detach().numpy())]
    pred_score=list(pred[0]['scores'].detach().numpy())
    pred_t=[pred_score.index(x) for x in pred_score if x>threshold]
    if len(pred_t)>0:
        for i in range(pred_t[-1]+1):
            if pred_class[i]=='cup':
                cup_boxes.append(pred_boxes[i])
        if len(cup_boxes)>0:
            return 1, cup_boxes
        else:
            return 0, None
    else:
        return 0, None




# +--------------------+
# | COMBINED FUNCTIONS |
# +--------------------+

def are_not_null(argslist) :
    for arg in argslist :
        if not(arg >= 0 or arg < 0) :
            return False
    return True



def setup_camera_world_frame(zed) :
    """
    Points A, B and C refer to three specific corners of the robot's workbench :

    A --------- B
    |           |
    |           |
    |     R     |
    + --------- C

    These points will be used to identify the robot's position relative to the camera.
    In order to be found more easily, these points will be associated with a colored sticker (or the like)
    so that a piece of OpenCV code can locate them fast on a picture. Points A, B and C will be associated
    to, respectively, Red, Green and Blue.
    """

    ### Create trackbars for calibration
    cv2.namedWindow("Calibration controls")
    cv2.createTrackbar("color_min", "Calibration controls", 200, 255, do_nothing)
    cv2.createTrackbar("other_color_max", "Calibration controls", 100, 255, do_nothing)

    ### Display real-time depth map to calibrate
    stop_signal = False
    depth_detection_ok = False
    while not(stop_signal) or not(depth_detection_ok) :
        # Recover trackbar calibration values
        color_min = cv2.getTrackbarPos("color_min", "Calibration controls")
        other_color_max = cv2.getTrackbarPos("other_color_max", "Calibration controls")

        # Take picture and look for the three points on it
        image, depth, point_cloud = left_eye_retrieve(zed)
        image_cv2 = np.array(convert_slMat(image))
        depth_cv2 = depth.get_data()
        red = color_filter(image_cv2, "red", color_min, other_color_max)
        green = color_filter(image_cv2, "green", color_min, other_color_max)
        blue = color_filter(image_cv2, "blue", color_min, other_color_max)

        points_list = find_dots(convert_slMat(image), color_min, other_color_max)
        nb_points = len(points_list)

        if nb_points == 3 :
            pointA, pointB, pointC = points_list
            lineA, colA = pointA
            lineB, colB = pointB
            lineC, colC = pointC

            # Read point coordinates in camera frame thanks to point cloud
            xA, yA, zA = read_point_cloud_pos(point_cloud, lineA, colA)
            xB, yB, zB = read_point_cloud_pos(point_cloud, lineB, colB)
            xC, yC, zC = read_point_cloud_pos(point_cloud, lineC, colC)

            # Check that camera could properly read the depth of all 3 points
            depth_detection_ok = are_not_null([xA, yA, zA, xB, yB, zB, xC, yC, zC])

            if depth_detection_ok :
                # Depth detection worked at all 3 of the located points
                # Display a green circular icon in top-left hand corner
                cv2.circle(image_cv2, (0, 0), 30, (255, 255, 255), thickness = 50)
                cv2.circle(image_cv2, (0, 0), 30, (0, 255, 0), thickness = 40)
                cv2.circle(image_cv2, (colA, lineA), 10, (255, 0, 0), thickness = 5)
                cv2.circle(image_cv2, (colB, lineB), 10, (0, 255, 0), thickness = 5)
                cv2.circle(image_cv2, (colC, lineC), 10, (0, 0, 255), thickness = 5)

            else :
                # Depth detection didn't work at 1 of the located points or more
                # Display a red circular icon in top-left hand corner
                cv2.circle(image_cv2, (0, 0), 30, (255, 255, 255), thickness = 50)
                cv2.circle(image_cv2, (0, 0), 30, (0, 0, 255), thickness = 40)

        else :
            # An error occurred in the attempt below
            # Display a black circular icon in top-left hand corner
            print("[DEBUG]", nb_points, "found instead of 3")
            cv2.circle(image_cv2, (0, 0), 30, (255, 255, 255), thickness = 50)
            cv2.circle(image_cv2, (0, 0), 30, (0, 0, 0), thickness = 40)

        # Display depth map and camera vision in real time
        cv2.imshow("Frame", image_cv2)
        cv2.imshow("Depth", depth_cv2)
        cv2.imshow("red", red)
        cv2.imshow("blue", blue)
        cv2.imshow("green", green)

        a = cv2.waitKey(200)
        if a == 27: # press ESC key to give stop signal
            if not(stop_signal) :
                stop_signal = True
            else :  # press ESC again to exit program
                cv2.destroyAllWindows()
                close_zed2(zed)
                exit()
    
    cv2.destroyAllWindows()

    # [STEP 3] Define position of robot frame origin (point R)
    # Projecting the relation OR = OC + (4/30)*CB + (1/2)*BA into the camera frame, there is:
    xR = xC + (4/30)*(xB - xC) + (1/2)*(xA - xB)
    yR = yC + (4/30)*(yB - yC) + (1/2)*(yA - yB)
    zR = zC + (4/30)*(zB - zC) + (1/2)*(zA - zB)

    # [STEP 4] Define rotation of robot frame relative to camera frame
    # [SUBSTEP 4.1] Define rotation of robot abscissa axis (x_rob) relative camera's (x_cam = (1, 0, 0))
    # --> x_rob comes from a rotation of angle acos(x_cam.x_rob) around vector (x_cam x x_rob)
    vector_CB = np.array([xB - xC, yB - yC, zB - zC]) 
    norm_CB = scalar_product(vector_CB, vector_CB)**(1/2)
    vector_BA = np.array([xA - xB, yA - yB, zA - zB]) 
    norm_BA = scalar_product(vector_BA, vector_BA)**(1/2)

    x_rob = vector_CB/norm_CB
    y_rob = vector_BA/norm_BA
    z_rob = vector_product(x_rob, y_rob)

    x_cam = np.array([1, 0, 0])
    y_cam = np.array([0, 1, 0])
    z_cam = np.array([0, 0, 1])
    
    print("[DEBUG] Check vectors --------")
    print("     ||x_rob|| =", np.around(scalar_product(x_rob, x_rob)**(1/2), 3))
    print("     ||y_rob|| =", np.around(scalar_product(y_rob, y_rob)**(1/2), 3))
    print("     ||z_rob|| =", np.around(scalar_product(z_rob, z_rob)**(1/2), 3))
    print("     x_rob.y_rob =", np.around(scalar_product(x_rob, y_rob), 3))
    print("     x_rob.z_rob =", np.around(scalar_product(x_rob, z_rob), 3))
    print("     y_rob.z_rob =", np.around(scalar_product(y_rob, z_rob), 3))

    theta = acos(scalar_product(x_cam, x_rob))
    axis_unnormed = vector_product(x_cam, x_rob)
    norm_axis = scalar_product(axis_unnormed, axis_unnormed)**(1/2)
    axis = axis_unnormed/norm_axis

    # Define quaternion associated to rotation
    ox1 = scalar_product(axis, x_cam)*sin(theta/2)
    oy1 = scalar_product(axis, y_cam)*sin(theta/2)
    oz1 = scalar_product(axis, z_cam)*sin(theta/2)
    ow1 = cos(theta/2) 
    q1 = (ox1, oy1, oz1, ow1)
    q1_inverse = (-ox1, -oy1, -oz1, ow1)

    # [SUBSTEP 4.2] Apply inverse rotation to have x_cam and x_rob go in the same direction 
    # and then recover last rotation components
    x_rob_2 = apply_quaternion_rotation(x_rob, q1_inverse)
    y_rob_2 = apply_quaternion_rotation(y_rob, q1_inverse)
    z_rob_2 = apply_quaternion_rotation(z_rob, q1_inverse)

    print("[DEBUG] Check applied rotation --------")
    print("     x_rob_2 =", np.around(x_rob_2), 3)

    # Now, the only difference between (x_cam, y_cam, z_cam) and (x_rob_2, y_rob_2, z_rob_2) is a rotation
    # around x_cam that we can identify
    alpha = acos(scalar_product(y_cam, y_rob_2))
    q2 = (sin(alpha/2), 0, 0, cos(alpha/2))

    # [SUBSTEP 4.3] Combine all this into a single rotation quaternion
    # All in all, in order to get the robot frame, we need to apply q2 and then q1 to the camera frame, 
    # which is equivalent to applying q1 x q2.
    q = quaternion_product(q1, q2)
    set_world_frame(xR, yR, zR, q[0], q[1], q[2], q[3])

    return (xR, yR, zR), q



def setup_camera_world_frame_manual(zed) :
    """
    Points A, B and C refer to three specific corners of the robot's workbench :

    A --------- B
    |           |
    |           |
    |     R     |
    + --------- C

    These points will be used to identify the robot's position relative to the camera.
    """

    corners = []

    ### Display real-time depth map to calibrate
    for i in range(3) :
        ### Create trackbars for calibration
        cv2.namedWindow("Calibration controls")
        cv2.createTrackbar("color_min", "Calibration controls", 200, 255, do_nothing)
        cv2.createTrackbar("other_color_max", "Calibration controls", 100, 255, do_nothing)

        stop_signal = False
        depth_detection_ok = False
        while not(stop_signal) or not(depth_detection_ok) :
            # Recover trackbar calibration values
            color_min = cv2.getTrackbarPos("color_min", "Calibration controls")
            other_color_max = cv2.getTrackbarPos("other_color_max", "Calibration controls")

            # Take picture and look for the three points on it
            image, depth, point_cloud = left_eye_retrieve(zed)
            image_cv2 = np.array(convert_slMat(image))
            depth_cv2 = depth.get_data()
            red = color_filter(image_cv2, "red", color_min, other_color_max)

            points_list = find_red_dot(convert_slMat(image), color_min, other_color_max)
            nb_points = len(points_list)

            if nb_points == 1 :
                point = points_list[0]
                line, col = point

                # Read point coordinates in camera frame thanks to point cloud
                x, y, z = read_point_cloud_pos(point_cloud, line, col)

                # Check that camera could properly read the depth of all 3 points
                depth_detection_ok = are_not_null([x, y, z])

                if depth_detection_ok :
                    # Depth detection worked at all 3 of the located points
                    # Display a green circular icon in top-left hand corner
                    cv2.circle(image_cv2, (0, 0), 30, (255, 255, 255), thickness = 50)
                    cv2.circle(image_cv2, (0, 0), 30, (0, 255, 0), thickness = 40)
                    cv2.circle(image_cv2, (col, line), 10, (255, 0, 0), thickness = 5)

                else :
                    # Depth detection didn't work at 1 of the located points or more
                    # Display a red circular icon in top-left hand corner
                    cv2.circle(image_cv2, (0, 0), 30, (255, 255, 255), thickness = 50)
                    cv2.circle(image_cv2, (0, 0), 30, (0, 0, 255), thickness = 40)

            else :
                # An error occurred in the attempt below
                # Display a black circular icon in top-left hand corner
                print("[DEBUG]", nb_points, "found instead of 3")
                cv2.circle(image_cv2, (0, 0), 30, (255, 255, 255), thickness = 50)
                cv2.circle(image_cv2, (0, 0), 30, (0, 0, 0), thickness = 40)

            # Display depth map and camera vision in real time
            cv2.imshow("Frame " + str(i), image_cv2)
            cv2.imshow("Depth " + str(i), depth_cv2)
            cv2.imshow("red " + str(i), red)

            a = cv2.waitKey(200)
            if a == 27: # press ESC key to give stop signal
                if not(stop_signal) :
                    stop_signal = True
                else :  # press ESC again to exit program
                    cv2.destroyAllWindows()
                    close_zed2(zed)
                    exit()
        
        cv2.destroyAllWindows()
        corners += [(x, y, z)]

    # [STEP 3] Define position of robot frame origin (point R)
    pA, pB, pC = corners
    xA, yA, zA = pA
    xB, yB, zB = pB
    xC, yC, zC = pC

    # Projecting the relation OR = OC + (4/30)*CB + (1/2)*BA into the camera frame, there is:
    xR = xC + (4/30)*(xB - xC) + (1/2)*(xA - xB)
    yR = yC + (4/30)*(yB - yC) + (1/2)*(yA - yB)
    zR = zC + (4/30)*(zB - zC) + (1/2)*(zA - zB)

    # [STEP 4] Define rotation of robot frame relative to camera frame
    # [SUBSTEP 4.1] Define rotation of robot abscissa axis (x_rob) relative camera's (x_cam = (1, 0, 0))
    # --> x_rob comes from a rotation of angle acos(x_cam.x_rob) around vector (x_cam x x_rob)
    vector_CB = np.array([xB - xC, yB - yC, zB - zC]) 
    norm_CB = scalar_product(vector_CB, vector_CB)**(1/2)
    vector_BA = np.array([xA - xB, yA - yB, zA - zB]) 
    norm_BA = scalar_product(vector_BA, vector_BA)**(1/2)

    x_rob = vector_CB/norm_CB
    y_rob = vector_BA/norm_BA
    z_rob = vector_product(x_rob, y_rob)

    x_cam = np.array([1, 0, 0])
    y_cam = np.array([0, 1, 0])
    z_cam = np.array([0, 0, 1])
    
    print("[DEBUG] Check vectors --------")
    print("     ||x_rob|| =", np.around(scalar_product(x_rob, x_rob)**(1/2), 3))
    print("     ||y_rob|| =", np.around(scalar_product(y_rob, y_rob)**(1/2), 3))
    print("     ||z_rob|| =", np.around(scalar_product(z_rob, z_rob)**(1/2), 3))
    print("     x_rob.y_rob =", np.around(scalar_product(x_rob, y_rob), 3))
    print("     x_rob.z_rob =", np.around(scalar_product(x_rob, z_rob), 3))
    print("     y_rob.z_rob =", np.around(scalar_product(y_rob, z_rob), 3))

    theta = acos(scalar_product(x_cam, x_rob))
    axis_unnormed = vector_product(x_cam, x_rob)
    norm_axis = scalar_product(axis_unnormed, axis_unnormed)**(1/2)
    axis = axis_unnormed/norm_axis

    # Define quaternion associated to rotation
    ox1 = scalar_product(axis, x_cam)*sin(theta/2)
    oy1 = scalar_product(axis, y_cam)*sin(theta/2)
    oz1 = scalar_product(axis, z_cam)*sin(theta/2)
    ow1 = cos(theta/2) 
    q1 = (ox1, oy1, oz1, ow1)
    q1_inverse = (-ox1, -oy1, -oz1, ow1)

    # [SUBSTEP 4.2] Apply inverse rotation to have x_cam and x_rob go in the same direction 
    # and then recover last rotation components
    x_rob_2 = apply_quaternion_rotation(x_rob, q1_inverse)
    y_rob_2 = apply_quaternion_rotation(y_rob, q1_inverse)
    z_rob_2 = apply_quaternion_rotation(z_rob, q1_inverse)

    print("[DEBUG] Check applied rotation --------")
    print("     x_rob_2 =", np.around(x_rob_2), 3)

    # Now, the only difference between (x_cam, y_cam, z_cam) and (x_rob_2, y_rob_2, z_rob_2) is a rotation
    # around x_cam that we can identify
    alpha = acos(scalar_product(y_cam, y_rob_2))
    q2 = (sin(alpha/2), 0, 0, cos(alpha/2))

    # [SUBSTEP 4.3] Combine all this into a single rotation quaternion
    # All in all, in order to get the robot frame, we need to apply q2 and then q1 to the camera frame, 
    # which is equivalent to applying q1 x q2.
    q = quaternion_product(q1, q2)
    set_world_frame(xR, yR, zR, q[0], q[1], q[2], q[3])

    return (xR, yR, zR), q



def cup_detection(zed, threshold=0.6, rect_th=3) :
    cups_coords = []

    # Capture image as well as 3D coordinates in robot frame
    runtime_parameters = sl.RuntimeParameters()
    runtime_parameters.measure3D_reference_frame = sl.REFERENCE_FRAME.WORLD

    real_time_window(zed)

    while True :
        img, depth, point_cloud = left_eye_retrieve(zed)
        image_cv2 = convert_slMat(img)
        depth_cv2 = depth.get_data()
        cv2.imshow("Frame", image_cv2)
        cv2.imshow("Depth", depth_cv2)
        a = cv2.waitKey(200)
        if a == 27: #ESC key
            cv2.destroyAllWindows()
            break   

    image = np.copy(image_cv2)

    # Initiate cup detection on image
    detected, boxes= get_prediction(image_cv2, threshold)

    # Switch to RGB to display a single picture with plt later
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Display detected cup(s?) on image
    if detected==1:
        print("[DEBUG] Cup detection outline boxes -----")
        print(boxes)
        for i in range(len(boxes)):
            # Corners of the rectangle bounding the cup's "region of interest"
            x_top_left=int(boxes[i][0][0])
            y_top_left=int(boxes[i][0][1])
            x_bot_right=int(boxes[i][1][0])
            y_bot_right=int(boxes[i][1][1])

            # Coordinates of the point that the robot should target, i.e.
            # the middle of the bouding rectangle along the x-axis and 
            # two thirds of its height along the y-axis
            # (x axis goes from left to right, y axis goes from top to bottom)
            x=(x_top_left+x_bot_right)//2
            y=y_top_left-(y_top_left-y_bot_right)//3

            # Read position in robot frame thanks to point cloud
            x_cup, y_cup, z_cup = read_point_cloud_pos(point_cloud, y, x)

            if are_not_null([x_cup, y_cup, z_cup]) :
                cups_coords += [(x_cup, y_cup, z_cup)]
                color = (0, 255, 0)
            else :
                color = (255, 0, 0)

            # Display bouding rectangle and target point
            cv2.rectangle(image , (x_top_left , y_top_left) , (x_bot_right , y_bot_right), color=color, thickness= rect_th)
            cv2.circle(image, (x,y), 5 , color=color, thickness= rect_th)

    plt.imshow(image)
    plt.show()
    plt.clf()

    return cups_coords

def main():
    rclpy.init(args=None)

    objectif = Objectif()

    zed = open_zed2()
    setup_camera_world_frame_manual(zed)
    cups_coords = cup_detection(zed)
    print("[FINAL RESULT] ---------------")
    print(cups_coords)
    close_zed2(zed)

    objectif.x, objectif.y, objectif.z = cups_coords[0]
    rclpy.spin_once(objectif)

    objectif.destroy_node()
    rclpy.shutdown()






if __name__ == "__main__" :
    main()