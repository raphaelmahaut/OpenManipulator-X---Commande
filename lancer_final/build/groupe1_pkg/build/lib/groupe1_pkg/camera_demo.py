import rclpy
from rclpy.node import Node


from geometry_msgs.msg import Pose2D

###############
### IMPORTS ###
###############

import cv2 # pip install opencv-python
import numpy as np
from copy import deepcopy


#############################
### RESOURCES AND CREDITS ###
#############################

# https://www.youtube.com/watch?v=Z846tkgl9-U
# https://www.youtube.com/watch?v=oXlwWbU8l2o


########################
### CAMERA FUNCTIONS ###
########################

def openCapture(camera_number_on_pc) :
    """
    This function opens and returns a camera *capture*. It represents the camera being active, and allows to recover frames.
    Once a program is done recovering frames from the camera, the capture *must* be closed with the following lines 
    (or with the closeCapture function below) :

    capture.release()
    cv2.destroyAllWindows()
    """

    print("Opening capture ...")
    capture = cv2.VideoCapture(camera_number_on_pc)
    print("capture is opened:",capture.isOpened())

    while not capture.isOpened() :
        capture = cv2.VideoCapture(camera_number_on_pc)
        print("capture is opened:",capture.isOpened())

    print("-----")
    return capture



def closeCapture(capture) :
    capture.release()
    cv2.destroyAllWindows()



def undistortImage(img, mtx, newcameramtx, dist) :
    h,w = img.shape[:2]

    # Method 1 to undistort the image
    undistorted = cv2.undistort(img, mtx, dist, None, newcameramtx)

    # Method 2 to undistort the image
    mapx,mapy=cv2.initUndistortRectifyMap(mtx,dist,None,newcameramtx,(w,h),5)
    undistorted = cv2.remap(img,mapx,mapy,cv2.INTER_LINEAR)

    return undistorted



mtx = np.array([[426, 0, 332], [0, 426, 198], [0, 0, 1]])
distorsion_coefficients = np.array([[-0.266, 0.07, -0.006, 0.0008, 0.0058]])



############################
### PROCESSING FUNCTIONS ###
############################

def chopChop(img, additional_crop) :
    nb_lines, nb_columns, nb_colors = img.shape
    crop = int((nb_columns - nb_lines)/2)
    return img[additional_crop:nb_lines-additional_crop-1 , crop+additional_crop:nb_columns-crop-additional_crop-1]



def colorQuantization(img, k) : # To reduce the number of colors in an image
    return (np.array(img)//k)*k



def cannyEdgeDetector(img, threshold1, threshold2, color_divider = 8, thickness = 5) :
    
    # Color reduction
    reduced = colorQuantization(img, color_divider)

    # Blur
    blur = cv2.GaussianBlur(reduced, (7, 7), cv2.BORDER_DEFAULT)

    # Convert to grayscale :
    # gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)

    # Canny edge detector
    canny = cv2.Canny(blur, threshold1, threshold2)

    # Dilate the edges to make them thicker
    canny = cv2.dilate(canny, np.ones((thickness,thickness)), iterations=1)

    return canny, blur



def isPlankMisaligned(min_cos_alpha, robot_line, robot_column, plank) :
    # plank is a list of 5 2D points : the center of gravity and the 4 corners of a wooden plank.
    # this function will determine whether this plank is properly aligned to be seized by the robot,
    # and will do so by computing a scalar product.

    x_o, y_o = robot_line, robot_column
    x_g, y_g = plank[0] # center of gravity
    x_a, y_a = plank[1] # first corner
    x_b, y_b = plank[2] # next corner counterclockwise
    x_c, y_c = plank[3] # second next corner counterclockwise

    scalar_product_og_ab = (x_g - x_o)*(x_b - x_a) + (y_g - y_o)*(y_b - y_a)
    scalar_product_og_bc = (x_g - x_o)*(x_c - x_b) + (y_g - y_o)*(y_c - y_b)

    norm_og = ((x_g - x_o)**2 + (y_g - y_o)**2)**(1/2)

    norm_ab = ((x_b - x_a)**2 + (y_b - y_a)**2)**(1/2)
    norm_bc = ((x_c - x_b)**2 + (y_c - y_b)**2)**(1/2)

    if norm_ab > norm_bc : # if AB is the longest edge of the plank
        cos_alpha = abs(scalar_product_og_ab/(norm_og*norm_ab))
    else : # if BC is the longest edge of the plank :
        cos_alpha = abs(scalar_product_og_bc/(norm_og*norm_bc))
    
    # print('cos_alpha', cos_alpha)

    return cos_alpha <= min_cos_alpha



def euclidianNorm2D(u) :
    return (u[0]**2 + u[1]**2)**(1/2)



def is_among_planks(plank_0, planks_list, epsilon) :
    among_planks = False
    g0x, g0y = plank_0[0] # center of gravity
    a0x, a0y = plank_0[1] # point A
    b0x, b0y = plank_0[2] # point B, next to A counterclockwise
    n = len(plank_0) # = 5

    for plank in planks_list :
        same_plank = False

        gx, gy = plank[0]
        same_gravity = euclidianNorm2D((g0x-gx, g0y-gy)) < epsilon

        if same_gravity :
            # if the planks have the same center of gravity, it is enough to check whether they have
            # two adjacent points in common to determine if they are the same.

            for i in range(1, n) :
                ax, ay = plank[i]
                if i == 4 :
                    bx, by = plank[1]
                else :
                    bx, by = plank[i+1]
                
                same_a = euclidianNorm2D((a0x-ax, a0y-ay)) < epsilon
                same_b = euclidianNorm2D((b0x-bx, b0y-by)) < epsilon

                if same_a and same_b :
                    same_plank = True
            
        
        among_planks = among_planks or same_plank
    return among_planks



def cleanContours(edges_img, edges_img_canvas, contours, min_area, max_area) :
    # edges_img is an image of edges, like the one returned by cannyEdgeDetector().
    # contours is a list of its contours as detected by cv2.
    # This function will sort through these contours and eliminate the unwanted ones
    # by erasing them directly on edges_img.

    found_noisy_contour = False

    for contour in contours :
        # Some of the contours might be noise. One way to filter them out is to compute their area.
        area = cv2.contourArea(contour)

        if area > max_area or min_area > area :
            found_noisy_contour = True
            cv2.drawContours(edges_img, contour, -1, color = (0, 0, 0), thickness = 5)
            cv2.drawContours(edges_img_canvas, contour, -1, color = (128, 128, 128), thickness = 5)
        
        else :
            # Each contour is a huge array of points. Fortunately, cv2 can approximate its shape :
            perimeter = cv2.arcLength(contour, closed = True)
            epsilon = 0.03*perimeter
            approximate_shape_corners = cv2.approxPolyDP(contour, epsilon, closed = True)

            # We wish to detect 2D rectangular wooden planks, so we will only process trapezes (4 edges) :
            if len(approximate_shape_corners) != 4 :
                found_noisy_contour = True
                cv2.drawContours(edges_img, contour, -1, color = (0, 0, 0), thickness = 5)
                cv2.drawContours(edges_img_canvas, contour, -1, color = (128, 128, 128), thickness = 5)

    return found_noisy_contour



def findCentersGravity(img, edges_img, edges_img_canvas, min_area, max_area, robot_line, robot_column, min_cos_alpha, draw_contours = True) :

    # The following lines will "extract the contours from the image of the edges".
    # This might seem odd at first glance, why not call this alone and skip detecting the edges beforehand ?
    # That's merely because the canny edge detector is easier to calibrate.
    # All in all, this line is simply used here to separate really fast the shapes drawn by the edges.

    contours, hierarchy = cv2.findContours(edges_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    noisy_contour_present = cleanContours(edges_img, edges_img_canvas, contours, min_area, max_area)

    while noisy_contour_present :
        contours, hierarchy = cv2.findContours(edges_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        noisy_contour_present = cleanContours(edges_img, edges_img_canvas, contours, min_area, max_area)


    planks_list = []

    for contour in contours :

        # Each contour is a huge array of points. Fortunately, cv2 can approximate its shape :
        perimeter = cv2.arcLength(contour, closed = True)
        epsilon = 0.03*perimeter
        approximate_shape_corners = cv2.approxPolyDP(contour, epsilon, closed = True)
        # The variable approximate_shape_corners now contains a very crucial information :
        # The coordinates of each approximated corner 
        # (as in, the lines and the columns of the matrix where they are located).
        # This allows the computation of their center of gravity as well as a change of coordinates 
        # to make them usable by the robot.

        # Let us compute the coordinates of the center of gravity
        sum_column = 0
        sum_line = 0
        points_list = [] #This will store a reordered approximate_shape_corners with the center of gravity as a bonus

        for corner in approximate_shape_corners :
            corner_column = corner[0][0]
            corner_line = corner[0][1] 
            sum_column += corner_column
            sum_line += corner_line
            points_list.append((corner_line, corner_column))

        gravity_column = int(sum_column/4)
        gravity_line = int(sum_line/4)
        points_list = [(gravity_line, gravity_column)] + points_list

        planks_list.append(points_list)

        """
        #########################
        DRAWING ZONE
        #########################
        """

        if draw_contours :

            misaligned = isPlankMisaligned(min_cos_alpha, robot_line, robot_column, points_list)

            # draw plank
            if misaligned :
                chosen_color = (0, 0, 255)
            else :
                chosen_color = (0, 255, 0)
                    
            cv2.drawContours(img, contour, -1, color = chosen_color, thickness = 5)
            cv2.circle(img, (gravity_column, gravity_line), 5, chosen_color, thickness = 5)
            cv2.line(img, (robot_column, robot_line), (gravity_column, gravity_line), (255, 0, 0), thickness = 3)
    
    return planks_list



def convertPlankCoordinates(planks_list, nb_lines_frame, nb_columns_frame, robot_line, robot_column, distance_ratio_x, distance_ratio_y, min_cos_alpha) :
    planks_converted = []
    column_center = nb_columns_frame/2
    line_center = nb_lines_frame/2

    for plank in planks_list :
        plank_converted = []

        for (line, column) in plank :
            x_c = distance_ratio_x*(column - column_center)
            y_c = -distance_ratio_y*(line - line_center)
            x = -y_c+0.11
            y = x_c
            plank_converted.append((x, y))

        plank_converted.append(isPlankMisaligned(min_cos_alpha, robot_line, robot_column, plank))
        planks_converted.append(plank_converted)
    
    return planks_converted



#####################
### CAMERA VISION ###
#####################

def do_nothing(useless_argument) : # despite its looks, this is useful for trackbars used in calibration below.
    pass

def cameraVision_calibrate(camera_number_on_pc, mtx, distorsion_coefficients) :
    min_cos_alpha = 0.5

    cv2.namedWindow("parameters")
    cv2.resizeWindow("parameters", 500, 300)
    cv2.createTrackbar("min area", "parameters", 30, 100, do_nothing)
    cv2.createTrackbar("max area", "parameters", 10, 100, do_nothing)
    cv2.createTrackbar("threshold 1", "parameters", 10, 255, do_nothing)
    cv2.createTrackbar("threshold 2", "parameters", 70, 255, do_nothing)
    cv2.createTrackbar("crop", "parameters", 0, 100, do_nothing)
    cv2.createTrackbar("thickness", "parameters", 4, 14, do_nothing)
    cv2.createTrackbar("color divider", "parameters", 3, 7, do_nothing)
    first_picture = True

    capture = openCapture(camera_number_on_pc)

    while(True):
        ret, frame = capture.read()

        if ret:
            min_area = 100*cv2.getTrackbarPos("min area", "parameters")
            max_area = 1000*cv2.getTrackbarPos("max area", "parameters")
            threshold1 = cv2.getTrackbarPos("threshold 1", "parameters")
            threshold2 = cv2.getTrackbarPos("threshold 2", "parameters")
            crop = cv2.getTrackbarPos("crop", "parameters")
            thickness = 1 + cv2.getTrackbarPos("thickness", "parameters")
            color_divider = 2**cv2.getTrackbarPos("color divider", "parameters")


            frame = undistortImage(frame, mtx, mtx, distorsion_coefficients)
            frame = chopChop(frame, crop)

            if first_picture :
                first_picture = False
                nb_lines, nb_columns, nb_colors = frame.shape
                cv2.namedWindow("elements")
                cv2.resizeWindow("elements", 500, 250)
                cv2.createTrackbar("rob line", "elements", int((nb_lines-1)/20), nb_lines-1, do_nothing)
                cv2.createTrackbar("rob column", "elements", int((nb_columns-1)/2), nb_columns-1, do_nothing)
                cv2.createTrackbar("A line", "elements", int((nb_lines-1)/20), nb_lines-1, do_nothing)
                cv2.createTrackbar("A column", "elements", int((nb_columns-1)/20), nb_columns-1, do_nothing)
                cv2.createTrackbar("B line", "elements", int(19*(nb_lines-1)/20), nb_lines-1, do_nothing)
                cv2.createTrackbar("B column", "elements", int(19*(nb_columns-1)/20), nb_columns-1, do_nothing)
            
            robot_line = cv2.getTrackbarPos("rob line", "elements")
            robot_column = cv2.getTrackbarPos("rob column", "elements")
            a_line = cv2.getTrackbarPos("A line", "elements")
            a_column = cv2.getTrackbarPos("A column", "elements")
            b_line = cv2.getTrackbarPos("B line", "elements")
            b_column = cv2.getTrackbarPos("B column", "elements")

            canny, img2 = cannyEdgeDetector(frame, threshold1, threshold2, color_divider, thickness)
            canny_canvas = deepcopy(canny)
            plank_list = findCentersGravity(img2, canny, canny_canvas, min_area, max_area, robot_line, robot_column, min_cos_alpha)
            cv2.circle(img2, (robot_column, robot_line), 10, (255, 0, 0), thickness = 4)
            cv2.circle(img2, (a_column, a_line), 10, (255, 255, 0), thickness = 4)
            cv2.circle(img2, (b_column, b_line), 10, (255, 0, 255), thickness = 4)

            cv2.imshow("source", img2)
            cv2.imshow("canny", canny)
            cv2.imshow("canny canvas", canny_canvas)

        a = cv2.waitKey(200)
        if a == 27: #ESC key
            break 

    # cv2.destroyAllWindows()
    # cv2.imshow("source", img2)
    # cv2.waitKey(0)
    distance_ratio_x = 0.3/abs(a_column - b_column)
    distance_ratio_y = 0.3/abs(a_line - b_line)
    print('\n######################################\n')
    print(distance_ratio_x)
    print(distance_ratio_y)

    closeCapture(capture)
    return min_area, max_area, threshold1, threshold2, crop, thickness, color_divider, robot_line, robot_column, distance_ratio_x, distance_ratio_y



def cameraVision(camera_number_on_pc, mtx, distorsion_coefficients, min_area, max_area, threshold1, threshold2, crop, thickness, color_divider, robot_line, robot_column, distance_ratio_x, distance_ratio_y) :
    min_cos_alpha = 0.5

    capture = openCapture(camera_number_on_pc)
    list_of_lists = []

    for i in range(20):
        ret, frame = capture.read()

        if ret:
            frame = undistortImage(frame, mtx, mtx, distorsion_coefficients)
            frame = chopChop(frame, crop)
            nb_lines, nb_columns, nb_colors = frame.shape

            canny, img2 = cannyEdgeDetector(frame, threshold1, threshold2, color_divider, thickness)
            canny_canvas = deepcopy(canny)
            planks_list_1frame = findCentersGravity(img2, canny, canny_canvas, min_area, max_area, robot_line, robot_column, min_cos_alpha)

            # cv2.imshow("source", img2)
            # cv2.imshow("canny", canny)
            # cv2.imshow("canny canvas", canny_canvas)

        cv2.waitKey(200)
        list_of_lists.append(planks_list_1frame)
    
    closeCapture(capture)
    planks_list_final = []

    for planks_list in list_of_lists :
        for plank in planks_list :
            gx, gy = plank[0]
            ax, ay = plank[1]
            epsilon = euclidianNorm2D((gx-ax, gy-ay))/3
            if not(is_among_planks(plank, planks_list_final, epsilon)) :
                planks_list_final.append(plank)
                # cv2.circle(frame, (gy, gx), 10, (255, 0, 0), thickness = 4)
    
    # cv2.imshow("detected planks :"+str(len(planks_list_final)), frame)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    planks_converted = convertPlankCoordinates(planks_list_final, nb_lines, nb_columns, robot_line, robot_column, distance_ratio_x, distance_ratio_y, min_cos_alpha)

    
    

    if len(planks_converted) != 0 :
        if len(planks_converted[0]) != 0 :
            print("gravity :",planks_converted[0][0])
            print("\n#######################\n")
            return planks_converted[0][0]
        else :
            print("Gravity : C'est vide")
            print("\n#######################\n")
            return (0.00, 0.09)
    else :
        print("Gravity : C'est vide")
        print("\n#######################\n")
        return (0.00, 0.09)



def cameraVision_test(camera_number_on_pc) :
    min_cos_alpha = 0.5
    mtx = np.array([[426, 0, 332], [0, 426, 198], [0, 0, 1]])
    distorsion_coefficients = np.array([[-0.266, 0.07, -0.006, 0.0008, 0.0058]])
    min_area = 3000
    max_area = 10000
    threshold1 = 10
    threshold2 = 70
    crop = 0
    thickness = 5
    color_divider = 8
    robot_line = 62
    robot_column = 176
    distance_ratio_x = 0.0009677
    distance_ratio_y = 0.001

    

    capture = openCapture(camera_number_on_pc)
    list_of_lists = []

    for i in range(20):
        ret, frame = capture.read()

        if ret:
            frame = undistortImage(frame, mtx, mtx, distorsion_coefficients)
            frame = chopChop(frame, crop)
            nb_lines, nb_columns, nb_colors = frame.shape

            canny, img2 = cannyEdgeDetector(frame, threshold1, threshold2, color_divider, thickness)
            canny_canvas = deepcopy(canny)
            planks_list_1frame = findCentersGravity(img2, canny, canny_canvas, min_area, max_area, robot_line, robot_column, min_cos_alpha)

        cv2.waitKey(200)
        list_of_lists.append(planks_list_1frame)
    
    closeCapture(capture)
    planks_list_final = []

    for planks_list in list_of_lists :
        for plank in planks_list :
            gx, gy = plank[0]
            ax, ay = plank[1]
            epsilon = euclidianNorm2D((gx-ax, gy-ay))/3
            if not(is_among_planks(plank, planks_list_final, epsilon)) :
                planks_list_final.append(plank)

    planks_converted = convertPlankCoordinates(planks_list_final, nb_lines, nb_columns, robot_line, robot_column, distance_ratio_x, distance_ratio_y, min_cos_alpha)
    print("\n#######################\n")
    print("gravity :",planks_converted[0][0])

    return planks_converted[0][0]



def test() :
    min_area, max_area, threshold1, threshold2, crop, thickness, color_divider, robot_line, robot_column, distance_ratio_x, distance_ratio_y = cameraVision_calibrate(0, mtx, distorsion_coefficients)
    planks_list = cameraVision(0, mtx, distorsion_coefficients, min_area, max_area, threshold1, threshold2, crop, thickness, color_divider, robot_line, robot_column, distance_ratio_x, distance_ratio_y)

    print('\n######################################\n')
    print("Nb of planks:",len(planks_list))
    print('\n######################################\n')

    for plank in planks_list :
        print(plank)
        print('\n--------------------------------------\n')


parameters = 0

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Pose2D, 'camera', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        global parameters
        min_area, max_area, threshold1, threshold2, crop, thickness, color_divider, robot_line, robot_column, distance_ratio_x, distance_ratio_y = parameters
        mtx = np.array([[426, 0, 332], [0, 426, 198], [0, 0, 1]])
        distorsion_coefficients = np.array([[-0.266, 0.07, -0.006, 0.0008, 0.0058]])

        msg = Pose2D()
        msg.x, msg.y= cameraVision(2, mtx, distorsion_coefficients, min_area, max_area, threshold1, threshold2, crop, thickness, color_divider, robot_line, robot_column, distance_ratio_x, distance_ratio_y)
        self.publisher_.publish(msg)
        self.get_logger().info('envoie de données')
        self.i += 1


def main(args=None):
    global parameters
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    mtx = np.array([[426, 0, 332], [0, 426, 198], [0, 0, 1]])
    distorsion_coefficients = np.array([[-0.266, 0.07, -0.006, 0.0008, 0.0058]])
    parameters = cameraVision_calibrate(2, mtx, distorsion_coefficients)

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()