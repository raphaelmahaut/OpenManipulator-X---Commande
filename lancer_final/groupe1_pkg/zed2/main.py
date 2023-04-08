# +---------+
# | IMPORTS |
# +---------+

import zed2.image_processing_cup_detection as ip
import zed2.geometry as g
import zed2.zed2_handling as zed2
import cv2
import numpy as np

DEBUG = True
MEASURE_UNCERTAINTY = True
NB_MEASURES = 10


# +-----------------------------------------------+
# | COMBINE MODULES TO DETECT ROBOT FRAME AND CUP |
# +-----------------------------------------------+

def detect_robot_frame_manual(zed):
    """
    Parameters:
        - zed: sl.Camera(), open instance of sl.Camera()

    Returns:
        - m_cam2rob: ndarray float[][], frame change matrix used to change from camera frame
        to robot frame

    This function's purpose is to detect the robot within the Zed2 camera's vision to the build and
    return the frame change matrix allowing to change from camera frame to robot frame.

    This function requires a manual "calibration" by the user. In order for this function to work, 
    the user will be prompted thrice to position a small, scarlet red item on a corner of the robot's
    workbench (In sequence: top-left corner (point A), then top-right corner (point B), then 
    bottom-right corner (point C)). The Zed2 camera will measure the 3D coordinates of each of these
    points, which will be used to reconstruct the robot's position and frame relative to the 
    camera frame in order to build the frame change matrix that this function returns.

    Calling this function will display 4 windows, 3 times:
        - "Left eye frame": this window displays a livefeed of what the Zed2 camera's left eye can see, 
        albeit with a very low framerate. If "scarlet red pixels" (as defined by the calibration controls
        explained below) are visible on the livefeed, a colored circle will indicate their barycenter 
        (green if the Zed2 camera can measure the depth of the barycenter, red if it cannot).
        - "Left eye depth": This window displays the depth map associated to the "Left eye frame" 
        livefeed.
        - "Left eye frame (red)": this window displays a filtered version of the "Left eye frame"
        livefeed, only the pixels that are both "red enough" and "sufficiently not blue nor green"
        (referred to as "scarlet red"), as defined by the calibration controls explained below, are
        visible. The circle visible on "Left eye frame" is the barycenter of the pixels visible here.
        - "Calibration controls": This window displays 3 trackbars used to handle the detection of the
        scarlet red objects placed by the user:
            - "color_min": any pixel with a red component lower than this will not be considered
            scarlet red. Default value of 200 should work well under most lightings.
            - "other_color_max": any pixel with a blue or green component higher than this will not
            be considered scarlet red. Default value of 100 might need to be lowered a little if needed.
            - "ready": used to order the Zed2 camera to stop its livefeed and measure the 
            coordinates (in the camera frame) of the barycenter of the scalet red pixels. As scarlet red
            isn't such a common color, the other 2 trackbars should be adjusted so that only the
            scarlet red object placed by the user is visible on "Left eye frame (red)".

    Once the user slides the trackbar "ready" in the "Calibration controls" window to the value 1, 
    all windows will close and the position of the scarlet red object will be measured. The windows
    will reappear if less than 3 positions have been measured this way. Once 3 measures have been done, 
    this function will return the frame change matrix allowing to change from camera frame to robot
    frame.

    [WARNING]: Once this function is called, do NOT move in any way whatsoever the camera or the robot's
    workbench until the robot ceases to need the camera's assistance. Otherwise, every measurement taken
    will be wrong.
    """
    corners = []

    # Start calibration : detect a red bottle cap (or the like) thrice,
    # associated to points A, B and C in order
    for bottlecap in range(3):
        # Create trackbars for calibration
        cv2.namedWindow("Calibration controls")
        cv2.createTrackbar("color_min", "Calibration controls",
                           200, 255, ip.do_nothing)
        cv2.createTrackbar("other_color_max",
                           "Calibration controls", 100, 255, ip.do_nothing)
        cv2.createTrackbar("ready", "Calibration controls",
                           0, 1, ip.do_nothing)

        stop_signal = False  # Slide "ready" trackbar to 1 to activate this
        # Turns to True when detected bottlecap's depth can be measured by zed2
        depth_detection_ok = False

        while not(stop_signal and depth_detection_ok):
            # Recover trackbar calibration values
            color_min = cv2.getTrackbarPos("color_min", "Calibration controls")
            other_color_max = cv2.getTrackbarPos(
                "other_color_max", "Calibration controls")
            stop_signal_float = cv2.getTrackbarPos(
                "ready", "Calibration controls")

            # Take picture, depth map and point cloud with zed2
            image, depth, point_cloud = zed2.left_eye_retrieve(zed)
            image_cv2 = np.array(zed2.convert_slMat(image))
            depth_img = depth.get_data()
            depth_3Dimg = np.array([[[x, x, x] for x in line]
                                   for line in depth_img])
            red = ip.color_filter(image_cv2, "red", color_min, other_color_max)

            # Look for red bottlecap
            points_list = ip.find_red_dot(
                image_cv2, color_min, other_color_max)
            found_point = len(points_list) >= 1

            if found_point:
                point = points_list[0]
                line, col = point[0], point[1]

                # Read point coordinates in camera frame thanks to point cloud
                x, y, z = zed2.read_point_cloud_pos(point_cloud, line, col)
                try:
                    x = float(x)
                    y = float(y)
                    z = float(z)
                    all_are_numbers = ip.are_numbers([x, y, z])
                except:
                    all_are_numbers = False

                if DEBUG:
                    print("[DEBUG] line, col :", line, col)
                    print("        x, y, z :", x, y, z)
                    print("        all_are_numbers :", all_are_numbers)

                if all_are_numbers:
                    # At this point, the zed2 managed to read the depth of the detected red bottlecap
                    # But let's not update depth_detection_ok yet, let's just indicate on an image
                    # where the thing was found with a green color.
                    color = (0, 255, 0)  # Green in BGR
                else:
                    color = (0, 0, 255)  # Red in BGR

                # Display that a red bottlecap was found, with an appropriate color to indicate
                # if zed2 could sense its depth
                cv2.circle(image_cv2, (col, line), 10, color, thickness=5)
                cv2.circle(depth_3Dimg, (col, line), 10, color, thickness=5)

            else:
                if DEBUG:
                    print("[DEBUG] Found no red dot")

            # Display depth map and camera vision in real time
            cv2.imshow("Left-eye frame " + str(bottlecap), image_cv2)
            cv2.imshow("Left-eye depth " + str(bottlecap), depth_3Dimg)
            cv2.imshow("Left-eye frame (red) " + str(bottlecap), red)
            a = cv2.waitKey(200)
            if a == 27:  # press ESC key to exit program
                cv2.destroyAllWindows()
                zed2.close_zed2(zed)
                exit()

            # Update if depth detection was ok and if stop signal was given
            try:
                depth_detection_ok = all_are_numbers
                stop_signal = (stop_signal_float > 0.99)
            except:
                depth_detection_ok = False
                stop_signal = False

        # We just exited the while() loop, hence we found the red bottlecap in space
        # We can recover the point found and add it to the list of the robot's workbench corners
        corners += [(x, y, z)]
        cv2.destroyAllWindows()

    # We just exited the for() loop, therefore we have found all 3 corners necessary to indentify
    # the robot's frame. First, let's recover the coordinates of these points:
    pA, pB, pC = corners[0], corners[1], corners[2]
    xA, yA, zA = pA[0], pA[1], pA[2]
    xB, yB, zB = pB[0], pB[1], pB[2]
    xC, yC, zC = pC[0], pC[1], pC[2]

    # Projecting the relation OR = OC + (4/30)*CB + (1/2)*BA into the camera frame, we can
    # find the coordinates of the robot's frame's origin:
    """
    A --------- B
    |     ^ x   |
    |     |     |
    |  y<-R     |
    + --------- C
    """
    xR = xC + (4/30)*(xB - xC) + (1/2)*(xA - xB)
    yR = yC + (4/30)*(yB - yC) + (1/2)*(yA - yB)
    zR = zC + (4/30)*(zB - zC) + (1/2)*(zA - zB)

    # Now, let's use points A, B and C to define the unit vectors of the robot's frame:
    vector_CB = np.array([xB - xC, yB - yC, zB - zC])
    norm_CB = g.norm_3D(vector_CB)
    vector_BA = np.array([xA - xB, yA - yB, zA - zB])
    norm_BA = g.norm_3D(vector_BA)

    x_rob = vector_CB/norm_CB
    y_rob = vector_BA/norm_BA
    z_rob = g.vector_product_3D(x_rob, y_rob)

    # Usually, x and y are not exactly orthogonal, so we redefine y using x and a renormalized z
    z_rob_norm = g.norm_3D(z_rob)
    z_rob = z_rob / z_rob_norm
    y_rob = g.vector_product_3D(z_rob, x_rob)

    if DEBUG:
        m_rob2cam_rotation, m_cam2rob_rotation = g.frame_change_rotation_matrices(
            x_rob, y_rob, z_rob)
        print()
        print("[DEBUG] Robot frame as seen from camera frame:")
        print("        x_rob =", np.around(np.dot(mat, [1, 0, 0, 1]), 3))
        print("        y_rob =", np.around(np.dot(mat, [0, 1, 0, 1]), 3))
        print("        z_rob =", np.around(np.dot(mat, [0, 0, 1, 1]), 3))
        print()
        print("[DEBUG] Position of robot frame origin as seen from camera frame:")
        print("        x_R =", xR)
        print("        y_R =", yR)
        print("        z_R =", zR)

    m_rob2cam, m_cam2rob = g.frame_change_matrices(
        [xR, yR, zR], x_rob, y_rob, z_rob)

    return m_cam2rob


def detect_cup_manual(zed):
    """
    Parameters:
        - zed: sl.Camera(), open instance of sl.Camera()

    Returns:
        - array float[], array of length 3 of the 3D coordinates in the camera frame
        of a detected cup

    This function's purpose is to detect drinking cups visible by the Zed2 camera, and return the 3D
    coordinates in the camera frame of one of them.

    Calling this function will display 3 windows:
        - "Left eye frame": This window displays a livefeed of what the Zed2 camera's left eye can see, 
        albeit with a extremely low framerate. Each cup detected on the picture will be highlighted by 
        a colored rectangle - green if the Zed2 camera can measure its depth, red if it cannot.
        - "Left eye depth": This window displays the depth map associated to the "Left eye frame" 
        livefeed.
        - "Calibration controls": This window displays 2 trackbars used to handle the cup detection:
            - "100 * threshold": used to handle the threshold used by the cup detection
            aglorithm. Its default value of 60 (threshold = 0.6) should work in most cases.
            - "ready": used to order the Zed2 camera to stop its livefeed and measure the 
            coordinates (in the camera frame) of the next cup it detects.

    Once the user slides the trackbar "ready" in the "Calibration controls" window to the value 1, 
    all windows will close and this function will return the 3D coordinates in the camera frame of 
    a random visible cup the depth of which the Zed2 camera managed to measure.
    """
    # Create trackbars for calibration
    cv2.namedWindow("Calibration controls")
    cv2.createTrackbar("100 * threshold",
                       "Calibration controls", 60, 100, ip.do_nothing)
    cv2.createTrackbar("ready", "Calibration controls", 0, 1, ip.do_nothing)

    stop_signal = False  # Slide "ready" trackbar to 1 to activate this
    # Turns to True when detected cup's depth can be measured by zed2
    depth_detection_ok = False

    while not(stop_signal and depth_detection_ok):
        # Recover trackbar calibration values
        threshold = cv2.getTrackbarPos(
            "100 * threshold", "Calibration controls") / 100
        stop_signal_float = cv2.getTrackbarPos("ready", "Calibration controls")

        # Take picture, depth map and point cloud with zed2
        image, depth, point_cloud = zed2.left_eye_retrieve(zed)
        image_cv2 = np.array(zed2.convert_slMat(image))
        depth_img = depth.get_data()
        depth_3Dimg = np.array([[[x, x, x] for x in line]
                               for line in depth_img])

        # Initiate cup detection on image
        image_cv2_copy = np.copy(image_cv2)
        detected, boxes = ip.get_prediction(image_cv2_copy, threshold)

        if detected:
            all_are_numbers = [False for q in boxes]
            colors = [(0, 0, 255) for q in boxes]
            cups_coords = [[0, 0, 0] for q in boxes]

            # if DEBUG :
            #print("[DEBUG] Cup detection outline boxes -----")
            # print(boxes)

            for i in range(len(boxes)):
                # Corners of the rectangle bounding the cup's "region of interest"
                x_top_left = int(boxes[i][0][0])
                y_top_left = int(boxes[i][0][1])
                x_bot_right = int(boxes[i][1][0])
                y_bot_right = int(boxes[i][1][1])

                # Coordinates of the point that the robot should target, i.e.
                # the middle of the bouding rectangle along the x-axis and
                # two thirds of its height along the y-axis
                # (x axis goes from left to right, y axis goes from top to bottom)
                col_target = (x_top_left+x_bot_right)//2
                line_target = y_top_left-(y_top_left-y_bot_right)//3

                # Read position in camera frame thanks to point cloud
                x_cup, y_cup, z_cup = zed2.read_point_cloud_pos(
                    point_cloud, line_target, col_target)
                try:
                    x_cup = float(x_cup)
                    y_cup = float(y_cup)
                    z_cup = float(z_cup)
                    all_are_numbers[i] = ip.are_numbers([x_cup, y_cup, z_cup])
                except:
                    all_are_numbers[i] = False

                if DEBUG:
                    print(
                        "        [DEBUG] Coordinates in camera frame of cup n°", i)
                    print("                x_cup, y_cup, z_cup :",
                          x_cup, y_cup, z_cup)
                    print("                all_are_numbers :",
                          all_are_numbers[i])

                if all_are_numbers[i]:
                    # At this point, the zed2 managed to read the depth of the detected cup
                    # But let's not update depth_detection_ok yet, let's just indicate on an image
                    # where the thing was found with a green color.
                    colors[i] = (0, 255, 0)  # Green in BGR
                    # Also, let's store their coords
                    cups_coords[i] = [x_cup, y_cup, z_cup]

                # Display that a cup was found, with an appropriate color to indicate
                # if zed2 could sense its depth
                cv2.circle(image_cv2, (col_target, line_target),
                           10, colors[i], thickness=5)
                cv2.circle(depth_3Dimg, (col_target, line_target),
                           10, colors[i], thickness=5)
                # Also display bouding rectangle
                cv2.rectangle(image_cv2, (x_top_left, y_top_left),
                              (x_bot_right, y_bot_right), color=colors[i], thickness=3)
                cv2.rectangle(depth_3Dimg, (x_top_left, y_top_left),
                              (x_bot_right, y_bot_right), color=colors[i], thickness=3)

        else:
            if DEBUG:
                print("[DEBUG] Found no cup")

        # Display depth map and camera vision in real time
        cv2.imshow("Left-eye frame", image_cv2)
        cv2.imshow("Left-eye depth", depth_3Dimg)
        a = cv2.waitKey(200)
        if a == 27:  # press ESC key to exit program
            cv2.destroyAllWindows()
            zed2.close_zed2(zed)
            exit()

        # Update if depth detection was ok and if stop signal was given
        try:
            depth_detection_ok = np.any(all_are_numbers)
            stop_signal = (stop_signal_float > 0.99)
        except:
            depth_detection_ok = False
            stop_signal = False

    # We just exited the while() loop, therefore we found at least one cup
    # the coordinates of which the zed2 managed to grab. Let's find it.
    flag = True
    ok_cup_index = -1
    for i in range(len(all_are_numbers)):
        if all_are_numbers[i] and flag:
            ok_cup_index = i
            flag = False

    if DEBUG:
        print("[DEBUG] Found a cup with properly read coords")
        print("        at index", ok_cup_index, "in following array :")
        print("        ", all_are_numbers)
        print("        with coords :", cups_coords[ok_cup_index])

    return cups_coords[ok_cup_index]


# +--------------------------+
# | MEASURE ZED2 UNCERTAINTY |
# +--------------------------+

def measure_zed2_uncertainty(zed, nb_measures):
    """
    Parameters:
        - zed: sl.Camera(), open instance of sl.Camera()
        - nb_measures: int, number of measures to be taken for each point

    Returns:
        - measures: array float[][], measures[i] is the array of measures taken for point i

    This function is used to try and measure the uncertainty on the Zed2 camera's position readings
    during the "calibration" phase of the function detect_robot_frame_manual(zed). It works exactly like
    the latter, except for what is returned: the measures for each corner of the workbench.
    """
    measures = [[], [], []]

    for bottlecap in range(3):
        # Create trackbars for calibration
        cv2.namedWindow("Calibration controls")
        cv2.createTrackbar("color_min", "Calibration controls",
                           200, 255, ip.do_nothing)
        cv2.createTrackbar("other_color_max",
                           "Calibration controls", 100, 255, ip.do_nothing)
        cv2.createTrackbar("ready", "Calibration controls",
                           0, 1, ip.do_nothing)
        cv2.createTrackbar("stop", "Calibration controls", 0, 1, ip.do_nothing)

        start_signal = False  # Slide "ready" trackbar to 1 to activate this
        stop_signal = False  # Slide "stop" trackbar to 1 to activate this
        count = 0

        while not(stop_signal) and count < nb_measures:
            # Recover trackbar calibration values
            color_min = cv2.getTrackbarPos("color_min", "Calibration controls")
            other_color_max = cv2.getTrackbarPos(
                "other_color_max", "Calibration controls")
            start_signal_float = cv2.getTrackbarPos(
                "ready", "Calibration controls")
            try:
                start_signal = (start_signal_float > 0.99)
            except:
                start_signal = False
            stop_signal_float = cv2.getTrackbarPos(
                "stop", "Calibration controls")

            # Take picture, depth map and point cloud with zed2
            image, depth, point_cloud = zed2.left_eye_retrieve(zed)
            image_cv2 = np.array(zed2.convert_slMat(image))
            depth_img = depth.get_data()
            depth_3Dimg = np.array([[[x, x, x] for x in line]
                                   for line in depth_img])
            red = ip.color_filter(image_cv2, "red", color_min, other_color_max)

            # Look for red bottlecap
            points_list = ip.find_red_dot(
                image_cv2, color_min, other_color_max)
            found_point = len(points_list) >= 1

            if found_point:
                point = points_list[0]
                line, col = point[0], point[1]

                # Read point coordinates in camera frame thanks to point cloud
                x, y, z = zed2.read_point_cloud_pos(point_cloud, line, col)
                try:
                    x = float(x)
                    y = float(y)
                    z = float(z)
                    all_are_numbers = ip.are_numbers([x, y, z])
                except:
                    all_are_numbers = False

                if all_are_numbers:
                    # At this point, the zed2 managed to read the depth of the detected red bottlecap
                    color = (0, 255, 0)  # Green in BGR
                    if start_signal:
                        count += 1
                        measures[bottlecap] += [(np.around(x, 5),
                                                 np.around(y, 5), np.around(z, 5))]
                else:
                    color = (0, 0, 255)  # Red in BGR

                # Display that a red bottlecap was found, with an appropriate color to indicate
                # if zed2 could sense its depth
                cv2.circle(image_cv2, (col, line), 10, color, thickness=5)
                cv2.circle(depth_3Dimg, (col, line), 10, color, thickness=5)

            # Display depth map and camera vision in real time
            cv2.imshow("Left-eye frame " + str(bottlecap), image_cv2)
            cv2.imshow("Left-eye depth " + str(bottlecap), depth_3Dimg)
            cv2.imshow("Left-eye frame (red) " + str(bottlecap), red)
            a = cv2.waitKey(200)
            if a == 27:  # press ESC key to exit program
                cv2.destroyAllWindows()
                zed2.close_zed2(zed)
                exit()

            # Update if stop signal was given
            try:
                stop_signal = (stop_signal_float > 0.99)
            except:
                stop_signal = False

            print("[POINT "+str(bottlecap)+"] measure count :", count)

        cv2.destroyAllWindows()

    return measures


def coordonnees():

    if MEASURE_UNCERTAINTY:
        zed = zed2.open_zed2()
        measures = measure_zed2_uncertainty(zed, NB_MEASURES)
        zed2.close_zed2(zed)

        print("----- Measures for point A -----")
        print(measures[0])
        print()
        print("----- Measures for point B -----")
        print(measures[1])
        print()
        print("----- Measures for point C -----")
        print(measures[2])
        print()

    else:
        zed = zed2.open_zed2()
        mat = detect_robot_frame_manual(zed)
        cup_coords = detect_cup_manual(zed)
        zed2.close_zed2(zed)

        cup_coords_np = np.array(
            [cup_coords[0], cup_coords[1], cup_coords[2], 1])
        cup_coords_rob = np.dot(mat, cup_coords_np)

        print("----- CUP COORDS IN ROBOT FRAME: -----")
        print("x_cup =", np.around(cup_coords_rob[0], 3))
        print("y_cup =", np.around(cup_coords_rob[1], 3))
        print("z_cup =", np.around(cup_coords_rob[2], 3))

    return cup_coords_rob[0], cup_coords_rob[1], cup_coords_rob[2]
