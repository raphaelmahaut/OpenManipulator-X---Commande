# +---------+
# | IMPORTS |
# +---------+

import numpy as np
import torchvision.transforms as T
import torchvision 
import numbers



# +----------------------------+
# | IMAGE PROCESSING FUNCTIONS |
# +----------------------------+

def do_nothing(a) :
    # A function that does nothing, used as the mandatory callback function for cv2 trackbars
    pass



def are_numbers(argslist) :
    for arg in argslist :
        try :
            bool1 = arg >= 0
            bool2 = arg < 0
            if not(bool1 or bool2) :
                return False
        except :
            return False
    return True



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
    cols_nbs = np.array([[j for j in range(cols)] for i in range(lines)])

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
    cols_nbs = np.array([[j for j in range(cols)] for i in range(lines)])

    #print("[DEBUG FIND RED DOT]", lines, cols)
    #print(lines_nbs)
    #print(cols_nbs)

    colornp = color_filter(img, "red", color_min, other_color_max)
    weight_sum = np.sum(np.sum(colornp))

    if weight_sum > 0 :
        line_times_weight_sum = np.sum(np.sum(colornp * lines_nbs))
        col_times_weight_sum = np.sum(np.sum(colornp * cols_nbs))

        #print("[DEBUG FIND RED DOT] iM[i, j], jM[i, j], M[i, j] :", line_times_weight_sum, col_times_weight_sum, weight_sum)

        centroid_line = int(line_times_weight_sum/weight_sum)
        centroid_col = int(col_times_weight_sum/weight_sum)
        points = points + [(centroid_line, centroid_col)]

    return points



# +------------------------+
# | CUP DETECTION FUNCTION |
# +------------------------+

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
            return True, cup_boxes
        else:
            return False, None
    else:
        return False, None