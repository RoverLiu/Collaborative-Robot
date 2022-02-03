import cv2
import numpy as np
# from sklearn.linear_model import LinearRegression
import collections
import pyrealsense2 as rs
import json, ast
import matplotlib.pyplot as plt

def camera_init():
    # import RealSense preset configuration
    jsonObj = json.load(open("/home/rover/librealsense/config/MidDensityPreset.json"))
    jsonObj = ast.literal_eval(json.dumps(jsonObj))
    json_string = str(jsonObj).replace("'", '\"')

    # print(1)
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    freq = int(jsonObj['stream-fps'])
    config.enable_stream(rs.stream.depth, int(jsonObj['stream-width']), int(jsonObj['stream-height']), rs.format.z16, int(jsonObj['stream-fps']))
    config.enable_stream(rs.stream.color, int(jsonObj['stream-width']), int(jsonObj['stream-height']), rs.format.bgr8, int(jsonObj['stream-fps']))
    print('width: ' + str(int(jsonObj['stream-width'])) + ' height: ' + str(int(jsonObj['stream-height'])) + ' fps: ' + str(int(jsonObj['stream-fps'])))
    cfg = pipeline.start(config)
    dev = cfg.get_device()
    advnc_mode = rs.rs400_advanced_mode(dev)
    advnc_mode.load_json(json_string)
    return 1

def process_image(pipeline, calibrate):
    cX = 0
    cY = 0
    count = 0
    camera_coordinates={}
    rectangles = dict() # store bounding rectangles in the form of dict. key = cX, val = (cY, angle)

    # print("Enter to start")
    # raw_input()

    ## begin real time detection
    camera_coordinates={}
    count += 1
    print ("================================begin loop "+str(count)+"==============================")
    pre_frame = pipeline.wait_for_frames()
    color_frame = pre_frame.get_color_frame()
    frame = np.asanyarray(color_frame.get_data())

    depth_frame = pre_frame.get_depth_frame()
    depth_image = np.asanyarray(depth_frame.get_data())
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

    ######################## RGB IMAGE PROCESSING ###################################
    gray_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    #blur = cv2.GaussianBlur(gray_frame,(5,5),cv2.BORDER_DEFAULT)
    blur = cv2.bilateralFilter(gray_frame,3, 75, 75)

    thresh,th1 = cv2.threshold(blur,127,255,cv2.THRESH_BINARY)
    print('threshold used: ' + str(thresh))

    th2 = cv2.adaptiveThreshold(th1,255,cv2.ADAPTIVE_THRESH_MEAN_C,\
            cv2.THRESH_BINARY,3,2)
    th3 = cv2.adaptiveThreshold(th1,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
            cv2.THRESH_BINARY,3,2)

    kernel = np.ones((5,5), np.uint8)
    erosion = cv2.erode(th3,kernel, iterations = 1)
    _,contours, _ = cv2.findContours(erosion, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    cv2.imshow('binary',th1)
    cv2.imshow('mean',th2)
    cv2.imshow('gauss',erosion)
    centre = 0;

    ####################### BLOCK DETECTION #################################
    for contour in contours:
        area = cv2.contourArea(contour)

        # area of a block laid flat is around 1400, on its side is 1000
        if area < 900 or area > 2500:
            continue

        peri = cv2.arcLength(contour,True)
        approx = cv2.approxPolyDP(contour,0.02*peri, True)
        (x,y,w,h) = cv2.boundingRect(approx)

        # find center
        M = cv2.moments(contour)

        temp_x = cX
        temp_y = cY
        cX = int(M["m10"]/M["m00"])
        cY = int(M["m01"]/M["m00"])

        # disregard blocks in the construction site
        if cX >= 260 and cX <= 370 and cY >= 320:
            continue

        # to avoid double scan
        if (np.abs(temp_x - cX) < 20 and np.abs(temp_y - cY) < 20):
            continue

        cv2.circle(frame,(cX,cY),7,(0,255,0),-1)
        centre += 1

        # find distance
        distance = depth_frame.get_distance(int(cX),int(cY))

        rect = cv2.minAreaRect(contour)

        # make sure length > width and angle is positive, measured clockwise from vertical and in range (0,180)
        temp_rect = list(rect)
        if rect[1][0] > rect[1][1]:
            temp_rect_size = list(temp_rect[1])
            temp_rect_size[0] = rect[1][1]
            temp_rect_size[1] = rect[1][0]
            temp_rect[1] = tuple(temp_rect_size)
            temp_rect[2] = (temp_rect[2] - 90) % 180
        else:
            temp_rect[2] = temp_rect[2] % 180

        rect = tuple(temp_rect)
        rectangles[rect[0][0]] = (rect[0][1], rect[2])

        box = cv2.boxPoints(rect) # cv2.boxPoints(rect) for OpenCV 3.x
        box = np.int0(box)
        cv2.drawContours(frame,[box],0,(0,0,255),2)
        print("block " + str(len(rectangles)-1) + " X: "+ str(int(rect[0][0])) + ", Y: "+ str(int(rect[0][1])) +
        ", w: "+ str(int(rect[1][0])) +", l: "+ str(int(rect[1][1])) +", distance: " + str(distance)+ ", area: " + str(area))

    ############################# DEPTH FRAME POST PROCESSING ######################################3
    colorizer = rs.colorizer()

    decimation = rs.decimation_filter()
    spatial = rs.spatial_filter()
    temporal = rs.temporal_filter()
    hole_filling = rs.hole_filling_filter()
    depth_to_disparity = rs.disparity_transform(True)
    disparity_to_depth = rs.disparity_transform(False)

    # apply post processing
    depth_frame = decimation.process(depth_frame)
    depth_frame = depth_to_disparity.process(depth_frame)
    depth_frame = spatial.process(depth_frame)
    depth_frame = temporal.process(depth_frame)
    depth_frame = disparity_to_depth.process(depth_frame)
    depth_frame = hole_filling.process(depth_frame)

    colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())
    plt.rcParams["axes.grid"] = False
    plt.rcParams['figure.figsize'] = [8, 4]

    plt.imshow(colorized_depth)
    #plt.show()

    #images = np.hstack((frame, depth_colormap))
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', frame)
    cv2.waitKey(300000)
    cv2.destroyAllWindows()

    block_coords = sort_xy(rectangles).items()

    ########## CAMERA ROBOT COORDINATE TRANSFORM CALIBRAION ###########
    if calibrate == 'true':
        print('----left x-----')
        poly_lx = find_coord_transform(block_coords, "x", "left")
        print('----left y----')
        poly_ly = find_coord_transform(block_coords, "y", "left")
        print('----right x----')
        poly_rx = find_coord_transform(block_coords, "x", "right")
        print('----right y----')
        poly_ry = find_coord_transform(block_coords, "y", "right")
        calibrate = 'false'
        transforms = [poly_lx, poly_ly, poly_rx, poly_ry]
        transform_file = open("transform_file.py", "w")
        print(transforms)
        transform_file.write('from numpy import poly1d\n')
        transform_file.write('transforms = ' + repr(transforms))
        transform_file.write("")
        transform_file.close()
        return process_image(pipeline, 'false')
    else:
        return block_coords

def sort_xy(cor):
    sort_cor = collections.OrderedDict(sorted(cor.items()))
    return sort_cor

def find_coord_transform(camera, mode, arm):

    # IMPORTANT: arrange robot coords from rightmost to leftmost block (ascending y)
    if arm == "left":
        if mode == "x":
            robot_coords = np.array([0.294920862983, 0.498012230063, 0.27425901116, 0.416192585244, 0.256556702179])
        else:
            robot_coords = np.array([-0.198502716329, -0.125283012758, -0.0151784497153, 0.0316408725846, 0.181887151335])
    else:
        if mode == "x":
            robot_coords = np.array([0.291230358078, 0.493605238349, 0.271157570317, 0.41236923714, 0.254027575456])
        else:
            robot_coords = np.array([-0.191638368408, -0.125145788267, -0.0110518234848, 0.036689806807, 0.185925557766])

    camera_coords = []
    for i in range(len(camera)):
        if mode == "y":
            camera_coords.append(camera[i][0]) # camera x coords correspond to robot y coords and vice versa
        else:
            camera_coords.append(camera[i][1][0])
    camera_array = np.asarray(camera_coords)
    poly = np.poly1d(np.polyfit(camera_array, robot_coords, 1))
    print(poly)
    plt.figure(5)
    plt.scatter(camera_array, robot_coords)
    xx = np.linspace(camera_array.min(), camera_array.max(), 500)
    plt.plot(xx, poly(xx))
    plt.show()
    return poly


def main():
    ################################### BLOCK DETECTION ########################################################
    profile = camera_init()
    pipeline = rs.pipeline()

    if profile == 1:
        pipeline.start(rs.config())

    # define variables
    # calibrate = sys.argv[1]
    block_coords = process_image(pipeline, True)
    pipeline.stop()

    print('block_coords: ')
    print(block_coords)

    

if __name__=='__main__':
    main()