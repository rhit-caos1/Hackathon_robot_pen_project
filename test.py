from __future__ import print_function
from __future__ import division
# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2


import argparse

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

#setupslidebar
minH=110
maxH=150
minS=48
maxS=250
minV=30
maxV=250
alpha_slider_max = 359
def nothing(x):
    pass
cv2.namedWindow('controls')
cv2.createTrackbar('max','controls',150,alpha_slider_max,nothing)
cv2.createTrackbar('min','controls',110,alpha_slider_max,nothing)
cv2.createTrackbar('maxs','controls',255,alpha_slider_max,nothing)
cv2.createTrackbar('mins','controls',94,alpha_slider_max,nothing)
cv2.createTrackbar('maxv','controls',255,alpha_slider_max,nothing)
cv2.createTrackbar('minv','controls',67,alpha_slider_max,nothing)

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 1 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

align_to = rs.stream.color
align = rs.align(align_to)
i=0
# Streaming loop

try:
    while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 153
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)
        #print(f"depthimage{depth_image}",f"colorimage{color_image}")



        # Render images:
        #   depth align to color on left
        #   depth on right
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((bg_removed, depth_colormap))

        color_image_hsv = cv2.cvtColor(color_image,cv2.COLOR_BGR2HSV)
        lower_purple = np.array([minH,minS,minV])
        upper_purple = np.array([maxH,maxS,maxV])
        mask = cv2.inRange(color_image_hsv,lower_purple,upper_purple)
        burmask = cv2.blur(mask,(40,40))
        res = cv2.bitwise_and(color_image,color_image,mask = burmask)
        ret,thresh = cv2.threshold(burmask,1,255,cv2.THRESH_OTSU)
        contours,hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(color_image,contours,-1,(0,250,0),3)
        
        contoursArea = {}
        for i in contours:
            area = cv2.contourArea(i)
            contoursArea = {area:i}

        maxarea = sorted(contoursArea.keys())
        

        if len(maxarea)>1:
            contours = contoursArea[maxarea[-1]]
        elif len(maxarea)==1:
            contours = contoursArea[maxarea[0]]
        
        
        cnt = contours#[0]
        M = cv2.moments(cnt)
        if M['m00']!=0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(color_image,[cx,cy],10,(255,0,255),5)

        cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
        cv2.imshow('Align Example', color_image)
        cv2.imshow('mask',burmask)
        minH = int(cv2.getTrackbarPos('min','controls'))
        maxH = int(cv2.getTrackbarPos('max','controls'))
        minS = int(cv2.getTrackbarPos('mins','controls'))
        maxS = int(cv2.getTrackbarPos('maxs','controls'))
        minV = int(cv2.getTrackbarPos('minv','controls'))
        maxV = int(cv2.getTrackbarPos('maxv','controls'))
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
        i=i+1
finally:
    pipeline.stop()