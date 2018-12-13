import cv2                                # state of the art computer vision algorithms library
import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API
import imutils
import math
from imutils.video import VideoStream
import zmq
print("Environment Ready")

# Setup:>
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)

# Create networking with PUB/SUB
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://127.0.0.1:5556")
pub_topic = "coords"

# Skip 5 first frames to give the Auto-Exposure time to adjust
for x in range(50):
    pipeline.wait_for_frames()

try:
    while True:
        # Store next frameset for later processing:
        frameset = pipeline.wait_for_frames()

        # align frameset to color stream
        aligned_frames = align.process(frameset)
        
        aligned_color_frame = aligned_frames.get_color_frame()
        aligned_depth_frame = aligned_frames.get_depth_frame()

        color = np.asanyarray(aligned_color_frame.get_data())

        hole_filling = rs.hole_filling_filter()  # hole filling filter
        filled_depth = hole_filling.process(aligned_depth_frame)  # filled depth data

        colorized_depth = np.asanyarray(aligned_depth_frame.get_data())
        colorized_depth_filled = np.asanyarray(filled_depth.get_data())

        colorizer = rs.colorizer()
        disp = np.asanyarray(colorizer.colorize(filled_depth).get_data())
        disp2 = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())

        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        # rgb = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)

        orangeLower = np.array([5, 130, 79], dtype=np.uint8)
        orangeUpper = np.array([21, 248, 255], dtype=np.uint8)

        lower_white_rgb = (180, 180, 170)
        upper_white_rgb = (255, 255, 255)

        greenLower = np.array([29, 86, 6], dtype=np.uint8)
        greenUpper = np.array([64, 255, 255], dtype=np.uint8)

        orangeLower2 = (5, 130, 110)
        orangeUpper2 = (8, 254, 249)

        mask = cv2.inRange(hsv, orangeLower, orangeUpper)

        res = cv2.bitwise_and(color, color, mask=mask)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]
        center = None

        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                # cv2.circle(res, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                # cv2.circle(res, center, 5, (0, 0, 255), -1)

                # cv2.circle(colorized_depth_filled, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                # cv2.circle(colorized_depth_filled, center, 5, (0, 0, 255), -1)

                rectX = abs(int(x) - int(radius/2))
                rectY = abs(int(y) - int(radius/2))

                rectX_max = rectX + int(radius)
                rectY_max = rectY + int(radius)

                cv2.rectangle(disp2, (int(rectX), int(rectY)), (int(rectX_max), int(rectY_max)), (255, 255, 255), 2)

        calculateDepth = colorized_depth
        calculateDepth2 = calculateDepth[int(rectY):int(rectY_max), int(rectX):int(rectX_max)].astype(float)

        cropped = disp2[int(rectY):int(rectY_max), int(rectX):int(rectX_max)]

        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        calculateDepth3 = calculateDepth2 * depth_scale
        dist, _, _, _ = cv2.mean(calculateDepth3)

        intrin = aligned_depth_frame.profile.as_video_stream_profile().get_intrinsics()

        world_coords = rs.rs2_deproject_pixel_to_point(intrin, [x, y], dist)

        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow('circle_track', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('circle_track', res)
        cv2.imshow('RealSense', disp2)

        cv2.waitKey(1)

        socket.send_pyobj([pub_topic, world_coords[0], world_coords[1], world_coords[2]])

finally:
    # Stop streaming
    pipeline.stop()
