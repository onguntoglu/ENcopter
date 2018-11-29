import cv2                                # state of the art computer vision algorithms library
import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API
import imutils
from imutils.video import VideoStream
print("Environment Ready")

# Setup:
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)


# Skip 5 first frames to give the Auto-Exposure time to adjust
for x in range(50):
  pipeline.wait_for_frames()



try:
    while True:
        # Store next frameset for later processing:
        frameset = pipeline.wait_for_frames()
        color_frame = frameset.get_color_frame()
        depth_frame = frameset.get_depth_frame()

        # Get color and depth frames
        color = np.asanyarray(color_frame.get_data())
        colorizer = rs.colorizer()
        colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())

        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        rgb = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
          
        lower_white = np.array([0, 0, 0], dtype=np.uint8)
        upper_white = np.array([255, 0, 0], dtype=np.uint8)

        lower_white_rgb = (180, 180, 170)
        upper_white_rgb = (255, 255, 255)

        greenLower = (29, 86, 6)
        greenUpper = (64, 255, 255)

        mask = cv2.inRange(hsv, greenLower, greenUpper)

        # Create alignment primitive with color as its target stream:
        align = rs.align(rs.stream.color)
        frameset = align.process(frameset)

        # Update color and depth frames:
        aligned_depth_frame = frameset.get_depth_frame() # use this variable for filtering

        hole_filling = rs.hole_filling_filter()
        filled_depth = hole_filling.process(aligned_depth_frame)

        colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())
        colorized_depth_filled = np.asanyarray(colorizer.colorize(filled_depth).get_data())       

        res = cv2.bitwise_and(color,color, mask= mask)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]
        center = None

        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                #cv2.circle(res, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                #cv2.circle(res, center, 5, (0, 0, 255), -1)

                #cv2.circle(colorized_depth_filled, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                #cv2.circle(colorized_depth_filled, center, 5, (0, 0, 255), -1)

                rectX = int(x) - int(radius)
                rectY = int(y) - int(radius)

                rectX_max = rectX + 2*int(radius)
                rectY_max = rectY + 2*int(radius)

                cv2.rectangle(colorized_depth_filled,(int(rectX), int(rectY)),(int(rectX_max), int(rectY_max)),(0, 255, 255), 2)

        images = np.hstack((color,colorized_depth_filled))

        calculateDepth = np.asanyarray(aligned_depth_frame.get_data())
        calculateDepth2 = calculateDepth[int(rectX):int(rectX_max), int(rectY):int(rectY_max)].astype(float)

        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        calculateDepth2 = calculateDepth2 * depth_scale
        dist,_,_,_ = cv2.mean(calculateDepth2)

        print("Detected top at {0:.3} meters away.".format(dist))

        cv2.namedWindow('RealSense_stacked', cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow('circle_track', cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow('rect_track', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense_stacked', images)
        cv2.imshow('circle_track',res)
        cv2.imshow('rect_track',res)
        cv2.waitKey(1)

finally:
    # Stop streaming
    pipeline.stop()


