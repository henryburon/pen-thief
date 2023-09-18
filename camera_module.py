import pyrealsense2 as rs
import numpy as np
import cv2
import math

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream different resolutions of color and depth streams
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
    print("This process requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# Remove the background of objects more than clipping_distance_in_meters meters away
clipping_distance_in_meters = 1 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

# Clear pipeline files
with open('x_robot.txt', 'w') as file:
    pass
            
with open('theta_robot.txt', 'w') as file:
    pass

with open('z_robot.txt', 'w') as file:
    pass

# Streaming loop
try:
 
    centroid_positions = []

    while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()

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

        # Convert BGR image to HSV for easier color isolation
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # Set upper and lower purple threshold values
        lower_purple = np.array([115, 50, 50])
        upper_purple = np.array([170, 255, 255])

        mask = cv2.inRange(hsv, lower_purple, upper_purple)
        result = cv2.bitwise_and(hsv, hsv, mask=mask)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Initialize variables to keep track of the largest contour
        largest_contour = None
        largest_area = 0

        # Find the largest contour
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > largest_area:
                largest_area = area
                largest_contour = contour
        
        # Create a copy of the original color image
        image_with_box = color_image.copy()

        # Initialize variables to store the centroid coordinates
        centroid_x = -1
        centroid_y = -1

        # Draw a green rectangle around the largest contour (i.e. the purple pen)
        if largest_contour is not None:
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(image_with_box, (x,y), (x+w, y+h), (0,255,0), 2) # Green rectangle

            # Calculate the centroid of the largest contour
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                centroid_x = int(M["m10"] / M["m00"])
                centroid_y = int(M["m01"] / M["m00"])

                centroid_positions.append((centroid_x, centroid_y))

                # Average past several calculated centroids to smooth result
                if len(centroid_positions) > 20:
                    centroid_positions.pop(0)

                if centroid_positions:
                    avg_x = sum([pos[0] for pos in centroid_positions]) / len(centroid_positions)
                    avg_y = sum([pos[1] for pos in centroid_positions]) / len(centroid_positions)

                    centroid_x = int(avg_x)
                    centroid_y = int(avg_y)

            # Draw a red circle at the centroid position
            if centroid_x != -1 and centroid_y != -1:
                cv2.circle(image_with_box, (centroid_x, centroid_y), 5, (0, 0, 255), -1)  # Red circle

                # Get the depth value at the centroid coordinates
                depth_value = aligned_depth_frame.get_distance(centroid_x, centroid_y)

                # Convert depth value to meters
                depth_meters = (depth_value / depth_scale)/1000

                # Display the depth value as text at the bottom of the image
                depth_text = f"Depth: {depth_meters:.2f} meters"
                cv2.putText(image_with_box, depth_text, (10, color_image.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                # Display the coordinate value as text at the bottom of the image
                coordinate_text = f"Pixel coordinate: ({centroid_x}, {centroid_y})"
                cv2.putText(image_with_box, coordinate_text, (10, color_image.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # Convert the depth map and pixel coordinates into real (xyz) coordinates
            profile2 = profile.get_stream(rs.stream.color)
            intrinsics = profile2.as_video_stream_profile().get_intrinsics()
            xyz = rs.rs2_deproject_pixel_to_point(intrinsics, [centroid_x, centroid_y], depth_meters)

            # Initialize the (0,0,0) position as the end effector's 'home' position
            x = round((xyz[0] * 10) - .165, 3)
            y = round((xyz[1] * 10) - .19, 3)
            z = round((xyz[2] * 10) - 2.74, 3)

            # Convert cartesian to cylindrical
            r = round(math.sqrt(x**2 + y**2), 3)
            theta = abs(math.atan2(y, x))
            theta_robot = round(theta * .5, 3)

            # Convert the y-value to a z-value in robot space
            if y >= -0.54 and y <= 0.60:
                if y >= -0.54 and y <= 0.0:
                    percent_y1 = y / -0.54
                    z_robot = percent_y1 * 0.05
                elif y <= 0.60 and y > 0.0:
                    percent_y2 = y / 0.60
                    z_robot = percent_y2 * -0.09
            else:
                z_robot = None

            # Convert the x-value to an x-value in robot space
            if x >= -1.28 and x <= 0.35:
                if x >= -1.28 and x <= 0.0:
                    percent_x1 = x / -1.28
                    x_robot = percent_x1 * -0.14
                elif x <= 0.35 and x > 0.0:
                    percent_x2 = x / 0.35
                    x_robot = percent_x2 * 0.04
            else:
                x_robot = None
        
            # Append the x (r), theta, and z values into the pipeline files
            with open('/home/henry/Desktop/two_at_once/x_robot.txt', 'a') as file:
                file.write(str(x_robot) + "\n")
            
            with open('/home/henry/Desktop/two_at_once/theta_robot.txt', 'a') as file:
                file.write(str(theta_robot) + "\n")

            with open('/home/henry/Desktop/two_at_once/z_robot.txt', 'a') as file:
                file.write(str(z_robot) + "\n")

            # Display the cartesian and cylindrical coordinates on the output window
            cartesian_coordinates = f"X/Y/Z: ({x}, {y}, {z}"
            cv2.putText(image_with_box, cartesian_coordinates, (10, color_image.shape[0] - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            cylindrical_coordinates = f"r/theta/z: ({r}, {theta_robot}, {z})"
            cv2.putText(image_with_box, cylindrical_coordinates, (10, color_image.shape[0] - 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 153
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

        # Render images: depth align to color on left, depth on right
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((bg_removed, depth_colormap))

        #cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
        #cv2.imshow('Align Example', images)
        #cv2.imshow('HSV Purple Image', result) # Shows the HSV image that masks everything except purple
        cv2.imshow('Image with Contour, Centroid, and Depth', image_with_box)

        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
    pipeline.stop()