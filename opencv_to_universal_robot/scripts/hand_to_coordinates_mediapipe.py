import pyrealsense2 as rs
import mediapipe as mp
import cv2
import numpy as np
import datetime as dt


class HandDetection:
    def __init__(self):

        #mediapipe hand recognition variables
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands()
        self.mpDraw = mp.solutions.drawing_utils


    def frame_processing(self, depth_frame, color_frame):
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_image_flipped = cv2.flip(depth_image, 1)
        color_image = np.asanyarray(color_frame.get_data())

        depth_image_3d = np.dstack(
            (depth_image, depth_image, depth_image))  # Depth image is 1 channel, while color image is 3
        background_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0),
                                      background_removed_color, color_image)

        processing_images = cv2.flip(background_removed, 1)
        color_image = cv2.flip(color_image, 1)
        color_images_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

        self.hand_processing(depth_image_flipped, processing_images, color_images_rgb)

    def hand_processing(self, hand_depth_image_flipped, hand_images, hand_color_images_rgb):
        results = self.hands.process(hand_color_images_rgb)
        if results.multi_hand_landmarks:
            number_of_hands = len(results.multi_hand_landmarks)
            i = 0
            for handLms in results.multi_hand_landmarks:
                self.mpDraw.draw_landmarks(hand_images, handLms, self.mpHands.HAND_CONNECTIONS)
                org2 = (20, org[1] + (20 * (i + 1)))
                hand_side_classification_list = results.multi_handedness[i]
                hand_side = hand_side_classification_list.classification[0].label
                middle_finger_knuckle = results.multi_hand_landmarks[i].landmark[9]
                x = int(middle_finger_knuckle.x * len(hand_depth_image_flipped[0]))
                y = int(middle_finger_knuckle.y * len(hand_depth_image_flipped))
                if x >= len(hand_depth_image_flipped[0]):
                    x = len(hand_depth_image_flipped[0]) - 1
                if y >= len(hand_depth_image_flipped):
                    y = len(hand_depth_image_flipped) - 1
                mfk_distance = hand_depth_image_flipped[y, x] * depth_scale  # meters
                hand_images = cv2.putText(hand_images,
                                     f"{hand_side} Hand Distance: {mfk_distance:0.3} meters away on position x:{x} and y:{y}",
                                     org2, font, fontScale, color, thickness, cv2.LINE_AA)

                i += 1
            hand_images = cv2.putText(hand_images, f"Hands: {number_of_hands}", org, font, fontScale, color, thickness,
                                 cv2.LINE_AA)
        else:
            hand_images = cv2.putText(hand_images, "No Hands", org, font, fontScale, color, thickness, cv2.LINE_AA)

        self.hand_images = hand_images


font = cv2.FONT_HERSHEY_SIMPLEX
org = (10, 10)
fontScale = .5
color = (0, 50, 255)
thickness = 1

# ====== Realsense ======
realsense_ctx = rs.context()
device = realsense_ctx.devices[0].get_info(rs.camera_info.serial_number)
pipeline = rs.pipeline()
config = rs.config()
background_removed_color = 153  # Grey


# ====== Enable Streams ======
config.enable_device(device)

# # For worse FPS, but better resolution:
# stream_res_x = 1280
# stream_res_y = 720
# # For better FPS. but worse resolution:
stream_res_x = 640
stream_res_y = 480

stream_fps = 30

config.enable_stream(rs.stream.depth, stream_res_x, stream_res_y, rs.format.z16, stream_fps)
config.enable_stream(rs.stream.color, stream_res_x, stream_res_y, rs.format.bgr8, stream_fps)
profile = pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)

# ====== Get depth Scale ======
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print(f"\tDepth Scale for Camera SN {device} is: {depth_scale}")

# ====== Set clipping distance ======
clipping_distance_in_meters = 1
clipping_distance = clipping_distance_in_meters / depth_scale
print(f"\tConfiguration Successful for SN {device}")

# ====== Get and process images ======
print(f"Starting to capture images on SN: {device}")

hand = HandDetection()

while True:
    start_time = dt.datetime.today().timestamp()

    # Get and align frames
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    input_color_image = aligned_frames.get_color_frame()

    if not aligned_depth_frame or not input_color_image:
        continue

    hand.frame_processing(aligned_depth_frame, input_color_image)
    images = hand.hand_images

    time_diff = dt.datetime.today().timestamp() - start_time
    fps = int(1 / time_diff)
    org3 = (20, org[1] + 60)
    images = cv2.putText(images, f"FPS: {fps}", org3, font, fontScale, color, thickness, cv2.LINE_AA)

    name_of_window = 'SN: ' + str(device)

    # Display images
    cv2.namedWindow(name_of_window, cv2.WINDOW_AUTOSIZE)
    cv2.rectangle(images, (20, 137), (600, 343), (0, 255, 0), 2)
    cv2.putText(images, f"65 cm from camera, highest point on the robot.", (20, 135), font, fontScale, color, thickness,
                cv2.LINE_AA)
    cv2.rectangle(images, (96, 175), (530, 305), (255, 0, 0), 2)
    cv2.imshow(name_of_window, images)
    key = cv2.waitKey(1)
    # Press esc or 'q' to close the image window
    if key & 0xFF == ord('q') or key == 27:
        print(f"User pressed break key for SN: {device}")
        break

print(f"Application Closing")
pipeline.stop()
print(f"Application Closed.")