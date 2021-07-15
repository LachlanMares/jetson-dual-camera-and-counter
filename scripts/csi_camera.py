#!/usr/bin/env python3

import cv2
import threading
import numpy as np
import rospy
import warnings

warnings.filterwarnings("ignore")


class CsiCamera:
    def __init__(self, cam_id=0, camera_mode=3, display_width=1280, display_height=720, frame_rate=30, flip_method=0):
        # Class for CSI-2 Interface Cameras, i.e. Raspberry Pi v2

        # Create video capture variable
        self.cv2_video_capture_obj = None

        # Gather some info about the camera
        self.cam_id = cam_id
        self.camera_mode = camera_mode
        self.camera_mode_resolution = [[3280, 2464], [3280, 1848], [1920, 1080], [1280, 720], [1280, 720]]

        # Output image dimensions
        self.display_width = display_width
        self.display_height = display_height

        # The last captured image from the camera
        self.frame = None
        self.ret = False

        # The threads where the video capture runs
        self.read_thread = None
        self.read_lock = threading.Lock()
        self.running = False

        capture_width, capture_height = self.camera_settings(self.camera_mode)

        # Populate gstreamer string
        self.g_string = ("nvarguscamerasrc sensor-id=%d sensor-mode=%d ! "
                         "video/x-raw(memory:NVMM), "
                         "width=(int)%d, height=(int)%d, "
                         "format=(string)NV12, framerate=(fraction)%d/1 ! "
                         "nvvidconv flip-method=%d ! "
                         "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
                         "videoconvert ! "
                         "video/x-raw, format=(string)BGR ! appsink"
                         % (self.cam_id, self.camera_mode, capture_width, capture_height, frame_rate, flip_method,
                            self.display_width, self.display_height)
                         )
        try:
            # Try to start opencv video capture object
            self.cv2_video_capture_obj = cv2.VideoCapture(self.g_string, cv2.CAP_GSTREAMER)

        except RuntimeError:
            self.cv2_video_capture_obj = None
            self.running = False
            rospy.logerr(f"Unable to open using this gstreamer pipeline: {self.g_string}")
            return

        self.ret, self.frame = self.cv2_video_capture_obj.read()

    def camera_settings(self, camera_mode):
        # return the image height and width based on camera mode
        assert 0 <= camera_mode <= 4
        return self.camera_mode_resolution[camera_mode]

    def start_thread(self):
        # Start up a separate thread to capture images
        if self.cv2_video_capture_obj is not None:
            self.running = True
            self.ret, self.frame = self.cv2_video_capture_obj.read()

            # Set thread to run update_camera function continually
            self.read_thread = threading.Thread(target=self.update_camera)
            self.read_thread.start()

        return self

    def update_camera(self):
        # This is the thread to read images from the camera
        while self.running:
            try:
                ret, frame = self.cv2_video_capture_obj.read()

                # Secure the variable before trying to write
                with self.read_lock:
                    self.ret, self.frame = ret, np.array(frame)

            except RuntimeError:
                rospy.logerr(f"Could not read camera {self.cam_id}")

    def frame_exists(self):
        with self.read_lock:
            ret = self.ret
        return ret

    def get_frame(self):
        with self.read_lock:
            frame = self.frame.copy()
        return frame

    def is_open(self):
        return self.cv2_video_capture_obj.isOpened()

    def stop(self):
        self.running = False
        self.read_thread.join()

    def release(self):
        # Release cv2 object
        if self.cv2_video_capture_obj is not None:
            self.cv2_video_capture_obj.release()
            self.cv2_video_capture_obj = None

        # Kill thread
        if self.read_thread is not None:
            self.read_thread.join()
