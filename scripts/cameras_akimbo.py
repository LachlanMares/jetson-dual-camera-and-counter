#!/usr/bin/env python3

import rospy
import warnings

from Ppetual.msg import ImagesAndCount
from .pulse_counter import PulseCounter
from .csi_camera import CsiCamera

warnings.filterwarnings("ignore")


if __name__ == "__main__":
    desc = """Dual wield Raspberry Pi cameras with a Jetson Nano"""

    # Create a ROS node
    rospy.init_node("cameras_akimbo")
    rospy.loginfo("*** Cameras Akimbo Node Starting ***")

    c_mode = rospy.get_param(param_name="~camera_mode", default=3)
    c_width = rospy.get_param(param_name="~image_width", default=1280)
    c_height = rospy.get_param(param_name="~image_height", default=720)
    c_fr = rospy.get_param(param_name="~camera_frame_rate", default=30)
    c_fl = rospy.get_param(param_name="~image_flip", default=0)

    # Create instances of CsiCamera
    camera_0 = CsiCamera(cam_id=0, camera_mode=c_mode, display_width=c_width, display_height=c_height, frame_rate=c_fr, flip_method=c_fl)
    camera_1 = CsiCamera(cam_id=1, camera_mode=c_mode, display_width=c_width, display_height=c_height, frame_rate=c_fr, flip_method=c_fl)

    # Create instance of PulseCounter
    pulsecounter = PulseCounter()

    camera_0.start_thread()
    camera_1.start_thread()

    if camera_0.is_open() and camera_1.is_open():

        # Create an Image publisher
        image_publisher = rospy.Publisher('camera_image', ImagesAndCount, queue_size=1)

        # Create ROS Message
        image_and_count_msg = ImagesAndCount()

        # Set the publishing rate
        rate = rospy.Rate(c_fr)

        # Clear the pulse counter
        pulsecounter.reset_count()

        while not rospy.is_shutdown():
            # Check to see if there is a frame ready to be grabbed
            if camera_0.frame_exists() and camera_1.frame_exists():

                # Stuff the image with treats
                image_and_count_msg.header.stamp = rospy.Time.now()
                image_and_count_msg.camera_0_image = camera_0.get_frame()
                image_and_count_msg.camera_1_image = camera_1.get_frame()
                image_and_count_msg.count = pulsecounter.read_32bit_counter(reset_after_read=True)

                # Publish message
                image_publisher.publish(image_and_count_msg)

                # Sleep to maintain right frame rate
                rate.sleep()

        # Join/stop the camera threads
        camera_0.stop()
        camera_0.release()
        camera_1.stop()
        camera_1.release()

        rospy.loginfo("*** Cameras Akimbo Node Shutdown ***")
        SystemExit(0)

    else:
        rospy.loginfo("*** Cameras Akimbo Node Unable To Open Cameras ***")
        SystemExit(0)
