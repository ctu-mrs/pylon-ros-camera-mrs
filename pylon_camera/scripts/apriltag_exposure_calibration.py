#! /usr/bin/python3

from threading import current_thread
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from apriltag_ros.msg import AprilTagCorners
from apriltag_ros.srv import SetId
from camera_control_msgs.srv import SetExposure
import time
import message_filters

# ------------ config ---------------
compressed_image_topic = "/uav1/tag_detections_image/compressed"
apriltag_corners_topic = "/uav1/tag_corners"
exposure_service = "/uav1/basler_left/set_exposure"
tag_id_service = "/uav1/set_tag_id_for_corner_publishing"

id_of_tag_to_calibrate = 2
lowest_camera_exposure = int(10.0)
highest_camera_exposure = int(200000.0)
testing = False
autoexpose_apriltag = True


# ------------ config ---------------

class auto_exposure:
    def __init__(self):
        self.cv_image = None
        self.bridge = CvBridge()
        self.compressed_image = None

        # Image characteristics
        self.image_std = 0.0
        self.image_brightness = 0.0
        self.apriltag_brightness = 0.0
        self.apriltag_std = 0.0

        # Delays and Timestamps
        self.measured_exposure_rise_delay = 0.0
        self.compressed_image_header = None

        # AprilTag variables
        self.apriltag_corner = AprilTagCorners()
        self.apriltag_corner.corner_0[0] = -1
        self.initial_benchmark = 0.0
        self.exposure_for_initial_benchmark = 0.0

        # ROS Services
        self.service_exposure_ = rospy.ServiceProxy(
            exposure_service, SetExposure)
        self.service_tag_id_ = rospy.ServiceProxy(tag_id_service, SetId)

    def callbackImageCompressed(self, image_msg):
        self.compressed_image_header = image_msg.header
        np_arr = np.fromstring(image_msg.data, np.uint8)
        self.compressed_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.image_std = self.compressed_image.std()
        self.image_brightness = self.compressed_image.mean()

    def callbackImage(self, image_msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(
            image_msg, desired_encoding='passthrough')
        print("Normal Image", self.cv_image.std())

        # TODO : Corner should be only set if the timestamp of the detections is within a certain window of the corner or we will have bad corners
    def callbackCorner(self, corner_msg):
        self.apriltag_corner = corner_msg
        if (self.compressed_image is None):
            print("No compressed_image received")
            return
        if (self.apriltag_corner.header.stamp.to_sec() != self.compressed_image_header.stamp.to_sec()):
            print("Stamps for corner and image did not match")
            return
        if (self.apriltag_corner.corner_0[0] != -1):
            mask = np.zeros(self.compressed_image.shape, dtype=np.uint8)
            mask_corners = np.array([[(self.apriltag_corner.corner_0, self.apriltag_corner.corner_1,
                                    self.apriltag_corner.corner_2, self.apriltag_corner.corner_3)]], dtype=np.int32)
            channel_count = self.compressed_image.shape[2]
            ignore_mask_color = (255,)*channel_count
            cv2.fillPoly(mask, mask_corners, ignore_mask_color)
            masked_image = cv2.bitwise_and(self.compressed_image, mask)
            only_tag_in_masked_image = masked_image[masked_image != 0]
            self.apriltag_brightness = only_tag_in_masked_image.mean()
            self.apriltag_std = only_tag_in_masked_image.std()
            return
        self.apriltag_std = 0.0001
        self.apriltag_brightness = 0.01
        rospy.loginfo_throttle(
            0.5, "AprilTag not found in the image, setting std and brightness to zero")

    def callbackImageAndCorners(self, image_msg, corner_msg):
        self.compressed_image_header = image_msg.header
        np_arr = np.fromstring(image_msg.data, np.uint8)
        self.compressed_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.image_std = self.compressed_image.std()
        self.image_brightness = self.compressed_image.mean()

        self.apriltag_corner = corner_msg
        if (self.compressed_image is None):
            print("No compressed_image received")
            return
        if (self.apriltag_corner.header.stamp.to_sec() != self.compressed_image_header.stamp.to_sec()):
            print("Stamps for corner and image did not match")
            return
        if (self.apriltag_corner.corner_0[0] != -1):
            mask = np.zeros(self.compressed_image.shape, dtype=np.uint8)
            mask_corners = np.array([[(self.apriltag_corner.corner_0, self.apriltag_corner.corner_1,
                                    self.apriltag_corner.corner_2, self.apriltag_corner.corner_3)]], dtype=np.int32)
            channel_count = self.compressed_image.shape[2]
            ignore_mask_color = (255,)*channel_count
            cv2.fillPoly(mask, mask_corners, ignore_mask_color)
            masked_image = cv2.bitwise_and(self.compressed_image, mask)
            only_tag_in_masked_image = masked_image[masked_image != 0]
            self.apriltag_brightness = only_tag_in_masked_image.mean()
            self.apriltag_std = only_tag_in_masked_image.std()
            return
        self.apriltag_std = 0.0001
        self.apriltag_brightness = 0.01
        # rospy.loginfo_throttle(
        #     0.5, "AprilTag not found in the image, setting std and brightness to zero")

    def node_work_ahead(self):
        rospy.init_node("auto_exposure_apriltag", anonymous=True)
        # rospy.Subscriber(compressed_image_topic,
        #                 CompressedImage, self.callbackImageCompressed, queue_size=1, buff_size=2000000)
        rospy.Subscriber("~uncompressed_image_input",
                         Image, self.callbackImage)
        # rospy.Subscriber(apriltag_corners_topic,
        #                 AprilTagCorners, self.callbackCorner, queue_size=1)
        compressed_image_sub = message_filters.Subscriber(
            compressed_image_topic, CompressedImage, queue_size=1, buff_size=2000000)
        corner_sub = message_filters.Subscriber(
            apriltag_corners_topic, AprilTagCorners, queue_size=1)

        ts = message_filters.TimeSynchronizer(
            [compressed_image_sub, corner_sub], 1)
        ts.registerCallback(self.callbackImageAndCorners)
        # TODO : Check that the set exposure is the same as reached exposure on the camera service return
        print(self.service_tag_id_(id_of_tag_to_calibrate))
        rate = rospy.Rate(10)
        if (not testing):
            self.findExposureDelay()
            good_exposure = self.binarySearchHighestContrastandBrightness(
                lowest_camera_exposure, highest_camera_exposure, self.benchmarkFunctionA)
            print("initial search results ", self.initial_benchmark,
                  self.exposure_for_initial_benchmark)
            if (autoexpose_apriltag):
                lowest_apriltag_exposure = self.binarySearchAprilTagExposureLowerLimit(
                    lowest_camera_exposure, self.exposure_for_initial_benchmark, self.benchmarkFunctionB)
                highest_apriltag_exposure = self.binarySearchAprilTagExposureHigherLimit(
                    self.exposure_for_initial_benchmark, highest_camera_exposure, self.benchmarkFunctionB)
                best_apriltag_exposure = self.binarySearchBestAprilTagContrast(
                    lowest_apriltag_exposure, highest_apriltag_exposure, self.benchmarkFunctionB)
                print("Best apriltag exposure ever ", best_apriltag_exposure)
                self.service_exposure_(best_apriltag_exposure)
        time.sleep(3)
        while not rospy.is_shutdown():
            print("AprilTag benchmark", self.benchmarkFunctionB(
                self.apriltag_brightness, self.apriltag_std))
            rate.sleep()

    def findExposureDelay(self):
        while (self.compressed_image is None and not rospy.is_shutdown()):
            print("Sleeping for a second while we wait for compressed image")
            time.sleep(1)
        self.service_exposure_(lowest_camera_exposure)
        print("Setting lowest_camera_exposure of ", lowest_camera_exposure)
        time.sleep(2)
        low_exposure_brightness = self.image_brightness
        print("recorded low exposure contrast is ", low_exposure_brightness)
        low_time = rospy.Time.now().to_sec()
        print("Setting highest_camera_exposure of ",
              highest_camera_exposure, "at time: ", rospy.Time.now().to_sec())
        self.service_exposure_(highest_camera_exposure)
        if (self.image_brightness <= 2*low_exposure_brightness):
            while (self.image_brightness <= 2*low_exposure_brightness and not rospy.is_shutdown()):
                rospy.loginfo_throttle(0.1, "Waiting for exposure to rise")
                time.sleep(0.01)
        high_time = rospy.Time.now().to_sec()
        self.measured_exposure_rise_delay = high_time - low_time
        print("Found that it takes ", self.measured_exposure_rise_delay,
              " seconds to change exposure")
        self.measured_exposure_rise_delay = 1.5*self.measured_exposure_rise_delay
        print("Choosing 50% larger value at ", self.measured_exposure_rise_delay,
              " seconds to change exposure")

    def binarySearchHighestContrastandBrightness(self, low_exposure, high_exposure, benchmarkFunc):
        self.service_exposure_(low_exposure)
        time.sleep(self.measured_exposure_rise_delay)
        low_exposure_benchmark = benchmarkFunc(
            self.image_brightness, self.image_std)
        self.storeAndCompareAprilTag(low_exposure)

        self.service_exposure_(high_exposure)
        time.sleep(self.measured_exposure_rise_delay)
        high_exposure_benchmark = benchmarkFunc(
            self.image_brightness, self.image_std)
        self.storeAndCompareAprilTag(high_exposure)

        if (high_exposure-low_exposure > (0.1*high_exposure)):
            mid_exposure = int((low_exposure+high_exposure)/2)
        else:
            mid_exposure = int((low_exposure+high_exposure)/2)
            print("Interval too short, found best value at ", mid_exposure)
            return mid_exposure
        self.service_exposure_(mid_exposure)
        time.sleep(self.measured_exposure_rise_delay)
        new_benchmark = benchmarkFunc(
            self.image_brightness, self.image_std)
        self.storeAndCompareAprilTag(mid_exposure)
        if ((new_benchmark > low_exposure_benchmark) and (high_exposure_benchmark > low_exposure_benchmark)):
            print("Found better value at ", mid_exposure, " between ",
                  low_exposure, " and ", high_exposure)
            return self.binarySearchHighestContrastandBrightness(
                mid_exposure, high_exposure, benchmarkFunc)
        elif ((new_benchmark > high_exposure_benchmark) and (low_exposure_benchmark > high_exposure_benchmark)):
            print("Found better value at ", mid_exposure, " between ",
                  low_exposure, " and ", high_exposure)
            return self.binarySearchHighestContrastandBrightness(
                low_exposure, mid_exposure, benchmarkFunc)
        else:
            if (low_exposure_benchmark > high_exposure_benchmark):
                return low_exposure
            else:
                return high_exposure

    def binarySearchAprilTagExposureLowerLimit(self, low_exposure, high_exposure, benchmarkFunc):
        self.service_exposure_(low_exposure)
        time.sleep(self.measured_exposure_rise_delay)
        low_exposure_benchmark = benchmarkFunc(
            self.apriltag_brightness, self.apriltag_std)

        self.service_exposure_(high_exposure)
        time.sleep(self.measured_exposure_rise_delay)
        high_exposure_benchmark = benchmarkFunc(
            self.apriltag_brightness, self.apriltag_std)

        if (high_exposure-low_exposure > 400):
            mid_exposure = int((low_exposure+high_exposure)/2)
        else:
            print("[LOW_HUNT]:Found lowest value at ", high_exposure)
            return high_exposure
        self.service_exposure_(mid_exposure)
        time.sleep(self.measured_exposure_rise_delay)
        new_benchmark = benchmarkFunc(
            self.apriltag_brightness, self.apriltag_std)
        print("Benchmarks: ", low_exposure_benchmark,
              new_benchmark, high_exposure_benchmark)
        print("Exposures: ", low_exposure, mid_exposure, high_exposure)

        if (new_benchmark > low_exposure_benchmark):
            print("[LOW_HUNT]:Found lower value")
            return self.binarySearchAprilTagExposureLowerLimit(
                low_exposure, mid_exposure, benchmarkFunc)
        elif (new_benchmark < high_exposure_benchmark):
            print("[LOW_HUNT]:Found higher value")
            return self.binarySearchAprilTagExposureLowerLimit(
                mid_exposure, high_exposure, benchmarkFunc)
        else:
            print("[LOW_HUNT]:AprilTag not found anywhere")
            return lowest_camera_exposure

    def binarySearchAprilTagExposureHigherLimit(self, low_exposure, high_exposure, benchmarkFunc):
        self.service_exposure_(low_exposure)
        time.sleep(self.measured_exposure_rise_delay)
        low_exposure_benchmark = benchmarkFunc(
            self.apriltag_brightness, self.apriltag_std)

        self.service_exposure_(high_exposure)
        time.sleep(self.measured_exposure_rise_delay)
        high_exposure_benchmark = benchmarkFunc(
            self.apriltag_brightness, self.apriltag_std)

        if ((high_exposure-low_exposure) > 400):
            mid_exposure = int((low_exposure+high_exposure)/2)
        else:
            print(high_exposure-low_exposure)
            print("[HIGH_HUNT]:Found highest value at ", low_exposure)
            return low_exposure
        self.service_exposure_(mid_exposure)
        time.sleep(self.measured_exposure_rise_delay)
        new_benchmark = benchmarkFunc(
            self.apriltag_brightness, self.apriltag_std)
        print("Benchmarks: ", low_exposure_benchmark,
              new_benchmark, high_exposure_benchmark)
        print("Exposures: ", low_exposure, mid_exposure, high_exposure)

        if (new_benchmark > high_exposure_benchmark):
            print("[HIGH_HUNT]:Found lower value")
            return self.binarySearchAprilTagExposureHigherLimit(
                mid_exposure, high_exposure, benchmarkFunc)
        elif (new_benchmark < low_exposure_benchmark):
            print("[HIGH_HUNT]:Found higher value")
            return self.binarySearchAprilTagExposureHigherLimit(
                low_exposure, mid_exposure, benchmarkFunc)
        else:
            print("[HIGH_HUNT]:AprilTag not found anywhere")
            return highest_camera_exposure

    def binarySearchBestAprilTagContrast(self, low_exposure, high_exposure, benchmarkFunc):
        self.service_exposure_(low_exposure)
        time.sleep(self.measured_exposure_rise_delay)
        low_exposure_benchmark = benchmarkFunc(
            self.apriltag_brightness, self.apriltag_std)

        self.service_exposure_(high_exposure)
        time.sleep(self.measured_exposure_rise_delay)
        high_exposure_benchmark = benchmarkFunc(
            self.apriltag_brightness, self.apriltag_std)

        if (high_exposure-low_exposure > (0.1*high_exposure)):
            mid_exposure = int((low_exposure+high_exposure)/2)
        else:
            mid_exposure = int((low_exposure+high_exposure)/2)
            print("Found optimum value at ", mid_exposure)
            return mid_exposure
        self.service_exposure_(mid_exposure)
        time.sleep(self.measured_exposure_rise_delay)
        new_benchmark = benchmarkFunc(
            self.apriltag_brightness, self.apriltag_std)

        print(low_exposure_benchmark, "(", low_exposure, ") ", new_benchmark,
              "(", mid_exposure, ") ", high_exposure_benchmark, "(", high_exposure, ") ")
        if ((low_exposure_benchmark == new_benchmark) and (new_benchmark == high_exposure_benchmark)):
            print("AprilTag wasn't found")
            return
        if ((new_benchmark > high_exposure_benchmark) and (high_exposure_benchmark > low_exposure_benchmark)):
            print("Found better optimum at ", mid_exposure, " between ",
                  low_exposure, " and ", high_exposure)
            return self.binarySearchBestAprilTagContrast(
                mid_exposure, high_exposure, benchmarkFunc)
        elif ((new_benchmark > low_exposure_benchmark) and (low_exposure_benchmark > high_exposure_benchmark)):
            print("Found better optimum at ", mid_exposure, " between ",
                  low_exposure, " and ", high_exposure)
            return self.binarySearchBestAprilTagContrast(
                low_exposure, mid_exposure, benchmarkFunc)
        elif ((new_benchmark > low_exposure_benchmark) and (new_benchmark > high_exposure_benchmark)):
            print("Found better optimum at ", mid_exposure, " between ",
                  low_exposure, " and ", high_exposure)
            return mid_exposure
        else:
            if (low_exposure_benchmark > high_exposure_benchmark):
                return low_exposure
            else:
                return high_exposure

    def benchmarkFunctionA(self, brightness, contrast):
        return (contrast**1.0)/(brightness**1.0)

    def benchmarkFunctionB(self, brightness, contrast):
        return contrast

    def storeAndCompareAprilTag(self, exposure):
        current_benchmark = self.benchmarkFunctionB(
            self.apriltag_brightness, self.apriltag_std)
        if (current_benchmark > self.initial_benchmark):
            print("Apriltag benchmark improved ", current_benchmark)
            self.initial_benchmark = current_benchmark
            self.exposure_for_initial_benchmark = exposure


if __name__ == '__main__':
    autoexposure = auto_exposure()
    try:
        autoexposure.node_work_ahead()
    except rospy.ROSInterruptException:
        pass
