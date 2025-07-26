#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes

class BoxDrawerNode:
    def __init__(self):
        rospy.init_node('box_drawer', anonymous=True)

        # Topics (adjust if you renamed in your launch)
        image_topic = rospy.get_param('~image_topic', '/camera/rgb/image_raw')
        boxes_topic = rospy.get_param('~boxes_topic', '/darknet_ros/bounding_boxes')

        self.bridge = CvBridge()
        self.latest_boxes = []

        # Subscribers
        rospy.Subscriber(image_topic, Image, self.image_cb, queue_size=1)
        rospy.Subscriber(boxes_topic, BoundingBoxes, self.boxes_cb, queue_size=1)

        rospy.loginfo("Box drawer node started. Listening to:\n • %s\n • %s", image_topic, boxes_topic)
        rospy.spin()

    def boxes_cb(self, msg):
        # store latest boxes for next image frame
        self.latest_boxes = msg.bounding_boxes

    def image_cb(self, img_msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
            return

        # draw each bounding box
        for bb in self.latest_boxes:
            # bb.xmin, ymin, xmax, ymax are ints
            p1 = (bb.xmin, bb.ymin)
            p2 = (bb.xmax, bb.ymax)
            cv2.rectangle(cv_img, p1, p2, (0, 255, 0), 2)
            label = "{}:{:.2f}".format(bb.Class, bb.probability)
            cv2.putText(cv_img, label, (bb.xmin, bb.ymin - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

        # show image
        cv2.imshow("YOLO Detections", cv_img)
        cv2.waitKey(1)


if __name__ == '__main__':
    try:
        BoxDrawerNode()
    except rospy.ROSInterruptException:
        pass
