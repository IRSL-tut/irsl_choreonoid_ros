import rospy
from sensor_msgs.msg import CompressedImage
import cv2

# %%
class compressedImagePublisher(object):
    def __init__(self, topic, queue_size=1, img_height=None):
        self.pub = rospy.Publisher(topic, CompressedImage, queue_size=1)
        self.height = img_height
    def publish(self, cv_img):
        if self.height is not None:
            sh = cv_img.shape
            rs_img = cv2.resize(img, (int(self.height*sh[1]/sh[0]), self.height))
        else:
            rs_img = cv_img
        ret, buf = cv2.imencode('.jpeg', rs_img)
        if ret:
            msg = CompressedImage()
            msg.format = 'jpeg'
            msg.data = buf.tobytes()
            self.pub.publish(msg)
