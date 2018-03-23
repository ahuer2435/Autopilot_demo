import message_filters
from sensor_msgs.msg import Image, CameraInfo

def callback(image, camera_info):
  # Solve all of perception here...

image_sub = message_filters.Subscriber('image', Image)
info_sub = message_filters.Subscriber('camera_info', CameraInfo)

ts = message_filters.TimeSynchronizer([image_sub, info_sub], 10)
ts.registerCallback(callback)
rospy.spin()
