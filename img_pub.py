#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def publish_image():
    # 初始化ROS节点
    rospy.init_node('img_pub', anonymous=True)
    rospy.loginfo("Node initialized")

    # 创建图像发布器
    pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    rospy.loginfo("Publisher created")

    # 创建cv_bridge对象
    bridge = CvBridge()

    # 打开摄像头
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not cap.isOpened():
        rospy.logerr("摄像头无法打开！")
        return
    rospy.loginfo("摄像头已打开")

    rate = rospy.Rate(10)

    try:
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                rospy.logwarn("无法读取摄像头帧！")
                continue

            rospy.loginfo("读取到一帧图像")

            # 转换BGR图像到ROS Image消息
            try:
                ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
                pub.publish(ros_image)
                rospy.loginfo("图像发布成功")
            except Exception as e:
                rospy.logerr("图像转换失败：%s", str(e))

            rate.sleep()
    finally:
        cap.release()
        cv2.destroyAllWindows()
        rospy.loginfo("摄像头资源已释放")

if __name__ == '__main__':
    try:
        publish_image()
    except rospy.ROSInterruptException:
        pass

