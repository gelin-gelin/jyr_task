#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def process_image(msg):
    bridge = CvBridge()
    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr("图像转换失败：%s", str(e))
        return
    median_frame = cv2.medianBlur(frame, 7)
    hsv = cv2.cvtColor(median_frame, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    # 使用形态学操作来去除噪声
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)  
    mask = cv2.dilate(mask, kernel, iterations=2) 

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for contour in contours:
        if cv2.contourArea(contour) > 5000:  # 增加面积过滤，过滤掉小轮廓
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(median_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            center_x = x + w // 2
            center_y = y + h // 2
            center_coord = f"Center: ({center_x}, {center_y})"
            cv2.putText(median_frame, center_coord, (x, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            rospy.loginfo(f"矩形框中心坐标: ({center_x}, {center_y})")
    cv2.imshow("Processed Image", median_frame)
    cv2.waitKey(1)

    # 发布处理后的图像
    pub = rospy.Publisher('/camera/processed_image', Image, queue_size=10)
    try:
        processed_image = bridge.cv2_to_imgmsg(median_frame, "bgr8")
        pub.publish(processed_image)
    except Exception as e:
        rospy.logerr("图像转换失败：%s", str(e))

def main():
    rospy.init_node('img_sub', anonymous=True)
    
    # 创建cv_bridge对象
    bridge = CvBridge()

    # 创建订阅者
    rospy.Subscriber('/camera/image_raw', Image, process_image)

    # 设置发布者
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
