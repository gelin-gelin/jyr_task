import cv2
import numpy as np

def red_identify(img):
    a,b,c=img.shape
    print(a,b,c)
    original_img = img.copy()
    img = cv2.GaussianBlur(img, (5, 5), 0,0)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)# 将图像转换为HSV色彩空间
    lower_apple = np.array([0, 0, 10])
    higher_apple = np.array([10, 255, 255])  
    mask = cv2.inRange(hsv, lower_apple, higher_apple) # 阈值化处理，得到二值化的掩膜
    cnts, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    max_cnt = max(cnts, key=cv2.contourArea)
    (x, y, w, h) = cv2.boundingRect(max_cnt)
    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 3)
    center_x = x + w // 2
    center_y = y + h // 2
    center_ = f"Center: ({center_x}, {center_y})"
    cv2.putText(img, center_, (x, y-20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    result = np.hstack((original_img, img))
    cv2.imshow("Red Identification", result)
    print(f"矩形框中心的坐标: ({center_x}, {center_y})")


if __name__ == "__main__":
    img = cv2.imread('/home/zy/after_springfestival/redball.jpg')
    red_identify(img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()