import cv2

if __name__ == '__main__':

    name = f"data_low_light/1000.png"
    img = cv2.imread(name)

    p1=(900,528)
    p2=(905,536)
    cv2.rectangle(img, p1, p2, (255, 0, 0), 1)

    p1=(858,494)
    p2=(946,572)
    cv2.rectangle(img, p1, p2, (0, 255, 0), 1)

    p1=(902,532)
    p2=(903,532)
    cv2.rectangle(img, p1, p2, (0, 0, 255), 1)

    cv2.imshow("the image", img)
    cv2.waitKey(0)

