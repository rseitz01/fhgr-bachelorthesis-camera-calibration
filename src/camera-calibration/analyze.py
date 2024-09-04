import cv2
import numpy as np
import matplotlib.pyplot as plt

def load_images():
    images = []
    for i in range(0, 10+1):
        name = f"data_low_light/{i*100:04d}.png"
        image = cv2.imread(name)
        if image is None:
            print(name, 'not loaded')
        else:
            print(name, 'loaded')
            images.append(image)

    assert(len(images) == 11) # we expect 11 images...
    return images

def calculage_avgs(images, p1=(900,528), p2=(905,536), show_image=False):
    avgs = []

    for image in images:

        image_cut = image[p1[1]:p2[1]+1, p1[0]:p2[0]+1]
        avg = np.average(image_cut)
        avgs.append(avg)
        if show_image:
            print(avg)
            cv2.imshow("image", image_cut)
            cv2.waitKey(0)

    return avgs

if __name__ == '__main__':

    images = load_images();
    show_image = False
    x = (0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0)

    avgs = calculage_avgs(images, p1=(902,532), p2=(903,532), show_image=show_image)
    #p=(904,568)
    #avgs = calculage_avgs(images, p1=(p[0],p[1]), p2=(p[0]+1,p[1]), show_image=show_image)
    res = (avgs - min(avgs)) / (max(avgs) - min(avgs))
    plt.plot(x, res, label='1x2 Pixel')
    print('AVERAGE over 1x2 pixels')
    for i in range(10+1):
        print(f'{i*10:5}% : {res[i]*100:7.3f}%')

    avgs = calculage_avgs(images, show_image=show_image)
    #avgs = calculage_avgs(images, p1=(p[0]-5,p[1]-5), p2=(p[0]+5,p[1]+5), show_image=show_image)
    res = (avgs - min(avgs)) / (max(avgs) - min(avgs))
    plt.plot(x, res, label='5x8 Pixels')
    print('AVERAGE over 5x8 Pixels')
    for i in range(10+1):
        print(f'{i*10:5}% : {res[i]*100:7.3f}%')

    avgs = calculage_avgs(images, p1=(858,494), p2=(946,572), show_image=show_image)
    #avgs = calculage_avgs(images, p1=(p[0]-40,p[1]-40), p2=(p[0]+40,p[1]+40), show_image=show_image)
    res = (avgs - min(avgs)) / (max(avgs) - min(avgs))
    plt.plot(x, res, label='86x78 Pixels')
    print('AVERAGE over 86x78 Pixels')
    for i in range(10+1):
        print(f'{i*10:5}% : {res[i]*100:7.3f}%')

    plt.plot(x, x, label='Ideal', color='red')

    plt.grid()
    plt.legend(loc='best')
    plt.xlabel('eingestellt / soll')
    plt.ylabel('gemessen')
    plt.title('Duty Cycle Soll vs. Gemessen')
    plt.show()
    #print(avgs)


