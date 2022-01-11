import imageio as iio
import matplotlib.pyplot as plt


img = iio.imread('image/JZ_192*192.png')
# print(img.shape)
plt.imshow(img)
plt.show()
# print(img[50][50][0])