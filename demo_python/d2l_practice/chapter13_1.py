import torch
import torchvision
from torch import nn
from PIL import Image
import matplotlib.pyplot as plt
from chapter2 import set_figsize


set_figsize()
img = Image.open('./img/cat1.jpg')
plt.imshow(img)
plt.show()

