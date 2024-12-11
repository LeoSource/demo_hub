import torch
import torchvision
from torch import nn
from torch.nn import functional as F
from chapter3_5 import show_images
from chapter5_6 import try_all_gpus
from chapter13_1 import train_ch13
from chapter13_9 import load_data_voc,VOC_COLORMAP,read_voc_images
import matplotlib.pyplot as plt


def bilinear_kernel(in_channels, out_channels, kernel_size):
    factor = (kernel_size + 1) // 2
    if kernel_size % 2 == 1:
        center = factor - 1
    else:
        center = factor - 0.5
    og = (torch.arange(kernel_size).reshape(-1, 1),
          torch.arange(kernel_size).reshape(1, -1))
    filt = (1 - torch.abs(og[0] - center) / factor) * \
           (1 - torch.abs(og[1] - center) / factor)
    weight = torch.zeros((in_channels, out_channels,
                          kernel_size, kernel_size))
    weight[range(in_channels), range(out_channels), :, :] = filt
    return weight

def loss(inputs, targets):
    return F.cross_entropy(inputs, targets, reduction='none').mean(1).mean(1)

def predict(net,img):
    transform = torchvision.transforms.Normalize(
            mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    X = transform(img.float() / 255).unsqueeze(0)
    devices = try_all_gpus()
    # X = test_iter.dataset.normalize_image(img).unsqueeze(0)
    pred = net(X.to(devices[0])).argmax(dim=1)
    return pred.reshape(pred.shape[1], pred.shape[2])

def label2image(pred):
    colormap = torch.tensor(VOC_COLORMAP, device=devices[0])
    X = pred.long()
    return colormap[X, :]


if __name__ == '__main__':
    pretrained_net = torchvision.models.resnet18(weights=torchvision.models.ResNet18_Weights.DEFAULT)
    # print(list(pretrained_net.children())[-3:])

    net = nn.Sequential(*list(pretrained_net.children())[:-2])
    X = torch.rand(size=(1, 3, 320, 480))
    # print(net(X).shape)

    num_classes = 21
    net.add_module('final_conv',nn.Conv2d(512,num_classes,kernel_size=1))
    net.add_module('transpose_conv',nn.ConvTranspose2d(num_classes,num_classes,kernel_size=64,padding=16,stride=32))

    is_train = False
    if is_train:
        W = bilinear_kernel(num_classes, num_classes, 64)
        net.transpose_conv.weight.data.copy_(W)
        batch_size, crop_size = 16, (320, 480)
        train_iter, test_iter = load_data_voc(batch_size, crop_size)

        num_epochs, lr, wd, devices = 5, 0.001, 1e-3, try_all_gpus()
        trainer = torch.optim.SGD(net.parameters(), lr=lr, weight_decay=wd)
        train_ch13(net, train_iter, test_iter, loss, trainer, num_epochs, devices)
        torch.save(net.state_dict(),'net.pt')
        plt.show()
    else:
        net.load_state_dict(torch.load('net.pt'))
        devices = try_all_gpus()
        net.to(devices[0])
        net.eval()
        voc_dir = '../data/VOCdevkit/VOC2012'
        test_images, test_labels = read_voc_images(voc_dir, False)
        n, imgs,start_idx = 4, [], 50
        for i in range(n):
            crop_rect = (0, 0, 320, 480)
            X = torchvision.transforms.functional.crop(test_images[i+start_idx], *crop_rect)
            pred = label2image(predict(net,X))
            imgs += [X.permute(1,2,0), pred.cpu(),
                    torchvision.transforms.functional.crop(
                        test_labels[i+start_idx], *crop_rect).permute(1,2,0)]
        show_images(imgs[::3] + imgs[1::3] + imgs[2::3], 3, n, scale=2)
        plt.show()

