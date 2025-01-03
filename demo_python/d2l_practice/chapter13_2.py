import os
import torch
import torchvision
from torch import nn
from chapter3_5 import show_images
from chapter4_10 import download_extract,DATA_HUB,DATA_URL
from chapter5_6 import try_all_gpus
from chapter13_1 import train_ch13
import matplotlib.pyplot as plt


# 如果param_group=True，输出层中的模型参数将使用十倍的学习率
def train_fine_tuning(net, learning_rate, batch_size=128, num_epochs=5,
                      param_group=True):
    train_iter = torch.utils.data.DataLoader(torchvision.datasets.ImageFolder(
        os.path.join(data_dir, 'train'), transform=train_augs),
        batch_size=batch_size, shuffle=True)
    test_iter = torch.utils.data.DataLoader(torchvision.datasets.ImageFolder(
        os.path.join(data_dir, 'test'), transform=test_augs),
        batch_size=batch_size)
    devices = try_all_gpus()
    loss = nn.CrossEntropyLoss(reduction="none")
    if param_group:
        params_1x = [param for name, param in net.named_parameters()
             if name not in ["fc.weight", "fc.bias"]]
        trainer = torch.optim.SGD([{'params': params_1x},
                                   {'params': net.fc.parameters(), 'lr': learning_rate * 10}],
                                lr=learning_rate, weight_decay=0.001)
    else:
        trainer = torch.optim.SGD(net.parameters(), lr=learning_rate,
                                  weight_decay=0.001)
    train_ch13(net, train_iter, test_iter, loss, trainer, num_epochs, devices)


if __name__=='__main__':
    DATA_HUB['hotdog'] = (DATA_URL + 'hotdog.zip',
                            'fba480ffa8aa7e0febbb511d181409f899b9baa5')

    data_dir = download_extract('hotdog')

    train_imgs = torchvision.datasets.ImageFolder(os.path.join(data_dir, 'train'))
    test_imgs = torchvision.datasets.ImageFolder(os.path.join(data_dir, 'test'))

    # hotdogs = [train_imgs[i][0] for i in range(8)]
    # not_hotdogs = [train_imgs[-i - 1][0] for i in range(8)]
    # show_images(hotdogs + not_hotdogs, 2, 8, scale=1.4)
    # plt.show()


    # 使用RGB通道的均值和标准差，以标准化每个通道
    normalize = torchvision.transforms.Normalize(
        [0.485, 0.456, 0.406], [0.229, 0.224, 0.225])

    train_augs = torchvision.transforms.Compose([
        torchvision.transforms.RandomResizedCrop(224),
        torchvision.transforms.RandomHorizontalFlip(),
        torchvision.transforms.ToTensor(),
        normalize])

    test_augs = torchvision.transforms.Compose([
        torchvision.transforms.Resize([256, 256]),
        torchvision.transforms.CenterCrop(224),
        torchvision.transforms.ToTensor(),
        normalize])
    
    # pretrained_net = torchvision.models.resnet18(pretrained=True)
    # print(pretrained_net)
    # print(pretrained_net.fc)

    # finetune_net = torchvision.models.resnet18(pretrained=True)
    # finetune_net.fc = nn.Linear(finetune_net.fc.in_features, 2)
    # nn.init.xavier_uniform_(finetune_net.fc.weight)
    # train_fine_tuning(finetune_net, 5e-5)

    scratch_net = torchvision.models.resnet18()
    scratch_net.fc = nn.Linear(scratch_net.fc.in_features, 2)
    train_fine_tuning(scratch_net, 5e-4, param_group=False)

    plt.show()
