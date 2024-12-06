import torch
import torchvision
from torch import nn
from PIL import Image
import matplotlib.pyplot as plt
from chapter2 import set_figsize
from chapter3_1 import Timer
from chapter3_5 import show_images,get_dataloader_workers
from chapter3_6 import accuracy,Animator,Accumulator
from chapter5_6 import try_all_gpus
from chapter6_6 import evaluate_accuracy_gpu
from chapter12_6 import resnet18


def apply(img, aug, num_rows=2, num_cols=4, scale=1.5):
    Y = [aug(img) for _ in range(num_rows * num_cols)]
    show_images(Y, num_rows, num_cols, scale=scale)

def test_apply():
    set_figsize()
    img = Image.open('./img/cat1.jpg')
    # plt.imshow(img)

    # apply(img, torchvision.transforms.RandomHorizontalFlip())
    # apply(img, torchvision.transforms.RandomVerticalFlip())

    shape_aug = torchvision.transforms.RandomResizedCrop(
        (200, 200), scale=(0.1, 1), ratio=(0.5, 2))
    # apply(img, shape_aug)

    # apply(img, torchvision.transforms.ColorJitter(
    #     brightness=0.5, contrast=0, saturation=0, hue=0))

    # apply(img, torchvision.transforms.ColorJitter(
    #     brightness=0, contrast=0, saturation=0, hue=0.5))
    
    color_aug = torchvision.transforms.ColorJitter(
        brightness=0.5, contrast=0.5, saturation=0.5, hue=0.5)
    # apply(img, color_aug)

    augs = torchvision.transforms.Compose([
        torchvision.transforms.RandomHorizontalFlip(), color_aug, shape_aug])
    apply(img, augs)

    plt.show()


def load_cifar10(is_train, augs, batch_size):
    dataset = torchvision.datasets.CIFAR10(root="../data", train=is_train,
                                           transform=augs, download=True)
    dataloader = torch.utils.data.DataLoader(dataset, batch_size=batch_size,
                    shuffle=is_train, num_workers=get_dataloader_workers())
    return dataloader

#@save
def train_batch_ch13(net, X, y, loss, trainer, devices):
    """用多GPU进行小批量训练"""
    if isinstance(X, list):
        # 微调BERT中所需
        X = [x.to(devices[0]) for x in X]
    else:
        X = X.to(devices[0])
    y = y.to(devices[0])
    net.train()
    trainer.zero_grad()
    pred = net(X)
    l = loss(pred, y)
    l.sum().backward()
    trainer.step()
    train_loss_sum = l.sum()
    train_acc_sum = accuracy(pred, y)
    return train_loss_sum, train_acc_sum

#@save
def train_ch13(net, train_iter, test_iter, loss, trainer, num_epochs,
               devices=try_all_gpus()):
    """用多GPU进行模型训练"""
    timer, num_batches = Timer(), len(train_iter)
    animator = Animator(xlabel='epoch', xlim=[1, num_epochs], ylim=[0, 1],
                            legend=['train loss', 'train acc', 'test acc'])
    net = nn.DataParallel(net, device_ids=devices).to(devices[0])
    for epoch in range(num_epochs):
        # 4个维度：储存训练损失，训练准确度，实例数，特点数
        metric = Accumulator(4)
        for i, (features, labels) in enumerate(train_iter):
            timer.start()
            l, acc = train_batch_ch13(
                net, features, labels, loss, trainer, devices)
            metric.add(l, acc, labels.shape[0], labels.numel())
            timer.stop()
            if (i + 1) % (num_batches // 5) == 0 or i == num_batches - 1:
                animator.add(epoch + (i + 1) / num_batches,
                             (metric[0] / metric[2], metric[1] / metric[3],
                              None))
        test_acc = evaluate_accuracy_gpu(net, test_iter)
        animator.add(epoch + 1, (None, None, test_acc))
    print(f'loss {metric[0] / metric[2]:.3f}, train acc '
          f'{metric[1] / metric[3]:.3f}, test acc {test_acc:.3f}')
    print(f'{metric[2] * num_epochs / timer.sum():.1f} examples/sec on '
          f'{str(devices)}')

def train_with_data_aug(train_augs, test_augs, net, lr=0.001):
    train_iter = load_cifar10(True, train_augs, batch_size)
    test_iter = load_cifar10(False, test_augs, batch_size)
    loss = nn.CrossEntropyLoss(reduction="none")
    trainer = torch.optim.Adam(net.parameters(), lr=lr)
    train_ch13(net, train_iter, test_iter, loss, trainer, 10, devices)





batch_size, devices, net = 256, try_all_gpus(), resnet18(10, 3)

if __name__ == '__main__':
    # test_apply()


    all_images = torchvision.datasets.CIFAR10(train=True, root="../data",
                                          download=True)
    # show_images([all_images[i][0] for i in range(32)], 4, 8, scale=0.8)
    # plt.show()

    train_augs = torchvision.transforms.Compose([
        torchvision.transforms.RandomHorizontalFlip(),
        torchvision.transforms.ToTensor()])

    test_augs = torchvision.transforms.Compose([
        torchvision.transforms.ToTensor()])


    def init_weights(m):
        if type(m) in [nn.Linear, nn.Conv2d]:
            nn.init.xavier_uniform_(m.weight)
    net.apply(init_weights)
    train_with_data_aug(train_augs, test_augs, net)
    plt.show()
