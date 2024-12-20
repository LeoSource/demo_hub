import torch
from torch import nn



def trans_conv(X, K):
    h, w = K.shape
    Y = torch.zeros((X.shape[0] + h - 1, X.shape[1] + w - 1))
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            Y[i: i + h, j: j + w] += X[i, j] * K
    return Y



if __name__ == '__main__':
    X = torch.tensor([[0.0, 1.0], [2.0, 3.0]])
    K = torch.tensor([[0.0, 1.0], [2.0, 3.0]])
    print(trans_conv(X, K))

    X, K = X.reshape(1, 1, 2, 2), K.reshape(1, 1, 2, 2)
    tconv = nn.ConvTranspose2d(1, 1, kernel_size=2, bias=False)
    tconv.weight.data = K
    print(tconv(X))

    X = torch.rand(size=(1, 10, 16, 16))
    conv = nn.Conv2d(10, 20, kernel_size=5, padding=2, stride=3)
    tconv = nn.ConvTranspose2d(20, 10, kernel_size=5, padding=2, stride=3)
    print(tconv(conv(X)).shape == X.shape)







