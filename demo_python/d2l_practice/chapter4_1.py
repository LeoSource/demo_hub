import torch
from chapter2 import plot
import matplotlib.pyplot as plt

def plot_relu():
    x = torch.arange(-8.0, 8.0, 0.1, requires_grad=True)
    y = torch.relu(x)
    plot(x.detach(), y.detach(), 'x', 'relu(x)', figsize=(5, 2.5))

    y.backward(torch.ones_like(x), retain_graph=True)
    plot(x.detach(), x.grad, 'x', 'grad of relu', figsize=(5, 2.5))


def plot_sigmoid():
    x = torch.arange(-8.0, 8.0, 0.1, requires_grad=True)
    y = torch.sigmoid(x)
    plot(x.detach(), y.detach(), 'x', 'sigmoid(x)', figsize=(5, 2.5))

    y.backward(torch.ones_like(x), retain_graph=True)
    plot(x.detach(), x.grad, 'x', 'grad of sigmoid', figsize=(5, 2.5))


def plot_tanh():
    x = torch.arange(-8.0, 8.0, 0.1, requires_grad=True)
    y = torch.tanh(x)
    plot(x.detach(), y.detach(), 'x', 'tanh(x)', figsize=(5, 2.5))

    y.backward(torch.ones_like(x), retain_graph=True)
    plot(x.detach(), x.grad, 'x', 'grad of tanh', figsize=(5, 2.5))

if __name__ == "__main__":
    # plot_relu()
    # plot_sigmoid()
    plot_tanh()