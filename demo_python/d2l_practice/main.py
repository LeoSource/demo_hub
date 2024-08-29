import os
import re
import torch
from torch import nn
from torch.nn import functional as F
import torch.utils
from torch.utils.data import Dataset, DataLoader, random_split
import numpy as np
import torch.utils.data
import matplotlib.pyplot as plt
import random


class MapMinMaxForward(nn.Module):
    def __init__(self,size):
        super().__init__()
        self.xoffset = nn.Parameter(torch.randn((size,)))
        self.gain = nn.Parameter(torch.randn((size,)))
        self.ymin = -1

    def forward(self,x):
        y = x - self.xoffset
        y *= self.gain
        y += self.ymin
        return y

class MapMinMaxBackward(nn.Module):
    def __init__(self,size):
        super().__init__()
        self.xoffset = nn.Parameter(torch.randn((size,)))
        self.gain = nn.Parameter(torch.randn((size,)))
        self.ymin = -1
    def forward(self,y):
        x = y - self.ymin
        x /= self.gain
        x += self.xoffset
        return x

def set_seed(seed):
    torch.manual_seed(seed)           # 设置PyTorch CPU的随机数种子
    torch.cuda.manual_seed(seed)      # 设置PyTorch GPU的随机数种子
    torch.cuda.manual_seed_all(seed)  # 如果你在使用多个GPU，需设置所有GPU的随机数种子
    np.random.seed(seed)              # 设置NumPy的随机数种子
    random.seed(seed)                 # 设置Python的随机数种子

    # 保证在使用不同卷积算法时，结果也是一致的
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False

# data with compensation
file_dir= 'F:/0_project/prca/data/2#datasets'
filename = 'validata_rpy.*.txt'
def find_all_files(directory,filename=None):
    file_list = []
    for name in os.listdir(directory):
        if filename is None:
            file_list.append(os.path.join(directory,name))
        else:
            if re.match(filename,name):
                file_list.append(os.path.join(directory,name))
    return file_list

def load_all_data(file_list):
    x_data_list = []
    y_data_list = []
    for file in file_list:
        data = np.loadtxt(file)
        yd = data[1:,2:4]
        xd = np.zeros((yd.shape[0],4))
        for idx in range(data.shape[0]-1):
            xd[idx,:] = np.hstack((data[idx,0:2],data[idx+1,0:2]))
        x_data_list.append(xd)
        y_data_list.append(yd)
    x_data = np.vstack(x_data_list)
    y_data = np.vstack(y_data_list)
    return x_data,y_data

def split_data(x_data,y_data,test_split=0.2,shuffle_dataset=True,random_seed=42):
    dataset_size = x_data.shape[0]
    indices = list(range(dataset_size))
    split = int(np.floor(test_split*dataset_size))
    if shuffle_dataset:
        np.random.seed(random_seed)
        np.random.shuffle(indices)
    train_indices, test_indices = indices[split:], indices[:split]
    return x_data[train_indices], y_data[train_indices],x_data[test_indices],y_data[test_indices]

x_data,y_data = load_all_data(find_all_files(file_dir))
def pdf_data(data,npoints):
    xdata = np.linspace(data.min(),data.max(),npoints)
    width = np.abs(xdata[1]-xdata[0])
    edges = np.append(xdata-width/2,xdata[-1]+width/2)
    hist,bin_edges = np.histogram(data,bins=edges)
    return xdata,hist/len(data)


x_torch = torch.tensor(x_data,dtype=torch.float32)
y_torch = torch.tensor(y_data,dtype=torch.float32)
datasets = torch.utils.data.TensorDataset(*(x_torch,y_torch))
train_size = int(0.85*len(datasets))
test_size = len(datasets) - train_size
train_dataset,test_dataset = random_split(datasets,
                                          [train_size,test_size],
                                          generator=torch.Generator().manual_seed(42))
train_dataloader = DataLoader(train_dataset,batch_size=256,shuffle=True)

net = nn.Sequential(MapMinMaxForward(4),
                    nn.Linear(4,32),
                    nn.Tanh(),
                    nn.Linear(32,2),
                    MapMinMaxBackward(2))
# net = nn.Sequential(nn.Linear(4,256),
#                     nn.ReLU(),
#                     nn.Linear(256,64),
#                     nn.Tanh(),
#                     nn.Linear(64,2))
def init_weights(m):
    if isinstance(m, nn.Linear):
        nn.init.normal_(m.weight, mean=0, std=0.1)

net.apply(init_weights)
loss = nn.MSELoss(reduction='mean')
trainer = torch.optim.SGD(net.parameters(),lr=0.01)
num_epochs = 800
err = 10
epoch = 0
for epoch in range(num_epochs):
    for x,y in train_dataloader:
        y_hat = net(x)
        l = loss(y_hat,y)
        # err = l.item()

        trainer.zero_grad()
        l.backward()
        trainer.step()

    err = loss(net(x_torch),y_torch)
    if (epoch+1) % 10 == 0:
        # print(net.parameters())
        print('Epoch [{}/{}], Loss: {:.8f}'.format(epoch+1, num_epochs, err))

y_predict = net(x_torch).detach().numpy()
rx_err_before = np.rad2deg(y_data[:,0]-x_data[:,2])
ry_err_before = np.rad2deg(y_data[:,1]-x_data[:,3])
rx_err_after = np.rad2deg(y_predict[:,0]-y_data[:,0])
ry_err_after = np.rad2deg(y_predict[:,1]-y_data[:,1])
xp1,yp1 = pdf_data(rx_err_before,100)
xp2,yp2 = pdf_data(rx_err_after,100)
plt.figure()
plt.plot(xp1,yp1,label='before')
plt.plot(xp2,yp2,label='after')
plt.legend()
plt.title('Rx pdf')
xp1,yp1 = pdf_data(ry_err_before,100)
xp2,yp2 = pdf_data(ry_err_after,100)
plt.figure()
plt.plot(xp1,yp1,label='before')
plt.plot(xp2,yp2,label='after')
plt.legend()
plt.title('Ry pdf')
plt.show()