import os
import re
import torch
from torch import nn
from torch.nn import functional as F
import numpy as np


# data with compensation
file_dir= 'F:/0_project/prca/data/2#datasets'
filename = 'validata_rpy2_2024-082.*.txt'
def find_all_files(directory,filename):
    file_list = []
    for name in os.listdir(directory):
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

x_data,y_data = load_all_data(find_all_files(file_dir,filename))
x_train,y_train,x_test,y_test = split_data(x_data,y_data)


net = nn.Sequential(nn.Linear(4,256),nn.ReLU(),nn.Linear(256,2))
def init_weights(m):
    if isinstance(m, nn.Linear):
        nn.init.normal_(m.weight, mean=0, std=0.1)

net.apply(init_weights)
loss = nn.MSELoss()
trainer = torch.optim.SGD(net.parameters(),lr=0.01)
num_epochs = 1000
err = 10
epoch = 0
while err > 1e-7:
    imputs = torch.tensor(x_train,dtype=torch.float32)
    targets = torch.tensor(y_train,dtype=torch.float32)

    outputs = net(imputs)
    l = loss(outputs,targets)
    err = l.item()

    trainer.zero_grad()
    l.backward()
    trainer.step()

    if (epoch+1) % 100 == 0:
        print('Epoch [{}/{}], Loss: {:.8f}'.format(epoch+1, num_epochs, l.item()))
    epoch += 1
