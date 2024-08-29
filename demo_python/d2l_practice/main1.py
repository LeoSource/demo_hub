import torch
from torch import nn
import torch.utils
from torch.utils.data import TensorDataset, DataLoader, random_split
import numpy as np
import torch.utils.data
import matplotlib.pyplot as plt
from dataset import pdf_data, load_as_numpy, load_as_torch, mapminmax
from model import MyNetModule, SimpleModel
from trainer import train
from predicter import predict


file_dir= 'F:/0_project/prca/data/2#datasets'
x_data,y_data = load_as_numpy(file_dir)
# x_torch,y_torch = dataset.load_as_torch(file_dir)

# 指定归一化区间为 [-1, 1]
feature_range = (-2, 2)
# 归一化处理
x_data_norm, input_scaler = mapminmax(x_data, feature_range)
y_data_norm, output_scaler = mapminmax(y_data, feature_range)
x_torch = torch.tensor(x_data_norm,dtype=torch.float64)
y_torch = torch.tensor(y_data_norm,dtype=torch.float64)
datasets = TensorDataset(x_torch,y_torch)

training = True
net = nn.Sequential(nn.Linear(4,32,dtype=torch.float64),
                    nn.LayerNorm(32,dtype=torch.float64),
                    nn.Tanh(),
                    nn.Linear(32,2,dtype=torch.float64))
# net = SimpleModel(4,20,2,training)

if training:
    train(net=net,datasets=datasets,train_ratio=0.85,max_epochs=100)
else:
    net = nn.Sequential(nn.Linear(4,32),nn.Tanh(),nn.Linear(32,2))
    w1 = np.loadtxt('F:/0_project/prca/src/w1.txt')
    w2 = np.loadtxt('F:/0_project/prca/src/w2.txt')
    b1 = np.loadtxt('F:/0_project/prca/src/b1.txt')
    b2 = np.loadtxt('F:/0_project/prca/src/b2.txt')
    net[0].weight.data = torch.tensor(w1,dtype=torch.float64)
    net[0].bias.data = torch.tensor(b1,dtype=torch.float64)
    net[2].weight.data = torch.tensor(w2,dtype=torch.float64)
    net[2].bias.data = torch.tensor(b2,dtype=torch.float64)


# predict and compare
x_data,y_data = load_as_numpy('F:/0_project/prca/data/2#datasets')
y_predict = predict(net,x_data,input_scaler,output_scaler)

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
