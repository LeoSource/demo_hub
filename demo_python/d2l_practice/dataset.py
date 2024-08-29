import os
import re
import numpy as np
import torch
from sklearn.preprocessing import MinMaxScaler


def find_all_files(directory,filename=None):
    file_list = []
    for name in os.listdir(directory):
        if filename is None:
            file_list.append(os.path.join(directory,name))
        else:
            if re.match(filename,name):
                file_list.append(os.path.join(directory,name))
    return file_list

def load_all_data(file_list,dtype=np.float64):
    x_data_list = []
    y_data_list = []
    for file in file_list:
        data = np.loadtxt(file,dtype=dtype)
        yd = data[1:,2:4]
        xd = np.zeros((yd.shape[0],4))
        for idx in range(data.shape[0]-1):
            xd[idx,:] = np.hstack((data[idx,0:2],data[idx+1,0:2]))
        x_data_list.append(xd)
        y_data_list.append(yd)
    x_data = np.vstack(x_data_list)
    y_data = np.vstack(y_data_list)
    return x_data,y_data

def load_as_numpy(directory,filename=None,dtype=np.float64):
    return load_all_data(find_all_files(directory,filename),dtype)

def load_as_torch(directory,filename=None,dtype=torch.float64):
    x_data,y_data = load_as_numpy(directory,filename)
    return torch.tensor(x_data,dtype=dtype),torch.tensor(y_data,dtype=dtype)

def load_as_dataset(directory,filename=None,dtype=torch.float64):
    x_data,y_data = load_as_torch(directory,filename,dtype=dtype)
    return torch.utils.data.TensorDataset(x_data,y_data)



def pdf_data(data,npoints):
    xdata = np.linspace(data.min(),data.max(),npoints)
    width = np.abs(xdata[1]-xdata[0])
    edges = np.append(xdata-width/2,xdata[-1]+width/2)
    hist,bin_edges = np.histogram(data,bins=edges)
    return xdata,hist/len(data)


def mapminmax(data,feature_range=(0, 1)):
    scaler = MinMaxScaler(feature_range=feature_range)
    data_norm = scaler.fit_transform(data)
    return data_norm, scaler