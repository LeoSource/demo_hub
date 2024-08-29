import torch


def predict(net,x_data,input_scaler,output_scaler,dtype=torch.float64):
    x = input_scaler.transform(x_data)
    y = net(torch.tensor(x,dtype=dtype))
    y = output_scaler.inverse_transform(y.detach().numpy())
    return y

