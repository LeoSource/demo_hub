import torch
from dataset import pdf_data, load_as_numpy
import matplotlib.pyplot as plt
from predicter import predict
import joblib
import numpy as np


# model = torch.load('nn_model2.pth')
# input_scaler = joblib.load('input_scaler2.joblib')
# output_scaler = joblib.load('output_scaler2.joblib')

model = joblib.load('xgb_model2.joblib')



# predict and compare
x_data,y_data = load_as_numpy('F:/0_project/prca/data/2#datasets')
# y_predict = predict(model,x_data,input_scaler,output_scaler)
y_predict = model.predict(x_data)

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