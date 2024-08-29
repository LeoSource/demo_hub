from xgboost import XGBRegressor
from dataset import pdf_data, load_as_numpy, load_as_torch, mapminmax
import matplotlib.pyplot as plt
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error


x_data,y_data = load_as_numpy('F:/0_project/prca/data/2#datasets')
x_train,x_test,y_train,y_test = train_test_split(x_data, y_data, test_size=0.2)


model = XGBRegressor(booster='gbtree',  # gblinear
                     n_estimators=300,  # 迭代次数
                     learning_rate=0.01,  # 步长
                     max_depth=10,  # 树的最大深度
                     # min_child_weight=0.5,  # 决定最小叶子节点样本权重和
                     seed=123,  # 指定随机种子，为了复现结果
                     )

model.fit(x_train, y_train, verbose=True)

y_predict = model.predict(x_test)

mse = mean_squared_error(y_test, y_predict)
print(f'mse:{mse:.8f}')

rx_err_before = np.rad2deg(y_data[:,0]-x_data[:,2])
ry_err_before = np.rad2deg(y_data[:,1]-x_data[:,3])
rx_err_after = np.rad2deg(y_predict[:,0]-y_test[:,0])
ry_err_after = np.rad2deg(y_predict[:,1]-y_test[:,1])
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