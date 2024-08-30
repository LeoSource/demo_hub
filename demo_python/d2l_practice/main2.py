from xgboost import XGBRegressor
from dataset import pdf_data, load_as_numpy, load_as_torch, mapminmax
import matplotlib.pyplot as plt
import numpy as np
from sklearn.model_selection import train_test_split, GridSearchCV, RandomizedSearchCV, cross_val_score
from sklearn.metrics import mean_squared_error
import joblib


x_data,y_data = load_as_numpy('F:/0_project/prca/data/2#datasets')
# 指定归一化区间为 [-1, 1]
feature_range = (-2, 2)
# 归一化处理
x_data_norm, input_scaler = mapminmax(x_data, feature_range)
y_data_norm, output_scaler = mapminmax(y_data, feature_range)
x_train,x_test,y_train,y_test = train_test_split(x_data_norm, y_data_norm,
                                                 test_size=0.2, random_state=42)


model = XGBRegressor(booster='gbtree',  # gblinear
                     n_estimators=300,  # 迭代次数
                     learning_rate=0.03,  # 步长
                     max_depth=10,  # 树的最大深度
                     # min_child_weight=0.5,  # 决定最小叶子节点样本权重和
                     seed=123,  # 指定随机种子，为了复现结果
                     )

param_dist = {
    'n_estimators': [500],
    'max_depth': [5],  # 限制树的深度，避免过复杂的模型
    'learning_rate': [0.03],
    'subsample': [1],  # 控制样本采样比例
    'colsample_bytree': [1.0],  # 控制特征采样比例
    'gamma': [0],  # 控制树的分裂
    'lambda': [5],  # L2正则化，防止过拟合
    'alpha': [0]  # L1正则化，防止过拟合
}
# 自定义fit_params，用于传递早停法参数
fit_params = {
    'eval_set': [(x_test, y_test)],
    'eval_metric': 'rmse',
    'early_stopping_rounds': 10,
    'verbose': False  # 设为True会输出每轮的性能评估结果
}

grid_serach = GridSearchCV(estimator=model,
                           param_grid=param_dist,
                           cv=10,
                           scoring='neg_mean_squared_error',
                           n_jobs=-1,
                           verbose=1)
grid_serach.fit(x_train, y_train)
print(f'best_params:{grid_serach.best_params_}')
print(f'best_score:{grid_serach.best_score_:.8f}')

best_model = grid_serach.best_estimator_
# save model
joblib.dump(best_model, './xgb_model2.joblib')

y_predict = best_model.predict(x_data)

mse = mean_squared_error(y_data, y_predict)
print(f'mse:{mse:.8f}')

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