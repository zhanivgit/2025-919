import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression

plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False

t = np.arange(0, 50, 0.1)
y_true = np.sin(t)

X = []
y_target = []

for i in range(2, len(y_true)):
    X.append([y_true[i-1], y_true[i-2]])
    y_target.append(y_true[i])

X = np.array(X)
y_target = np.array(y_target)

model = LinearRegression()
model.fit(X, y_target)

a1, a2 = model.coef_
intercept = model.intercept_

print("模型求解结果:")
print(f"系数 a1 (y(k-1)的系数): {a1:.6f}")
print(f"系数 a2 (y(k-2)的系数): {a2:.6f}")
print(f"截距: {intercept:.6f}")

y_fit = model.predict(X)

forecast_steps = 100
y_forecast = []

last_points = list(y_true[-2:])

for _ in range(forecast_steps):
    current_input = np.array([last_points[-1], last_points[-2]]).reshape(1, -1)
    next_pred = model.predict(current_input)[0]
    y_forecast.append(next_pred)
    last_points.append(next_pred)

y_forecast = np.array(y_forecast)

plt.figure(figsize=(16, 8))

plt.plot(t, y_true, label='原始数据 y=sin(t)', color='blue', linewidth=2)

plt.plot(t[2:], y_fit, label='AR(2)模型拟合 (一步预测)', color='green', linestyle='--', linewidth=2)

last_t = t[-1]
step = t[1] - t[0]
t_forecast = np.arange(last_t + step, last_t + step * (forecast_steps + 1), step)
plt.plot(t_forecast, y_forecast, label='AR(2)模型预测 (多步预测)', color='red', linestyle='-.', linewidth=2)

plt.title('AR(2)模型对sin(t)时间序列的拟合与预测', fontsize=16)
plt.xlabel('时间 t', fontsize=12)
plt.ylabel('y 值', fontsize=12)
plt.legend(fontsize=12)
plt.grid(True)
plt.axvline(x=t[-1], color='gray', linestyle=':', label='预测起始点')
plt.show()