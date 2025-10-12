import numpy as np
import matplotlib.pyplot as plt

T = 0.115
state = np.loadtxt('/home/pooh/car_code_ws/state.txt')
error_state = np.loadtxt('/home/pooh/car_code_ws/state.txt')
control = np.loadtxt('/home/pooh/car_code_ws/control.txt')
X_o_d = np.loadtxt('/home/pooh/car_code_ws/X_o_d.txt')
n = int(np.size(error_state)/3)
m = int(np.size(control)/2)
xod = int(np.size(X_o_d)/3)
print(n)
print(m)
error_state = error_state.reshape(n,3)
control = control.reshape(m,2)
X_o_d = X_o_d.reshape(xod,3)
# print(control)
# print(error_state)
# # make data
time = np.linspace(0, n*T, n)
time2 = np.linspace(0, m*T, m)
time3 = np.linspace(0, xod*T, xod)
# 画图
fig = plt.figure()
# plot
plt.xlabel('x',fontsize=20)
plt.ylabel('y',fontsize=20)
plt.grid(True)
plt.plot(error_state[:,0], error_state[:,1], linewidth=2.0)
plt.plot(X_o_d[:,0], X_o_d[:,1], linewidth=2.0)

fig = plt.figure()
# plot
plt.xlabel('time/s',fontsize=20)
plt.ylabel('x',fontsize=20)
plt.grid(True)
plt.plot(time, error_state[0:n,0], linewidth=2.0)
plt.plot(time, X_o_d[0:n,0], linewidth=2.0)

fig = plt.figure()
# plot
plt.xlabel('time/s',fontsize=20)
plt.ylabel('y',fontsize=20)
plt.grid(True)
plt.plot(time, error_state[0:n,1], linewidth=2.0)
plt.plot(time, X_o_d[0:n,1], linewidth=2.0)

fig = plt.figure()
# plot
plt.xlabel('time/s',fontsize=20)
plt.ylabel('phi',fontsize=20)
plt.grid(True)
plt.plot(time, error_state[0:n,1], linewidth=2.0)
plt.plot(time, X_o_d[0:n,1], linewidth=2.0)

fig = plt.figure()
# plot
plt.xlabel('time/s',fontsize=20)
plt.ylabel('Velocity',fontsize=20)
plt.grid(True)
plt.plot(time2, control[:,0], linewidth=2.0)


fig = plt.figure()
# plot
plt.xlabel('time/s',fontsize=20)
plt.ylabel('Angular Velocity',fontsize=20)
plt.grid(True)
plt.plot(time2, control[:,1], linewidth=2.0)

# fig = plt.figure()
# # plot
# plt.xlabel('time/s',fontsize=20)
# plt.ylabel('c Velocity',fontsize=20)
# plt.grid(True)
# plt.plot(time, c_seq[:,1], linewidth=2.0)

# fig = plt.figure()
# # plot
# plt.xlabel('time/s',fontsize=20)
# plt.ylabel('c Angular Velocity',fontsize=20)
# plt.grid(True)
# plt.plot(time, c_seq[:,1], linewidth=2.0)

plt.show()
