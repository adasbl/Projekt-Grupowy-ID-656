import numpy as np
import matplotlib.pyplot as plt

axel_length = 0.148
wheel_radius = 0.07
L = axel_length
R = wheel_radius
dt = 0.1

est_x = [0.0]
est_y = [0.0]
est_alfa = [0.0]
real_x = [0.0]
real_y = [0.0]
real_alfa = [0.0]

n_steps = 100
w_l_true = np.linspace(5.0, 4.0, n_steps)
w_r_true = np.linspace(5.0, 6.0, n_steps)

w_l_noisy = []
w_r_noisy = []
for i in range(n_steps):
    w_l_noisy.append(w_l_true[i] + np.random.normal(0, 0.2))
    w_r_noisy.append(w_r_true[i] + np.random.normal(0, 0.2))

for i in range(n_steps):
    w_l = w_l_noisy[i]
    w_r = w_r_noisy[i]
    
    d_l = R * w_l * dt
    d_r = R * w_r * dt

    d_c = (d_r + d_l) / 2
    d_alfa = (d_r - d_l) / L

    curr_x = est_x[-1]
    curr_y = est_y[-1]
    curr_alfa = est_alfa[-1]

    est_x.append(curr_x + np.cos(curr_alfa + d_alfa / 2.0) * d_c)
    est_y.append(curr_y + np.sin(curr_alfa + d_alfa / 2.0) * d_c)
    est_alfa.append(curr_alfa + d_alfa)

    w_l = w_l_true[i]
    w_r = w_r_true[i]
    
    d_l = R * w_l * dt
    d_r = R * w_r * dt

    d_c = (d_r + d_l) / 2
    d_alfa = (d_r - d_l) / L

    curr_x = real_x[-1]
    curr_y = real_y[-1]
    curr_alfa = real_alfa[-1]

    real_x.append(curr_x + np.cos(curr_alfa + d_alfa / 2.0) * d_c)
    real_y.append(curr_y + np.sin(curr_alfa + d_alfa / 2.0) * d_c)
    real_alfa.append(curr_alfa + d_alfa)

#estymacja z IMU
acc_est_x = [0.0]
acc_est_y = [0.0]
acc_est_alfa = [0.0]

n_steps = 100
acc_x = []
gyro_z = []

prev_v = R * (w_r_true[0] + w_l_true[0]) / 2.0

for i in range(n_steps):
    v_true = R * (w_r_true[i] + w_l_true[i]) / 2.0
    acc = (v_true - prev_v) / dt
    prev_v = v_true
    acc_x.append(acc + np.random.normal(0, 0.05))
    
    w_true = R * (w_r_true[i] - w_l_true[i]) / L
    gyro_z.append(w_true + np.random.normal(0, 0.05))

curr_v = R * (w_r_true[0] + w_l_true[0]) / 2.0 

for i in range(n_steps):
    curr_x = acc_est_x[-1]    
    curr_y = acc_est_y[-1]    
    curr_alfa = acc_est_alfa[-1]    

    d_c = curr_v * dt + (acc_x[i] * dt**2) / 2.0 
    curr_v += acc_x[i] * dt
    
    d_alfa = gyro_z[i] * dt
    
    acc_est_x.append(curr_x + np.cos(curr_alfa + d_alfa / 2.0) * d_c)
    acc_est_y.append(curr_y + np.sin(curr_alfa + d_alfa / 2.0) * d_c)
    acc_est_alfa.append(curr_alfa + d_alfa)    

#estymacja z lidara
lidar_x = []
lidar_y = []

for i in range(len(real_x)):
    lidar_x.append(real_x[i] + np.random.normal(0, 0.05))
    lidar_y.append(real_y[i] + np.random.normal(0, 0.05))

#filtr kalmana
P = np.diag([0.01,0.01,0.01])
Q = np.diag([0.05,0.05,0.05])
R_mat = np.diag([0.3,0.3])
H = np.array([[1.0, 0.0, 0.0],
             [0.0, 1.0, 0.0]])

kal_est_x = [0.0]
kal_est_y = [0.0]
kal_est_alfa = [0.0]

curr_v = R * (w_r_true[0] + w_l_true[0]) / 2.0
for i in range(n_steps):
    curr_x = kal_est_x[-1]
    curr_y = kal_est_y[-1]
    curr_alfa = kal_est_alfa[-1]

    d_c = curr_v * dt + (acc_x[i] * dt**2) / 2.0
    d_alfa = gyro_z[i]*dt
    
    curr_v += acc_x[i]*dt

    x_new = curr_x + d_c*np.cos(curr_alfa + d_alfa/2.0)
    y_new = curr_y + d_c*np.sin(curr_alfa + d_alfa/2.0)
    alfa_new = curr_alfa + d_alfa

    new_state = np.array([x_new,y_new,alfa_new])

    F = np.array([[1.0, 0.0, -d_c*np.sin(curr_alfa+d_alfa/2.0)],
                 [0.0, 1.0,  d_c*np.cos(curr_alfa+d_alfa/2.0)],
                 [0.0, 0.0,  1.0]])

    P = F @ P @ F.T + Q

    lidar_state = np.array([lidar_x[i+1],lidar_y[i+1]])

    Y = lidar_state - H @ new_state

    S = H @ P @ H.T + R_mat

    K = P @ H.T @ np.linalg.inv(S)

    #skorygowanie estymacji
    corrected_state = new_state + K @ Y
    P = (np.eye(3) - K @ H) @ P

    kal_est_x.append(corrected_state[0])
    kal_est_y.append(corrected_state[1]) 
    kal_est_alfa.append(corrected_state[2])

plt.figure(figsize=(10, 8))
plt.plot(est_x, est_y, label="Odometria z zaszumionych enkoderow", color="blue")
plt.plot(real_x, real_y, label="Rzeczywista trasa robota", color="red", linewidth=2)
plt.plot(acc_est_x, acc_est_y, label="Odometria z IMU", color="green", linestyle="--")
plt.scatter(lidar_x, lidar_y, label="Scan Matching (Lidar)", color="orange", marker="x", s=20, zorder=5)
plt.plot(kal_est_x,kal_est_y,label = 'Filtr Kalmana', color = 'black')

plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.title("Estymacja trasy robota (Enkodery, IMU, Lidar)")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.show()