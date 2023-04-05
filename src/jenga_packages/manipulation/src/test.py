import numpy as np
joints_traj = np.random.rand(15,7)
min_jerk_traj = np.zeros(((joints_traj.shape[0]-1)*20 + 1, joints_traj.shape[1]))
t_interp = np.linspace(0.05, 1, 20) # expand each trajectory edge to 20 points
min_jerk_traj[0,:] = joints_traj[0,:]
for i in range(1, joints_traj.shape[0]):
    for t_i in range(len(t_interp)):
        dt = t_interp[t_i]
        interp_traj_i = joints_traj[i,:]*dt + joints_traj[i-1,:]*(1-dt)
        min_jerk_traj[(i-1)*20 + t_i+1,:] = interp_traj_i
print('min_jerk_traj shape: ', min_jerk_traj.shape)
import matplotlib.pyplot as plt
plt.plot(min_jerk_traj[:,3])
plt.figure()
plt.plot(joints_traj[:,3])
plt.show()