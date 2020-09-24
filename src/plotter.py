import pandas as pd
import matplotlib.pyplot as plt

XY = pd.read_csv('~/mdmi_data/I0/XY.csv')
Z = pd.read_csv('~/mdmi_data/I0/Z.csv')

attitude = pd.read_csv('~/mdmi_data/I0/Attitude.csv')
attitude_cmd = pd.read_csv('~/mdmi_data/I0/CommandAttitude.csv')
yawdata = pd.read_csv('~/mdmi_data/I0/Yaw.csv')
yawdata_cmd = pd.read_csv('~/mdmi_data/I0/CommandYaw.csv')

# attitude = pd.read_csv('~/mdmi_data/I0/PQ.csv')
# attitude_cmd = pd.read_csv('~/mdmi_data/I0/CommandPQ.csv')
# omega_df = pd.read_csv('~/mdmi_data/I0/PWM.csv')
# wrench = pd.read_csv('~/mdmi_data/I0/Wrench.csv')


xe = XY['x'].to_numpy()
ye = XY['y'].to_numpy()
txy = XY['t'].to_numpy() + XY['tsec'].to_numpy()/1e9;

z = Z['z'].to_numpy()
tz = Z['t'].to_numpy() + Z['tsec'].to_numpy()/1e9;


t = attitude['t'].to_numpy()
tsec = attitude['tsec'].to_numpy()
t = t + tsec/1e9

roll = attitude['roll'].to_numpy()
pitch = attitude['pitch'].to_numpy()

t_y = yawdata['t']
tsec_y = yawdata['tsec']
t_y = t_y + tsec_y/1e9

yaw = yawdata['yaw'].to_numpy()


t_cmd = attitude_cmd['t'].to_numpy()
tsec_cmd = attitude_cmd['tsec'].to_numpy()
t_cmd = t_cmd + tsec_cmd/1e9

roll_cmd = attitude_cmd['roll'].to_numpy()
pitch_cmd = attitude_cmd['pitch'].to_numpy()

yaw_cmd = yawdata_cmd['yaw'].to_numpy()
t_yc = yawdata_cmd['t']
tsec_yc = yawdata_cmd['tsec']
t_yc = t_yc + tsec_yc/1e9


# omega = omega_df.filter(regex='omega').to_numpy()
# ptorq = [((omg[2]**2+omg[3]**2)-(omg[0]**2+omg[1]**2))/100000000 for omg in omega]
# qtorq = [((omg[1]**2+omg[2]**2)-(omg[0]**2+omg[3]**2))/100000000 for omg in omega]
# rtorq = [(-(omg[0]**2+omg[2]**2)+(omg[1]**2+omg[3]**2))/200000000 for omg in omega]
# t_omg = omega_df['t'].to_numpy()
# tsec_omg = omega_df['tsec'].to_numpy()
# t_omg = t_omg + tsec_omg/1e9
# t_wr = wrench['t'].to_numpy()
# tsec_wr = wrench['tsec'].to_numpy()
# t_wr = t_wr + tsec_wr/1e9
# ptorq = wrench['roll'].to_numpy()*100
# qtorq = wrench['pitch'].to_numpy()*100
# rtorq = wrench['yaw'].to_numpy()*100

# print(t_cmd)

# plt.plot(t, roll, '-', label='actual roll')
# plt.plot(t_cmd, roll_cmd, '-', label='roll command')
# # plt.plot(t_omg, ptorq, '-', label='roll torque')

# plt.plot(t, pitch, '-', label='actual pitch')
# plt.plot(t_cmd, pitch_cmd, '-', label='command')
# # plt.plot(t_omg, qtorq, '-', label='torque')

# plt.plot(t_y, yaw, '-', label='actual yao')
# plt.plot(t_yc, yaw_cmd, '-', label='command')
# plt.plot(t_omg, rtorq, '-', label='torque')

plt.plot(txy, xe, label='x')
plt.plot(txy, ye, label='y')
plt.plot(tz, z, label='z')

plt.grid()
plt.legend()
# plt.ylim(-0.001, 0.001)
plt.show()


