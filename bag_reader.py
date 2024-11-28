from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import math
from scipy.signal import detrend
#----------------------------------------------.bag파일과 시간축 설정------------------------------------------------------------------------
file_name = '2024-10-22-18-23-56'
b = bagreader(file_name+'.bag')
x_min = 0
x_max = 63550
#-------------------------------------------------------------------------------------------------------------------------------------------
print(b.topic_table)
imu_msg = b.message_by_topic(topic='/imu_data')# 토픽.csv생성
pwm_msg = b.message_by_topic(topic='/PWM')
pose_msg = b.message_by_topic(topic='/pose_cmd')
#bldc_msg = b.message_by_topic(topic='/bldc_test')
#tf_msg = b.message_by_topic(topic='/tf')
PPM_msg = b.message_by_topic(topic='/PPM')
#SBUS_msg = b.message_by_topic(topic='/SBUS')
vel_msg = b.message_by_topic(topic='/camera/odom/sample')
#yaw_msg = b.message_by_topic(topic='/yaw_cmd')

df_imu = pd.read_csv(file_name+'/imu_data.csv')
df_PWM = pd.read_csv(file_name+'/PWM.csv')
df_pose = pd.read_csv(file_name+'/pose_cmd.csv')
#df_bldc = pd.read_csv(file_name+'/bldc_test.csv')
#df_tf = pd.read_csv(file_name+'/tf.csv')
df_PPM = pd.read_csv(file_name+'/PPM.csv')
#df_SBUS = pd.read_csv(file_name+'/SBUS.csv')
df_vel = pd.read_csv(file_name+'/camera-odom-sample.csv')
#df_Yaw = pd.read_csv(file_name+'/yaw_cmd.csv')

df_imu['Time'] = pd.to_datetime(df_imu['Time'], unit='s')
df_PWM['Time'] = pd.to_datetime(df_PWM['Time'], unit='s')
df_pose['Time'] = pd.to_datetime(df_pose['Time'], unit='s')
#df_bldc['Time'] = pd.to_datetime(df_bldc['Time'], unit='s')
#df_tf['Time'] = pd.to_datetime(df_tf['Time'], unit='s')
df_PPM['Time'] = pd.to_datetime(df_PPM['Time'], unit='s')
#df_SBUS['Time'] = pd.to_datetime(df_SBUS['Time'], unit='s')
df_vel['Time'] = pd.to_datetime(df_vel['Time'], unit='s')
#df_Yaw['Time'] = pd.to_datetime(df_Yaw['Time'], unit='s')

#yaw_value = [x for x in df_Yaw['data']]

imu_list1 = [x for x in df_imu['orientation.x']]# Roll
imu_list2 = [x for x in df_imu['orientation.y']]# Pitch
imu_list3 = [x for x in df_imu['orientation.z']]# Yaw
imu_list4 = [x for x in df_imu['orientation.w']]# w_x
imu_list5 = [x for x in df_imu['linear_acceleration.y']]# w_y
imu_list6 = [x for x in df_imu['linear_acceleration.z']]# w_z
imu_list7 = [x for x in df_imu['angular_velocity.x']]# w_z
imu_list8 = [x for x in df_imu['angular_velocity.y']]# w_z
imu_list9 = [x for x in df_imu['angular_velocity.z']]# w_z
numbers_imu = [list(imu_data) for imu_data in zip(imu_list1, imu_list2, imu_list3, imu_list4, imu_list7, imu_list8)]


pwm_list1 = [x for x in df_PWM['data_0']]
pwm_list2 = [x for x in df_PWM['data_1']]
pwm_list3 = [x for x in df_PWM['data_2']]
pwm_list4 = [x for x in df_PWM['data_3']]
numbers_pwm = [list(pwm_data) for pwm_data in zip(pwm_list1, pwm_list2, pwm_list3, pwm_list4)]


pose_list1 = [-x for x in df_pose['x']]
pose_list2 = [x for x in df_pose['y']]
pose_list3 = [x+80  for x in df_pose['z']]
pose_compare = [(x+20)/20 for x in df_pose['z']]
numbers_pose = [list(pose_data) for pose_data in zip(pose_list1, pose_list2)]#, pose_list3)]


vel_list1 = [-x+0.25 for x in df_vel['pose.pose.position.x']]
vel_list2 = [x for x in df_vel['pose.pose.position.y']]
vel_list3 = [x for x in df_vel['pose.pose.position.z']]
vel_list4 = [x for x in df_vel['pose.pose.orientation.x']]
vel_list5 = [x for x in df_vel['pose.pose.orientation.y']]
vel_list6 = [x for x in df_vel['pose.pose.orientation.z']]
vel_list7 = [x for x in df_vel['pose.pose.orientation.w']]
vel_list8 = [x for x in df_vel['twist.twist.linear.x']]
vel_list9 = [x for x in df_vel['twist.twist.linear.y']]
vel_list10 = [x for x in df_vel['twist.twist.linear.z']]
vel_list11 = [x for x in df_vel['twist.twist.angular.x']]
vel_list12 = [x for x in df_vel['twist.twist.angular.y']]
vel_list13 = [x for x in df_vel['twist.twist.angular.z']]
numbers_vel = [list(vel_data) for vel_data in zip(vel_list1, vel_list2)]#, vel_list3, vel_list4, vel_list5, vel_list6, vel_list7, vel_list8, vel_list9, vel_list10, vel_list11, vel_list12, vel_list13)]

'''
bldc_list1 = [x for x in df_bldc['data_0']]
bldc_list2 = [x for x in df_bldc['data_1']]
bldc_list3 = [x for x in df_bldc['data_2']]
bldc_list4 = [x for x in df_bldc['data_3']]
numbers_bldc = [list(bldc_data) for bldc_data in zip(bldc_list1, bldc_list2, bldc_list3, bldc_list4)]
'''

PPM_list1 = [x for x in df_PPM['data_0']]#
PPM_list2 = [x-1500 for x in df_PPM['data_1']]#yaw
PPM_list3 = [-0.2*(x-1500)/500 for x in df_PPM['data_2']]#pitch -0.3*(x-1500)/500
PPM_list4 = [(1500-x)/4+100 for x in df_PPM['data_3']]#thrust
PPM_thrust = [((1500-x)/4+100)/100 for x in df_PPM['data_3']]#thrust
PPM_list5 = [0.2*(x-1500)/500 for x in df_PPM['data_4']]#roll 0.3*(x-1500)/500
PPM_list6 = [x for x in df_PPM['data_5']]#emergency mode
PPM_list7 = [x for x in df_PPM['data_6']]#
numbers_PPM = [list(PPM_data) for PPM_data in zip(PPM_list3)]#, PPM_list2, PPM_list3, PPM_list4, PPM_list5, PPM_list6, PPM_list7)]

'''
SBUS_list1 = [x for x in df_SBUS['data_0']]#
SBUS_list2 = [x for x in df_SBUS['data_1']]#yaw
SBUS_list3 = [x for x in df_SBUS['data_2']]#pitch
SBUS_list4 = [x for x in df_SBUS['data_3']]#thrust
SBUS_list5 = [x for x in df_SBUS['data_4']]#roll
SBUS_list6 = [x for x in df_SBUS['data_5']]#emergency mode
SBUS_list7 = [x for x in df_SBUS['data_6']]#
numbers_SBUS = [list(SBUS_data) for SBUS_data in zip(SBUS_list4, SBUS_list5, SBUS_list3, SBUS_list2, SBUS_list6, SBUS_list6, SBUS_list7)]
'''

time_values_imu = df_imu['Time'].dt.strftime('%H:%M:%S.%f').str.split(':').apply(lambda x: int(x[0]) * 3600 + int(x[1]) * 60 + float(x[2]))
time_values_pwm = df_PWM['Time'].dt.strftime('%H:%M:%S.%f').str.split(':').apply(lambda x: int(x[0]) * 3600 + int(x[1]) * 60 + float(x[2]))
time_values_pose = df_pose['Time'].dt.strftime('%H:%M:%S.%f').str.split(':').apply(lambda x: int(x[0]) * 3600 + int(x[1]) * 60 + float(x[2]))
#time_values_bldc = df_bldc['Time'].dt.strftime('%H:%M:%S.%f').str.split(':').apply(lambda x: int(x[0]) * 3600 + int(x[1]) * 60 + float(x[2]))
time_values_PPM = df_PPM['Time'].dt.strftime('%H:%M:%S.%f').str.split(':').apply(lambda x: int(x[0]) * 3600 + int(x[1]) * 60 + float(x[2]))
#time_values_SBUS = df_SBUS['Time'].dt.strftime('%H:%M:%S.%f').str.split(':').apply(lambda x: int(x[0]) * 3600 + int(x[1]) * 60 + float(x[2]))
time_values_vel = df_vel['Time'].dt.strftime('%H:%M:%S.%f').str.split(':').apply(lambda x: int(x[0]) * 3600 + int(x[1]) * 60 + float(x[2]))
#time_values_Yaw = df_Yaw['Time'].dt.strftime('%H:%M:%S.%f').str.split(':').apply(lambda x: int(x[0]) * 3600 + int(x[1]) * 60 + float(x[2]))

selected_time_values_imu = time_values_imu[(time_values_imu >= x_min) & (time_values_imu <= x_max)]
selected_numbers_imu = [numbers_imu[i] for i in range(len(numbers_imu)) if x_min <= time_values_imu[i] <= x_max]
selected_time_values_imu = selected_time_values_imu - x_min


selected_time_values_vel = time_values_vel[(time_values_vel >= x_min) & (time_values_vel <= x_max)]
selected_numbers_vel_x = [vel_list1[i] for i in range(len(vel_list1)) if x_min <= time_values_vel[i] <= x_max]
selected_numbers_vel_y = [vel_list2[i] for i in range(len(vel_list2)) if x_min <= time_values_vel[i] <= x_max]
selected_time_values_vel = selected_time_values_vel - x_min

'''
selected_time_values_PPM = time_values_PPM[(time_values_PPM >= x_min) & (time_values_PPM <= x_max)]
selected_numbers_PPM = [numbers_PPM[i] for i in range(len(numbers_PPM)) if x_min <= time_values_PPM[i] <= x_max]
selected_time_values_PPM = selected_time_values_PPM - x_min
'''

selected_time_values_pwm = time_values_pwm[(time_values_pwm >= x_min) & (time_values_pwm <= x_max)]
selected_numbers_pwm = [numbers_pwm[i] for i in range(len(numbers_pwm)) if x_min <= time_values_pwm[i] <= x_max]
selected_time_values_pwm = selected_time_values_pwm - x_min

'''
selected_time_values_bldc = time_values_bldc[(time_values_bldc >= x_min) & (time_values_bldc <= x_max)]
selected_numbers_bldc = [numbers_bldc[i] for i in range(len(numbers_bldc)) if x_min <= time_values_bldc[i] <= x_max]
selected_time_values_bldc = selected_time_values_bldc - x_min
'''
'''
for i in range(len(selected_numbers_pwm)):#스케일링
    #center_value = max(selected_numbers_pwm[i]) / 2 + min(selected_numbers_pwm[i]) / 2
    #selected_numbers_pwm[i] = [x - center_value for x in selected_numbers_pwm[i]] 
    selected_numbers_pwm[i] = [(x-1500)/1000 for x in selected_numbers_pwm[i]]#/20
'''
'''
for i in range(len(selected_numbers_bldc)):#스케일링
    #center_value = max(selected_numbers_pwm[i]) / 2 + min(selected_numbers_pwm[i]) / 2
    #selected_numbers_pwm[i] = [x - center_value for x in selected_numbers_pwm[i]]
    selected_numbers_bldc[i] = [x/1000 for x in selected_numbers_bldc[i]]#/20 
'''


roll_data_PWM = [[-(pwm[0] + pwm[1] - pwm[2] - pwm[3])/5000] for pwm in selected_numbers_pwm]
pitch_data_PWM = [[(pwm[0] - pwm[1] - pwm[2] + pwm[3])/5000] for pwm in selected_numbers_pwm]
yaw_data_PWM = [[(pwm[0] - pwm[1] + pwm[2] - pwm[3])/5000] for pwm in selected_numbers_pwm]

#roll_data_bldc = [[bldc[0] + bldc[1] - bldc[2] - bldc[3]] for bldc in selected_numbers_bldc]
#pitch_data_bldc = [[bldc[0] - bldc[1] - bldc[2] + bldc[3]] for bldc in selected_numbers_bldc]
#t0 = [[2*(imu[3] * imu[0] + imu[1] * imu[2])] for imu in selected_numbers_imu]# +2.0 * (w * x + y * z)
#t1 = [[1-2*(imu[0] * imu[0] + imu[1] * imu[1])] for imu in selected_numbers_imu]#+1.0 - 2.0 * (x * x + y * y)

imu_roll = [[math.atan2(2*(imu[3] * imu[0] + imu[1] * imu[2]),1-2*(imu[0] * imu[0] + imu[1] * imu[1]))] for imu in selected_numbers_imu]
imu_pitch = [[math.asin(2 * (imu[3] * imu[1] - imu[2] * imu[0]))] for imu in selected_numbers_imu]
imu_yaw = [[math.atan2(2*(imu[3]*imu[1]+imu[0]*imu[2]),1-2*(imu[0]*imu[0]+imu[1]*imu[1]))] for imu in selected_numbers_imu]
'''
vel_roll = [[math.atan2(2*(imu[6] * imu[3] + imu[4] * imu[5]),1-2*(imu[3] * imu[3] + imu[4] * imu[4]))+0.06] for imu in numbers_vel]
vel_pitch = [[math.asin(2 * (imu[6] * imu[4] - imu[5] * imu[3]))-0.718] for imu in numbers_vel]
'''

dc_remove_x = imu_list7 - np.mean(imu_list7)
dc_remove_y = imu_list8 - np.mean(imu_list8)
dc_remove_z = imu_list9 - np.mean(imu_list9)

dc_remove_x = detrend(dc_remove_x)
dc_remove_y = detrend(dc_remove_y)
dc_remove_z = detrend(dc_remove_z)

fft_vel_x = np.fft.rfft(dc_remove_x)/len(dc_remove_x)
fft_vel_y = np.fft.rfft(dc_remove_y)/len(dc_remove_y)
fft_vel_z = np.fft.rfft(dc_remove_z)/len(dc_remove_z)

magnitude_vel_x = np.abs(fft_vel_x)
magnitude_vel_y = np.abs(fft_vel_y)
magnitude_vel_z = np.abs(fft_vel_z)

# Get corresponding frequencies
sampling_interval = (time_values_imu[1] - time_values_imu[0])  # assuming uniform sampling
freqs = np.fft.rfftfreq(len(time_values_imu), d=sampling_interval)
'''
common_time = np.intersect1d(selected_time_values_imu, time_values_pose)

# IMU 데이터에서 roll 값과 pose_cmd에서 x값을 가져와 차이 계산
imu_roll_interp = np.interp(common_time, selected_time_values_imu, [imu[0] for imu in selected_numbers_imu])  # imu_list1 (Roll)
pose_x_interp = np.interp(common_time, time_values_pose, pose_list1)

# 차이 계산
diff = imu_roll_interp - pose_x_interp
'''

"""fig,ax=plt.subplots(2,3, figsize = (15,7))

ax[0,0].plot(selected_time_values_imu, imu_roll, label='imu.R')
ax[0,0].plot(time_values_pose, pose_list1, label='PPM.R')

ax[0,1].plot(selected_time_values_imu, imu_pitch, label='imu.P')
ax[0,1].plot(time_values_pose, pose_list2, label='PPM.R')

ax[0,2].plot(time_values_pwm, pwm_list4, label='PPM.Y')
################################################################
ax[1,0].plot(selected_numbers_vel_x, selected_numbers_vel_y, label='Gyro.x_freq')
#ax[1,0].plot(time_values_vel, vel_list2, label='imu.ang_vel.y')
ax[1,1].plot(time_values_vel, vel_list2, label='imu.ang_vel.y')
#ax[1,1].plot(time_values_pose, pose_compare, label='Gyro.x_freq')

ax[1,2].plot(time_values_vel, vel_list3, label='imu.ang_vel.y')  
ax[1,2].plot(time_values_PPM, PPM_thrust, label='PPM.Y')
#ax[1,2].plot(time_values_vel, vel_list3, label='imu.P')

ax[0,0].set_ylabel('Rad') #y 라벨
ax[0,0].set_title("Roll") #그래프 이름

ax[0,1].set_ylabel('Rad') #y 라벨
ax[0,1].set_title("Pitch") #그래프 이름

ax[1,0].set_xlabel('time')
ax[1,0].set_ylabel("Rad/s") #y 라벨
ax[1,0].set_title("position.x") #그래프 이름

ax[1,1].set_xlabel('time') #x 라벨
ax[1,1].set_ylabel('Rad/s') #y 라벨
ax[1,1].set_title("position.y") #그래프 이름


#ax[0,2].set_ylabel('Rad/s') #y 라벨
ax[0,2].set_title("Thrust input") #그래프 이름

ax[1,2].set_xlabel('time') #x 라벨
#ax[1,2].set_ylabel('Rad/s') #y 라벨
ax[1,2].set_title("position.z") #그래프 이름
"""
plt.figure(1)

plt.plot(selected_numbers_vel_x, selected_numbers_vel_y, label='Position')

plt.xlabel('X(m)')
plt.ylabel('Y(m)')#('imu to Roll, Pitch')
plt.xlim([-1,1])
plt.ylim([-1,1])
plt.title('Position')#('PWM and Selected IMU Data Comparison (X axis range: {} - {})'.format(x_min, x_max))
plt.legend()

"""plt.figure(2)

plt.plot(time_values_imu, imu_list8, label='imu.ang_Vel.y')
plt.xlabel('Time (seconds)')
plt.ylabel('rad')
plt.title('Pitch position cmd')
plt.legend()

plt.figure(3)

plt.plot(freqs, magnitude_vel_x, label='gyro.x fft')

plt.xlabel('Time (seconds)')
plt.ylabel('rad/s')
plt.title('FFT_Roll')
plt.legend()"""

plt.show()
