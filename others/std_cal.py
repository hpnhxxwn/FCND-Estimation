import numpy as np

gps_x_val = np.loadtxt('GPS_X.txt',delimiter=',',dtype='Float64',skiprows=1)[:,1]

gps_x_std  = np.std(gps_x_val)

acc_x_val = np.loadtxt('ACC_X.txt',delimiter=',',dtype='Float64',skiprows=1)[:,1]

acc_x_std  = np.std(acc_x_val)

print("GPS X Standard Deviation (MeasuredStdDev_GPSPosXY):",gps_x_std)

print("ACC X Standard Deviation (MeasuredStdDev_AccelXY):",acc_x_std)
