import numpy as np

MAX_SPEED  = 0.20 #[m/s] not too fast
MIN_SPEED  = 0.15 #[m/s] #0.08-0.10not too slow
MAX_STEER  = np.deg2rad(25.0) # 30 # maximum steering angle [rad] #0.436 #0.5236 not turn too much
MAX_DSTEER = np.deg2rad(45.0) # maximum steering rate[rad/s]#not turn too suddenly
MAX_ACCEL  = 0.20 # maximum acceleration [m/s^2] #not speed up too quickly
DT         = 0.1  #[s] #time step #controller is running every 0.1 sec #10Hz
WB         = 0.256 #[m] #wheel base distance between the front axle and the rear axle
RADIUS     = 1.0
TARGET_SPEED = 0.20
MAX_TIME   = 60.0 #change this during experiments
DS         = 0.05
