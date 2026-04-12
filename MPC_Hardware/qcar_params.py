import numpy as np

MAX_SPEED  = 0.10 #[m/s] not too fast
MIN_SPEED  = 0.08#[m/s] #0.08-0.10not too slow
MAX_STEER  = np.deg2rad(30.0) # 30 # maximum steering angle [rad] #0.436 #0.5236 not turn too much
MAX_DSTEER = np.deg2rad(5.0) # maximum steering rate[rad/s]#not turn too suddenly
MAX_ACCEL  = 0.05 # maximum acceleration [m/s^2] #not speed up too quickly
DT         = 0.1  #[s] #time step #controller is running every 0.2 sec #5Hz
WB         = 0.256 #[m] #wheel base distance between the front axle and the rear axle
RADIUS     = 0.8   #[m] circle radius
LENGTH     = 2  #[m] straight line length
TARGET_SPEED = 0.09
MAX_TIME   = 15.0 #change this during experiments
DS         = 0.02
