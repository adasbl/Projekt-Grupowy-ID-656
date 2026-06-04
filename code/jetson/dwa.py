class DWAConfig:
    # linear velocity limit
    max_linear_vel = 0.0541
    min_linear_vel = -0.0541

    # angular velocity limit
    max_angular_vel = 0.731
    min_angular_vel = -0.731

    # limits of velocities' acceleration
    max_linear_acc = 0.1
    max_angular_acc = 3.5

    # resolution for predicting velocities (sample of each)
    linear_resolution = 0.002
    angular_resolution = 0.05

    # using for predict velocity: v = a * dt
    dt = 0.1
    # lower -> worse prediction 
    predict_time = 4.0

    # weights to adjust (increase if specific paramater is more important)
    w_heading   = 1.0 # angular direction to target point
    w_clearance = 0.8 # bypassing obstacles
    w_velocity  = 0.6 # speed value

    # radius of wheel platform
    robo_radius = 0.17
    # lower -> less computations
    lidar_max_range = 3.0
    # keep robot from hitting obstacle
    inflation_radius = 0.20