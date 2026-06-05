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
    predict_time = 5.0

    # weights to adjust (increase if specific paramater is more important)
    w_heading   = 1.5 # (Zwiększone z 1.0) Mocniej nagradzaj patrzenie w stronę celu
    w_clearance = 0.5 # (Zmniejszone z 0.8) Mniej agresywnie reaguj na bycie blisko obiektów
    w_velocity  = 1.0 # (Zwiększone z 0.6) Znacznie bardziej nagradzaj jazdę do przodu

    # radius of wheel platform
    robo_radius = 0.17
    # lower -> less computations
    lidar_max_range = 3.0
    # keep robot from hitting obstacle
    inflation_radius = 0.12
