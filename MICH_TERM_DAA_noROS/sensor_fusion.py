import numpy as np
from image_distance_fusion import ultrasonic_readings, depthnet_readings
ultrasonic_measurements = np.array(ultrasonic_readings) 
depthnet_measurements = np.array(depthnet_readings)    

# Kalman Filter Parameters
initial_estimate = 0.0
initial_estimate_error = 1.0
measurement_error_ultrasonic = 0.5  
measurement_error_depthnet = 0.5   
process_noise = 0.1                 

def kalman_filter(measurements1, measurements2, measurement_error1, measurement_error2):
    estimate = initial_estimate
    estimate_error = initial_estimate_error

    fused_estimates = []

    for m1, m2 in zip(measurements1, measurements2):
        # Measurement update
        kalman_gain = estimate_error / (estimate_error + measurement_error1)
        estimate = estimate + kalman_gain * (m1 - estimate)
        estimate_error = (1 - kalman_gain) * estimate_error

        # Process update
        estimate = estimate
        estimate_error = estimate_error + process_noise

        # Repeat with second measurement
        kalman_gain = estimate_error / (estimate_error + measurement_error2)
        estimate = estimate + kalman_gain * (m2 - estimate)
        estimate_error = (1 - kalman_gain) * estimate_error

        fused_estimates.append(estimate)

    return fused_estimates

fused_distances = kalman_filter(ultrasonic_measurements, depthnet_measurements, measurement_error_ultrasonic, measurement_error_depthnet)

print(fused_distances)


