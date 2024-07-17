import numpy as np
from pykalman import KalmanFilter

initial_state_mean = np.array(
    [0, 0, 0, 0], dtype=np.uint8)  # [x, x_dot, y, y_dot]

transition_matrix = np.array([[1, 1, 0, 0],
                              [0, 1, 0, 0],
                              [0, 0, 1, 1],
                              [0, 0, 0, 1]], dtype=np.uint8)  # F matrix

observation_matrix = np.array([[1, 0, 0, 0],
                               [0, 0, 1, 0]], dtype=np.uint8)  # H matrix

# Did not implement the Q (transition-covarience) R (sensor noise or sensor covarience) matrices, U has been assumed to be null matrix as of now.

kf1 = KalmanFilter(transition_matrices=transition_matrix,
                   observation_matrices=observation_matrix,
                   initial_state_mean=initial_state_mean,
                   n_dim_obs=2)


def predict(measurements):
    n_timesteps = measurements.shape[0]
    n_dim_state = transition_matrix.shape[0]
    filtered_state_means = np.zeros((n_timesteps, n_dim_state), dtype=np.uint8)
    filtered_state_covariances = np.zeros(
        (n_timesteps, n_dim_state, n_dim_state), dtype=np.uint8)



    for t in range(n_timesteps - 1):
        if t == 0:
            filtered_state_means[t] = initial_state_mean
            filtered_state_covariances[t] = np.eye(4)*500  # Intital state covarience. A large value of initial covarience is selected as initial state is purely guessed. The only known thing is that the object should be moving along x axis at 1 m/s,
        filtered_state_means[t + 1], filtered_state_covariances[t + 1] = (
            kf1.filter_update(
                filtered_state_means[t],
                filtered_state_covariances[t],
                measurements[t + 1],
            )
        )
    return (filtered_state_means[-1], filtered_state_covariances[-1])
