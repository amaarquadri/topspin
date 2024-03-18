import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    reference_npy = np.load("impedance_2.npy", allow_pickle=True)
    traj_npy = np.load("impedance_1.npy", allow_pickle=True)

    reference = [point.translation[2] for point in reference_npy]
    traj = [point.translation[2] for point in traj_npy]

    plt.plot(reference, label="reference")
    plt.plot(traj, label='traj')
    plt.legend()
    plt.show()