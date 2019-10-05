import matplotlib.pyplot as plt
import numpy as np


def eulers_to_dcm(dcm, theta, phi, psy):
    dcm[0, 0] = np.cos(psy) * np.cos(theta) - np.sin(phi) * np.sin(psy) * np.sin(theta)
    dcm[0, 1] = -np.cos(phi) * np.sin(psy)
    dcm[0, 2] = np.cos(psy) * np.sin(theta) + np.cos(theta) * np.sin(phi) * np.sin(psy)
    dcm[1, 0] = np.cos(theta) * np.sin(psy) + np.cos(psy) * np.sin(phi) * np.sin(theta)
    dcm[1, 1] = np.cos(phi) * np.cos(psy)
    dcm[1, 2] = np.sin(psy) * np.sin(theta) - np.cos(psy) * np.cos(theta) * np.sin(phi)
    dcm[2, 0] = -np.cos(phi) * np.sin(theta)
    dcm[2, 1] = np.sin(phi)
    dcm[2, 2] = np.cos(phi) * np.cos(theta)


def dcm_to_eulers(r):
    if r[2, 1] < 1.0:
        if r[2, 1] > -1.0:
            phi = np.arcsin(r[2, 1])
            psy = np.arctan2(-r[0, 1], r[1, 1])
            theta = np.arctan2(-r[2, 0], r[2, 2])
        else: # r21 = -1
            phi = -np.pi / 2.0
            psy = -np.arctan2(r[0, 2], r[0, 0])
            theta = 0.0
    else: # r21 = 1
        phi = np.pi / 2.0
        psy = np.arctan2(r[0, 2], r[0, 0])
        theta = 0.0

    return [psy, phi, theta]


def main():
    # read data
    f = open('gyro_data_mov_3.txt', 'r')
    w_x = np.array([])
    w_y = np.array([])
    w_z = np.array([])
    for line in f:
        split = line.split()
        w_x = np.append(w_x, float(split[0]))
        w_y = np.append(w_y, float(split[1]))
        w_z = np.append(w_z, float(split[2]))
    f.close()

    # radians
    w_x = w_x / (180.0 / np.pi)
    w_y = w_y / (180.0 / np.pi)
    w_z = w_z / (180.0 / np.pi)

    # bias
    n_bias = 500
    bias_x = w_x[0: n_bias].mean()
    bias_y = w_y[0: n_bias].mean()
    bias_z = w_z[0: n_bias].mean()

    # remove bias
    w_x = w_x - bias_x
    w_y = w_y - bias_y
    w_z = w_z - bias_z

    # plot signal
    plt.plot(w_x)
    plt.show()

    # plot the Gaussian noise distribution
    plt.hist(w_x, bins=100)
    plt.show()

    # smooth
    # n = 5
    # kernel = np.ones(n) / n
    # w_x = np.convolve(w_x, kernel, 'same')
    # w_y = np.convolve(w_y, kernel, 'same')
    # w_z = np.convolve(w_z, kernel, 'same')

    # spectrum
    # noise_fft = np.fft.fft(w_x)
    # plt.plot(abs(noise_fft) / max(abs(noise_fft)))
    # plt.show()

    # initial dcm
    dcm = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
    phi = 0.0
    psy = 0.0
    theta = 0.0
    eulers_to_dcm(dcm, theta, phi, psy)

    # integrate
    result_length = w_x.shape[0] - n_bias
    psy_result = np.zeros(result_length)
    phi_result = np.zeros(result_length)
    theta_result = np.zeros(result_length)
    deltat = 0.00375 # sec
    for i in range(n_bias, w_x.shape[0]):
        b = deltat * np.array([[0.0, -w_z[i], w_y[i]],
                               [w_z[i], 0.0, -w_x[i]],
                               [-w_y[i], w_x[i], 0.0]])
        sigma = deltat * np.sqrt(w_x[i]**2 + w_y[i]**2 + w_z[i]**2)
        increment = np.eye(3) + \
                    np.sinc(sigma) * b + \
                    ((1.0 - np.cos(sigma)) / sigma**2) * b.dot(b)
        dcm = dcm.dot(increment)
        eulers = dcm_to_eulers(dcm)
        psy_result[i - n_bias] = eulers[0] * 180.0 / np.pi
        phi_result[i - n_bias] = eulers[1] * 180.0 / np.pi
        theta_result[i - n_bias] = eulers[2] * 180.0 / np.pi

    # plot result vs time
    plt.plot(deltat * np.arange(result_length), psy_result, label='psy')
    plt.plot(deltat * np.arange(result_length), phi_result, label='phi')
    plt.plot(deltat * np.arange(result_length), theta_result, label='theta')
    plt.legend(frameon=False)
    plt.ylim((-200.0, 200.0))
    plt.show()


if __name__ == "__main__":
    main()
