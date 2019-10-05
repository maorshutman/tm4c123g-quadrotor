import matplotlib.pyplot as plt
import numpy as np


class State:
    def __init__(self, psi_des, phi_des, theta_des):
        # Euler angles
        self.psi = 0
        self.phi = 0
        self.theta = 0
        self.psi_des = psi_des
        self.phi_des = phi_des
        self.theta_des = theta_des

        # angular velocities in world frame
        self.psi_dot = 0
        self.phi_dot = 0
        self.theta_dot = 0
        self.psi_dot_des = 0
        self.phi_dot_des = 0
        self.theta_dot_des = 0

        # angular velocities in body frame
        self.w_z = 0.0
        self.w_x = 0.0
        self.w_y = 0.0


class Params:
    def __init__(self):
        self.m = 0.66  # kg
        self.k = (0.62 * 9.81) / ((2 * np.pi * 6360 / 60)**2)  # N * Hz^-2
        self.l = 0.25  # m
        self.g = 9.81  # m * s^-2
        self.b = 5.0e-6  # N * Hz^-2, taken form Mellinger paper
        # motor: 47g, rod: 43g, battery: 180g, frame center: 120g
        self.i_xx = 2 * 0.047 * 0.25**2 + 2 * 0.043 * 0.25**2 * 0.33 + \
            0.4 * 0.3 * 0.1**2  # kg * m^2
        self.i_yy = 2 * 0.047 * 0.25**2 + 2 * 0.043 * 0.25**2 * 0.33 + \
            0.4 * 0.3 * 0.1**2  # kg * m^2
        self.i_zz = 4 * 0.047 * 0.25**2 + 4 * 0.043 * 0.25**2 * 0.33 + \
            0.4 * 0.3 * 0.1**2  # kg * m^2
        self.deltat = 0.004  # sec
        self.kp = 4.0
        self.kd = 3.0
        self.max_motor_gamma = (2.0 * np.pi * 6360 / 60)**2  # max motor rpm
        # residual random bias after bias subtraction
        self.w_x_bias = np.random.normal(0.0, 0.001)
        self.w_y_bias = np.random.normal(0.0, 0.001)
        self.w_z_bias = np.random.normal(0.0, 0.001)


def error_to_input(params, state):
    total_thrust = params.m * params.g / \
        (4.0 * params.k * np.cos(state.phi) * np.cos(state.theta))

    # PD errors
    e_phi = -params.kp * (state.phi_des - state.phi) +\
        -params.kd * (state.phi_dot_des - state.phi_dot)
    e_theta = -params.kp * (state.theta_des - state.theta) +\
        -params.kd * (state.theta_dot_des - state.theta_dot)
    e_psi = -params.kp * (state.psi_des - state.psi) +\
        -params.kd * (state.psi_dot_des - state.psi_dot)

    # gamma_i = omega_i^2
    gamma_1 = total_thrust - \
        (2 * params.b * params.i_xx * e_phi +
         params.k * params.l * params.i_zz * e_psi) / \
        (4.0 * params.b * params.k * params.l)

    gamma_2 = total_thrust + \
        params.i_zz * e_psi / (4.0 * params.b) - \
        params.i_yy * e_theta / (2.0 * params.k * params.l)

    gamma_3 = total_thrust - \
        (-2 * params.b * params.i_xx * e_phi +
         params.k * params.l * params.i_zz * e_psi) / \
        (4.0 * params.b * params.k * params.l)

    gamma_4 = total_thrust + \
        params.i_zz * e_psi / (4.0 * params.b) + \
        params.i_yy * e_theta / (2.0 * params.k * params.l)

    # limit motor angular velocity
    if gamma_1 > params.max_motor_gamma:
        gamma_1 = params.max_motor_gamma
    if gamma_2 > params.max_motor_gamma:
        gamma_2 = params.max_motor_gamma
    if gamma_3 > params.max_motor_gamma:
        gamma_3 = params.max_motor_gamma
    if gamma_4 > params.max_motor_gamma:
        gamma_4 = params.max_motor_gamma

    return [gamma_1, gamma_2, gamma_3, gamma_4]


def update_angular_velocity(params, state, torque_phi, torque_psi, torque_theta):
    # transformation of angular velocity in body frame to world frame
    world_to_body =\
        np.array([[1.0, 0.0, -np.sin(state.theta)],
                  [0.0, np.cos(state.phi), np.cos(state.theta) * np.sin(state.phi)],
                  [0.0, -np.sin(state.phi), np.cos(state.theta) * np.cos(state.phi)]])
    body_to_world = np.linalg.inv(world_to_body)

    # update angular velocities in the body frame
    state.w_x = state.w_x + params.deltat * \
        (torque_phi / params.i_xx - (params.i_yy - params.i_zz) / params.i_xx * state.w_y * state.w_z)

    state.w_y = state.w_y + params.deltat * \
        (torque_theta / params.i_yy - (params.i_zz - params.i_xx) / params.i_yy * state.w_x * state.w_z)

    state.w_z = state.w_z + params.deltat * \
        (torque_psi / params.i_zz - (params.i_xx - params.i_yy) / params.i_zz * state.w_x * state.w_y)

    # measured angular velocities with Gaussian noise
    w_x_meas = state.w_x + np.random.normal(params.w_x_bias, 0.00167)
    w_y_meas = state.w_y + np.random.normal(params.w_x_bias, 0.00167)
    w_z_meas = state.w_z + np.random.normal(params.w_x_bias, 0.00167)

    # Update angular velocities in world frame. The angular velocity
    # in the world frame is computed with w_x, w_y, w_z corrupted by
    # Gaussian noise.
    state.phi_dot = \
        body_to_world[0, 0] * w_x_meas + \
        body_to_world[0, 1] * w_y_meas + \
        body_to_world[0, 2] * w_z_meas
    state.theta_dot = \
        body_to_world[1, 0] * w_x_meas + \
        body_to_world[1, 1] * w_y_meas + \
        body_to_world[1, 2] * w_z_meas
    state.psi_dot = \
        body_to_world[2, 0] * w_x_meas + \
        body_to_world[2, 1] * w_y_meas + \
        body_to_world[2, 2] * w_z_meas


def main():
    # desired orientation
    phi_des = 0.0 / (180.0 / np.pi)
    theta_des = 0.0 / (180.0 / np.pi)
    psi_des = 0.0 / (180.0 / np.pi)

    # initialize parameters and system state
    params = Params()
    state = State(psi_des, phi_des, theta_des)

    # simulation steps
    steps = 40000

    # initially there are no torques
    torque_phi = 0.0
    torque_theta = 0.0
    torque_psi = 0.0

    # store results
    res_phi = np.zeros(steps)
    res_theta = np.zeros(steps)
    res_psi = np.zeros(steps)
    z_force = np.zeros(steps)
    thrust_1 = np.zeros(steps)
    thrust_2 = np.zeros(steps)
    thrust_3 = np.zeros(steps)
    thrust_4 = np.zeros(steps)

    # run simulation
    for i in range(steps):
        # update angular velocities in world frame
        tmp_phi_dot = state.phi_dot
        tmp_theta_dot = state.theta_dot
        tmp_psi_dot = state.psi_dot
        update_angular_velocity(params, state, torque_phi, torque_psi, torque_theta)

        # update angles
        state.phi = state.phi + params.deltat * tmp_phi_dot
        state.theta = state.theta + params.deltat * tmp_theta_dot
        state.psi = state.psi + params.deltat * tmp_psi_dot

        # compute required motor angular velocities using updated angles and
        # angular velocities in the world frame
        [gamma_1, gamma_2, gamma_3, gamma_4] = error_to_input(params, state)

        # update torques
        torque_phi = params.l * params.k * (gamma_1 - gamma_3)
        torque_theta = params.l * params.k * (gamma_2 - gamma_4)
        torque_psi = params.b * (gamma_1 - gamma_2 + gamma_3 - gamma_4)

        # results
        res_phi[i] = state.phi
        res_theta[i] = state.theta
        res_psi[i] = state.psi
        z_force[i] = params.g * params.m / (np.cos(state.phi) * np.cos(state.theta))
        thrust_1[i] = params.k * gamma_1
        thrust_2[i] = params.k * gamma_2
        thrust_3[i] = params.k * gamma_3
        thrust_4[i] = params.k * gamma_4

    # plot angles vs time
    time = params.deltat * np.arange(steps)
    plt.plot(time, res_phi * 180.0 / np.pi, label='phi')
    plt.plot(time, res_theta * 180.0 / np.pi, label='theta')
    plt.plot(time, res_psi * 180.0 / np.pi, label='psi')
    plt.legend(frameon=False)
    plt.ylim((-200.0, 200.0))
    plt.show()

    # plot force in z direction
    plt.plot(time, z_force / params.m, label='f_z', color='k')
    plt.ylim((0.0, 15.0))
    plt.show()

    # plot thrusts
    plt.plot(time, thrust_1 / params.g, label='1')
    plt.plot(time, thrust_2 / params.g, label='2')
    plt.plot(time, thrust_3 / params.g, label='3')
    plt.plot(time, thrust_4 / params.g, label='4')
    plt.ylim((-0.2, 0.2))
    plt.legend(frameon=False)
    plt.show()


if __name__ == "__main__":
    main()
