import math
import numpy as np
import matplotlib.pyplot as plt

g = 9.81  # m/s^2

def calculate_buoyancy(V: float, density_fluid: float) -> float:
    """
    Calculate the buoyant force on an object submerged in a fluid.

    Parameters:
    V (float): Volume of the object (m^3)
    density_fluid (float): Density of the fluid (kg/m^3)

    Returns:
    float: Buoyant force (N)
    """
    if V <= 0 or density_fluid <= 0:
        raise ValueError("Volume and density must be positive values.")
    return V * density_fluid * g


def will_it_float(V: float, mass: float) -> bool:
    """
    Determine if an object will float in water.

    Parameters:
    V (float): Volume of the object (m^3)
    mass (float): Mass of the object (kg)

    Returns:
    bool: True if the object will float, False otherwise
    """
    if V <= 0 or mass <= 0:
        raise ValueError("Volume and mass must be positive values.")
    return V * 1000 >= mass


def calculate_pressure(h: float) -> float:
    """
    Calculate the pressure at a given depth in water.

    Parameters:
    h (float): Depth in water (m)

    Returns:
    float: Pressure (Pa)
    """
    if h < 0:
        raise ValueError("Depth must be a non-negative value.")
    return g * h * 1000 + 101325  # Added atmospheric pressure at sea level (Pa)


def calculate_angular_acceleration(tau: float, I: float) -> float:
    """
    Calculate the angular acceleration of an object.

    Parameters:
    tau (float): Torque applied (Nm)
    I (float): Moment of inertia (kg·m^2)

    Returns:
    float: Angular acceleration (rad/s^2)
    """
    if I <= 0:
        raise ValueError("Moment of inertia must be a positive value.")
    return tau / I


def calculate_torque(F_magnitude: float, F_direction: float, r: float) -> float:
    """
    Calculate the torque exerted by a force.

    Parameters:
    F_magnitude (float): Magnitude of the force (N)
    F_direction (float): Direction of the force (degrees)
    r (float): Distance from the pivot point (m)

    Returns:
    float: Torque (Nm)
    """
    if F_magnitude <= 0 or r <= 0:
        raise ValueError("Force magnitude and distance must be positive values.")
    return F_magnitude * math.sin(F_direction) * r


def calculate_moment_of_inertia(m: float, r: float) -> float:
    """
    Calculate the moment of inertia of a point mass.

    Parameters:
    m (float): Mass (kg)
    r (float): Distance from the axis of rotation (m)

    Returns:
    float: Moment of inertia (kg·m^2)
    """
    if m <= 0 or r <= 0:
        raise ValueError("Mass and distance must be positive values.")
    return m * r * r


def calculate_auv_acceleration(
    F_magnitude: float, F_angle: float, mass: float = 100
) -> float:
    """
    Calculate the linear acceleration of an AUV.

    Parameters:
    F_magnitude (float): Magnitude of the applied force (N)
    F_angle (float): Angle of the applied force (degrees)
    mass (float): Mass of the AUV (kg), default is 100 kg

    Returns:
    float: Linear acceleration (m/s^2)
    """
    if F_magnitude <= 0 or mass <= 0:
        raise ValueError("Force magnitude and mass must be positive values.")
    return (math.sin(F_angle) * F_magnitude) / mass


def calculate_auv_angular_acceleration(
    F_magnitude: float, F_angle: float, thruster_distance: float, mass: float = 100
) -> float:
    """
    Calculate the angular acceleration of an AUV.

    Parameters:
    F_magnitude (float): Magnitude of the applied force (N)
    F_angle (float): Angle of the applied force (degrees)
    thruster_distance (float): Distance from the AUV's center of mass to the thruster (m)
    mass (float): Mass of the AUV (kg), default is 100 kg

    Returns:
    float: Angular acceleration (rad/s^2)
    """
    if F_magnitude <= 0 or thruster_distance <= 0 or mass <= 0:
        raise ValueError(
            "Force magnitude, thruster distance, and mass must be positive values."
        )
    return (math.sin(F_angle) * F_magnitude) / (mass * thruster_distance)


def calculate_auv2_acceleration(
    T: np.ndarray, alpha: float, theta: float, mass: float = 100
) -> float:
    """
    Calculate the linear acceleration of a multi-thruster AUV.

    Parameters:
    T (np.ndarray): Thrust magnitudes from four thrusters (N)
    alpha (float): Angle of the thrusters relative to the AUV's body axis (radians)
    theta (float): Orientation angle of the AUV (radians)
    mass (float): Mass of the AUV (kg), default is 100 kg

    Returns:
    float: Linear acceleration (m/s^2)
    """
    if (not isinstance(T, np.ndarray) or len(T) != 4 or mass <= 0):
        raise ValueError(
            "Thrust values must be positive numbers, and mass must be positive."
        )
    
    ax = (
        (np.sin(alpha + theta) * T[0])
        + (np.sin(theta - alpha) * T[1])
        + (np.sin(np.pi + theta + alpha) * T[2])
        + (np.sin(np.pi + theta - alpha) * T[3])
    ) / mass

    ay = (
        (np.cos(alpha + theta) * T[0])
        + (np.cos(theta - alpha) * T[1])
        + (np.cos(np.pi + theta + alpha) * T[2])
        + (np.cos(np.pi + theta - alpha) * T[3])
    ) / mass

    return np.sqrt(ax**2 + ay**2)


def calculate_auv2_angular_acceleration(
    T: np.ndarray, alpha: float, L: float, l: float, theta: float, inertia: float = 100
) -> float:
    """
    Calculate the angular acceleration of a AUV.

    Parameters:
    T (np.ndarray): Thrust magnitudes from four thrusters (N)
    alpha (float): Angle of the thrusters relative to the AUV's body axis (radians)
    L (float): Longitudinal distance from the center of mass to the thrusters (m)
    l (float): Lateral distance from the center of mass to the thrusters (m)
    theta (float): Orientation angle of the AUV (radians)
    inertia (float): Moment of inertia of the AUV (kg·m^2), default is 100 kg·m^2

    Returns:
    float: Angular acceleration (rad/s^2)
    """
    if (not isinstance(T, np.ndarray) or len(T) != 4 or L <= 0 or l <= 0 or inertia <= 0):
        raise ValueError(
            "Thrust values must be positive numbers, angles must be non-negative, and distances and inertia must be positive."
        )

    torque_longitudinal = L * (
        (np.sin(alpha + theta) * T[0])
        + (np.sin(theta - alpha) * T[1])
        + (np.sin(np.pi - theta - alpha) * T[2])
        + (np.sin(np.pi - theta + alpha) * T[3])
    )

    torque_lateral = l * (
        (np.cos(alpha + theta) * T[0])
        + (np.cos(theta - alpha) * T[1])
        + (np.cos(np.pi - theta - alpha) * T[2])
        + (np.cos(np.pi - theta + alpha) * T[3])
    )

    total_torque = torque_longitudinal + torque_lateral

    return total_torque / inertia

def simulate_auv2_motion(
    T: np.ndarray,
    alpha: float,
    L: float,
    l: float,
    mass: float = 100,
    inertia: float = 100,
    dt: float = 0.1,
    t_final: float = 10,
    x0: float = 0,
    y0: float = 0,
    theta0: float = 0,
):
    """
    Simulate the motion of a AUV in the 2D plane.

    Parameters:
    T (np.ndarray): Thrust magnitudes from four thrusters (N)
    alpha (float): Angle of the thrusters relative to the AUV's body axis (radians)
    L (float): Longitudinal distance from the center of mass of the AUV to the thrusters (m)
    l (float): Lateral distance from the center of mass of the AUV to the thrusters (m)
    mass (float): Mass of the AUV (kg), default is 100 kg
    inertia (float): Moment of inertia of the AUV (kg·m^2), default is 100 kg·m^2
    dt (float): Time step for the simulation (s), default is 0.1 s
    t_final (float): Total simulation time (s), default is 10 s
    x0 (float): Initial x position (m), default is 0 m
    y0 (float): Initial y position (m), default is 0 m
    theta0 (float): Initial orientation angle (radians), default is 0 radians

    Returns:
    np.ndarray: Time, x positions, y positions, orientation angles, velocities, angular velocities, accelerations
    """
    if (not isinstance(T, np.ndarray) or len(T) != 4 or L <= 0 or l <= 0 or mass <= 0 or inertia <= 0 or dt <= 0 or t_final <= 0):
        raise ValueError(
            "Thrust values must be non-negative numbers, angles must be non-negative, and distances, mass, inertia, and time values must be positive."
        )
              
    n_steps = int(t_final / dt)

    x = np.zeros(n_steps)
    y = np.zeros(n_steps)
    theta = np.zeros(n_steps)
    v = np.zeros(n_steps)
    omega = np.zeros(n_steps)
    a = np.zeros(n_steps)

    x[0], y[0], theta[0] = x0, y0, theta0

    for i in range(1, n_steps):
        # Calculate linear acceleration
        acc = calculate_auv2_acceleration(T, alpha, theta[i - 1], mass)
        # Calculate angular acceleration
        ang_acc = calculate_auv2_angular_acceleration(T, alpha, L, l, theta[i - 1], inertia)

        # Update linear velocity
        v[i] = v[i - 1] + acc * dt
        # Update angular velocity
        omega[i] = omega[i - 1] + ang_acc * dt

        # Update orientation angle
        theta[i] = theta[i - 1] + omega[i] * dt
        # Update x position
        x[i] = x[i - 1] + v[i] * np.cos(theta[i]) * dt
        # Update y position
        y[i] = y[i - 1] + v[i] * np.sin(theta[i]) * dt
        # Store acceleration
        a[i] = acc

    t = np.linspace(0, t_final, n_steps)

    return t, x, y, theta, v, omega, a

def plot_auv2_motion(
    t: np.ndarray,
    x: np.ndarray,
    y: np.ndarray,
    theta: np.ndarray,
    v: np.ndarray,
    omega: np.ndarray,
    a: np.ndarray,
):
    """
    Plot the motion of the AUV in the 2D plane

    Parameters:
    t (np.ndarray): Time steps of the simulation (s)
    x (np.ndarray): x-positions of the AUV (m)
    y (np.ndarray): y-positions of the AUV (m)
    theta (np.ndarray): Angles of the AUV (radians)
    v (np.ndarray): Velocities of the AUV (m/s)
    omega (np.ndarray): Angular velocities of the AUV (rad/s)
    a (np.ndarray): Accelerations of the AUV (m/s^2)
    """
    fig, axs = plt.subplots(3, 2, figsize=(12, 12))

    axs[0, 0].plot(t, x)
    axs[0, 0].set_title("X Position vs Time")
    axs[0, 0].set_xlabel("Time (s)")
    axs[0, 0].set_ylabel("X Position (m)")

    axs[0, 1].plot(t, y)
    axs[0, 1].set_title("Y Position vs Time")
    axs[0, 1].set_xlabel("Time (s)")
    axs[0, 1].set_ylabel("Y Position (m)")

    axs[1, 0].plot(t, theta)
    axs[1, 0].set_title("Angle vs Time")
    axs[1, 0].set_xlabel("Time (s)")
    axs[1, 0].set_ylabel("Angle (rad)")

    axs[1, 1].plot(t, v)
    axs[1, 1].set_title("Velocity vs Time")
    axs[1, 1].set_xlabel("Time (s)")
    axs[1, 1].set_ylabel("Velocity (m/s)")

    axs[2, 0].plot(t, omega)
    axs[2, 0].set_title("Angular Velocity vs Time")
    axs[2, 0].set_xlabel("Time (s)")
    axs[2, 0].set_ylabel("Angular Velocity (rad/s)")

    axs[2, 1].plot(t, a)
    axs[2, 1].set_title("Acceleration vs Time")
    axs[2, 1].set_xlabel("Time (s)")
    axs[2, 1].set_ylabel("Acceleration (m/s^2)")

    plt.tight_layout()
    plt.show()
