"""
Numerical Integration Methods for Physics Simulation

Available methods:
- Explicit Euler: Simple forward integration
- Semi-implicit (Symplectic) Euler: Better energy conservation
- Implicit (Backward) Euler: Unconditionally stable, good for stiff systems
"""

import numpy as np
import matplotlib.pyplot as plt


def explicit_euler_step(pos, vel, theta, omega, force, torque, mass, inertia, dt):
    """
    Explicit Euler integration: x(t+dt) = x(t) + v(t)*dt, v(t+dt) = v(t) + a(t)*dt

    Args:
        pos: Current position (array)
        vel: Current velocity (array)
        theta: Current angle (scalar)
        omega: Current angular velocity (scalar)
        force: Total force (array)
        torque: Total torque (scalar)
        mass: Mass (scalar)
        inertia: Moment of inertia (scalar)
        dt: Timestep (scalar)

    Returns:
        (pos_new, vel_new, theta_new, omega_new, acc, alpha): Updated state with accelerations
    """
    # Accelerations from current forces
    acc = force / mass
    alpha = torque / inertia

    # Update positions with current velocities
    pos_new = pos + vel * dt
    theta_new = theta + omega * dt

    # Update velocities with current accelerations
    vel_new = vel + acc * dt
    omega_new = omega + alpha * dt
    
    # raise NotImplementedError
    return pos_new, vel_new, theta_new, omega_new, acc, alpha


def semi_implicit_euler_step(pos, vel, theta, omega, force, torque, mass, inertia, dt):
    """
    Semi-implicit (Symplectic) Euler integration:
    v(t+dt) = v(t) + a(t)*dt, x(t+dt) = x(t) + v(t+dt)*dt

    Args:
        pos: Current position (array)
        vel: Current velocity (array)
        theta: Current angle (scalar)
        omega: Current angular velocity (scalar)
        force: Total force (array)
        torque: Total torque (scalar)
        mass: Mass (scalar)
        inertia: Moment of inertia (scalar)
        dt: Timestep (scalar)

    Returns:
        (pos_new, vel_new, theta_new, omega_new, acc, alpha): Updated state with accelerations
    """
    # Accelerations from current forces
    acc = force / mass
    alpha = torque / inertia

    # Update velocities first
    vel_new = vel + acc * dt
    omega_new = omega + alpha * dt

    # Update positions with new velocities
    pos_new = pos + vel_new * dt
    theta_new = theta + omega_new * dt

    # raise NotImplementedError
    return pos_new, vel_new, theta_new, omega_new, acc, alpha


def implicit_euler_step(pos, vel, theta, omega, force, torque, mass, inertia, dt,
                        compute_forces_func, max_iters=10, tol=1e-6):
    """
    Implicit (Backward) Euler integration:
    v(t+dt) = v(t) + a(t+dt)*dt, x(t+dt) = x(t) + v(t+dt)*dt

    Requires iterative solution since forces depend on next state.
    Uses fixed-point iteration: guess next state, compute forces, update guess.

    Args:
        pos: Current position (array)
        vel: Current velocity (array)
        theta: Current angle (scalar)
        omega: Current angular velocity (scalar)
        force: Initial force estimate (array)
        torque: Initial torque estimate (scalar)
        mass: Mass (scalar)
        inertia: Moment of inertia (scalar)
        dt: Timestep (scalar)
        compute_forces_func: Function that computes (force, torque) given (pos, vel, theta, omega)
        max_iters: Maximum number of iterations (default: 10)
        tol: Convergence tolerance (default: 1e-6)

    Returns:
        (pos_new, vel_new, theta_new, omega_new, acc, alpha): Updated state with accelerations

    Pros: Unconditionally stable, good for stiff systems
    Cons: Computationally expensive, requires iteration
    """
    # Initial guess: use semi-implicit Euler as starting point
    pos_guess = pos + vel * dt
    vel_guess = vel + (force / mass) * dt
    theta_guess = theta + omega * dt
    omega_guess = omega + (torque / inertia) * dt

    # TODO: use other methods to solve for the next accelerations

    # Using fixed-point iteration method to solve for a_{t+1} and \alpha_{t+1}
    for iteration in range(max_iters):
        # Compute forces at guessed next state
        force_next, torque_next = compute_forces_func(pos_guess, vel_guess, theta_guess, omega_guess)

        # Update using implicit formula
        acc_next = force_next / mass
        alpha_next = torque_next / inertia

        vel_new = vel + acc_next * dt
        omega_new = omega + alpha_next * dt

        pos_new = pos + vel_new * dt
        theta_new = theta + omega_new * dt

        # Check convergence
        pos_err = np.linalg.norm(pos_new - pos_guess)
        vel_err = np.linalg.norm(vel_new - vel_guess)
        theta_err = abs(theta_new - theta_guess)
        omega_err = abs(omega_new - omega_guess)

        max_err = max(pos_err, vel_err, theta_err, omega_err)

        if max_err < tol:
            break

        # Update guess for next iteration
        pos_guess = pos_new
        vel_guess = vel_new
        theta_guess = theta_new
        omega_guess = omega_new

    # Return the accelerations from the converged next state
    # raise NotImplementedError
    return pos_new, vel_new, theta_new, omega_new, acc_next, alpha_next


def integrate_step(method, pos, vel, theta, omega, force, torque, mass, inertia, dt,
                  compute_forces_func=None, max_iters=10, tol=1e-6):
    """
    Wrapper function to call the appropriate integration method.

    Args:
        method: Integration method - 'explicit', 'semi-implicit', or 'implicit'
        pos: Current position (array)
        vel: Current velocity (array)
        theta: Current angle (scalar)
        omega: Current angular velocity (scalar)
        force: Total force (array)
        torque: Total torque (scalar)
        mass: Mass (scalar)
        inertia: Moment of inertia (scalar)
        dt: Timestep (scalar)
        compute_forces_func: Function to compute forces (required for implicit)
        max_iters: Maximum iterations for implicit solver (default: 10)
        tol: Convergence tolerance for implicit solver (default: 1e-6)

    Returns:
        (pos_new, vel_new, theta_new, omega_new, acc, alpha): Next state with accelerations
    """
    if method == 'explicit':
        return explicit_euler_step(pos, vel, theta, omega, force, torque, mass, inertia, dt)

    elif method == 'semi-implicit':
        return semi_implicit_euler_step(pos, vel, theta, omega, force, torque, mass, inertia, dt)

    elif method == 'implicit':
        if compute_forces_func is None:
            raise ValueError("compute_forces_func required for implicit integration")
        return implicit_euler_step(pos, vel, theta, omega, force, torque, mass, inertia, dt,
                                  compute_forces_func, max_iters, tol)

    else:
        raise ValueError(f"Unknown integration method: {method}. Use 'explicit', 'semi-implicit', or 'implicit'")


def compare_integration_methods(
    methods_to_test,
    initial_state,
    num_steps,
    dt,
    mass,
    inertia,
    compute_forces_func,
    save_every=10,
    max_iters=10,
    tol=1e-6,
    verbose=True
):
    """
    Run simulations with different integration methods and compare results.

    Args:
        methods_to_test: List of method names, e.g., ['explicit', 'semi-implicit', 'implicit']
        initial_state: Dict with keys 'pos', 'vel', 'theta', 'omega'
        num_steps: Number of simulation steps
        dt: Timestep
        mass: Mass
        inertia: Moment of inertia
        compute_forces_func: Function(pos, vel, theta, omega) -> (force, torque)
        save_every: Save state every N steps (default: 10)
        max_iters: Max iterations for implicit (default: 10)
        tol: Tolerance for implicit (default: 1e-6)
        verbose: Print progress (default: True)

    Returns:
        Dict mapping method name to array of states
        Each state is (pos, theta, vel, omega, acc, alpha)
    """
    if verbose:
        print("Comparing integration methods...")
        print("="*60)

    integration_results = {}

    for method in methods_to_test:
        if verbose:
            print(f"\nRunning with {method} integration...")

        # Reset initial conditions
        pos = initial_state['pos'].copy()
        vel = initial_state['vel'].copy()
        theta = initial_state['theta']
        omega = initial_state['omega']

        states = []

        # Run simulation
        for step in range(num_steps):
            # Compute forces at current state
            force, torque = compute_forces_func(pos, vel, theta, omega)

            # Integration step (returns accelerations computed within the method)
            pos, vel, theta, omega, acc, alpha = integrate_step(
                method=method,
                pos=pos,
                vel=vel,
                theta=theta,
                omega=omega,
                force=force,
                torque=torque,
                mass=mass,
                inertia=inertia,
                dt=dt,
                compute_forces_func=compute_forces_func,
                max_iters=max_iters,
                tol=tol
            )

            # Save state with accelerations
            if step % save_every == 0:
                states.append((pos.copy(), theta, vel.copy(), omega, acc.copy(), alpha))

        integration_results[method] = np.array(states, dtype=object)

        if verbose:
            print(f"  Final position: x={pos[0]:.4f} m, height={pos[1]:.4f} m")
            print(f"  Final angular velocity: {omega:.3f} rad/s")

    if verbose:
        print("\n" + "="*60)
        print("Comparison complete!")

    return integration_results


def plot_integration_comparison(integration_results, dt, save_every, radius=None, mass=None, inertia=None, gravity=-9.81):
    """
    Create comparison plots for different integration methods.

    Args:
        integration_results: Dict from compare_integration_methods()
        dt: Timestep used in simulation
        save_every: How often states were saved
        radius: Optional radius to plot as reference line (default: None)
        mass: Mass of the object (required for energy calculation)
        inertia: Moment of inertia (required for energy calculation)
        gravity: Gravitational acceleration (default: -9.81 m/s^2)
    """
    fig, axes = plt.subplots(3, 3, figsize=(18, 14))

    colors = {'explicit': 'red', 'semi-implicit': 'blue', 'implicit': 'green'}
    linestyles = {'explicit': ':', 'semi-implicit': '-', 'implicit': '--'}

    methods = list(integration_results.keys())

    time_array = np.arange(len(integration_results[methods[0]])) * save_every * dt

    # ============================================
    # FIRST ROW: Position and Velocity
    # ============================================

    # Plot 1: X Position
    ax = axes[0, 0]
    for method in methods:
        positions = np.array([s[0] for s in integration_results[method]])
        ax.plot(time_array, positions[:, 0],
               color=colors.get(method, 'black'),
               linestyle=linestyles.get(method, '-'),
               linewidth=2, label=method, alpha=0.7)
    if radius is not None:
        ax.axhline(radius, color='gray', linestyle='--', linewidth=1, label='Reference')
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('X Position (m)', fontsize=11)
    ax.set_title('X Position vs Time', fontsize=12, fontweight='bold')
    ax.legend(fontsize=10)
    ax.grid(alpha=0.3)

    # Plot 2: Height
    ax = axes[0, 1]
    for method in methods:
        heights = np.array([s[0][1] for s in integration_results[method]])
        ax.plot(time_array, heights,
               color=colors.get(method, 'black'),
               linestyle=linestyles.get(method, '-'),
               linewidth=2, label=method, alpha=0.7)
    if radius is not None:
        ax.axhline(radius, color='gray', linestyle='--', linewidth=1, label='Reference')
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('Height (m)', fontsize=11)
    ax.set_title('Height vs Time', fontsize=12, fontweight='bold')
    ax.legend(fontsize=10)
    ax.grid(alpha=0.3)

    # Plot 3: Angular velocity
    ax = axes[0, 2]
    for method in methods:
        omegas = np.array([s[3] for s in integration_results[method]])
        ax.plot(time_array, omegas,
               color=colors.get(method, 'black'),
               linestyle=linestyles.get(method, '-'),
               linewidth=2, label=method, alpha=0.7)
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('Angular Velocity (rad/s)', fontsize=11)
    ax.set_title('Angular Velocity vs Time', fontsize=12, fontweight='bold')
    ax.legend(fontsize=10)
    ax.grid(alpha=0.3)

    # ============================================
    # SECOND ROW: Velocities and Energy
    # ============================================

    # Plot 4: X Velocity
    ax = axes[1, 0]
    for method in methods:
        velocities = np.array([s[2] for s in integration_results[method]])
        ax.plot(time_array, velocities[:, 0],
               color=colors.get(method, 'black'),
               linestyle=linestyles.get(method, '-'),
               linewidth=2, label=method, alpha=0.7)
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('Velocity X (m/s)', fontsize=11)
    ax.set_title('Horizontal Velocity', fontsize=12, fontweight='bold')
    ax.legend(fontsize=10)
    ax.grid(alpha=0.3)

    # Plot 5: Height/Z Velocity
    ax = axes[1, 1]
    for method in methods:
        velocities = np.array([s[2] for s in integration_results[method]])
        ax.plot(time_array, velocities[:, 1],
               color=colors.get(method, 'black'),
               linestyle=linestyles.get(method, '-'),
               linewidth=2, label=method, alpha=0.7)
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('Velocity Z/Height (m/s)', fontsize=11)
    ax.set_title('Vertical Velocity', fontsize=12, fontweight='bold')
    ax.legend(fontsize=10)
    ax.grid(alpha=0.3)

    # Plot 6: Total Energy
    ax = axes[1, 2]
    if mass is not None and inertia is not None:
        for method in methods:
            states = integration_results[method]
            positions = np.array([s[0] for s in states])
            velocities = np.array([s[2] for s in states])
            omegas = np.array([s[3] for s in states])

            # Kinetic energy: translational + rotational
            ke_translational = 0.5 * mass * np.sum(velocities**2, axis=1)
            ke_rotational = 0.5 * inertia * omegas**2
            kinetic_energy = ke_translational + ke_rotational

            # Potential energy: m * g * h (height is y-coordinate)
            potential_energy = mass * np.abs(gravity) * positions[:, 1]

            # Total energy
            total_energy = kinetic_energy + potential_energy

            ax.plot(time_array, total_energy,
                   color=colors.get(method, 'black'),
                   linestyle=linestyles.get(method, '-'),
                   linewidth=2, label=method, alpha=0.7)
        ax.set_xlabel('Time (s)', fontsize=11)
        ax.set_ylabel('Total Energy (J)', fontsize=11)
        ax.set_title('Total Energy (Kinetic + Potential)', fontsize=12, fontweight='bold')
        ax.legend(fontsize=10)
        ax.grid(alpha=0.3)
    else:
        ax.text(0.5, 0.5, 'Energy calculation requires\nmass and inertia parameters',
               ha='center', va='center', transform=ax.transAxes, fontsize=12)
        ax.set_xlabel('Time (s)', fontsize=11)
        ax.set_title('Total Energy (Kinetic + Potential)', fontsize=12, fontweight='bold')
        ax.grid(alpha=0.3)

    # ============================================
    # THIRD ROW: Accelerations
    # ============================================

    # Plot 7: X Acceleration
    ax = axes[2, 0]
    for method in methods:
        accelerations = np.array([s[4] for s in integration_results[method]])
        ax.plot(time_array, accelerations[:, 0],
               color=colors.get(method, 'black'),
               linestyle=linestyles.get(method, '-'),
               linewidth=2, label=method, alpha=0.7)
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('Acceleration X (m/s²)', fontsize=11)
    ax.set_title('Horizontal Acceleration', fontsize=12, fontweight='bold')
    ax.legend(fontsize=10)
    ax.grid(alpha=0.3)

    # Plot 8: Height/Z Acceleration
    ax = axes[2, 1]
    for method in methods:
        accelerations = np.array([s[4] for s in integration_results[method]])
        ax.plot(time_array, accelerations[:, 1],
               color=colors.get(method, 'black'),
               linestyle=linestyles.get(method, '-'),
               linewidth=2, label=method, alpha=0.7)
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('Acceleration Z/Height (m/s²)', fontsize=11)
    ax.set_title('Vertical Acceleration', fontsize=12, fontweight='bold')
    ax.legend(fontsize=10)
    ax.grid(alpha=0.3)

    # Plot 9: Angular acceleration
    ax = axes[2, 2]
    for method in methods:
        angular_accelerations = np.array([s[5] for s in integration_results[method]])
        ax.plot(time_array, angular_accelerations,
               color=colors.get(method, 'black'),
               linestyle=linestyles.get(method, '-'),
               linewidth=2, label=method, alpha=0.7)
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('Angular Acceleration (rad/s²)', fontsize=11)
    ax.set_title('Angular Acceleration', fontsize=12, fontweight='bold')
    ax.legend(fontsize=10)
    ax.grid(alpha=0.3)

    plt.tight_layout()
    return fig
