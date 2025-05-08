import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches

# System parameters
M = 0.2  # Mass of diabolo (kg)
m = 0.1  # Mass of counterweight (kg)
R = 0.065  # Radius of diabolo (m)
d = 0.03  # Distance from center to counterweight (m)
I_d = 505.45539e-6  # Moment of inertia of diabolo (kg·m²)
g = 9.81  # Gravitational acceleration (m/s²)

# Total moment of inertia (diabolo + wheel effect of mass)
I_total = I_d + M * R**2

class DiaboloSystem:
    def __init__(self):
        self.position = 0.0  # x position of diabolo center
        self.velocity = 0.0  # linear velocity of diabolo
        self.angle = 0.0     # counterweight angle (0 = bottom position)
        self.omega_d = 0.0   # angular velocity of diabolo
        
    def update_state(self, position, velocity, angle):
        self.position = position
        self.velocity = velocity
        self.angle = angle
        self.omega_d = velocity / R
        
    def get_state(self):
        return {
            'position': self.position,
            'velocity': self.velocity,
            'angle': self.angle,
            'omega_d': self.omega_d
        }
    
    def calculate_torque(self, theta):
        """Calculate torque about contact point due to counterweight at angle theta"""
        return m * g * d * np.sin(theta)
    
    def calculate_acceleration(self, theta):
        """Calculate linear acceleration of diabolo for a given counterweight angle"""
        torque = self.calculate_torque(theta)
        alpha = torque / I_total  # Angular acceleration
        return alpha * R  # Linear acceleration
        
    def simulate_motion(self, counterweight_trajectory, t_span, dt=0.01):
        """Simulate diabolo motion based on a given counterweight trajectory function"""
        t_eval = np.arange(t_span[0], t_span[1], dt)
        
        # Function to compute derivatives for solver
        def system_dynamics(t, y):
            position, velocity = y
            angle = counterweight_trajectory(t)
            acceleration = self.calculate_acceleration(angle)
            return [velocity, acceleration]
        
        # Initial conditions
        y0 = [self.position, self.velocity]
        
        # Solve the system
        sol = solve_ivp(
            system_dynamics, 
            t_span, 
            y0, 
            t_eval=t_eval, 
            method='RK45'
        )
        
        # Extract results
        t = sol.t
        positions = sol.y[0]
        velocities = sol.y[1]
        angles = np.array([counterweight_trajectory(t_i) for t_i in t])
        accelerations = np.array([self.calculate_acceleration(angle) for angle in angles])
        
        return {
            't': t,
            'position': positions,
            'velocity': velocities,
            'angle': angles,
            'acceleration': accelerations
        }

def optimal_counterweight_trajectory(t, cycle_time=1.0, duty_cycle=0.3):
    """
    Generate an optimal counterweight trajectory:
    - Quick lifting (from 0 to π)
    - Slow lowering (from π back to 0)
    
    Parameters:
    - t: Time
    - cycle_time: Time for one complete cycle
    - duty_cycle: Fraction of cycle spent on lifting (should be < 0.5 for optimization)
    
    Note: Angle is measured from the bottom position. 
    - 0 = counterweight at bottom
    - π = counterweight at top
    """
    phase = (t % cycle_time) / cycle_time
    
    if phase < duty_cycle:
        # Fast upward motion (0 to π)
        normalized_phase = phase / duty_cycle
        return np.pi * normalized_phase
    else:
        # Slower downward motion (π to 0)
        normalized_phase = (phase - duty_cycle) / (1 - duty_cycle)
        return np.pi * (1 - normalized_phase)

def custom_counterweight_trajectory(t, params):
    """
    Generate a custom counterweight trajectory based on parametric function
    
    Parameters:
    - t: Time
    - params: Dictionary of parameters that define the trajectory
    """
    cycle_time = params.get('cycle_time', 1.0)
    a = params.get('amplitude', np.pi)
    phase_shift = params.get('phase_shift', 0)
    
    # Basic sinusoidal pattern
    return a * (1 - np.cos(2 * np.pi * t / cycle_time + phase_shift)) / 2

def analyze_performance(results):
    """Analyze the performance of a trajectory"""
    max_velocity = np.max(results['velocity'])
    avg_velocity = np.mean(results['velocity'])
    final_position = results['position'][-1]
    
    print(f"Maximum velocity: {max_velocity:.4f} m/s")
    print(f"Average velocity: {avg_velocity:.4f} m/s")
    print(f"Final position: {final_position:.4f} m")
    
    # Calculate acceleration efficiency
    total_time = results['t'][-1]
    efficiency = final_position / total_time
    print(f"Position/time efficiency: {efficiency:.4f} m/s")

def plot_results(results, title="Diabolo Motion"):
    """Plot the results of the simulation"""
    t = results['t']
    
    fig, axs = plt.subplots(4, 1, figsize=(15, 16), sharex=True)
    
    # Position plot
    axs[0].plot(t, results['position'], 'b-', linewidth=2)
    axs[0].set_ylabel('Position (m)', fontsize=14)
    axs[0].set_title(title, fontsize=18)
    axs[0].grid(True)
    axs[0].tick_params(axis='both', which='major', labelsize=12)
    
    # Velocity plot
    axs[1].plot(t, results['velocity'], 'g-', linewidth=2)
    axs[1].set_ylabel('Velocity (m/s)', fontsize=14)
    axs[1].grid(True)
    axs[1].tick_params(axis='both', which='major', labelsize=12)
    
    # Acceleration plot
    axs[2].plot(t, results['acceleration'], 'r-', linewidth=2)
    axs[2].set_ylabel('Acceleration (m/s²)', fontsize=14)
    axs[2].grid(True)
    axs[2].tick_params(axis='both', which='major', labelsize=12)
    
    # Counterweight angle plot
    axs[3].plot(t, np.degrees(results['angle']), 'k-', linewidth=2)  # Convert to degrees
    axs[3].set_ylabel('Angle (degrees)', fontsize=14)
    axs[3].set_xlabel('Time (s)', fontsize=14)
    axs[3].grid(True)
    axs[3].tick_params(axis='both', which='major', labelsize=12)
    
    plt.tight_layout()
    plt.show()

def animate_diabolo(results, save_animation=False):
    """Create an animation of the diabolo motion"""
    t = results['t']
    positions = results['position']
    angles = results['angle']
    velocities = results['velocity']
    
    # Calculate diabolo rotation angle at each step
    # Integral of angular velocity (v/R)
    diabolo_angles = np.zeros_like(positions)
    for i in range(1, len(positions)):
        diabolo_angles[i] = diabolo_angles[i-1] + (velocities[i] / R) * (t[i] - t[i-1])
    
    # Create figure and axis - MUCH LARGER
    fig, ax = plt.subplots(figsize=(20, 12))
    max_pos = max(positions)
    min_pos = min(positions)
    buffer = 3*R
    ax.set_xlim(min_pos - buffer, max_pos + buffer)
    ax.set_ylim(-2*R, 3*R)
    ax.set_aspect('equal')
    ax.grid(True)
    
    # Create ground line
    ground = ax.axhline(y=0, color='k', linestyle='-', linewidth=2, alpha=0.6)
    
    # Create center marker and axis lines for diabolo
    center_marker = plt.Line2D([positions[0]], [R], marker='+', color='black', markersize=14)
    ax.add_line(center_marker)
    
    # Create diabolo object (circle with visual indicators for rotation)
    diabolo_circle = plt.Circle((positions[0], R), R, fill=False, color='blue', lw=4)
    
    # Add spokes to show rotation more clearly
    num_spokes = 8
    diabolo_spokes = []
    for i in range(num_spokes):
        angle = 2 * np.pi * i / num_spokes
        spoke = plt.Line2D(
            [positions[0], positions[0] + 0.9*R*np.sin(diabolo_angles[0] + angle)],
            [R, R + 0.9*R*np.cos(diabolo_angles[0] + angle)], 
            color='blue', lw=2, alpha=0.7
        )
        diabolo_spokes.append(spoke)
        ax.add_line(spoke)
    
    # Create counterweight object - much larger for visibility
    counterweight = plt.Circle(
        (positions[0] + d*np.sin(angles[0]), R + d*np.cos(angles[0])), 
        0.05*R, color='red', zorder=3
    )
    
    # Create shaft connecting center to counterweight
    shaft = plt.Line2D(
        [positions[0], positions[0] + d*np.sin(angles[0])],
        [R, R + d*np.cos(angles[0])], 
        color='black', lw=3, alpha=0.8
    )
    
    # Add objects to plot
    ax.add_patch(diabolo_circle)
    ax.add_patch(counterweight)
    ax.add_line(shaft)
    
    # Add time and velocity text - larger font size
    time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes, fontsize=16)
    vel_text = ax.text(0.02, 0.90, '', transform=ax.transAxes, fontsize=16)
    angle_text = ax.text(0.02, 0.85, '', transform=ax.transAxes, fontsize=16)
    
    # Add title and axis labels
    plt.title('Diabolo Motion with Internal Counterweight', fontsize=20)
    plt.xlabel('Position (m)', fontsize=16)
    plt.ylabel('Height (m)', fontsize=16)
    ax.tick_params(axis='both', which='major', labelsize=14)
    
    def init():
        diabolo_circle.center = (positions[0], R)
        center_marker.set_data([positions[0]], [R])
        counterweight.center = (positions[0] + d*np.sin(angles[0]), R + d*np.cos(angles[0]))
        shaft.set_data(
            [positions[0], positions[0] + d*np.sin(angles[0])],
            [R, R + d*np.cos(angles[0])]
        )
        
        # Update spokes
        for i, spoke in enumerate(diabolo_spokes):
            angle = 2 * np.pi * i / num_spokes
            spoke.set_data(
                [positions[0], positions[0] + 0.9*R*np.sin(diabolo_angles[0] + angle)],
                [R, R + 0.9*R*np.cos(diabolo_angles[0] + angle)]
            )
            
        time_text.set_text('')
        vel_text.set_text('')
        angle_text.set_text('')
        return [diabolo_circle, center_marker, counterweight, shaft, time_text, vel_text, angle_text] + diabolo_spokes
    
    def animate(i):
        diabolo_circle.center = (positions[i], R)
        center_marker.set_data([positions[i]], [R])
        counterweight.center = (positions[i] + d*np.sin(angles[i]), R + d*np.cos(angles[i]))
        shaft.set_data(
            [positions[i], positions[i] + d*np.sin(angles[i])],
            [R, R + d*np.cos(angles[i])]
        )
        
        # Update spokes
        for j, spoke in enumerate(diabolo_spokes):
            angle = 2 * np.pi * j / num_spokes
            spoke.set_data(
                [positions[i], positions[i] + 0.9*R*np.sin(diabolo_angles[i] + angle)],
                [R, R + 0.9*R*np.cos(diabolo_angles[i] + angle)]
            )
            
        time_text.set_text(f'Time: {t[i]:.2f}s')
        vel_text.set_text(f'Velocity: {velocities[i]:.3f} m/s')
        angle_text.set_text(f'CW Angle: {angles[i]*180/np.pi:.1f}°')
        return [diabolo_circle, center_marker, counterweight, shaft, time_text, vel_text, angle_text] + diabolo_spokes
    
    # Use a higher interval for smoother animation
    ani = FuncAnimation(fig, animate, frames=len(t),
                        init_func=init, blit=True, interval=30)
    
    if save_animation:
        ani.save('diabolo_animation.gif', writer='pillow', fps=30)
    
    plt.tight_layout()
    plt.show()
    
    ani = FuncAnimation(fig, animate, frames=len(t),
                        init_func=init, blit=True, interval=50)
    
    if save_animation:
        ani.save('diabolo_animation.gif', writer='pillow', fps=30)
    
    plt.show()

def compare_trajectories():
    """Compare different counterweight trajectories"""
    diabolo = DiaboloSystem()
    t_span = (0, 5)  # 5 seconds of simulation
    
    # Define different trajectories
    trajectories = {
        "Optimal (30% duty cycle)": lambda t: optimal_counterweight_trajectory(t, cycle_time=1.0, duty_cycle=0.3),
        "Optimal (20% duty cycle)": lambda t: optimal_counterweight_trajectory(t, cycle_time=1.0, duty_cycle=0.2),
        "Optimal (10% duty cycle)": lambda t: optimal_counterweight_trajectory(t, cycle_time=1.0, duty_cycle=0.1),
        "Sinusoidal": lambda t: custom_counterweight_trajectory(t, {'cycle_time': 1.0, 'amplitude': np.pi}),
    }
    
    results = {}
    for name, trajectory in trajectories.items():
        results[name] = diabolo.simulate_motion(trajectory, t_span)
    
    # Plot position comparison
    plt.figure(figsize=(16, 10))
    for name, result in results.items():
        plt.plot(result['t'], result['position'], label=name, linewidth=2)
    
    plt.xlabel('Time (s)', fontsize=14)
    plt.ylabel('Position (m)', fontsize=14)
    plt.title('Comparison of Different Counterweight Trajectories', fontsize=18)
    plt.legend(fontsize=14)
    plt.grid(True)
    plt.tick_params(axis='both', which='major', labelsize=12)
    plt.show()
    
    # Plot velocity comparison
    plt.figure(figsize=(16, 10))
    for name, result in results.items():
        plt.plot(result['t'], result['velocity'], label=name, linewidth=2)
    
    plt.xlabel('Time (s)', fontsize=14)
    plt.ylabel('Velocity (m/s)', fontsize=14)
    plt.title('Velocity Comparison of Different Counterweight Trajectories', fontsize=18)
    plt.legend(fontsize=14)
    plt.grid(True)
    plt.tick_params(axis='both', which='major', labelsize=12)
    plt.show()
    
    # Plot counterweight angle comparison - for one cycle only
    plt.figure(figsize=(16, 10))
    t_cycle = np.linspace(0, 1.0, 100)  # One cycle time
    for name, trajectory in trajectories.items():
        angles = np.array([trajectory(t_i) for t_i in t_cycle])
        plt.plot(t_cycle, np.degrees(angles), label=name, linewidth=2)  # Convert to degrees
    
    plt.xlabel('Cycle Time (s)', fontsize=14)
    plt.ylabel('Counterweight Angle (degrees)', fontsize=14)
    plt.title('Counterweight Movement Patterns (One Cycle)', fontsize=18)
    plt.legend(fontsize=14)
    plt.grid(True)
    plt.tick_params(axis='both', which='major', labelsize=12)
    plt.show()
    
    # Print performance metrics
    print("\nPerformance Comparison:")
    print("-----------------------")
    for name, result in results.items():
        print(f"\n{name}:")
        analyze_performance(result)
        
    # Create animation of the most effective trajectory
    best_trajectory = max(results.items(), key=lambda x: x[1]['position'][-1])
    print(f"\nThe most effective trajectory is: {best_trajectory[0]}")
    print("Generating animation for this trajectory...")
    animate_diabolo(best_trajectory[1])

# Example usage
if __name__ == "__main__":
    # Create diabolo system
    diabolo = DiaboloSystem()
    
    # Define simulation time span (0 to 5 seconds)
    t_span = (0, 5)
    
    print("\nDiabolo System Parameters:")
    print(f"- Diabolo mass: {M*1000:.0f}g")
    print(f"- Counterweight mass: {m*1000:.0f}g")
    print(f"- Diabolo radius: {R*1000:.1f}mm")
    print(f"- Distance to counterweight: {d*1000:.1f}mm")
    print(f"- Moment of inertia: {I_d*1e6:.5f}kg·mm²")
    
    # Define and test different counterweight trajectories
    print("\nTesting different counterweight control patterns...")
    
    # Fast rise, slow fall (optimal pattern)
    print("\n1. Testing optimal trajectory (fast rise, slow fall):")
    trajectory_optimal = lambda t: optimal_counterweight_trajectory(t, cycle_time=1.0, duty_cycle=0.3)
    results_optimal = diabolo.simulate_motion(trajectory_optimal, t_span)
    analyze_performance(results_optimal)
    
    # Create animation of optimal trajectory
    print("\nGenerating animation for optimal trajectory...")
    animate_diabolo(results_optimal)
    
    # Plot detailed results
    plot_results(results_optimal, "Diabolo Motion with Optimal Counterweight Trajectory")
    
    # Compare different trajectories
    print("\nComparing different control strategies...")
    compare_trajectories()