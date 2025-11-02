import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

# Physical constants
RADIUS = 0.11  # Ball radius (m)
MASS = 0.43  # Ball mass (kg)
GRAV = -9.81  # Gravity (m/s^2)
GOAL_X = 10.0  # Goal line x-position (m)
GOAL_MOUTH_Y = 2.2  # Goal height (m)
FIELD_XLIM = (-1.0, 12.0)
FIELD_YLIM = (-0.5, 3.0)
DT = 1.0/60.0  # Time step (s)

# Force/friction parameters
AIR_DRAG = 0.12
GROUND_FRICTION = 0.8
RESTITUTION = 0.45  # Bounce coefficient
K_CONTACT = 3.0e4  # Contact stiffness
C_CONTACT = 120.0  # Contact damping
MU_SLIDE = 0.35  # Sliding friction
F_HELD = 20.0  # Force when key held (N)
KICK_IMPULSE = 2.2  # Kick impulse (N·s)
KICK_MIN_DIST_M = 1.0  # Minimum distance from goal to kick (m)

# Initial state
POS0 = np.array([1.0, RADIUS])
VEL0 = np.array([0.0, 0.0])

# Game state
pos = POS0.astype(float).copy()
vel = VEL0.astype(float).copy()
score = 0
held = {"up": False, "down": False, "left": False, "right": False}
held_wasd = {"w": False, "a": False, "s": False, "d": False}
aim_dir = np.array([1.0, 0.0])
running = True
last_goal_time = -1e9


def norm2(v):
    """Normalize vector and return (unit_vector, magnitude)"""
    m = np.hypot(v[0], v[1])
    if m < 1e-12:
        return np.array([0.0, 0.0]), 0.0
    return v/m, m


def distance_to_goal_line(x):
    """Calculate distance from ball to goal line"""
    return max(0.0, GOAL_X - x)


def contact_force_ball_on_ground(p, v):
    """Calculate contact force between ball and ground"""
    y_foot = p[1] - RADIUS
    depth = max(0.0, -y_foot)
    if depth <= 0.0:
        return np.array([0.0, 0.0])
    
    # Normal force
    v_n = v[1]
    Fn = K_CONTACT * depth - C_CONTACT * v_n
    if Fn < 0.0:
        Fn = 0.0
    
    # Tangential friction force
    v_t = v[0]
    Ft_visc = -GROUND_FRICTION * v_t * MASS
    Ft_max = MU_SLIDE * Fn
    Ft = np.clip(Ft_visc, -Ft_max, Ft_max)
    
    return np.array([Ft, Fn])


def reset():
    """Reset game state"""
    global pos, vel, score, last_goal_time
    pos = POS0.astype(float).copy()
    vel = VEL0.astype(float).copy()
    score = 0
    last_goal_time = -1e9


def step_physics(dt):
    """Update physics simulation"""
    global pos, vel, score, last_goal_time
    
    # User input forces
    F_user = np.zeros(2)
    if held["left"]:  F_user[0] -= F_HELD
    if held["right"]: F_user[0] += F_HELD
    if held["up"]:    F_user[1] += F_HELD
    if held["down"]:  F_user[1] -= F_HELD
    if held_wasd["a"]: F_user[0] -= F_HELD
    if held_wasd["d"]: F_user[0] += F_HELD
    if held_wasd["w"]: F_user[1] += F_HELD
    if held_wasd["s"]: F_user[1] -= F_HELD
    
    # Physics forces
    F_drag = -AIR_DRAG * MASS * vel
    F_grav = np.array([0.0, MASS * GRAV])
    F_contact = contact_force_ball_on_ground(pos, vel)
    
    # Net force and acceleration
    F_net = F_user + F_drag + F_grav + F_contact
    acc = F_net / MASS
    
    # Update velocity and position
    vel[:] = vel + acc * dt
    pos[:] = pos + vel * dt
    
    # Ground collision
    if pos[1] < RADIUS:
        pos[1] = RADIUS
        if vel[1] < 0.0:
            vel[1] = -vel[1] * RESTITUTION
    
    # Check for goal
    crossed = pos[0] - RADIUS > GOAL_X and pos[1] >= RADIUS and pos[1] <= GOAL_MOUTH_Y
    if crossed and (time.time() - last_goal_time) > 0.6:
        score += 1
        last_goal_time = time.time()


def _extract_key(keystr):
    """Extract key name from event"""
    if not isinstance(keystr, str):
        return keystr
    # Handle different key formats
    key = keystr.split("+")[-1].lower()
    # Map special keys
    key_map = {
        'up': 'up',
        'down': 'down', 
        'left': 'left',
        'right': 'right'
    }
    return key_map.get(key, key)


def _set_aim_from_key(k):
    """Update aim direction based on key pressed"""
    global aim_dir
    if k in ("up", "w"):
        aim_dir[:] = (0.0, 1.0)
    elif k in ("down", "s"):
        aim_dir[:] = (0.0, -1.0)
    elif k in ("left", "a"):
        aim_dir[:] = (-1.0, 0.0)
    elif k in ("right", "d"):
        aim_dir[:] = (1.0, 0.0)


def on_key_press(event):
    """Handle key press events"""
    global running, vel
    
    # Debug: print what key was pressed
    print(f"Key pressed: {event.key}")
    
    k = _extract_key(event.key)
    
    if k in ("q", "escape"):
        running = False
        plt.close(fig)
        return
    
    if k == "r":
        reset()
        return
    
    # Movement keys
    if k in held:
        held[k] = True
        _set_aim_from_key(k)
        print(f"Arrow key: {k}")
    if k in held_wasd:
        held_wasd[k] = True
        _set_aim_from_key(k)
        print(f"WASD key: {k}")
    
    # Kick
    if k in (" ", "space"):
        dist = distance_to_goal_line(pos[0])
        if dist >= KICK_MIN_DIST_M:
            u, _ = norm2(aim_dir)
            dv = (KICK_IMPULSE / MASS) * u
            vel[:] = vel + dv
            print(f"Kick! Velocity: {vel}")


def on_key_release(event):
    """Handle key release events"""
    k = _extract_key(event.key)
    if k in held:
        held[k] = False
    if k in held_wasd:
        held_wasd[k] = False


# Setup matplotlib figure
plt.rcParams["figure.figsize"] = (10, 5)
fig, ax = plt.subplots()
ax.set_aspect("equal")
ax.set_xlim(*FIELD_XLIM)
ax.set_ylim(*FIELD_YLIM)
ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.set_title("Interactive Soccer Simulator — WASD or Arrows to move, Space to kick")

# Draw field
ax.plot(FIELD_XLIM, [0, 0], color="k", lw=2, label="Ground")
ax.axvline(GOAL_X, ymin=0.0, ymax=0.73, color="red", ls="--", lw=2, label="Goal Line")
ax.plot([GOAL_X, GOAL_X], [0, GOAL_MOUTH_Y], color="red", ls="--", lw=2, alpha=0.4)

# Create ball and arrow
ball = plt.Circle(pos, RADIUS, fc="white", ec="black", lw=1.5)
ax.add_patch(ball)
aim_arrow = ax.arrow(pos[0], pos[1], 0.5, 0.0, width=0.02, color="navy")

# HUD text
hud = ax.text(0.02, 0.95, "", transform=ax.transAxes, va="top", 
              family="monospace", fontsize=9)

# Connect keyboard events
fig.canvas.mpl_connect("key_press_event", on_key_press)
fig.canvas.mpl_connect("key_release_event", on_key_release)


def update_frame(frame):
    """Animation update function"""
    global aim_arrow
    
    if running:
        step_physics(DT)
    
    # Update ball position
    ball.center = (pos[0], pos[1])
    
    # Remove old arrow and create new one
    if aim_arrow in ax.patches:
        aim_arrow.remove()
    
    u, _ = norm2(aim_dir)
    L = 0.8
    aim_arrow = ax.arrow(pos[0], pos[1], u[0]*L, u[1]*L, 
                         width=0.02, color="navy", head_width=0.08)
    
    # Update HUD
    dist = distance_to_goal_line(pos[0])
    kick_ok = dist >= KICK_MIN_DIST_M
    hud.set_text(
        f"Score: {score}\n"
        f"Position: x={pos[0]:.2f} m, y={pos[1]:.2f} m\n"
        f"Velocity: vx={vel[0]:.2f} m/s, vy={vel[1]:.2f} m/s\n"
        f"Distance to goal: {dist:.2f} m  |  Kick: {'YES' if kick_ok else 'NO (too close)'}\n"
        f"Controls: WASD/Arrows=move, Space=kick, R=reset, Q=quit"
    )
    
    return ball, aim_arrow, hud


# Create animation
ani = FuncAnimation(fig, update_frame, interval=int(DT*1000), blit=False)

plt.tight_layout()
plt.show()