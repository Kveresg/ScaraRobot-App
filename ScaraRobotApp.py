import tkinter as tk
import math
import serial
import serial.tools.list_ports # New import for finding ports
import threading
import time
import queue
import os
import contextlib

# Suppress pygame startup message/output
with open(os.devnull, "w") as f, contextlib.redirect_stdout(f):
    import pygame

# --- CONFIGURATION ---
BAUDRATE = 115200
MOTION_SEND_INTERVAL = 0.05 # Throttle: Send motion command every 50ms (20 FPS target)
JOYSTICK_POLL_INTERVAL = 50 # Milliseconds between joystick checks (20 Hz)
JOYSTICK_DEADZONE = 0.15 # Minimum axis value (0.0 to 1.0) to register movement

# --- GLOBAL STATE ---
ser = None
command_queue = queue.Queue()
angle_text_ids = {}
last_send_time = time.time()
stop_thread_flag = threading.Event()
elbow_up_config = True # True means right bend (default)
serial_status_label = None # To display connection status

# Joystick State
joystick = None
axis_states = {0: 0.0, 1: 0.0} # 4: 0.0 is for windows and 1:0.0 is for linux for some reason

joystick_active_movement = False # Flag to stop joystick movement overriding mouse
current_target_x, current_target_y = 0, 0 # Target position in Canvas coordinates
joystick_move_step_value = 2 # NEW: Global state for joystick speed (pixels per step)

z_axis_movement = {
    'up': False,    # Right Bumper (Button 5)
    'down': False   # Left Bumper (Button 4)
}

# IK Arm Lengths (must match the original arm configuration)
L1 = 91.6
L2 = 121.6

CANVAS_WIDTH, CANVAS_HEIGHT = 600, 600
radius = L1 + L2
center_x, center_y = CANVAS_WIDTH // 2, CANVAS_HEIGHT // 2 + 100 # Base pivot point

# Physical Inner Stop Radius (~75.0)
MIN_REACH = abs(L1 + 105 - L2)

MIN_GEOMETRIC_REACH = abs(L1 - L2)
L1_MAX_ANGLE_DEG = 249 * (90 / 200)
L2_MIN_REACH_ANGLE_DEG = 649 * (90 / 200)

# --- SERIAL AUTOCONNECTION LOGIC ---

def update_serial_status(status_text, color):
    """Updates the dedicated Tkinter label for serial connection status."""
    if serial_status_label:
        serial_status_label.config(text=f"Serial Status: {status_text}", fg=color)

def find_serial_port():
    """Scans for ports and prioritizes common Linux microcontroller devices."""
    ports = serial.tools.list_ports.comports()

    ports = [port for port in ports if port.device != '/dev/ttyAMA0'] # Ignore Raspberry Pi default serial port
    
    # Priority check for common Linux Arduino/ESP device names
    for port in ports:
        if port.device.startswith('/dev/ttyACM') or port.device.startswith('/dev/ttyUSB'):
            # You can add more specific checks here (e.g., matching VID/PID)
            print(f"Found potential device at: {port.device}")
            return port.device
            
    # Fallback to any available port if specific names aren't found
    if ports:
        print(f"Found fallback device at: {ports[0].device}")
        return ports[0].device
        
    return None

def serial_manager_thread():
    """Manages the connection state, continuously searching and reconnecting."""
    global ser, stop_thread_flag
    
    while not stop_thread_flag.is_set():
        if ser is None or not ser.is_open:
            update_serial_status("Searching...", "orange")
            print("Searching for serial port...")
            
            port_path = find_serial_port()

            if port_path:
                print(f"Port found: {port_path}. Attempting connection...")
                try:
                    # Attempt connection
                    ser = serial.Serial(port_path, BAUDRATE, timeout=1)
                    update_serial_status(f"Connected to {port_path}", "green")
                    print(f"Successfully connected to {port_path}")
                    # Give the device a moment to reboot/initialize after connecting
                    time.sleep(1.5) 
                except serial.SerialException as e:
                    update_serial_status("Connection Failed", "red")
                    print(f"Failed to connect to {port_path}: {e}")
                    ser = None # Ensure global state is clear
                    time.sleep(2) # Wait before next retry

            else:
                update_serial_status("Not Found", "red")
                # No port found, keep waiting and searching
                time.sleep(5)
        else:
            # Successfully connected, just wait and let reader/writer handle traffic
            time.sleep(1)

def serial_reader_thread():
    """Continuously reads incoming serial messages and handles disconnection."""
    global ser
    while not stop_thread_flag.is_set():
        try:
            # Only attempt to read if the serial port is open
            if ser and ser.is_open:
                if ser.in_waiting:
                    line = ser.readline().decode(errors='ignore').strip()
                    if line:
                        print(f"ESP32: {line}")
            else:
                 time.sleep(0.01) # Short wait if port is not ready
        
        except serial.SerialException as e:
            # Disconnection detected (cable pull, device crash)
            print(f"Serial reader detected disconnection: {e}. Notifying manager...")
            update_serial_status("DISCONNECTED", "red")
            if ser:
                ser.close()
            ser = None # Signal manager thread to begin scan/reconnect
            time.sleep(0.5) # Prevent high CPU usage during disconnect state
        
        except Exception as e:
            print(f"Error in serial reader: {e}")
            time.sleep(0.01)

def serial_writer_thread():
    """Dedicated thread to safely send commands and handles disconnection."""
    global ser
    while not stop_thread_flag.is_set():
        try:
            command = command_queue.get(timeout=0.1)
            
            if ser and ser.is_open:
                # Use a short timeout here to avoid blocking indefinitely if a disconnect occurs right before writing
                ser.write((command + "\n").encode())
                print(f"Sent: {command}")
            else:
                if command != "heartbeat":
                    print(f"SIMULATED SEND: {command} (No port connected)")
            
            command_queue.task_done()
        
        except queue.Empty:
            continue
            
        except serial.SerialException as e:
            # Disconnection detected during writing
            print(f"Serial writer detected disconnection: {e}. Notifying manager...")
            update_serial_status("DISCONNECTED", "red")
            if ser:
                ser.close()
            ser = None # Signal manager thread to begin scan/reconnect
            time.sleep(0.5)
            
        except Exception as e:
            print(f"Error in serial thread: {e}")

def start_serial_thread():
    """Initializes and starts all three serial communication threads."""
    manager = threading.Thread(target=serial_manager_thread, daemon=True)
    worker = threading.Thread(target=serial_writer_thread, daemon=True)
    reader = threading.Thread(target=serial_reader_thread, daemon=True)
    
    manager.start()
    worker.start()
    reader.start()

def on_closing():
    """Handles clean shutdown of the application, threads, and serial port."""
    stop_thread_flag.set()
    root.destroy()
    if ser and ser.is_open:
        ser.close()
    pygame.quit() # Cleanup pygame

# --- PYGAME JOYSTICK SETUP ---
def init_joystick():
    """Initializes pygame and connects to the first available joystick."""
    global joystick
    try:
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            print("No joystick detected.")
            return

        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"Connected to: {joystick.get_name()}")

    except pygame.error as e:
        print(f"Pygame/Joystick initialization error: {e}")
    except Exception as e:
        print(f"General error during joystick setup: {e}")

def check_joystick():
    """
    Polls joystick events and updates the robot arm state, including hotplug detection.
    This function is called repeatedly by root.after().
    """
    global axis_states, joystick_active_movement, current_target_x, current_target_y, last_send_time, z_axis_movement, joystick_move_step_value, joystick
    pygame.event.pump()

    if joystick is None and pygame.joystick.get_count() > 0:
        print("Controller detected! Attempting initialization...")
        init_joystick()
    
    if joystick is None:
        root.after(JOYSTICK_POLL_INTERVAL, check_joystick)
        return

    for event in pygame.event.get():
        if event.type == pygame.JOYDEVICEREMOVED:
            print(f"Controller removed. Resetting joystick state.")
            joystick = None
            break # Exit event loop; the next loop iteration will hit the 'if joystick is None' block
            
        # --- AXIS/D-pad MOVEMENT ---
        if event.type == pygame.JOYAXISMOTION:
            if event.axis in axis_states:
                axis_states[event.axis] = event.value

        # --- BUTTON DOWN ACTIONS ---
        elif event.type == pygame.JOYBUTTONDOWN:
            if event.button == 0: # X button
                send_serial_command("close_gripper")
                print("Button X (Close Gripper) pressed")
            elif event.button == 1: # A button -> NOW ELBOW TOGGLE
                toggle_elbow_direction()
                print("Button A (Toggle Elbow) pressed")
            elif event.button == 3: # Y button -> NOW OPEN GRIPPER
                send_serial_command("open_gripper")
                print("Button Y (Open Gripper) pressed")
            elif event.button == 9: # Start button
                home_command()
                print("Start button (Home) pressed")
            elif event.button == 8: # Back button
                calibration_command()
                print("Back button (Calibrate) pressed")
            
            elif event.button == 4: # Left Bumper -> Z- AXIS DOWN
                z_axis_movement['down'] = True
                print("Left Bumper (Z- continuous down) activated")
            elif event.button == 5: # Right Bumper -> Z+ AXIS UP
                z_axis_movement['up'] = True
                print("Right Bumper (Z+ continuous up) activated")

        # --- BUTTON UP ACTIONS ---
        elif event.type == pygame.JOYBUTTONUP:
            if event.button == 4: # Left Bumper
                z_axis_movement['down'] = False
            elif event.button == 5: # Right Bumper
                z_axis_movement['up'] = False
            pass
    
    if joystick is None:
        root.after(JOYSTICK_POLL_INTERVAL, check_joystick)
        return

    x_axis_val = axis_states.get(0, 0.0)
    y_axis_val = axis_states.get(1, 0.0) # On WINDOWS it is get(4, 0.0) and get(1, 0.0) is for LINUX for some reason

    is_moving = abs(x_axis_val) > JOYSTICK_DEADZONE or abs(y_axis_val) > JOYSTICK_DEADZONE
    
    if is_moving:
        joystick_active_movement = True
        dx = int(x_axis_val * joystick_move_step_value)
        dy = int(y_axis_val * joystick_move_step_value) 
        
        proposed_target_x = current_target_x + dx
        proposed_target_y = current_target_y + dy

        proposed_target_x = max(0, min(CANVAS_WIDTH, proposed_target_x))
        proposed_target_y = max(0, min(CANVAS_HEIGHT, proposed_target_y))
        
        if proposed_target_x != current_target_x or proposed_target_y != current_target_y:
            move_successful = try_move_ik(proposed_target_x, proposed_target_y)
            if move_successful:
                current_target_x = proposed_target_x
                current_target_y = proposed_target_y
                
    else:
        joystick_active_movement = False # Release joystick control
    
    is_z_moving = z_axis_movement['up'] or z_axis_movement['down']
    
    current_time = time.time()
    if is_z_moving and current_time - last_send_time >= MOTION_SEND_INTERVAL:
        if z_axis_movement['up']: # Right Bumper
            send_serial_command("relmove 0 0 5") # Z-UP 
            last_send_time = current_time
        elif z_axis_movement['down']: # Left Bumper
            send_serial_command("relmove 0 0 -5") # Z-DOWN 
            last_send_time = current_time

    root.after(JOYSTICK_POLL_INTERVAL, check_joystick)

# --- UTILITY FUNCTIONS ---
def inverse_kinematics(tx, ty, L1, L2):
    global elbow_up_config

    d = math.sqrt(tx**2 + ty**2)
    if d > L1 + L2 or d < abs(L1 - L2):
        return None, None

    cos_theta2 = (tx**2 + ty**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = max(-1.0, min(1.0, cos_theta2))

    if elbow_up_config:
        theta2 = math.acos(cos_theta2) # Positive sign (Elbow Up/Right Bend)
    else:
        theta2 = -math.acos(cos_theta2) # Negative sign (Elbow Down/Left Bend)

    k1 = L1 + L2 * math.cos(theta2)
    k2 = L2 * math.sin(theta2)
    theta1 = math.atan2(ty, tx) - math.atan2(k2, k1)

    return theta1, theta2

# Angle mapping functions (as in the original arm controller)
def get_L1_angle_deg(theta1):
    """Calculates L1 angle in degrees relative to the vertical (90 deg is right)."""
    angle_rad = math.pi / 2 - theta1
    angle_deg = math.degrees(angle_rad)
    angle_deg = (angle_deg + 180) % 360 - 180
    return angle_deg

def get_L2_relative_angle_deg(theta1, theta2):
    """Calculates L2 angle relative to L1 in degrees."""
    abs_angle_L2 = theta1 + theta2
    angle_L1 = math.pi / 2 - theta1
    angle_L2 = math.pi / 2 - abs_angle_L2
    relative_angle = math.degrees(angle_L2 - angle_L1)
    relative_angle = (relative_angle + 180) % 360 - 180
    return relative_angle

def map_angle_L1(angle_deg):
    """Maps L1 angle to stepper motor steps."""
    return (200 / 90) * angle_deg

def map_angle_L2(angle_deg):
    """Maps L2 angle to stepper motor steps (relative)."""
    return (400 / 90) * angle_deg

def send_serial_command(command):
    """Adds a command to the thread-safe queue."""
    command_queue.put(command)

# --- CORE IK & SEND LOGIC (Refactored) ---
def inverse_map_angle_L1(mapped_steps):
    """Inverse of map_angle_L1: converts motor steps back to degrees."""
    return (90 / 200) * mapped_steps

def get_theta1_rad(angle_deg):
    angle_rad = math.radians(angle_deg)
    theta1 = math.pi / 2 - angle_rad
    return theta1

import math
import time

def try_move_ik(target_x, target_y):
    global last_send_time

    dx = target_x - center_x
    dy = -(target_y - center_y)  # Inverted Y-axis
    distance = math.sqrt(dx**2 + dy**2)

    MAX_REACH = L1 + L2
    TOLERANCE = 30  # Max extension tolerance (pixels)

    canvas.itemconfig("cursor", fill="yellow")

    # --- Check Inner Minimum Reach ---
    if distance < MIN_REACH:
        canvas.itemconfig("cursor", fill="red")
        return False

    # --- Elbow positions ---
    angle_rad = math.radians(90 - L1_MAX_ANGLE_DEG)
    elbow_y = center_y - L1 * math.sin(angle_rad)
    elbow_x_r = center_x + L1 - 7
    elbow_x_l = center_x - L1 + 7
    L1_distance_right = math.sqrt((elbow_x_r - target_x)**2 + (elbow_y - target_y)**2)
    L1_distance_left = math.sqrt((elbow_x_l - target_x)**2 + (elbow_y - target_y)**2)

    # --- Initial theta1 and mapped_L1 ---
    theta1 = math.atan2(dy, dx)
    angle_L1 = get_L1_angle_deg(theta1)
    mapped_L1 = map_angle_L1(angle_L1)

    theta2 = None

    def compute_elbow_angles(L1_distance):
        nonlocal theta1, theta2, mapped_L1
        canvas.itemconfig("cursor", fill="green")

        mapped_L1 = max(-249, min(249, mapped_L1))  # Clamp to bounds
        clamped_angle_deg = inverse_map_angle_L1(mapped_L1)
        theta1 = get_theta1_rad(clamped_angle_deg)

        # New elbow position
        x1 = center_x + L1 * math.cos(theta1)
        y1 = center_y - L1 * math.sin(theta1)
        dx2 = target_x - x1
        dy2 = -(target_y - y1)
        theta2 = math.atan2(dy2, dx2) - theta1

    # --- Out-of-bounds angle corrections ---
    if mapped_L1 > 249:
        if not elbow_up_config:
            if L1_distance_right > L2 + TOLERANCE:
                canvas.itemconfig("cursor", fill="red")
                return False
            if L1_distance_right > L2:
                compute_elbow_angles(L1_distance_right)
            else:
                theta1, theta2 = inverse_kinematics(dx, dy, L1, L2)
        else:
            theta1, theta2 = inverse_kinematics(dx, dy, L1, L2)

    elif mapped_L1 < -249:
        if elbow_up_config:
            if L1_distance_left > L2 + TOLERANCE:
                canvas.itemconfig("cursor", fill="red")
                return False
            if L1_distance_left > L2:
                compute_elbow_angles(L1_distance_left)
            else:
                theta1, theta2 = inverse_kinematics(dx, dy, L1, L2)
        else:
            theta1, theta2 = inverse_kinematics(dx, dy, L1, L2)

    # --- Too far to reach ---
    elif distance > MAX_REACH + TOLERANCE:
        canvas.itemconfig("cursor", fill="red")
        return False

    # --- Slightly out of reach, force straight arm ---
    elif distance > MAX_REACH and dy >= -90:
        theta2 = 0.0

    # --- Normal zone ---
    else:
        theta1, theta2 = inverse_kinematics(dx, dy, L1, L2)

    if theta1 is None or theta2 is None:
        canvas.itemconfig("cursor", fill="red")
        return False

    # --- Final position calculations ---
    x1 = center_x + L1 * math.cos(theta1)
    y1 = center_y - L1 * math.sin(theta1)
    x2 = x1 + L2 * math.cos(theta1 + theta2)
    y2 = y1 - L2 * math.sin(theta1 + theta2)

    angle_L1 = get_L1_angle_deg(theta1)
    angle_L2 = get_L2_relative_angle_deg(theta1, theta2)
    mapped_L1 = map_angle_L1(angle_L1)
    mapped_L2 = map_angle_L2(angle_L2)
    corrected_L2 = mapped_L2 + mapped_L1

    # --- Safety checks for servo limits ---
    if not (-249 <= mapped_L1 <= 250):
        canvas.itemconfig("cursor", fill="red")
        return False

    if not (-649 <= corrected_L2 <= 650):
        canvas.itemconfig("cursor", fill="red")
        return False

    # --- UI Update ---
    canvas.delete("all")
    draw_workspace_visualization()
    draw_arm(x1, y1, x2, y2)
    draw_texts(angle_L1, angle_L2, mapped_L1, mapped_L2)
    canvas.create_oval(target_x - 5, target_y - 5, target_x + 5, target_y + 5, fill="yellow", tags="cursor")

    # --- Send command ---
    current_time = time.time()
    if current_time - last_send_time >= MOTION_SEND_INTERVAL:
        send_serial_command(f"move {mapped_L1:.0f} {corrected_L2:.0f}")
        last_send_time = current_time

    return True

def run_ik_and_send():
    """
    Wrapper to run IK for the current global target. This is used by mouse and UI buttons.
    """
    global current_target_x, current_target_y
    try_move_ik(current_target_x, current_target_y)

# --- EVENT HANDLERS FOR CONFIGURATION ---
def send_speed(speed_entry):
    root.focus_set()
    try:
        speed_val = int(speed_entry.get())
        send_serial_command(f"set_speed {speed_val}")
    except ValueError:
        print("Invalid speed value")

def send_accel(accel_entry):
    root.focus_set()
    try:
        accel_val = int(accel_entry.get())
        send_serial_command(f"set_accel {accel_val}")
    except ValueError:
        print("Invalid accel value")

def send_tolerance(tolerance_entry):
    root.focus_set()
    try:
        tol_val = int(tolerance_entry.get())
        send_serial_command(f"set_tolerance {tol_val}")
    except ValueError:
        print("Invalid tolerance value")

def update_joystick_move_step(value):
    """Updates the global joystick movement step based on the slider value."""
    global joystick_move_step_value
    try:
        joystick_move_step_value = int(float(value)) 
        print(f"Gamepad speed set to: {joystick_move_step_value}")
    except ValueError:
        print("Invalid speed value from slider")

# --- OTHER EVENT HANDLERS ---
def on_mouse_motion(event):
    """
    Handles mouse click/motion. Only update the target if joystick is inactive.
    """
    global current_target_x, current_target_y
    if joystick_active_movement:
        return
    
    proposed_target_x = event.x
    proposed_target_y = event.y

    if try_move_ik(proposed_target_x, proposed_target_y):
        current_target_x = proposed_target_x
        current_target_y = proposed_target_y


def on_scroll(event):
    """Handles mouse scroll for Z-axis control (relmove 0 0 Z)."""
    if event.delta > 0:
        send_serial_command("relmove 0 0 5")
    else:
        send_serial_command("relmove 0 0 -5")

def calibration_command():
    send_serial_command("set_speed 40000")
    send_serial_command("set_accel 20000")
    send_serial_command("calibrate")
    redraw_ui()

def home_command():
    send_serial_command("home")
    redraw_ui()

def toggle_elbow_direction(event=None):
    global elbow_up_config
    elbow_up_config = not elbow_up_config
    new_state = "Right Bend" if elbow_up_config else "Left Bend"
    elbow_button.config(text=f"Elbow: {new_state} (A)") # Updated label
    print(f"Elbow direction toggled to: {new_state}")
    run_ik_and_send() # Re-calculate IK with new elbow configuration

# --- UI DRAWING FUNCTIONS ---
def draw_workspace_visualization():
    """
    Draws the visual boundaries of the robot arm workspace based on L1 motor limits,
    including new visualizations for the L2 link's geometric minimum reach capability.
    """
    global radius
    L1_start_angle = 90 - L1_MAX_ANGLE_DEG 
    L1_extent_angle = 2 * L1_MAX_ANGLE_DEG
                   
    canvas.create_arc(center_x - radius, center_y - radius,
                   center_x + radius, center_y + radius,
                   start=L1_start_angle, extent=L1_extent_angle,
                   outline="#003366", width=2, fill="lightblue", tags="outer_boundary")
    
    canvas.create_arc(center_x - MIN_REACH, center_y - MIN_REACH,
                   center_x + MIN_REACH, center_y + MIN_REACH,
                   start=L1_start_angle, extent=L1_extent_angle,
                   outline="#cc0000", width=2, fill="#FF6666", tags="unreachable_zone_fill_physical")

    angle_rad = math.radians(L1_start_angle)
    elbow_y = center_y - L1 * math.sin(angle_rad)

    # L1 - 7 is because of the oval on L1's end
    canvas.create_arc(center_x + L1 - 7 - L2, elbow_y   - L2,
                   center_x + L1 - 7 + L2, elbow_y  + L2,
                   start=L1_start_angle, extent=-90,
                   outline="#ff4d00", width=2, fill="#ffae00", tags="outer_boundary")
    
    # L1 + 7 is because of the oval on L1's end (the other side) 
    canvas.create_arc(center_x - L1 + 7  - L2, elbow_y  - L2,
                   center_x - L1 + 7 + L2, elbow_y + L2,
                   start=L1_start_angle+L1_extent_angle, extent=90,
                   outline="#ff4d00", width=2, fill="#ffae00", tags="outer_boundary")

    limit_angle_rad = math.radians(90 - L1_MAX_ANGLE_DEG)
    x_limit_1 = center_x + radius * math.cos(limit_angle_rad)
    y_limit_1 = center_y - radius * math.sin(limit_angle_rad)
    
    limit_angle_rad_2 = math.radians(90 + L1_MAX_ANGLE_DEG)
    x_limit_2 = center_x + radius * math.cos(limit_angle_rad_2)
    y_limit_2 = center_y - radius * math.sin(limit_angle_rad_2)
    
    canvas.create_line(center_x, center_y, x_limit_1, y_limit_1, fill="#003366", width=1, tags="limit_line_1")
    canvas.create_line(center_x, center_y, x_limit_2, y_limit_2, fill="#003366", width=1, tags="limit_line_2")


def draw_arm(x1, y1, x2, y2):
    # Link 1 (Base to Elbow)
    canvas.create_line(center_x, center_y, x1, y1, width=8, fill="#0033aa", capstyle=tk.ROUND, tags="arm_L1")
    # Link 2 (Elbow to End-Effector)
    canvas.create_line(x1, y1, x2, y2, width=8, fill="#ff6600", capstyle=tk.ROUND, tags="arm_L2")

    canvas.create_oval(x1 - 8, y1 - 8, x1 + 8, y1 + 8, fill="#00aa00", outline="#000") # Elbow Joint
    canvas.create_oval(x2 - 8, y2 - 8, x2 + 8, y2 + 8, fill="#ffd700", outline="#000") # End Effector
    canvas.create_oval(center_x - 10, center_y - 10, center_x + 10, center_y + 10, fill="#000", outline="#fff", width=2) # Base

def draw_texts(angle_L1=0.0, angle_L2=0.0, mapped_L1=0.0, mapped_L2=0.0):
    texts = {
        "L1_angle": f"L1 angle: {angle_L1:.2f}°",
        "L1_mapped": f"L1 mapped: {mapped_L1:.0f} (Max: +/- 249)",
        "L2_angle": f"L2 angle (relative): {angle_L2:.2f}°",
        "L2_mapped": f"L2 mapped: {mapped_L2:.0f}",
        "L2_corrected": f"L2 corrected: {(mapped_L2 + mapped_L1):.0f} (Max: +/- 649)",
    }
    y_start = 10
    y_spacing = 25
    for i, (key, text) in enumerate(texts.items()):
        if key in angle_text_ids and angle_text_ids[key]:
            canvas.delete(angle_text_ids[key])
        angle_text_ids[key] = canvas.create_text(
            10, y_start + i * y_spacing, anchor="nw",
            text=text, font=("Arial", 12, "bold"), fill="#003366"
        )

def redraw_ui():
    """Sets the target to the home position and runs IK/draws."""
    global current_target_x, current_target_y
    current_target_x = center_x
    current_target_y = center_y - (L1 + L2)
    run_ik_and_send()


# --- TKINTER WIDGETS ---
def create_command_buttons(frame):
    global serial_status_label
    
    # Status Label (New)
    serial_status_label = tk.Label(frame, text="Serial Status: Initializing...", font=("Arial", 10, "bold"), fg="black")
    serial_status_label.grid(row=0, column=0, columnspan=3, pady=(0, 10))

    btn_calibrate = tk.Button(frame, text="Calibrate (Select)", width=15, command=calibration_command, relief=tk.RAISED, bg='#e0f7fa', activebackground='#b2ebf2')
    btn_home = tk.Button(frame, text="Home (Start)", width=15, command=home_command, relief=tk.RAISED, bg='#e8f5e9', activebackground='#c8e6c9')
    btn_stop = tk.Button(frame, text="Stop", width=15, command=lambda: send_serial_command("stop"), relief=tk.RAISED, bg='#ffebee', activebackground='#ffcdd2')
    
    # Mapped to Y button (Button 3)
    btn_open = tk.Button(frame, text="Open Gripper (Y)", width=15, command=lambda: send_serial_command("open_gripper"), relief=tk.RAISED, bg='#fff8e1', activebackground='#ffecb3')
    # Mapped to X button (Button 0)
    btn_close = tk.Button(frame, text="Close Gripper (X)", width=15, command=lambda: send_serial_command("close_gripper"), relief=tk.RAISED, bg='#fffde7', activebackground='#fff9c4')

    global elbow_button
    initial_state = "Right Bend" if elbow_up_config else "Left Bend"
    # Mapped to A button (Button 1)
    elbow_button = tk.Button(frame, text=f"Elbow: {initial_state} (A)", width=20, command=toggle_elbow_direction, relief=tk.RAISED, bg='#f3e5f5', activebackground='#e1bee7')

    # Dummy frame for entry widgets
    config_frame = tk.Frame(frame, bd=1, relief=tk.GROOVE, padx=5, pady=5)

    speed_label = tk.Label(config_frame, text="Speed:")
    speed_entry = tk.Entry(config_frame, width=6); speed_entry.insert(0, "40000")
    btn_set_speed = tk.Button(config_frame, text="Set Speed", width=12, command=lambda: send_speed(speed_entry))

    accel_label = tk.Label(config_frame, text="Accel:")
    accel_entry = tk.Entry(config_frame, width=6); accel_entry.insert(0, "20000")
    btn_set_accel = tk.Button(config_frame, text="Set Accel", width=12, command=lambda: send_accel(accel_entry))

    tolerance_label = tk.Label(config_frame, text="Tolerance:")
    tolerance_entry = tk.Entry(config_frame, width=6); tolerance_entry.insert(0, "10") # Default tolerance
    btn_set_tolerance = tk.Button(config_frame, text="Set Tolerance", width=12, command=lambda: send_tolerance(tolerance_entry))

    # --- New Joystick Speed Slider ---
    global joystick_move_step_value
    slider_label = tk.Label(config_frame, text="Gamepad Speed (1-30):")
    joystick_speed_slider = tk.Scale(
        config_frame,
        from_=1, to=30, 
        orient=tk.HORIZONTAL,
        command=update_joystick_move_step,
        length=180, 
        resolution=1 # Allow only integer steps
    )
    joystick_speed_slider.set(joystick_move_step_value) # Set initial value

    btn_calibrate.grid(row=1, column=0, padx=5, pady=5)
    btn_home.grid(row=1, column=1, padx=5, pady=5)
    btn_stop.grid(row=1, column=2, padx=5, pady=5)

    btn_open.grid(row=2, column=0, padx=5, pady=5)
    btn_close.grid(row=2, column=1, padx=5, pady=5)
    elbow_button.grid(row=2, column=2, padx=5, pady=5)

    config_frame.grid(row=3, column=0, columnspan=3, pady=10)
    
    speed_label.grid(row=0, column=0, padx=2, pady=2, sticky="e")
    speed_entry.grid(row=0, column=1, padx=2, pady=2)
    btn_set_speed.grid(row=0, column=2, padx=5, pady=2)

    accel_label.grid(row=1, column=0, padx=2, pady=2, sticky="e")
    accel_entry.grid(row=1, column=1, padx=2, pady=2)
    btn_set_accel.grid(row=1, column=2, padx=5, pady=2)

    tolerance_label.grid(row=2, column=0, padx=2, pady=2, sticky="e")
    tolerance_entry.grid(row=2, column=1, padx=2, pady=2)
    btn_set_tolerance.grid(row=2, column=2, padx=5, pady=2)
    
    slider_label.grid(row=3, column=0, padx=2, pady=5, sticky="w")
    joystick_speed_slider.grid(row=3, column=1, columnspan=2, padx=5, pady=5, sticky="ew")


# --- TKINTER SETUP ---
root = tk.Tk()
root.title("SCARA Robot Arm (Mouse + Gamepad Control)")
root.protocol("WM_DELETE_WINDOW", on_closing)

canvas = tk.Canvas(root, width=CANVAS_WIDTH, height=CANVAS_HEIGHT, bg='#f0f0f0', bd=2, relief=tk.SUNKEN)
canvas.pack(padx=10, pady=10)

control_frame = tk.Frame(root)
control_frame.pack(pady=10)

create_command_buttons(control_frame)
start_serial_thread()
init_joystick()
redraw_ui()

canvas.bind("<Motion>", on_mouse_motion) # Mouse control for X/Y
canvas.bind("<MouseWheel>", on_scroll) # Mouse scroll for Z-axis
root.bind("<Key-e>", toggle_elbow_direction) # Toggle Elbow (Keyboard fallback)

root.after(JOYSTICK_POLL_INTERVAL, check_joystick)

root.mainloop()