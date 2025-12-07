from controller import Robot, DistanceSensor, Camera, Motor, GPS
import random
import numpy as np
import cv2
import os

TIME_STEP = 32
robot = Robot()

# --- Device Setup ---
ds = [robot.getDevice(f"ps{i}") for i in range(8)]
for sensor in ds: sensor.enable(TIME_STEP)
camera = robot.getDevice("camera"); camera.enable(TIME_STEP)
ground_sensor = robot.getDevice("gs0"); ground_sensor.enable(TIME_STEP)
gps = robot.getDevice("gps"); gps.enable(TIME_STEP)
left_motor = robot.getDevice("left wheel motor"); right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf')); right_motor.setPosition(float('inf'))
left_motor.setVelocity(0); right_motor.setVelocity(0)

print("Robot initialized.")

color_names = {
    'red':    (200, 0, 0), 'green':  (0, 200, 0), 'blue':   (0, 0, 200),
    'yellow': (200, 200, 0), 'orange': (255, 150, 20), 'purple': (150, 80, 200),
    'pink':   (240, 100, 180), 'cyan':   (50, 200, 210),
}
last_seen_color = None
previous_seen_color = None

def get_box_color_name(r, g, b):
    closest_name = None; closest_dist = float('inf')
    for name, (cr, cg, cb) in color_names.items():
        dist = (r-cr)**2 + (g-cg)**2 + (b-cb)**2
        if dist < closest_dist:
            closest_dist = dist
            closest_name = name
    return closest_name if closest_dist < 5000 else None

# --- Flags for completion tracking ---
did_sleep = did_eat = did_drink = did_search_food = did_search_water = False
did_search_both = did_avoid_obstacle = did_beacon = did_box = did_brightness = False
did_photo_frame = False

hunger = 50
thirst = 50
tiredness = 0
photo_frame_message_shown = False
last_breadcrumb_step = 0
BREADCRUMB_INTERVAL = 80
step_counter = 0
last_brightness_state = None

def drop_breadcrumb(step_counter):
    global last_breadcrumb_step
    if (step_counter - last_breadcrumb_step) >= BREADCRUMB_INTERVAL:
        pos = gps.getValues()
        print(f"[BREADCRUMB] Dropped at ({pos[0]:.2f}, {pos[1]:.2f})")
        last_breadcrumb_step = step_counter

def detect_beacon():
    global did_beacon
    width, height, cx, cy = camera.getWidth(), camera.getHeight(), camera.getWidth()//2, camera.getHeight()//2
    img = camera.getImage()
    if img is None or width < 1 or height < 1: return False
    r = camera.imageGetRed(img, width, cx, cy); g = camera.imageGetGreen(img, width, cx, cy); b = camera.imageGetBlue(img, width, cx, cy)
    if (r > 200 and g > 100 and b < 100):
        print("[BEACON] Mission Objective: Orange beacon sighted! Pausing and inspecting beacon with caution.")
        did_beacon = True
        # SLOW DOWN and approach slowly
        for _ in range(7):  # less steps, smaller velocity!
            left_motor.setVelocity(1.5)
            right_motor.setVelocity(1.2)
            robot.step(TIME_STEP)
        # Pause briefly at inspection distance
        for _ in range(10):
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            robot.step(TIME_STEP)
        # Back up and turn away to avoid collision
        for _ in range(8):
            left_motor.setVelocity(-2.5)
            right_motor.setVelocity(-2.1)
            robot.step(TIME_STEP)
        # Random turn then proceed
        if random.random() < 0.5:
            for _ in range(10):
                left_motor.setVelocity(-3)
                right_motor.setVelocity(3)
                robot.step(TIME_STEP)
        else:
            for _ in range(10):
                left_motor.setVelocity(3)
                right_motor.setVelocity(-3)
                robot.step(TIME_STEP)
        left_motor.setVelocity(0); right_motor.setVelocity(0)
        return True  # skip all other actions this timestep
    return False

def close_to_photo_frame():
    global did_photo_frame
    width, height = camera.getWidth(), camera.getHeight()
    cx, cy = width // 2, height // 2
    img = camera.getImage()
    if not img: return False
    patch_size = 8; detect_thresh = 0.65; match_count = 0; total = patch_size * patch_size
    for dx in range(-patch_size//2, patch_size//2):
        for dy in range(-patch_size//2, patch_size//2):
            x = min(max(cx + dx, 0), width-1)
            y = min(max(cy + dy, 0), height-1)
            r = camera.imageGetRed(img, width, x, y)
            g = camera.imageGetGreen(img, width, x, y)
            b = camera.imageGetBlue(img, width, x, y)
            if ((r > 170 and g > 145 and b > 60 and abs(r-g) < 50 and (r-b) > 80) or
                (r > 140 and g > 120 and b > 60) or
                (r > 150 and g > 80 and b < 120)):
                match_count += 1
    if match_count >= int(total * detect_thresh):
        did_photo_frame = True
        return True
    return False

def avoid_photo_frame():
    global photo_frame_message_shown
    print_status("Avoiding photo frame...")
    for _ in range(30): left_motor.setVelocity(-5); right_motor.setVelocity(-5); robot.step(TIME_STEP)
    if random.random() < 0.5:
        for _ in range(25): left_motor.setVelocity(5); right_motor.setVelocity(-5); robot.step(TIME_STEP)
    else:
        for _ in range(25): left_motor.setVelocity(-5); right_motor.setVelocity(5); robot.step(TIME_STEP)
    for _ in range(20): left_motor.setVelocity(5); right_motor.setVelocity(5); robot.step(TIME_STEP)
    left_motor.setVelocity(0); right_motor.setVelocity(0)
    print_status("Photo frame avoidance complete. Resuming normal behavior.")
    if not photo_frame_message_shown: observe_photo_frame_and_speak_once()

def observe_photo_frame_and_speak_once():
    global photo_frame_message_shown
    if not photo_frame_message_shown:
        print("I see a family of four, with two kids—a boy and a girl—heading toward the sunset on the beach. It fills me with happiness to witness such a beautiful moment.")
        photo_frame_message_shown = True

def get_ground_color_label(value):
    if 455 <= value <= 500: return "Green (Food)"
    elif 290 <= value <= 380: return "Blue (Water)"
    return "Unknown"

def print_status(action):
    width, height, cx, cy = camera.getWidth(), camera.getHeight(), camera.getWidth()//2, camera.getHeight()//2
    cam_img = camera.getImage()
    r = camera.imageGetRed(cam_img, width, cx, cy) if cam_img else 0
    g = camera.imageGetGreen(cam_img, width, cx, cy) if cam_img else 0
    b = camera.imageGetBlue(cam_img, width, cx, cy) if cam_img else 0
    ground_value = round(ground_sensor.getValue(),1)
    ground_color = get_ground_color_label(ground_value)
    pos = gps.getValues()
    gps_str = f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})"
    print(f"{action} | Sleep: {tiredness:.1f}, Hunger: {hunger:.1f}, Thirst: {thirst:.1f} | Cam RGB: ({r},{g},{b}) | Ground: {ground_value} ({ground_color}) | GPS: {gps_str}")

def ground_is_food():
    readings = []
    for _ in range(3):
        if robot.step(TIME_STEP) == -1: break
        readings.append(ground_sensor.getValue())
    if not readings: return False
    base = readings[0]
    return all(abs(val-base) <= 15 for val in readings) and 455 <= base <= 500

def ground_is_water():
    readings = []
    for _ in range(3):
        if robot.step(TIME_STEP) == -1: break
        readings.append(ground_sensor.getValue())
    if not readings: return False
    base = readings[0]
    return all(abs(val-base) <= 30 for val in readings) and 290 <= base <= 380

def should_sleep(): return tiredness > 80
def should_eat():   return hunger > 75 and ground_is_food()
def should_drink(): return thirst > 75 and ground_is_water()
def should_avoid_obstacle(): return any(sensor.getValue() > 100 for sensor in ds)
def should_search_food_and_water(): return hunger > 75 and thirst > 75
def should_search_food(): return hunger > 75
def should_search_water(): return thirst > 75

def all_behaviours_done():
    return all([
        did_sleep, did_eat, did_drink, did_search_food, did_search_water,
        did_search_both, did_avoid_obstacle, did_beacon, did_box,
        did_brightness, did_photo_frame
    ])

def sleep():
    global tiredness, did_sleep
    left_motor.setVelocity(0); right_motor.setVelocity(0)
    print_status("Sleeping..."); did_sleep = True
    while tiredness > 10:
        if robot.step(TIME_STEP) == -1: break
        tiredness = max(tiredness - (8 * TIME_STEP / 100), 0)
        print_status("Sleeping...")

def eat():
    global hunger, did_eat
    print_status("Eating..."); left_motor.setVelocity(0); right_motor.setVelocity(0)
    did_eat = True
    while hunger > 10:
        if robot.step(TIME_STEP) == -1: break
        hunger = max(hunger - (10 * TIME_STEP / 300), 0)
        print_status("Eating...")

def drink():
    global thirst, did_drink
    print_status("Drinking..."); left_motor.setVelocity(0); right_motor.setVelocity(0)
    did_drink = True
    while thirst > 10:
        if robot.step(TIME_STEP) == -1: break
        thirst = max(thirst - (10 * TIME_STEP / 300), 0)
        print_status("Drinking...")

def avoid_obstacle():
    global did_avoid_obstacle
    print_status("Avoiding obstacle...")
    did_avoid_obstacle = True
    front_val = (ds[0].getValue() + ds[7].getValue()) / 2
    left_val = sum(ds[i].getValue() for i in range(1, 3)) / 2
    right_val = sum(ds[i].getValue() for i in range(4, 6)) / 2
    threshold = 100
    if front_val > threshold:
        for _ in range(12): left_motor.setVelocity(-3); right_motor.setVelocity(-3); robot.step(TIME_STEP)
        if left_val < right_val:
            for _ in range(13): left_motor.setVelocity(3); right_motor.setVelocity(-3); robot.step(TIME_STEP)
        else:
            for _ in range(13): left_motor.setVelocity(-3); right_motor.setVelocity(3); robot.step(TIME_STEP)
    elif left_val > threshold:
        for _ in range(10): left_motor.setVelocity(2.5); right_motor.setVelocity(-2.5); robot.step(TIME_STEP)
    elif right_val > threshold:
        for _ in range(10): left_motor.setVelocity(-2.5); right_motor.setVelocity(2.5); robot.step(TIME_STEP)
    for _ in range(5): left_motor.setVelocity(3); right_motor.setVelocity(3); robot.step(TIME_STEP)
    left_motor.setVelocity(0); right_motor.setVelocity(0)

def search_food_and_water():
    global did_search_both
    print_status("Searching for food and water..."); did_search_both = True
    wander()

def search_food():
    global did_search_food
    print_status("Searching for food..."); did_search_food = True
    wander()

def search_water():
    global did_search_water
    print_status("Searching for water..."); did_search_water = True
    wander()

def wander():
    left_motor.setVelocity(5 + random.uniform(-1.2,1.2))
    right_motor.setVelocity(5 + random.uniform(-1.2,1.2))

while robot.step(TIME_STEP) != -1:
    step_counter += 1
    tiredness += 0.1; hunger += 0.05; thirst += 0.05

    drop_breadcrumb(step_counter)
    width = camera.getWidth(); height = camera.getHeight()
    cx, cy = width // 2, height // 2
    img = camera.getImage()

    if img:
        r = camera.imageGetRed(img, width, cx, cy)
        g = camera.imageGetGreen(img, width, cx, cy)
        b = camera.imageGetBlue(img, width, cx, cy)
        box_color = get_box_color_name(r, g, b)
        if box_color and box_color != last_seen_color:
            if last_seen_color: previous_seen_color = last_seen_color
            print(f"Currently seeing: {box_color} box.")
            did_box = True
            if previous_seen_color:
                print(f"Previously seen: {previous_seen_color} box.")
            last_seen_color = box_color

    brightness = 0
    if img:
        r = camera.imageGetRed(img, width, cx, cy)
        g = camera.imageGetGreen(img, width, cx, cy)
        b = camera.imageGetBlue(img, width, cx, cy)
        brightness = (r + g + b) / 3
    if brightness > 180:
        if last_brightness_state != 'bright':
            print("It’s bright here!"); did_brightness = True
        last_brightness_state = 'bright'
    elif brightness < 70:
        if last_brightness_state != 'dark':
            print("It’s a bit dark in this part of the arena."); did_brightness = True
        last_brightness_state = 'dark'
    else:
        last_brightness_state = None

    if detect_beacon(): continue
    if close_to_photo_frame():
        print("[PHOTO FRAME] Photo frame detected up close!")
        if not photo_frame_message_shown: observe_photo_frame_and_speak_once()
        avoid_photo_frame()
        continue

    if should_avoid_obstacle(): avoid_obstacle(); continue
    if should_sleep(): sleep(); continue
    if should_eat(): eat(); continue
    if should_drink(): drink(); continue
    if should_search_food_and_water(): search_food_and_water(); continue
    if should_search_food(): search_food(); continue
    if should_search_water(): search_water(); continue

    if all_behaviours_done():
        print("I had a really good life. I am tired and gonna die. Goodbye.")
        left_motor.setVelocity(0); right_motor.setVelocity(0)
        break

    print_status("Wandering...")
    wander()
