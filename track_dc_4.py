import time
import cv2
import imutils
import numpy as np
from picamera2 import Picamera2, Preview
import serial

class PID:
    def __init__(self, P=0.2, I=0.0, D=0.0, current_time=None, deadzone=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.deadzone = deadzone 
        self.sample_time = 0.00
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time
        self.clear()

    def clear(self):
        self.SetPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.int_error = 0.0
        self.windup_guard = 2500.0
        self.output = 0.0

    def update(self, feedback_value, current_time=None):
        error = self.SetPoint - feedback_value

        if abs(error) < self.deadzone:
            error = error/2
            effective_Kp = self.Kp * 0.9
            effective_Kd = self.Kd * 0.9
        else:
            effective_Kp = self.Kp
            effective_Kd = self.Kd

        self.current_time = current_time if current_time is not None else time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if delta_time >= self.sample_time:
            self.PTerm = effective_Kp * error
            self.ITerm += self.Ki * error * delta_time

            if self.ITerm < -self.windup_guard:
                self.ITerm = -self.windup_guard
            elif self.ITerm > self.windup_guard:
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = effective_Kd * delta_error / delta_time

            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + self.ITerm + self.DTerm

        return self.output

    def setKp(self, proportional_gain):
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        self.Kd = derivative_gain

    def setWindup(self, windup):
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        self.sample_time = sample_time

def map_value(x, min_angle, max_angle, min_pos, max_pos):
    return min_angle + ((x + max_pos) / (max_pos - min_pos)) * (max_angle - min_angle)

def constraint(val, min_val, max_val):
    if val < min_val:
        val = min_val
    elif val >= max_val:
        val = max_val
    return val

def on_mouse_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        hsv = param[y, x]
        print(f"Clicked HSV value: {hsv}")

def initialize_camera():
    piCam2 = Picamera2()
    config = piCam2.create_preview_configuration(main={"size": (240, 240), "format": "RGB888"})
    piCam2.configure(config)
    piCam2.start()
    return piCam2

def capture_frame(piCam2):
    return piCam2.capture_array()

def preprocess_frame(frame):
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    return hsv

def create_color_mask(hsv, lower, upper):
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    return mask

def find_contour(mask):
    contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        return c
    return None

def get_center(contour):
    M = cv2.moments(contour)
    if M["m00"] != 0:
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        return center
    return None

def detect_path_cam(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([90, 30, 150])
    upper_blue = np.array([120, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    path_contours = [c for c in contours if cv2.arcLength(c, True) > 50 and cv2.contourArea(c) > 80]
    sorted_contours = sorted(path_contours, key=lambda c: cv2.boundingRect(c)[0])
    path_points = [tuple(point[0]) for contour in sorted_contours for point in contour]
    path_points = sorted(path_points, key=lambda point: point[0])
    return path_points

def draw_path(image, path_points):
    for point in path_points:
        cv2.circle(image, point, 3, (0, 255, 0), -1)
    return image

def sample_points(points, interval):
    sampled_points = []
    if not points:
        return sampled_points
    sampled_points.append(points[0])
    last_point = points[0]
    for point in points[1:]:
        distance = calculate_distance(last_point, point)
        if distance >= interval:
            sampled_points.append(point)
            last_point = point
    return sampled_points

def calculate_distance(point1, point2):
    return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

def main():
    min_angle_X = -75
    max_angle_X = 75
    min_angle_Y = -75
    max_angle_Y = 75

    ser1 = serial.Serial('/dev/ttyUSB1', 2000000)
    ser2 = serial.Serial('/dev/ttyUSB0', 2000000)
    ser1.reset_input_buffer()
    ser2.reset_input_buffer()
    print("Serial communication initialized")

    piCam2 = initialize_camera()

    SAMPLE_TIME = 0.15
    pid_x = PID(3.9, 0.01, 2.1 , deadzone=1)
    pid_y = PID(3.9, 0.01, 2.1 , deadzone=1)
    pid_x.setSampleTime(SAMPLE_TIME)
    pid_y.setSampleTime(SAMPLE_TIME)

    path_points = []
    initial_path_points = []
    ######################  sin_wave ########## 
    desired_interval = 10
    error_threshold = 25
    #############################################
    
    ######################  square wave ########## 
   # desired_interval = 20
    #error_threshold = 24
    #############################################

    HSV_LOWER = (170, 90, 100)    #red
    
   # HSV_LOWER=(160 ,90, 50)   #red dark
    HSV_LOWER=(160, 60, 50)
    HSV_UPPER = (180, 255, 255)

    while True:
        frame = capture_frame(piCam2)
        hsv = preprocess_frame(frame)
        mask = create_color_mask(hsv, HSV_LOWER, HSV_UPPER)
        contour = find_contour(mask)
        ball_position = get_center(contour) if contour is not None else None

        if not initial_path_points:
            initial_path_points = detect_path_cam(frame)
            initial_path_points = sample_points(initial_path_points, desired_interval)
            path_points = initial_path_points.copy()

        frame_with_path = draw_path(frame.copy(), path_points)
        if ball_position:
            cv2.circle(frame_with_path, ball_position, 10, (0, 255, 0), 2)
            cv2.circle(frame_with_path, ball_position, 3, (0, 255, 0), -1)

        if ball_position and path_points:
            next_point = path_points[0]
            pos_x = ball_position[0]
            pos_y = ball_position[1]

            pid_x.SetPoint = next_point[0]
            pid_y.SetPoint = next_point[1]

            control_x = pid_x.update(pos_x)
            control_y = pid_y.update(pos_y)

            output_x = control_x
            output_y = control_y

            control_x = map_value(output_x, min_angle_X, max_angle_X, -450, 450)
            control_y = map_value(output_y, min_angle_Y, max_angle_Y, -450, 450)

            control_x = constraint(control_x, -90, 90)
            control_y = constraint(control_y, -90, 90)

            motor_deg_sent = (int(control_x), int(control_y))

            msg1 = f"{motor_deg_sent[0]}\n"
            msg2 = f"{motor_deg_sent[1]}\n"

            ser1.write(msg1.encode('utf-8'))
            ser2.write(msg2.encode('utf-8'))

            ser1.reset_input_buffer()
            ser2.reset_input_buffer()

            if (abs(pos_x - next_point[0]) < error_threshold and abs(pos_y - next_point[1]) < error_threshold):
                path_points = path_points[1:]  # Move to the next point if within threshold
                if not path_points:  # Reset to initial path points
                    path_points = initial_path_points.copy()

        else:
            msg1 = f"{0}\n"
            msg2 = f"{0}\n"

            ser1.write(msg1.encode('utf-8'))
            ser2.write(msg2.encode('utf-8'))

            ser1.reset_input_buffer()
            ser2.reset_input_buffer()

        cv2.imshow('Detected Path and Objects', frame_with_path)
        cv2.setMouseCallback('Detected Path and Objects', on_mouse_click, hsv)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    piCam2.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
