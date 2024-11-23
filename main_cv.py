import cv2
import numpy as np
from picamera2 import Picamera2
from process_frames import *
from motor_controller import create_dc_motor
from servo_controller import create_servo
from BnoSensor_controller import creat_bno_sensor
import collections

import time
# Initialize Picamera2
picam2 = Picamera2()
RESIZE_FACTOR = 1
# picam2.sensor_mode = 4
#2304 x 1296-PC1B
cam_res = (2304 // RESIZE_FACTOR, 1296 // RESIZE_FACTOR)

video_config = picam2.create_video_configuration(raw={"size": cam_res, "format" : 'SGBRG10'}, main = {"size":(2304//10, 1296//10)})
picam2.configure(video_config)
picam2.start()

def _get_raw_rc_angle(bno):
        """Get current raw angle from sensor."""
        sensor_data = bno.get_sensor_data()
        if not sensor_data:
            print("no gyro data")
            return None
        yaw_angle_raw = sensor_data['gyro_angles'][2]
        yaw_angle_raw
        return yaw_angle_raw

def get_rc_angle(bno):
        yaw_angle = _get_raw_rc_angle(bno)
        yaw_angle = np.degrees(yaw_angle)
        yaw_angle = (yaw_angle + 180) % 360 - 180
        return yaw_angle

class stuck_checker:
    def __init__(self):
         pass
         self.currTime = time.time()
         self.similarity_start_time = 0
         self.previous_frame = None
    def frames_are_similar(self, frame1, frame2, threshold):
        if frame1 is None or frame2 is None:
            return False
        # Convert to grayscale for simpler comparison
        gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

        # Compute structural similarity
        difference = cv2.absdiff(gray1, gray2)
        _, diff_thresh = cv2.threshold(difference, 25, 255, cv2.THRESH_BINARY)

        # Calculate the ratio of non-difference pixels
        non_zero_count = np.count_nonzero(diff_thresh)
        total_pixels = frame1.shape[0] * frame1.shape[1]
        similarity_ratio = (total_pixels - non_zero_count) / total_pixels

        return similarity_ratio >= threshold
    
    def check_if_stuck( self, frame, similarity_threshold = 0.9, similarity_duration = 3):
        # Check for frame similarity over time (car is stuck)
        # print('in check')
        self.currTime = time.time()
        if self.previous_frame is None:
             self.previous_frame = frame.copy()
             return False
        
        # print("HHHHH")
        if self.frames_are_similar(self.previous_frame, frame, threshold=similarity_threshold):
            if self.similarity_start_time == 0:
                self.similarity_start_time = self.currTime
            elif self.currTime - self.similarity_start_time >= similarity_duration :
                print("stuck")
                
                self.similarity_start_time = 0  # Reset the timer
                return True
            else:
                print("checking stuck..")
        else:
            # Reset similarity timer if the frames are not similar
            self.similarity_start_time = 0
            self.previous_frame = frame.copy()
            return False
        
    


def main():
    motor = create_dc_motor()
    servo = create_servo(channel=7, max_pulse=2500, min_pulse=500, reverse_angle=True, max_angle=40)
    bno = creat_bno_sensor()
    servo.center()
    initial_point = -1
    prev_wall_point = -1
    counter = 0
    prev_time_counter = time.time()
    prev_time_SteerLock = time.time()
    lab_count = 0
    start_counter = False
    num_points_avg = 40 # How many points from the trace to avg
    steerLock_duration = 0
    stuck_checker_obj = stuck_checker()
    steer_angle = 0
    steer_angle_buffer = collections.deque(maxlen=3)
    run_speed = 0.2
    try:
        while True:
            curr_time = time.time()
            frame = picam2.capture_array()
            
            offset_steer, wall_point, comp_img = process_frame(frame.copy(), num_points_avg)
            cv2.imshow("preview", comp_img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
                 
            if steerLock_duration:
                 if curr_time - prev_time_SteerLock < steerLock_duration:
                    continue 
                 else:
                    motor.set_speed(0)
                    steerLock_duration = 0
            
            if stuck_checker_obj.check_if_stuck(frame):
                steerLock_duration = 3
                prev_time_SteerLock = curr_time
                steer_angle = -steer_angle
                servo.set_angle(steer_angle)
                if run_speed:
                    motor.set_speed(-0.1)
                continue
            
            #steering section
            max_offset = 40.0
            offset_steer = min(max_offset, max(-max_offset, offset_steer))
            steer_ratio = offset_steer / max_offset
            steer_angle = int(steer_ratio*servo.max_angle)
            steer_angle_buffer.append(steer_angle)
            smoothed_steering_angle  = sum(steer_angle_buffer)/len(steer_angle_buffer)
            servo.set_angle(smoothed_steering_angle)
            rc_angle = get_rc_angle(bno)
            motor.set_speed(run_speed)
            
            #counting labs section
            if wall_point is not None:
                if initial_point < 0:
                        prev_time_counter = curr_time
                        initial_point = wall_point[1] 
                        # prev_wall_point = 1000
                if curr_time - prev_time_counter > 0.5:
                    
                    if abs(rc_angle) < 40:
                        print(wall_point)
                        if wall_point[1] >= initial_point and prev_wall_point < initial_point:
                            if start_counter:
                                counter+=1
                                print("counter: ", counter)
                                if counter == 3:
                                    motor.set_speed(-0.1)
                                    time.sleep(0.2)
                                    break
                                prev_time_counter = curr_time
                    else:
                         start_counter = True
                         prev_wall_point = 0
                    prev_wall_point = wall_point[1]
        
    finally:
        picam2.stop()
        cv2.destroyAllWindows()
        servo.center()
        servo.unlock()
        motor.stop()
        

if __name__ == "__main__":
    main()