import time
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio




class ServoController:
    def __init__(self, channel=0, min_pulse=1000, max_pulse=2000, reverse_angle = False, max_angle = 90):
        # Initialize I2C bus and PCA9685
        self.i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 50  # Set frequency to 50Hz for servos
        self.center_fix = 0
        # Set up the servo
        self.max_angle = max_angle
        self.servo = self.pca.channels[channel]
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
        self.reverse_factor = -1 if reverse_angle else 1
    def set_angle(self, angle):
        """Set the servo to a specific angle (-90 to 90 degrees)."""
        angle += self.center_fix
        if -90 <= angle <= 90:
            pulse_width = self.min_pulse + (self.max_pulse - self.min_pulse) * ((self.reverse_factor*angle + 90) / 180)
            self.servo.duty_cycle = int(pulse_width * 0xFFFF / 20000)
        else:
            raise ValueError("Angle must be between -90 and 90 degrees")

    def sweep(self, start=-90, end=90, step=5, delay=0.1):
        """Perform a sweep from start to end angle."""
        for angle in range(start, end + 1, step):
            self.set_angle(angle)
            time.sleep(delay)

    def center(self):
        """Center the servo (set to 0 degrees)."""
        self.set_angle(90)
    def unlock(self):
        self.i2c.unlock()

def create_servo(channel=0, min_pulse=1000, max_pulse=2000, reverse_angle= False, max_angle = 90):
    """Create and return a ServoController instance."""
    return ServoController(channel, min_pulse, max_pulse, reverse_angle, max_angle)