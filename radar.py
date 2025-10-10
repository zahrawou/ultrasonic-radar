#!/usr/bin/env python3

import serial
import sys
import time
import os
import struct
import mmap
from periphery import PWM
import math

# --- USER CONFIG ---
START_ANGLE = 0                # Start of sweep (degrees)
END_ANGLE = 180                 # End of sweep (degrees)
SWEEP_DELAY_SEC = 0.1          # Delay between steps
MAX_DISTANCE_CM = 400           # Max distance for scaling
ECHO_LIFETIME_FRAMES = 100      # Echo fade frames
BEAM_THICKNESS = 2              # Beam thickness in pixels
GRID_THICKNESS = 1              # Grid dot thickness

# --- ULTRASONIC ---
PORT = '/dev/ttyS4'
BAUDRATE = 9600
TRIGGER_BYTE = b'\xA0'
WAIT_AFTER_TRIGGER = 0.05
READ_TIMEOUT = 0.1

# --- SERVO ---
PWM_CHANNEL = 10
PWM_PIN = 0
FREQUENCY = 50                  # 50Hz = 20ms period
MIN_PULSE_MS = 0.5              # 0° pulse width
MAX_PULSE_MS = 2.5              # 180° pulse width
PERIOD_MS = 20.0                # 50Hz = 20ms period

# Calculate min/max duty based on START_ANGLE and END_ANGLE
TOTAL_SERVO_RANGE_DEG = 180.0
MIN_DUTY = MIN_PULSE_MS / PERIOD_MS + (START_ANGLE / TOTAL_SERVO_RANGE_DEG) * ((MAX_PULSE_MS - MIN_PULSE_MS) / PERIOD_MS)
MAX_DUTY = MIN_PULSE_MS / PERIOD_MS + (END_ANGLE / TOTAL_SERVO_RANGE_DEG) * ((MAX_PULSE_MS - MIN_PULSE_MS) / PERIOD_MS)
STEP_SIZE_DEG = 1

print("Servo config: {}° to {}° sweep".format(START_ANGLE, END_ANGLE))
print("Duty cycle: min={:.4f}, max={:.4f}".format(MIN_DUTY, MAX_DUTY))

# --- DISPLAY COLORS ---
BG_COLOR = (0, 0, 0)
GRID_COLOR = (60, 60, 60)
BEAM_COLOR = (255, 255, 100)
WALL_BEAM_COLOR = (180, 180, 50)
ECHO_BASE_COLOR = (0, 255, 255)

FB_DEVICE = '/dev/fb0'
SYSFS_FB_PATH = '/sys/class/graphics/fb0/'

class SmartFramebuffer:
    def __init__(self, device=FB_DEVICE):
        self.device = device
        self.fb_fd = os.open(self.device, os.O_RDWR)
        self.xres = 0
        self.yres = 0
        self.bits_per_pixel = 0
        self.line_length = 0
        self.visual = 0
        self.byte_order = 'unknown'

        self.detect_screen_info()
        self.map_fb()

        print("Detected: {}x{}, {} bpp, line_length={}".format(self.xres, self.yres, self.bits_per_pixel, self.line_length))
        print("Visual: {}, Byte Order: {}".format(self.visual, self.byte_order))

        # Create off-screen buffer (RAM) for fast drawing
        self.buffer_size = self.line_length * self.yres
        self.screen_buffer = bytearray(self.buffer_size)

        # Precompute grid points once
        self.grid_points = []

    def detect_screen_info(self):
        try:
            self._get_screen_info_ioctl()
            return
        except Exception as e:
            print("Warning: ioctl failed: {}. Trying sysfs fallback...".format(e))

        try:
            self._get_screen_info_sysfs()
            return
        except Exception as e:
            print("Warning: sysfs failed: {}".format(e))

        raise Exception("Could not detect framebuffer info by any method.")

    def _get_screen_info_ioctl(self):
        from fcntl import ioctl

        fix_info = bytearray(128)
        var_info = bytearray(160)

        if ioctl(self.fb_fd, 0x4600, fix_info) == -1:
            raise Exception("FBIOGET_FSCREENINFO failed")

        self.line_length = struct.unpack_from('<I', fix_info, 32)[0]
        self.visual = fix_info[29]

        if ioctl(self.fb_fd, 0x4601, var_info) == -1:
            raise Exception("FBIOGET_VSCREENINFO failed")

        self.xres = struct.unpack_from('<I', var_info, 0)[0]
        self.yres = struct.unpack_from('<I', var_info, 4)[0]
        self.bits_per_pixel = struct.unpack_from('<I', var_info, 28)[0]

        if self.bits_per_pixel in (24, 32):
            self.byte_order = 'BGR'
        else:
            self.byte_order = 'RGB'

    def _get_screen_info_sysfs(self):
        def read_sysfs_file(filename):
            path = os.path.join(SYSFS_FB_PATH, filename)
            with open(path, 'r') as f:
                return f.read().strip()

        virtual_size = read_sysfs_file('virtual_size')
        self.xres, self.yres = map(int, virtual_size.split(','))

        try:
            self.bits_per_pixel = int(read_sysfs_file('bits_per_pixel'))
        except:
            self.bits_per_pixel = 32

        self.line_length = self.xres * ((self.bits_per_pixel + 7) // 8)

        try:
            with open(os.path.join(SYSFS_FB_PATH, 'name'), 'r') as f:
                name = f.read().lower()
                if 'bgr' in name:
                    self.byte_order = 'BGR'
                elif 'rgb' in name:
                    self.byte_order = 'RGB'
                else:
                    self.byte_order = 'BGR'
        except:
            self.byte_order = 'BGR'

        self.visual = 0

    def map_fb(self):
        screen_size = self.line_length * self.yres
        try:
            self.fb_map = mmap.mmap(self.fb_fd, screen_size, mmap.MAP_SHARED, mmap.PROT_WRITE | mmap.PROT_READ, offset=0)
        except Exception as e:
            raise Exception("mmap failed: {}. Screen size={}".format(e, screen_size))

    def set_pixel_in_buffer(self, x, y, color):
        if x < 0 or x >= self.xres or y < 0 or y >= self.yres:
            return

        r, g, b = color
        offset = y * self.line_length

        if self.bits_per_pixel == 16:
            pixel = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
            pixel_offset = offset + x * 2
            if pixel_offset + 2 <= len(self.screen_buffer):
                struct.pack_into('<H', self.screen_buffer, pixel_offset, pixel)
        elif self.bits_per_pixel in (24, 32):
            bytes_per_pixel = 4 if self.bits_per_pixel == 32 else 3
            pixel_offset = offset + x * bytes_per_pixel
            if pixel_offset + bytes_per_pixel > len(self.screen_buffer):
                return

            if self.byte_order == 'BGR':
                pixel_bytes = bytes([b, g, r])
            else:
                pixel_bytes = bytes([r, g, b])

            if self.bits_per_pixel == 32:
                pixel_bytes += b'\x00'

            self.screen_buffer[pixel_offset:pixel_offset+len(pixel_bytes)] = pixel_bytes

    def draw_thick_pixel_in_buffer(self, x, y, color, thickness):
        half = thickness // 2
        start_x = x - half
        start_y = y - half
        for dy in range(thickness):
            for dx in range(thickness):
                px = start_x + dx
                py = start_y + dy
                self.set_pixel_in_buffer(px, py, color)

    def fill_buffer(self, color):
        r, g, b = color

        if self.bits_per_pixel == 16:
            pixel = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
            row_data = struct.pack('<H', pixel) * (self.line_length // 2)
        elif self.bits_per_pixel == 24:
            if self.byte_order == 'BGR':
                row_data = bytes([b, g, r]) * (self.line_length // 3)
            else:
                row_data = bytes([r, g, b]) * (self.line_length // 3)
        else:  # 32-bit
            if self.byte_order == 'BGR':
                row_data = bytes([b, g, r, 0]) * (self.line_length // 4)
            else:
                row_data = bytes([r, g, b, 0]) * (self.line_length // 4)

        for y in range(self.yres):
            offset = y * self.line_length
            end_offset = offset + self.line_length
            if end_offset <= len(self.screen_buffer):
                self.screen_buffer[offset:end_offset] = row_data[:self.line_length]

    def precompute_grid(self, center_x, center_y, scale, max_distance, start_angle, end_angle):
        """Precompute grid for user-defined angular sector."""
        self.grid_points = []
        for ring in range(30, int(max_distance) + 1, 30):
            for theta in range(int(start_angle), int(end_angle) + 1, 2):
                x = int(center_x + ring * scale * math.cos(math.radians(theta)))
                y = int(center_y - ring * scale * math.sin(math.radians(theta)))
                self.grid_points.append((x, y))

    def draw_precomputed_grid(self):
        for x, y in self.grid_points:
            self.draw_thick_pixel_in_buffer(x, y, GRID_COLOR, GRID_THICKNESS)

    def polar_to_cartesian(self, angle_deg, distance, center_x, center_y, scale):
        angle_rad = math.radians(angle_deg)
        x = int(center_x + distance * scale * math.cos(angle_rad))
        y = int(center_y - distance * scale * math.sin(angle_rad))
        return x, y

    def flush(self):
        """Write entire buffer to real framebuffer."""
        self.fb_map.seek(0)
        self.fb_map.write(self.screen_buffer)

    def close(self):
        if hasattr(self, 'fb_map'):
            self.fb_map.close()
        if hasattr(self, 'fb_fd'):
            os.close(self.fb_fd)


def angle_to_duty(angle):
    """Map angle (START_ANGLE to END_ANGLE) to duty cycle."""
    if END_ANGLE == START_ANGLE:
        return MIN_DUTY
    return MIN_DUTY + ((angle - START_ANGLE) / (END_ANGLE - START_ANGLE)) * (MAX_DUTY - MIN_DUTY)

def process_3_bytes(data):
    if len(data) < 3:
        return None
    three_bytes = data[:3]
    combined = (three_bytes[0] << 16) + (three_bytes[1] << 8) + three_bytes[2]
    distance_cm = combined / 10000.0
    if not (1 <= distance_cm <= 500):
        return None
    return distance_cm


def main():
    global pwm, ser

    # Initialize smart framebuffer
    try:
        fb = SmartFramebuffer(FB_DEVICE)

        # ALWAYS BOTTOM CENTER — regardless of sweep angles
        CENTER_X = fb.xres // 2
        CENTER_Y = fb.yres - 10

        # Compute safe scale so beam never goes off-screen within [START_ANGLE, END_ANGLE]
        max_required_height = CENTER_Y  # from bottom to top

        # Compute max horizontal reach needed — check both start and end angle
        cos_start = abs(math.cos(math.radians(START_ANGLE)))
        cos_end = abs(math.cos(math.radians(END_ANGLE)))
        max_cos = max(cos_start, cos_end)

        usable_width = min(CENTER_X, fb.xres - CENTER_X) / max_cos if max_cos > 0 else min(CENTER_X, fb.xres - CENTER_X)

        scale_by_height = max_required_height / MAX_DISTANCE_CM
        scale_by_width = usable_width / MAX_DISTANCE_CM
        SCALE = min(scale_by_height, scale_by_width)

        print("Radar origin: ({}, {}), Scale: {:.3f} px/cm".format(CENTER_X, CENTER_Y, SCALE))

        # Initial clear
        fb.fill_buffer(BG_COLOR)

        # Precompute static grid for configured angular sector
        fb.precompute_grid(CENTER_X, CENTER_Y, SCALE, MAX_DISTANCE_CM, START_ANGLE, END_ANGLE)

    except Exception as e:
        print("Framebuffer init error: {}".format(e))
        sys.exit(1)

    # Initialize serial
    try:
        ser = serial.Serial(
            port=PORT,
            baudrate=BAUDRATE,
            timeout=READ_TIMEOUT,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        print("Connected to ultrasonic sensor on {} @ {} baud".format(PORT, BAUDRATE))
    except Exception as e:
        print("Serial error: {}".format(e), file=sys.stderr)
        try:
            fb.close()
        except:
            pass
        sys.exit(1)

    # Initialize PWM
    try:
        pwm = PWM(PWM_CHANNEL, PWM_PIN)
        pwm.frequency = FREQUENCY
        pwm.polarity = "normal"
        pwm.enable()
        print("Servo initialized.")
    except Exception as e:
        print("PWM error: {}".format(e), file=sys.stderr)
        ser.close()
        try:
            fb.close()
        except:
            pass
        sys.exit(1)

    try:
        echoes = {}
        current_angle = START_ANGLE
        direction = 1

        print("Starting continuous {}° to {}° radar sweep. Press Ctrl+C to stop.".format(START_ANGLE, END_ANGLE))

        while True:
            duty = angle_to_duty(current_angle)
            pwm.duty_cycle = round(duty, 6)
            time.sleep(0.1)  # Let servo settle at position

            ser.write(TRIGGER_BYTE)
            time.sleep(WAIT_AFTER_TRIGGER)

            distance = None
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                distance = process_3_bytes(data)

            # Store valid echo
            if distance is not None:
                if current_angle not in echoes:
                    echoes[current_angle] = []
                echoes[current_angle].append((distance, 1.0))

            # Decay old echoes
            for angle in list(echoes.keys()):
                new_list = []
                for dist, intensity in echoes[angle]:
                    new_intensity = intensity - 1.0 / ECHO_LIFETIME_FRAMES
                    if new_intensity > 0:
                        new_list.append((dist, new_intensity))
                echoes[angle] = new_list
                if len(new_list) == 0:
                    del echoes[angle]

            # --- REDRAW FULL SCENE IN BUFFER ---

            # 1. Clear buffer to background
            fb.fill_buffer(BG_COLOR)

            # 2. Draw static grid
            fb.draw_precomputed_grid()

            # 3. Draw all active echoes (fading ones included)
            for angle, dist_list in echoes.items():
                for dist, intensity in dist_list:
                    if dist > MAX_DISTANCE_CM:
                        continue
                    ex, ey = fb.polar_to_cartesian(angle, dist, CENTER_X, CENTER_Y, SCALE)
                    base_r, base_g, base_b = ECHO_BASE_COLOR
                    echo_color = (
                        int(base_r * intensity),
                        int(base_g * intensity),
                        int(base_b * intensity)
                    )
                    fb.draw_thick_pixel_in_buffer(ex, ey, echo_color, 2)

            # 4. Draw current beam
            display_distance = distance if distance and distance <= MAX_DISTANCE_CM else MAX_DISTANCE_CM
            beam_color = BEAM_COLOR if (distance and distance <= MAX_DISTANCE_CM) else WALL_BEAM_COLOR

            x_end, y_end = fb.polar_to_cartesian(current_angle, display_distance, CENTER_X, CENTER_Y, SCALE)

            # Draw beam line
            dx = x_end - CENTER_X
            dy = y_end - CENTER_Y
            steps = max(abs(dx), abs(dy))
            if steps == 0:
                fb.draw_thick_pixel_in_buffer(CENTER_X, CENTER_Y, beam_color, BEAM_THICKNESS)
            else:
                for i in range(steps + 1):
                    x = CENTER_X + dx * i // steps
                    y = CENTER_Y + dy * i // steps
                    fb.draw_thick_pixel_in_buffer(x, y, beam_color, BEAM_THICKNESS)

            # 5. Flush once per frame
            fb.flush()

            # Update angle, continuous back-and-forth between START_ANGLE and END_ANGLE
            next_angle = current_angle + direction * STEP_SIZE_DEG

            if next_angle >= END_ANGLE:
                next_angle = END_ANGLE
                direction = -1
            elif next_angle <= START_ANGLE:
                next_angle = START_ANGLE
                direction = 1

            current_angle = next_angle
            time.sleep(SWEEP_DELAY_SEC)

    except KeyboardInterrupt:
        print("\nUser interrupted. Cleaning up...")

    except Exception as e:
        print("Unexpected error: {}".format(e))

    finally:
        cleanup_items = []
        if 'ser' in locals() and ser.is_open:
            cleanup_items.append(("Serial port", ser.close))
        if 'pwm' in locals():
            cleanup_items.append(("PWM", pwm.close))
        if 'fb' in locals():
            cleanup_items.append(("Framebuffer", fb.close))

        for name, closer in cleanup_items:
            try:
                closer()
                print("{} closed.".format(name))
            except Exception as e:
                print("Failed to close {}: {}".format(name, e))


if __name__ == '__main__':
    main()
