#!usr/bin/env python
__author__ = "Xiaoguang Zhang"
__email__ = "xzhang@westwoodrobotics.net"
__copyright__ = "Copyright 2020 Westwood Robotics"
__date__ = "Jan 8, 2020"
__version__ = "0.0.1"
__status__ = "Prototype"

# -----------------------------
# Demo code to swing a bar with dexterous force detection

from pybear import Manager
import time
import os
import math
import select
import sys
import pdb

bear = Manager.BEAR(port="/dev/ttyUSB0", baudrate=8000000)
m_id = 1

# -----
# Motor setup
# PID idiq
bear.set_p_gain_iq((m_id, 0.1))
bear.set_i_gain_iq((m_id, 0.01))
bear.set_d_gain_iq((m_id, 0))
bear.set_p_gain_id((m_id, 0.2))
bear.set_i_gain_id((m_id, 0))
bear.set_d_gain_id((m_id, 0))
# PID vel
bear.set_p_gain_velocity((m_id, 0.5))
bear.set_i_gain_velocity((m_id, 0))
bear.set_d_gain_velocity((m_id, 0))
# PID pos
bear.set_p_gain_position((m_id, 5))
bear.set_i_gain_position((m_id, 0))
bear.set_d_gain_position((m_id, 0.5))
# PID direct force
bear.set_p_gain_force((m_id, 2))
bear.set_i_gain_force((m_id, 0))
bear.set_d_gain_force((m_id, 0.2))
# Limit
bear.set_limit_velocity_max((m_id, 5))
bear.set_limit_iq_max((m_id, 1.5))

# -----
# Motion setup
freq = 0.75  # Motion frequency, should be something between 0.25~1.5
T = 1/freq  # motion loop cycle time
rate = 100  # sinusoid motion trajectory sample rate, 100 should be fine.
INTERVAL = T/rate  # INTERVAL of motion trajectory sample points
i_touch_absolute = 0.35  # Absolute current threshold for touch detection
i_touch_delta = 0.2  # Relative current threshold for touch detection
acc_comp = 0
# -----
# Clear HOMING_OFFSET
bear.set_homing_offset((m_id, 0))
# Check setting
check = False
trial_count = 1
# debug_temp = bear.get_homing_offset(m_id)
while not check:
    try:
        if abs(bear.get_homing_offset(m_id)[0][0]) < 0.01:
            check = True
            print("HOMING_OFFSET cleared. Trails: %d." % trial_count)
        else:
            bear.set_homing_offset((m_id, 0))
            time.sleep(0.05)
            trial_count += 1
    except KeyboardInterrupt:
        check = True
        print("User interrupted.")
# Wait for 1 sec after setting HOMING_OFFSET
# time.sleep(1)

# -----
# Set start and end position
input("Move the bar to the start position and press enter.")
start_pos = bear.get_present_position(m_id)[0][0]
print("start_pos: %2.4f" % start_pos)

input("Move the bar to the end position and press enter.")
end_pos = bear.get_present_position(m_id)[0][0]
print("end_pos: %2.4f" % end_pos)

# Calculation for motion
delta = (end_pos - start_pos) / 2  # motion amplitude
home = (start_pos + end_pos) / 2
start_pos = -delta
end_pos = delta
wave = []
for i in range(rate):
    wave.append(delta*math.sin(i*2*math.pi/rate))
wave.append(delta*math.sin(2*math.pi))
print("Wave with %d points generated." % rate)
print("delta: %2.4f" % delta)
print("home: %2.4f" % home)

# Set middle as home
bear.set_homing_offset((m_id, -home))
# Check setting
check = False
trial_count = 1
# debug_temp = bear.get_homing_offset(m_id)
while not check:
    try:
        if abs(bear.get_homing_offset(m_id)[0][0] + home) < 0.01:
            check = True
            print("HOMING_OFFSET updated. Trails: %d." % trial_count)
        else:
            bear.set_homing_offset((m_id, 0))
            time.sleep(0.05)
            trial_count += 1
    except KeyboardInterrupt:
        check = True
        print("User interrupted.")

# Motor set to position mode
bear.set_mode((m_id, 2))
print("Present position: %2.4f" % bear.get_present_position(m_id)[0][0])
print("Moving to: %2.4f" % start_pos)
input("Press enter to continue...")
# Move to the start position
print("Bar moving to home...")
time.sleep(0.25)
run = True
bear.set_torque_enable((m_id, 1))
bear.set_goal_position((m_id, 0))
time.sleep(1)
while run:
    try:
        vel = bear.get_present_velocity(m_id)[0][0]
        if abs(vel) < 0.1:
            diff = abs(bear.get_present_position(m_id)[0][0])
            print("Seems like motor stopped. Absolute distance to goal: %2.4f" % diff)
            if diff < 0.1:
                print("Seems like motor is at the right place. Disabling now.")
                run = False
                bear.set_torque_enable((m_id, 0))
    except KeyboardInterrupt:
        run = False
        bear.set_torque_enable((m_id, 0))
        print("User interrupted.")

# Ready to start
bear.set_mode((m_id, 3))  # Change to Direct Force mode
input("Ready when you are. Hit ENTER to start.")

# -----
# Motion started
print("Bar moving, press ENTER to stop.")
current_history = []
bear.set_torque_enable((m_id, 1))
run = True
history_len = 20
filter_len = 12
for n in range(history_len):
    current_history.append(bear.get_present_iq(m_id)[0][0])
present_current = sum(current_history[-filter_len:])/filter_len
previous_current = sum(current_history[0:filter_len-1])/filter_len
print("Present current: %2.4f" % present_current)
print("Previous current: %2.4f" % previous_current)

touch = False
steady = False
start_time = time.time()
current_time = time.time()
i = 0
while run:
    try:
        # Here is the motion
        previous_time = current_time
        current_time = time.time()
        if (current_time - start_time) > i*INTERVAL:  # Time for next sample point
            if i == 100:  # End of a cycle
                i = 1
                start_time = current_time
                steady = True
            else:
                i += 1
            bear.set_goal_position((m_id, wave[i]))
        # # Calculate acceleration
        # current_vel =

        # Check for touch
        current_history.append(bear.get_present_iq(m_id)[0][0])
        current_history = current_history[-history_len:]
        present_current = sum(current_history[-filter_len:]) / filter_len
        previous_current = sum(current_history[0:filter_len-1]) / filter_len
        diff = abs(present_current-previous_current)
        os.system('clear')
        print("Present current: %2.4f" % present_current)
        print("Diff: %2.4f" % diff)
        if steady:
            if abs(present_current) > i_touch_absolute:
                # Set to damper if touch detected
                bear.set_p_gain_force((m_id, 0))
                print("Touch detected with ABSOLUTE current. ")
                # pdb.set_trace()
                touch = True
                run = False

            elif diff > i_touch_delta:
                # Set to damper if touch detected
                bear.set_p_gain_force((m_id, 0))
                print("Touch detected with RELEVANT current.")
                # pdb.set_trace()
                touch = True
                run = False

        # Press ENTER to stop.
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = input()
            run = False
    except KeyboardInterrupt:
        run = False
        print("User interrupted.")

print("Motion stopped. Exiting...")
bear.set_torque_enable((m_id, 0))  # Disable the motor.
