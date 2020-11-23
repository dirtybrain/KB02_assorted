#!usr/bin/env python
__author__ = "X Zhang"
__email__ = "xzhang@westwoodrobotics.net"
__copyright__ = "Copyright 2020 Westwood Robotics Corp."
__date__ = "Feb 14, 2020"

__version__ = "0.0.1"
__status__ = "Prototype"

'''
This program is used to test actual output torque of BEAR and seek peak torque
'''

from pybear import Manager
import time
import os
import math
import select
import sys
import matplotlib.pyplot as plt
import pdb


bear = Manager.BEAR(port="/dev/WR-UB02A", baudrate=8000000)
m_id = 1
bear.set_torque_enable((m_id, 0))
limit_max = math.pi
limit_min = -math.pi
bear.set_limit_position_max((m_id, limit_max))
bear.set_limit_position_min((m_id, limit_min))
bear.set_torque_enable((m_id, 1))
bear.set_torque_enable((m_id, 0))
# pdb.set_trace()
# -----
# Motor setup
# PID idiq
bear.set_p_gain_iq((m_id, 0.02))
bear.set_i_gain_iq((m_id, 0.02))
bear.set_d_gain_iq((m_id, 0))
bear.set_p_gain_id((m_id, 0.02))
bear.set_i_gain_id((m_id, 0.02))
bear.set_d_gain_id((m_id, 0))

# print(bear.get_limit_iq_max(m_id))
bear.set_limit_iq_max((m_id, 50))
print(bear.get_limit_iq_max(m_id))

goal_iq = input("Enter the goal iq and press enter.")
goal_iq = -abs(float(goal_iq))

while True:
    os.system('cls' if os.name == 'nt' else 'clear')
    print("Press any key to start test.")
    # Press any key to stop.
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        line = input()
        break

print("Test started, goal iq is: %f" % goal_iq)
run = True
bear.set_mode((m_id, 0))
bear.set_torque_enable((m_id, 1))
bear.set_goal_iq((m_id, goal_iq))
start_time = time.time()

position_log = []
iq_log = []
winding_temp_log = []
MOS_temp_log = []
ic_temp_log = []
time_log = []
low_iq_count = 0
while run:
    os.system('cls' if os.name == 'nt' else 'clear')
    print(bear.get_goal_iq(m_id))

    status = bear.get_bulk_status((m_id, 'present_position', 'present_iq', 'winding_temperature', 'powerstage_temperature', 'ic_temperature'))
    print(status)
    motor_err = status[0][1]
    position = status[0][0][0]
    iq = status[0][0][1]
    winding_temperature = status[0][0][2]
    MOSFET_temperature = status[0][0][3]
    ic_temperature = status[0][0][4]
    present_time = time.time()-start_time
    if motor_err != 128:
        run = False
        print("BEAR error: %d" % motor_err)
    elif abs(iq-goal_iq) > abs(0.5*goal_iq):
        print(abs(iq-goal_iq))
        if low_iq_count > 200:
            run = False
            print("BEAR iq is 15% different from goal.")
        else:
            low_iq_count += 1
            bear.set_goal_iq((m_id, goal_iq))
    else:
        position_log.append(position)
        iq_log.append(iq)
        winding_temp_log.append(winding_temperature)
        MOS_temp_log.append(MOSFET_temperature)
        ic_temp_log.append(ic_temperature)
        time_log.append(present_time)
        temperatures = [winding_temperature, MOSFET_temperature, ic_temperature]
        device_names = ["Winding", "MOSFET", "IC"]
        hot_item = temperatures.index(max(temperatures))
        print("Present iq is: %f\n" % iq)
        print("Hotest device is: %s, temperature: %f\n" % (device_names[hot_item], temperatures[hot_item]))
        if temperatures[hot_item] > 70:
            print("BEAR too hot, entering damping mode...\n")
            run = False
        else:
            # Press any key to stop.
            print("Press any key to stop.\n")
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                line = input()
                run = False
                break


print("Entering damping mode...\n")
bear.set_limit_position_max((m_id, 0.0))
bear.set_limit_position_min((m_id, 0.0))
# Press any key to release damping mode.
print("Press any key to release...\n")
while True:
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        line = input()
        limit_max = math.pi
        limit_min = -math.pi
        bear.set_limit_position_max((m_id, limit_max))
        bear.set_limit_position_min((m_id, limit_min))
        # Disable
        bear.set_torque_enable((m_id, 0))
        bear.set_torque_enable((m_id, 1))
        bear.set_torque_enable((m_id, 0))
        break

print("Plotting...")

plt.figure()
plt.subplot(311)
plt.plot(time_log, position_log)

plt.subplot(312)
plt.plot(time_log, iq_log)

plt.subplot(313)
plt.plot(time_log, winding_temp_log, 'r', time_log, MOS_temp_log, 'b', time_log, ic_temp_log, 'g')

plt.show()

print("Thanks for using BEAR!")





