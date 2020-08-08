#!usr/bin/env python
__author__ = "Xiaoguang Zhang"
__email__ = "xzhang@westwoodrobotics.net"
__copyright__ = "Copyright 2020 Westwood Robotics"
__date__ = "Jan 8, 2020"
__version__ = "0.0.1"
__status__ = "Prototype"

# -----------------------------
# sample code to ping, read, write, read.

from pybear import Manager


bear = Manager.BEAR(port="/dev/WR-UB0013", baudrate=8000000)




m_id = 1

print("Pinging BEAR 1")
data = bear.ping(m_id)
print(data)

print("Pinging BEAR 2")
data = bear.ping(2)
print(data)

data = bear.get_present_position(m_id, m_id, m_id)
print(data)

