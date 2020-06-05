import os
import select
import sys
import pdb

i = 0
while True:
    os.system('cls' if os.name == 'nt' else 'clear')
    print("I'm doing stuff. Press Enter to stop me!")
    print(i)
    # Press any key to stop.
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        line = input()
        break
    i += 1
