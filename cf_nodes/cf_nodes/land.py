"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

# from pycrazyswarm import Crazyswarm
import sys
from crazyflie_py import Crazyswarm

def main():
    swarm = Crazyswarm()
    th = swarm.timeHelper
    cfs = swarm.allcfs
    cfs.land(0.04,5)
    th.sleep(5)

if __name__ == "__main__":
    main()

