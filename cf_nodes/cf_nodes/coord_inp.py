"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

# from pycrazyswarm import Crazyswarm
import sys
from crazyflie_py import Crazyswarm


TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0

def parse(tx):
    strings = tx.split(',')
    return float(strings[0]),float(strings[1]),float(strings[2]),float(strings[3]),float(strings[4])

def main():
    swarm = Crazyswarm()
    th = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]
    flying = False
    x,y,z,yaw,t = float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[3]),float(sys.argv[4]),float(sys.argv[5])
    #x,y,z,yaw,t = parse(inp)
    if not flying:
        flying = True
        cf.takeoff(0.5,1.5)
        th.sleep(1.5)
    cf.goTo([x,y,z],yaw,t)
    th.sleep(t)

    #th.sleep(5)
    #cf.land(0.04,5)
    #th.sleep(5)

if __name__ == "__main__":
    main()


