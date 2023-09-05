import sys
from crazyflie_py import Crazyswarm

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    z = 1.0
    if len(sys.argv)>1:
        z = float(sys.argv[1])
    cf.takeoff(targetHeight=z, duration=(z+0.5))
    timeHelper.sleep((z+0.5))


if __name__ == "__main__":
    main()

