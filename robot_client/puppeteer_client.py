import naoqi
from naoqi import ALProxy
import qi
import motion
import argparse
import sys
import numpy as np
from time import sleep

import redis

from robot_client import robot_redis_client


def main(session_pupp, session_perf, puppeteer_ip, performer_ip):
    robot_client_puppeteer = robot_redis_client(session_pupp, puppeteer_ip, 'puppeteer')
    robot_client_performer = robot_redis_client(session_perf, performer_ip, 'performer')

    robot_client_puppeteer.initialize_subscribers()
    robot_client_performer.initialize_subscribers()

    robot_client_puppeteer.start_to_listen()
    robot_client_performer.start_to_listen()

    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("User forced to quit")
        robot_client_puppeteer.posture_service.goToPosture("Stand", 1.0)
        robot_client_performer.posture_service.goToPosture("Stand", 1.0)
        # robot_client.autonomous_life.setAutonomousAbilityEnabled("All", True)
        pass

    # robot_client.motion_service.setStiffnesses("RArm", 0.0)
    robot_client_puppeteer.stop()
    robot_client_performer.stop()
    print("[Robot Client]: Finish")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip_pupp", type=str, default="10.15.3.171",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--ip_perf", type=str, default="10.15.3.25",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    session_puppeteer = qi.Session()
    session_performer = qi.Session()

    try:
        session_puppeteer.connect("tcp://" + args.ip_pupp + ":" + str(args.port))
        session_performer.connect("tcp://" + args.ip_perf + ":" + str(args.port))
        print("Successfully connected")
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip_pupp + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)

    main(session_puppeteer, session_performer, args.ip_pupp, args.ip_perf)