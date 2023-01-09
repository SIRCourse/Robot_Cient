import naoqi
from naoqi import ALProxy
import qi
import motion
import argparse
import sys
import numpy as np
from time import sleep

import redis

class robot_redis_client:
    def __init__(self, session, ip='192.168.0.121', role=None):
        self.role = role

        # self.autonomous_life = session.service("ALAutonomousLife")
        self.autonomous_life = ALProxy("ALAutonomousLife", ip, 9559)
        # self.tts = session.service("ALTextToSpeech")
        self.tts = ALProxy("ALTextToSpeech", ip, 9559)
        # self.motion_service = session.service("ALMotion")
        self.motion_service = ALProxy("ALMotion", ip, 9559)
        # self.posture_service = session.service("ALRobotPosture")
        self.posture_service = ALProxy("ALRobotPosture", ip, 9559)

        self.leds_service = ALProxy("ALLeds", ip, 9559)
        # self.leds_service = session.service("ALLeds")
        self.led_names = ['RightEarLeds', 'RightEarLedsFront', 'RightEarLedsBack','ChestLeds']
        self.leds_service.off('EarLeds')
        self.leds_service.off('ChestLeds')

        self.autonomous_life.setAutonomousAbilityEnabled("All", False)
        if not self.role == 'puppeteer':
            self.autonomous_life.setAutonomousAbilityEnabled("BasicAwareness", True)
        self.motion_service.setExternalCollisionProtectionEnabled('Arms', False)
        self.posture_service.goToPosture("Stand", 0.5)
        self.motion_service.closeHand("RHand")

        # joint control parameters
        # self.motion_service.setStiffnesses("Head", 0.0)
        self.motion_service.setStiffnesses("RArm", 1.0)
        # self.joint_names = ["KneePitch", "HipPitch", "HipRoll", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll"]
        self.joint_names = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"]
        # self.joint_names_whole_body = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "KneePitch", "HipPitch", "HipRoll"]
        self.joint_names_whole_body = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "HipRoll"]
        self.chain_name = "RArm"
        self.frame = motion.FRAME_TORSO
        self.fractionMaxSpeed = 0.25
        self.target_position = [0.3, -0.2, 0.8]
        self.z_offset = 0.82

        self.message_id = 0
        self.r = redis.Redis(host='localhost', port=6379, db=0)

        if self.role is None:
            self.sub_say = self.r.pubsub()
            self.sub_joint = self.r.pubsub()
            self.sub_cartesian = self.r.pubsub()
            self.sub_retrieve_data = self.r.pubsub()
            self.sub_go_home_pose = self.r.pubsub()
            self.sub_set_stiffness = self.r.pubsub()
            self.sub_set_led = self.r.pubsub()
            self.sub_retrieve_hand_pose = self.r.pubsub()
            self.sub_set_stiffness_whole_body = self.r.pubsub()
            self.sub_retrieve_data_whole_body = self.r.pubsub()
            self.sub_joint_wrist = self.r.pubsub()
        else:
            if self.role == 'puppeteer':
                self.sub_say = self.r.pubsub()
                self.sub_retrieve_data = self.r.pubsub()
                self.sub_go_home_pose = self.r.pubsub()
                self.sub_set_stiffness = self.r.pubsub()
                self.sub_set_led = self.r.pubsub()
                self.sub_retrieve_hand_pose = self.r.pubsub()
                self.sub_set_stiffness_whole_body = self.r.pubsub()
                self.sub_retrieve_data_whole_body = self.r.pubsub()
            else:
                self.sub_say = self.r.pubsub()
                self.sub_joint = self.r.pubsub()
                self.sub_go_home_pose = self.r.pubsub()
                self.sub_set_stiffness = self.r.pubsub()
                self.sub_retrieve_data = self.r.pubsub()
                self.sub_retrieve_hand_pose = self.r.pubsub()
                self.sub_retrieve_data_whole_body = self.r.pubsub()
                self.sub_joint_wrist = self.r.pubsub()

        self.listening_thread_say = None
        self.listening_thread_joint = None
        self.listening_thread_cartesian = None
        self.listening_thread_retrieve_data = None
        self.listening_thread_go_home_pose = None
        self.listening_thread_set_stiffness = None
        self.listening_thread_set_led = None
        self.listening_thread_retrieve_hand_pose = None
        self.listening_thread_set_stiffness_whole_body = None
        self.listening_thread_retrieve_data_whole_body = None
        self.listening_thread_joint_wrist = None

    def initialize_subscribers(self):
        if self.role is None:
            self.sub_say.subscribe(**{'say': self.say_message_handler})
            self.sub_joint.subscribe(**{'target_joint_values': self.joint_message_handler})
            self.sub_cartesian.subscribe(**{'target_hand_pose': self.cartesian_message_handler})
            self.sub_retrieve_data.subscribe(**{'retrieve_joint_values': self.retrieve_data_message_handler})
            self.sub_go_home_pose.subscribe(**{'go_home_pose': self.go_home_pose_message_handler})
            self.sub_set_stiffness.subscribe(**{'set_stiffness': self.set_stiffness_message_handler})
            self.sub_set_led.subscribe(**{'set_led': self.set_led_message_handler})
            self.sub_retrieve_hand_pose.subscribe(**{'retrieve_hand_pose': self.retrieve_hand_pose_message_handler})
            self.sub_set_stiffness_whole_body.subscribe(**{'set_stiffness_whole_body': self.set_stiffness_whole_body_message_handler})
            self.sub_retrieve_data_whole_body.subscribe(**{'retrieve_joint_values_whole_body': self.retrieve_data_whole_body_message_handler})
            self.sub_joint_wrist.subscribe(**{'target_joint_values_wrist': self.joint_wrist_message_handler})
        else:
            if self.role == 'puppeteer':
                self.sub_say.subscribe(**{'puppeteer_say': self.say_message_handler})
                self.sub_retrieve_data.subscribe(**{'puppeteer_retrieve_joint_values': self.retrieve_data_message_handler})
                self.sub_go_home_pose.subscribe(**{'puppeteer_go_home_pose': self.go_home_pose_message_handler})
                self.sub_set_stiffness.subscribe(**{'puppeteer_set_stiffness': self.set_stiffness_message_handler})
                self.sub_set_led.subscribe(**{'puppeteer_set_led': self.set_led_message_handler})
                self.sub_retrieve_hand_pose.subscribe(**{'puppeteer_retrieve_hand_pose': self.retrieve_hand_pose_message_handler})
                self.sub_set_stiffness_whole_body.subscribe(**{'puppeteer_set_stiffness_whole_body': self.set_stiffness_whole_body_message_handler})
                self.sub_retrieve_data_whole_body.subscribe(**{'puppeteer_retrieve_joint_values_whole_body': self.retrieve_data_whole_body_message_handler})
            else:
                self.sub_say.subscribe(**{'performer_say': self.say_message_handler})
                self.sub_joint.subscribe(**{'performer_target_joint_values': self.joint_message_handler})
                self.sub_go_home_pose.subscribe(**{'performer_go_home_pose': self.go_home_pose_message_handler})
                self.sub_set_stiffness.subscribe(**{'performer_set_stiffness': self.set_stiffness_message_handler})
                self.sub_retrieve_data.subscribe(**{'performer_retrieve_joint_values': self.retrieve_data_message_handler})
                self.sub_retrieve_hand_pose.subscribe(**{'performer_retrieve_hand_pose': self.retrieve_hand_pose_message_handler})
                self.sub_retrieve_data_whole_body.subscribe(**{'performer_retrieve_joint_values_whole_body': self.retrieve_data_whole_body_message_handler})
                self.sub_joint_wrist.subscribe(**{'performer_target_joint_values_wrist': self.joint_wrist_message_handler})

    def start_to_listen(self):
        if self.role is None:
            self.listening_thread_say = self.sub_say.run_in_thread(sleep_time=0.01)
            self.listening_thread_joint = self.sub_joint.run_in_thread(sleep_time=0.01)
            self.listening_thread_cartesian = self.sub_cartesian.run_in_thread(sleep_time=2.0)
            self.listening_thread_retrieve_data = self.sub_retrieve_data.run_in_thread(sleep_time=0.01)
            self.listening_thread_go_home_pose = self.sub_go_home_pose.run_in_thread(sleep_time=0.01)
            self.listening_thread_set_stiffness = self.sub_set_stiffness.run_in_thread(sleep_time=0.01)
            self.listening_thread_set_led = self.sub_set_led.run_in_thread(sleep_time=0.01)
            self.listening_thread_retrieve_hand_pose = self.sub_retrieve_hand_pose.run_in_thread(sleep_time=0.01)
            self.listening_thread_set_stiffness_whole_body = self.sub_set_stiffness_whole_body.run_in_thread(sleep_time=0.01)
            self.listening_thread_retrieve_data_whole_body = self.sub_retrieve_data_whole_body.run_in_thread(sleep_time=0.01)
            self.listening_thread_joint_wrist = self.sub_joint_wrist.run_in_thread(sleep_time=0.01)
        else:
            if self.role == 'puppeteer':
                self.listening_thread_say = self.sub_say.run_in_thread(sleep_time=0.01)
                self.listening_thread_retrieve_data = self.sub_retrieve_data.run_in_thread(sleep_time=0.01)
                self.listening_thread_go_home_pose = self.sub_go_home_pose.run_in_thread(sleep_time=0.01)
                self.listening_thread_set_stiffness = self.sub_set_stiffness.run_in_thread(sleep_time=0.01)
                self.listening_thread_set_led = self.sub_set_led.run_in_thread(sleep_time=0.01)
                self.listening_thread_retrieve_hand_pose = self.sub_retrieve_hand_pose.run_in_thread(sleep_time=0.01)
                self.listening_thread_set_stiffness_whole_body = self.sub_set_stiffness_whole_body.run_in_thread(sleep_time=0.01)
                self.listening_thread_retrieve_data_whole_body = self.sub_retrieve_data_whole_body.run_in_thread(sleep_time=0.01)
            else:
                self.listening_thread_say = self.sub_say.run_in_thread(sleep_time=0.01)
                self.listening_thread_joint = self.sub_joint.run_in_thread(sleep_time=0.01)
                self.listening_thread_go_home_pose = self.sub_go_home_pose.run_in_thread(sleep_time=0.01)
                self.listening_thread_set_stiffness = self.sub_set_stiffness.run_in_thread(sleep_time=0.01)
                self.listening_thread_retrieve_data = self.sub_retrieve_data.run_in_thread(sleep_time=0.01)
                self.listening_thread_retrieve_hand_pose = self.sub_retrieve_hand_pose.run_in_thread(sleep_time=0.01)
                self.listening_thread_retrieve_data_whole_body = self.sub_retrieve_data_whole_body.run_in_thread(sleep_time=0.01)
                self.listening_thread_joint_wrist = self.sub_joint_wrist.run_in_thread(sleep_time=0.01)

    def stop(self):
        if self.role is None:
            self.listening_thread_say.stop()
            self.listening_thread_joint.stop()
            self.listening_thread_cartesian.stop()
            self.listening_thread_retrieve_data.stop()
            self.listening_thread_go_home_pose.stop()
            self.listening_thread_set_stiffness.stop()
            self.listening_thread_set_led.stop()
            self.listening_thread_retrieve_hand_pose.stop()
            self.listening_thread_set_stiffness_whole_body.stop()
            self.listening_thread_retrieve_data_whole_body.stop()
            self.listening_thread_joint_wrist.stop()
        else:
            if self.role == 'puppeteer':
                self.listening_thread_say.stop()
                self.listening_thread_retrieve_data.stop()
                self.listening_thread_go_home_pose.stop()
                self.listening_thread_set_stiffness.stop()
                self.listening_thread_set_led.stop()
                self.listening_thread_retrieve_hand_pose.stop()
                self.listening_thread_set_stiffness_whole_body.stop()
                self.listening_thread_retrieve_data_whole_body.stop()
            else:
                self.listening_thread_say.stop()
                self.listening_thread_joint.stop()
                self.listening_thread_go_home_pose.stop()
                self.listening_thread_set_stiffness.stop()
                self.listening_thread_retrieve_data.stop()
                self.listening_thread_retrieve_hand_pose.stop()
                self.listening_thread_retrieve_data_whole_body.stop()
                self.listening_thread_joint_wrist.stop()

    ''' Blocking Handlers '''
    def say_message_handler(self, message):
        print("[Robot Client]: Receive say command data: {}".format(message['data']))
        id = self.tts.post.say(message['data']) # non-blocking function
        self.tts.wait(id, 0)

        if self.role is None:
            self.r.publish('finished_say', str(0))
        else:
            self.r.publish(self.role + '_' + 'finished_say', str(0))
        # self.message_id += 1

    def go_home_pose_message_handler(self, message):
        print("[Robot Client]: Receive command to go to home posture")
        id = self.posture_service.post.goToPosture("Stand", 0.5)
        self.posture_service.wait(id, 0)
        id = self.motion_service.post.closeHand("RHand")
        self.motion_service.wait(id, 0)

        if self.role is None:
            self.r.publish('finished_go_home_pose', str(0))
        else:
            self.r.publish(self.role + '_' + 'finished_go_home_pose', str(0))

    def set_stiffness_message_handler(self, message):
        print("[Robot Client]: Receive command to set stiffness to {}".format(message['data']))
        # self.motion_service.killTasksUsingResources([self.chain_name])
        # print("Killed motion tasks before setting stiffness")
        id = self.motion_service.post.setStiffnesses("RArm", float(message['data']))
        self.motion_service.wait(id, 0)

        if self.role is None:
            self.r.publish('finished_set_stiffness', str(0))
        else:
            self.r.publish(self.role + '_' + 'finished_set_stiffness', str(0))

    def set_stiffness_whole_body_message_handler(self, message):
        print("[Robot Client]: Receive command to set stiffness whole body to {}".format(message['data']))

        stiffness = float(message['data'])
        if stiffness > 0.0:
            id = self.motion_service.post.wakeUp()
            self.motion_service.wait(id, 0)
        else:
            id = self.motion_service.post.rest()
            self.motion_service.wait(id, 0)

        if self.role is None:
            self.r.publish('finished_set_stiffness_whole_body', str(0))
        else:
            self.r.publish(self.role + '_' + 'finished_set_stiffness_whole_body', str(0))

    def set_led_message_handler(self, message):
        original_msg = [float(i) for i in message['data'].split()]
        led_id = int(original_msg[0])
        intensity = original_msg[1]
        print("[Robot Client]: Receive command to set {} led to {}".format(self.led_names[led_id], intensity))

        if intensity > 0.0:
            id = self.leds_service.post.on(self.led_names[led_id])
        else:
            id = self.leds_service.post.off(self.led_names[led_id])
        self.leds_service.wait(id, 0)

        if self.role is None:
            self.r.publish('finished_set_led', str(0))
        else:
            self.r.publish(self.role + '_' + 'finished_set_led', str(0))

    ''' Non-blocking Handlers'''
    def joint_message_handler(self, message):
        print("[Robot Client]: Receive joint command data: {}".format(message['data']))

        # Send joint command
        original_msg = [float(i) for i in message['data'].split()] # last element is the target velocity factor
        # joint_values = [float(i) for i in message['data'].split()]
        joint_values = original_msg[:-1]
        speed_factor = original_msg[-1]
        if speed_factor < 0:
            speed_factor = self.fractionMaxSpeed

        for i in range(len(joint_values)):
            self.motion_service.setAngles(self.joint_names_whole_body[i], joint_values[i], speed_factor)
        # self.motion_service.setAngles(self.joint_names, joint_values, self.fractionMaxSpeed)
        # self.message_id += 1
        print("[Robot Client]: Finish sending joint command")

        # self.r.publish('finished_take_action', str(0))
        # # Get real cartesian position of end-effector in world frame
        # real_pose = self.motion_service.getTransform("RArm", motion.FRAME_TORSO, True)
        # real_position = []
        # for i in range(3):
        #     real_position.append(real_pose[i*4 + 3])
        # real_position[2] = real_position[2] + self.z_offset
        #
        # # Calculate difference between target and real position
        # diff = np.array([real_position[0] - self.target_position[0],
        #                  real_position[1] - self.target_position[1],
        #                  real_position[2] - self.target_position[2]])
        # diff_value = np.linalg.norm(diff)
        #
        # print("[Robot Client]: Real position is: {}".format(real_position))
        # print("[Robot Client]: Target position is: {}".format(self.target_position))
        # print("[Robot Client]: Error distance is: {}".format(diff_value))
        # print("********************************************")

    def joint_wrist_message_handler(self, message):
        print("[Robot Client]: Receive joint wrist command data: {}".format(message['data']))

        wrist_angle = float(message['data'])
        self.motion_service.setAngles('RWristYaw', wrist_angle, 0.2)

        print("[Robot Client]: Finish sending joint wrist command")

    def retrieve_data_message_handler(self, message):
        print("[Robot Client]: Receive command to retrieve robot data")
        joint_values = ''
        # joint_angles = self.motion_service.getAngles(self.chain_name, True)
        # for i in range(len(self.joint_names)):
        #     joint_values += str(joint_angles[i]) + ' '

        for joint in self.joint_names:
            joint_angle = self.motion_service.getAngles(joint, True)[0]
            joint_values += str(joint_angle) + ' '

        # print("latest_joint_values: " + joint_values)
        if self.role is None:
            self.r.publish('latest_joint_values', joint_values)
        else:
            self.r.publish(self.role + '_' + 'latest_joint_values', joint_values)

    def retrieve_data_whole_body_message_handler(self, message):
        print("[Robot Client]: Receive command to retrieve robot data whole body")
        joint_values = ''

        for joint in self.joint_names_whole_body:
            joint_angle = self.motion_service.getAngles(joint, True)[0]
            joint_values += str(joint_angle) + ' '

        if self.role is None:
            self.r.publish('latest_joint_values_whole_body', joint_values)
        else:
            self.r.publish(self.role + '_' + 'latest_joint_values_whole_body', joint_values)

    def retrieve_hand_pose_message_handler(self, message):
        print("[Robot Client]: Receive command to retrieve hand position")
        hand_pose = ''

        pose = self.motion_service.getPosition("RHand", 1, True)
        for i in range(6):
            coordinate = pose[i]
            hand_pose += str(coordinate) + ' '

        if self.role is None:
            self.r.publish('latest_hand_pose', hand_pose)
        else:
            self.r.publish(self.role + '_' + 'latest_hand_pose', hand_pose)



    def cartesian_message_handler(self, message):
        print("[Robot Client]: Receive cartesian command data: {}".format(message['data']))
        target_pos = [float(i) for i in message['data'].split()]
        target_ori = [0.0, 0.0, 0.0]
        target_pose = target_pos + target_ori
        axisMask = 7 # for position-control only

        self.motion_service.setPositions(self.chain_name, self.frame, target_pose, self.fractionMaxSpeed, axisMask)
        print("[Robot Client]: Finish sending cartesian command")


def main(session, ip):
    robot_client = robot_redis_client(session, ip)
    robot_client.initialize_subscribers()
    robot_client.start_to_listen()
    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("User forced to quit")
        robot_client.posture_service.goToPosture("Stand", 0.5)
        # robot_client.autonomous_life.setAutonomousAbilityEnabled("All", True)
        pass

    # robot_client.motion_service.setStiffnesses("RArm", 0.0)
    robot_client.stop()
    print("[Robot Client]: Finish")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="10.15.3.171",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
        print("Successfully connected")
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    main(session, args.ip)
