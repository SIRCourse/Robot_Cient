Robot Client is a Redis-based python2.7 wrapper for Naoqi. It can receive commands sending from your python 3.7 (or higher) script and send these commands to the real robot to control its behaviour (e.g., moving arm, speaking). 

To start the robot client, first get the robot IP address by clicking the button on the chest of the Pepper robot. After you get the IP address (e.g., 192.168.0.164), you can start the robot client by running the command below in your terminal

$ python robot_client.py --ip=192.168.0.164


Some things to notice:
1) Make sure you install Naoqi successfully and are able to import it in your code
2) Make sure you run the command in the environment with python2.7 as the interpreter, since Naoqi only supports python2.7.
3) Make sure you first start the robot client, then run your own control script written in python 3.7 (or higher) 

