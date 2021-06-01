import UrCommunication
import time
import socket
import statistics

#ur = UrCommunication.UrRobot("192.168.2.141")
ur = UrCommunication.UrRobot('../Ini/Parameters.json')
ur.connectUr()

ur.sendRobotScriptFile('../RobotScript/GripCap1.txt',usedOnRobotGripper=True)
ur.waitForMessage()

ur.waitForMessage()

