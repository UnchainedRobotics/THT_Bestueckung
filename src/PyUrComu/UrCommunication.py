import socket
import struct
from . import TimeMeasure
import math
import numpy as np
from multiprocessing import Process, Array, Manager
import json
from scipy.spatial.transform import Rotation as R
import time



class UrRobot:
    realTimePort = 30013
    ProgrammPort = 30003
    JointPositions = [0, 0, 0, 0, 0, 0]  # degree
    JointVelocity = [0, 0, 0, 0, 0, 0]  #
    TcpForce = [0, 0, 0, 0, 0, 0]  # N
    MotorTemp = [0, 0, 0, 0, 0, 0]  # C
    JointCurrents = [0, 0, 0, 0, 0, 0]  # A
    JointVoltage = [0, 0, 0, 0, 0, 0]  # V
    TargetJointMoment = [0, 0, 0, 0, 0, 0]  # Nm
    ActualTcpPos = [0, 0, 0, 0, 0, 0]  # m
    ActualTcpSpeed=[0, 0, 0, 0, 0, 0]  # Actual speed of the tool given in Cartesian coordinates
    ActualTcpForce=[0, 0, 0, 0, 0, 0] #Generalised forces in the TCP
    ActualTcpRPY=[0,0,0] #rad
    SafetyMode=[0]
    SpeedScaling=[0]
    timestamp=[0]
    data_manager=Manager()
    RobotData=data_manager.dict()

    dData = 0
    MSGLEN = 1116
    byteArrayPos_JointPos = 252
    byteArrayPos_JointVelocity = 300
    byteArrayPos_TcpForce = 540
    byteArrayPos_MotorTemp = 692
    byteArrayPos_JointCurrents = 300
    byteArrayPos_JointVoltage = 996
    byteArrayPos_TargetJointMoment = 204
    byteArrayPos_ActualTcpPose = 444
    byteArrayPos_ActualTcpSpeed = 492
    byteArrayPos_ActualTcpForce = 540

    byteArrayPos_SafetyMode = 812
    byteArrayPos_SpeedScaling = 940

    Data = b''
    sharedData = Array('d', np.zeros(6 * 10 + 3))  # [0,0,0,0,0,0,0,0,0,0,0,0]
    stdOnRobotMsg=[]
    Parameters=[]
    recvSocket=None
    timeout=0
    ####### Initials
    def __init__(self, In):
        try:
            self.stdOnRobotMsg=self.readFile('RobotScript/StdOnRobotMsg.txt')
        except:
            print("No StdOnRobotMessage.txt found")
        if('192' in In):
            self.IpAdresse = In
        if('.json' in In):
            with open(In,'r') as fp:
                self.Parameters=json.load(fp)
            self.IpAdresse=self.Parameters['RobotIp']
            self.setRg2FT_Ip(self.Parameters['OnRobotIp'])


    def setRg2FT_Ip(self,IpAdresse):
        self.OnRobotIp=IpAdresse
        if(self.stdOnRobotMsg==[]):
            print("No predefined OnRobotMsg!")
        else:
            self.stdOnRobotMsg=''.join(self.stdOnRobotMsg).replace('*SensorIp*',IpAdresse)

    def closeUrConnection(self):

       self.socket.close()
       self.socketP.close()


    def reconnectUr(self):
        self.closeRealtimePort()
        self.closeRealtimePort()

    def closeRealtimePort(self):
        self.socket.close()

    def connectRealtimePort(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.IpAdresse, self.realTimePort))
        self.socket.settimeout(0)

    def connectUr(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.IpAdresse, self.realTimePort))
        self.socket.settimeout(0.002)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socketP= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socketP.connect((self.IpAdresse, self.ProgrammPort))

    def readData(self):
        msg = b''
        while len(msg) < self.MSGLEN:
            try:

                chunk = self.socket.recv(self.MSGLEN - len(msg))
                self.timeout=0
                msg = msg + chunk
                #print(len(msg),self.RobotData['JointPositions'])
            except socket.error:
                if(self.timeout==0):
                    self.timeout=time.monotonic()
                    #print("here")
                if(time.monotonic()-self.timeout>1 ):
                    self.timeout=0
                    self.socket.close()
                    self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.socket.connect((self.IpAdresse, self.realTimePort))
                    self.socket.settimeout(0.002)
                    self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    print(self.socket)
                    msg=b''



        self.Data = b''
        self.Data = msg
        self.convertData()


    def bla(self):
        msg = b''
        while len(msg) < self.MSGLEN:
            print("running")


            try:
                chunk = self.socket.recv(self.MSGLEN - len(msg))
                if chunk == b'':
                    raise RuntimeError("socket connection lost")
                msg = msg + chunk

                self.Data = b''
                self.Data = msg
                self.convertData()
            except:

                self.connectUr()
                print("Reconnect" + str(self.Counter))
                self.Counter+=1
                msg=b''


    def readrawData(self):
        msg = b''
        while len(msg) < self.MSGLEN:
            chunk = self.socket.recv(self.MSGLEN - len(msg))
            if chunk == b'':
                raise RuntimeError("socket connection lost")
            msg = msg + chunk
        self.Data = b''
        self.Data = msg

    def splitByteArray(self, lower, upper):
        temp = bytearray(self.Data[lower:upper])
        temp.reverse()
        return temp

    def convertData(self):
        #sT=time.monotonic()

        try:
            for i in range(6):
                # Joint Positions
                temp = self.splitByteArray(self.byteArrayPos_JointPos + 8 * i, self.byteArrayPos_JointPos + 8 * (i + 1))
                self.JointPositions[i] = math.degrees(np.frombuffer(temp))
                # Joint Velocity
                temp = self.splitByteArray(self.byteArrayPos_JointVelocity + 8 * i,
                                           self.byteArrayPos_JointVelocity + 8 * (i + 1))
                self.JointVelocity[i] =  math.degrees(np.frombuffer(temp))

                # TcpForce
                temp = self.splitByteArray(self.byteArrayPos_TcpForce + 8 * i, self.byteArrayPos_TcpForce + 8 * (i + 1))
                self.TcpForce[i] = np.frombuffer(temp)

                # Motor Temperatur
                temp = self.splitByteArray(self.byteArrayPos_MotorTemp + 8 * i, self.byteArrayPos_MotorTemp + 8 * (i + 1))
                self.MotorTemp[i] = np.frombuffer(temp)

                # Actual Joint Currents
                temp = self.splitByteArray(self.byteArrayPos_JointCurrents + 8 * i,
                                           self.byteArrayPos_JointCurrents + 8 * (i + 1))
                self.JointCurrents[i] = np.frombuffer(temp)

                # Actual Joint Voltage
                temp = self.splitByteArray(self.byteArrayPos_JointVoltage + 8 * i,
                                           self.byteArrayPos_JointVoltage + 8 * (i + 1))
                self.JointVoltage[i] = np.frombuffer(temp)

                # Target Joint Moment
                temp = self.splitByteArray(self.byteArrayPos_TargetJointMoment + 8 * i,
                                           self.byteArrayPos_TargetJointMoment + 8 * (i + 1))
                self.TargetJointMoment[i] = np.frombuffer(temp)

                # ActualTcpPos
                temp = self.splitByteArray(self.byteArrayPos_ActualTcpPose + 8 * i,
                                           self.byteArrayPos_ActualTcpPose + 8 * (i + 1))
                self.ActualTcpPos[i] = np.frombuffer(temp)

                # ActualTcpForce
                temp = self.splitByteArray(self.byteArrayPos_ActualTcpForce + 8 * i,
                                           self.byteArrayPos_ActualTcpForce+ 8 * (i + 1))
                self.ActualTcpForce[i] = np.frombuffer(temp)

                # ActualTcpSpeed
                temp = self.splitByteArray(self.byteArrayPos_ActualTcpSpeed + 8 * i,
                                           self.byteArrayPos_ActualTcpSpeed+ 8 * (i + 1))
                self.ActualTcpSpeed[i] = np.frombuffer(temp)

            #safetymode
            temp = self.splitByteArray(self.byteArrayPos_SafetyMode,self.byteArrayPos_SafetyMode+8)
            self.SafetyMode[0] = np.frombuffer(temp)

            #SpeedScaling
            temp = self.splitByteArray(self.byteArrayPos_SpeedScaling,self.byteArrayPos_SpeedScaling+8)
            self.SpeedScaling[0] = np.frombuffer(temp)

        except:
            print("empty Bytearray")


        #print("Time used for loop= " +str(time.monotonic()-sT))
    def convertSafetyMode(self,value):
        statements={
            0:'Waiting',
            1:'Normal',
            2:'Reduced',
            3:'Protective Stop',
            4:'Recovery',
            5:'Safeguard Stop',
            6:'System Emergency Stop',
            7:'Robot Emergency Stop',
            8:'Violation',
            9:'Fault',
            10:'Validate Joint ID',
            11:'Undefined Safety Mode'
        }
        return statements[int(value)]


    def convertStringList(self, List):
        TempPosStr = ""
        for elemetns in range(len(List)):
            if elemetns < len(List) - 1:
                TempPosStr += List[elemetns] + ","
            else:
                TempPosStr += List[elemetns]
        return TempPosStr

    def startUpdateProcess(self,str):
        if(str=='raw'):
            self.p = Process(target=self.UpdaterRaw, args=(self.sharedData, True))
            self.p.start()
        if(str=='data'):
            self.p = Process(target=self.Updater, args=(self.sharedData, True))
            self.p.start()
        if(str=='data_dict'):
            self.p = Process(target=self.Updater2, args=(True,self.RobotData))
            self.p.start()
    def stopUpdateProzess(self):
        self.p.terminate()

    def Updater(self, n, Continuously):
        run = True
        while (run):
            #print("running1")
            self.readData()
            #print("running2")
            n[:6]=self.JointPositions
            n[6:12]=self.JointVelocity
            n[12:18]=self.TcpForce
            n[18:24]=self.MotorTemp
            n[24:30]=self.JointCurrents
            n[30:36]=self.JointVoltage
            n[36:42]=self.TargetJointMoment
            n[42:48]=self.ActualTcpPos
            n[48]=self.SafetyMode[0]
            n[49]=self.SpeedScaling[0]
            n[50:56]=self.ActualTcpForce
            n[56:62]=self.ActualTcpSpeed
            n[62]=time.monotonic()

            #print("running")

            run = Continuously
    def Updater2(self, Continuously, RobotData):
        run = True
        while (run):
            self.readData()
            RobotData['JointPositions']=self.JointPositions
            RobotData['JointVelocity']=self.JointVelocity
            RobotData['TcpForce']=self.TcpForce
            RobotData['MotorTemp']=self.MotorTemp
            RobotData['JointCurrents']=self.JointCurrents
            RobotData['JointVoltage']=self.JointVoltage
            RobotData['TargetJointMoment']=self.TargetJointMoment
            RobotData['ActualTcpPos']=self.ActualTcpPos
            RobotData['SafetyMode']=self.SafetyMode
            RobotData['SpeedScaling']=self.SpeedScaling
            RobotData['ActualTcpForce']=self.ActualTcpForce
            RobotData['ActualTcpSpeed']=self.ActualTcpSpeed
            RobotData['timestamp']=time.monotonic()
            #print("running")

        run = Continuously
    def UpdaterRaw(self, n, Continuously):
        run = True
        while (run):
            self.readrawData()
            run = Continuously

    def formatSharedData(self):
        #print(self.sharedData[:6])
        for i in range(6):
            self.JointPositions[i] = self.sharedData[i]
            self.JointVelocity[i] = self.sharedData[i + 6]
            self.TcpForce[i] = self.sharedData[i + 12]
            self.MotorTemp[i] = self.sharedData[i + 18]
            self.JointCurrents[i] = self.sharedData[i + 24]
            self.JointVoltage[i] = self.sharedData[i + 30]
            self.TargetJointMoment[i] = self.sharedData[i + 36]
            self.ActualTcpPos[i] = self.sharedData[i + 42]
            self.ActualTcpSpeed[i]=self.sharedData[i+56]
            self.ActualTcpForce[i]=self.sharedData[i+50]
        self.SafetyMode=self.convertSafetyMode(self.sharedData[48])
        self.SpeedScaling=self.sharedData[49]
        self.timestamp=self.sharedData[62]

    def FormatRotvectoRPY(self,rotvec):
        r = R.from_rotvec(rotvec)
        self.ActualTcpRPY=r.as_euler('xyz')

    @TimeMeasure.getTimeStamp
    def readDataWithTime(self):
        self.formatSharedData()
        Data=[self.JointPositions,self.JointVelocity,self.TcpForce,
              self.MotorTemp,self.JointCurrents,self.JointVoltage,
              self.TargetJointMoment,self.ActualTcpPos,self.SafetyMode,
              self.SpeedScaling,self.ActualTcpSpeed,self.ActualTcpForce]
        return self.ActualTcpPos

    def DataWithTime(self,startTime):
        out=self.readDataWithTime()
        dur=out[0]-startTime
        out[0]=dur
        return out

    ######Program Sender
    def sendUrProgram(self, commands):
        completeProgram = "def completeProgram():\n"
        for i in range(len(commands)):
            completeProgram += " " + commands[i] + "\n"
        completeProgram += "end\n"
        #print(completeProgram)
        self.socketP.send(completeProgram.encode())

    def sendUrProgram2(self,Program,usedOnRobotGripper):
        if(usedOnRobotGripper):
            Program=self.stdOnRobotMsg+Program
        temp="def completeProgram():\n"
        temp2="end\n"
        completeProgram=temp+Program+temp2
        #print(completeProgram)
        self.socketP.send(completeProgram.encode())

    def sendCompleteUrProgram(self,Path):
        completeProgram=self.readFile(Path)
        completeProgram=completeProgram
        #print(completeProgram)
        self.socketP.send(completeProgram.encode())

    def sendRobotScriptFile(self,Path,usedOnRobotGripper):
        self.recvSocket=None
        ScriptProgram=self.readFile(Path)
        ScriptProgram=self.replaceStd_SockMessaging(ScriptProgram)
        self.sendUrProgram2(ScriptProgram,usedOnRobotGripper)


    ####RobotMover

    def sendMoveL(self,Pos,Tcp,v,a,r):
        self.recvSocket=None
        ScriptProgram=self.readFile('RobotScript/MoveL.txt')
        ScriptProgram=self.ConvertMoveScript(ScriptProgram,Pos,Tcp,v,a,r)
        self.sendUrProgram2(ScriptProgram,False)

    def sendMoveJ(self,Pos,Tcp,v,a,r):
        self.recvSocket=None
        ScriptProgram=self.readFile('RobotScript/MoveJ.txt')
        ScriptProgram=self.ConvertMoveScript(ScriptProgram,Pos,Tcp,v,a,r)
        self.sendUrProgram2(ScriptProgram,False)

    def OnRobot_width(self,width,force,depth_comp):
        self.recvSocket=None
        ScriptProgram=self.readFile('RobotScript/OnRobot_width.txt')
        ScriptProgram=self.replaceStd_SockMessaging(ScriptProgram)
        ScriptProgram=ScriptProgram.replace('*width*',str(width)).replace('*force*',str(force)).replace('*depth_comp*',str(depth_comp))
        self.sendUrProgram2(ScriptProgram,True)

    #######Convert Rovot Script Funktions
    def replaceStd_SockMessaging(self,Str):
        if(self.Parameters!=[]):
            Str=Str.replace('*HostIp*','"'+self.Parameters['HostIp']+'"')
            Str=Str.replace('*HostPort*',str(self.Parameters['HostPort']))
        else:
            print("No Parameters.json loaded!")
        return Str

    def replaceTcp(self,Str,Tcp):
        Str=Str.replace('*Tcp[0]*',str(Tcp[0])).replace('*Tcp[1]*',str(Tcp[1])).replace('*Tcp[2]*',str(Tcp[2]))
        return Str

    def replacePos(self,Str,Pos):
        Str=Str.replace('*rotvec[0]*',str(Pos[3])).replace('*rotvec[1]*',str(Pos[4])).replace('*rotvec[2]*',str(Pos[5]))
        Str=Str.replace('*Pos[0]*',str(Pos[0])).replace('*Pos[1]*',str(Pos[1])).replace('*Pos[2]*',str(Pos[2]))
        return Str

    def ConvertMoveScript(self,Str,Pos,Tcp,v,a,r):
        ScriptProgram=self.replaceStd_SockMessaging(Str)
        ScriptProgram=self.replaceTcp(ScriptProgram,Tcp)
        ScriptProgram=self.replacePos(ScriptProgram,Pos)
        ScriptProgram=ScriptProgram.replace('*v*',str(v)).replace('*a*',str(a)).replace('*r*',str(r))
        return ScriptProgram

    #######Helper
    def readFile(self,Path):
        f=open(Path)
        file=f.readlines()
        f.close()
        file=''.join(file)
        return file

    def waitForMessage(self):
        if(self.recvSocket==None):
            self.recvSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.recvSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.recvSocket.bind(('', self.Parameters['HostPort']))
            self.recvSocket.listen(5)
            self.conn, self.addr = self.recvSocket.accept()
        recvData = self.conn.recv(1024)
        print(recvData)
        return recvData
