import socket
import struct
import array
import math
import xlrd
import xlsxwriter
from time import sleep
from termcolor import colored
from colorama import Fore, init
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation as animation
import datetime as dt
# import cv2
from multiprocessing import Process, Queue, Value, Array
import TimeMeasure
import time
from scipy import signal


class UrRobot:
    realTimePort = 30003
    JointPositions = [0, 0, 0, 0, 0, 0]  # degree
    JointVelocity = [0, 0, 0, 0, 0, 0]  #
    TcpForce = [0, 0, 0, 0, 0, 0]  # N
    MotorTemp = [0, 0, 0, 0, 0, 0]  # C
    JointCurrents = [0, 0, 0, 0, 0, 0]  # A
    JointVoltage = [0, 0, 0, 0, 0, 0]  # V
    TargetJointMoment = [0, 0, 0, 0, 0, 0]  # Nm
    ActualTcpPos = [0, 0, 0, 0, 0, 0]  # m
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
    Data = b''
    sharedData = Array('d', np.zeros(6 * 8))  # [0,0,0,0,0,0,0,0,0,0,0,0]
    stdOnRobotMsg=[]

    def __init__(self, IpAdresse):
        self.IpAdresse = IpAdresse
        self.socket = None
        try:
            f=open('../RobotScript/StdOnRobotMsg.txt')
            self.stdOnRobotMsg=f.readlines()
            f.close()
        except:
            print("No StdOnRobotMessage.txt found")

    def connectUr(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.IpAdresse, self.realTimePort))

    def readData(self):
        msg = b''
        while len(msg) < self.MSGLEN:
            chunk = self.socket.recv(self.MSGLEN - len(msg))
            if chunk == b'':
                raise RuntimeError("socket connection lost")
            msg = msg + chunk
        self.Data = b''
        self.Data = msg
        self.convertData()

    def splitByteArray(self, lower, upper):
        temp = bytearray(self.Data[lower:upper])
        temp.reverse()
        return temp

    def convertData(self):
        for i in range(6):
            # Joint Positions
            temp = self.splitByteArray(self.byteArrayPos_JointPos + 8 * i, self.byteArrayPos_JointPos + 8 * (i + 1))
            zw = math.degrees(struct.unpack('<d', temp)[0])
            self.JointPositions[i] = zw

            # Joint Velocity
            temp = self.splitByteArray(self.byteArrayPos_JointVelocity + 8 * i,
                                       self.byteArrayPos_JointVelocity + 8 * (i + 1))
            zw = math.degrees(struct.unpack('<d', temp)[0])
            self.JointVelocity[i] = zw

            # TcpForce
            temp = self.splitByteArray(self.byteArrayPos_TcpForce + 8 * i, self.byteArrayPos_TcpForce + 8 * (i + 1))
            self.TcpForce[i] = struct.unpack('<d', temp)[0]

            # Motor Temperatur
            temp = self.splitByteArray(self.byteArrayPos_MotorTemp + 8 * i, self.byteArrayPos_MotorTemp + 8 * (i + 1))
            self.MotorTemp[i] = struct.unpack('<d', temp)[0]

            # Actual Joint Currents
            temp = self.splitByteArray(self.byteArrayPos_JointCurrents + 8 * i,
                                       self.byteArrayPos_JointCurrents + 8 * (i + 1))
            self.JointCurrents[i] = struct.unpack('<d', temp)[0]

            # Actual Joint Voltage
            temp = self.splitByteArray(self.byteArrayPos_JointVoltage + 8 * i,
                                       self.byteArrayPos_JointVoltage + 8 * (i + 1))
            self.JointCurrents[i] = struct.unpack('<d', temp)[0]

            # Target Joint Moment
            temp = self.splitByteArray(self.byteArrayPos_TargetJointMoment + 8 * i,
                                       self.byteArrayPos_TargetJointMoment + 8 * (i + 1))
            self.TargetJointMoment[i] = struct.unpack('<d', temp)[0]

            # ActualTcpPos
            temp = self.splitByteArray(self.byteArrayPos_ActualTcpPose + 8 * i,
                                       self.byteArrayPos_ActualTcpPose + 8 * (i + 1))
            self.ActualTcpPos[i] = struct.unpack('<d', temp)[0]

    def sendUrProgram(self, commands):
        completeProgram = "def completeProgram():\n"
        for i in range(len(commands)):
            completeProgram += " " + commands[i] + "\n"
        completeProgram += "end\n"
        # print(completeProgram)
        self.socket.send(completeProgram.encode())

    def sendCompleteUrProgram(self,Path):
        f=open(Path)
        completeProgram=f.readlines()
        f.close()
        completeProgram=''.join(completeProgram)
        self.socket.send(completeProgram.encode())

    def convertStringList(self, List):
        TempPosStr = ""
        for elemetns in range(len(List)):
            if elemetns < len(List) - 1:
                TempPosStr += List[elemetns] + ","
            else:
                TempPosStr += List[elemetns]
        return TempPosStr

    def checkPosition(self, *argv):
        # first Position than TCP
        Pos = argv[0]

        if (len(argv) > 1):
            Tcp = argv[1]
            if type(Tcp) is list:
                if type(Tcp[0]) is float or type(Tcp[0]) is int:
                    TempTcp = ["%.4f" % x for x in Tcp]

                    if isinstance(Tcp[0], str):
                        TempTcp = Tcp
                    TempTcpStr = self.convertStringList(TempTcp)

            if isinstance(Tcp, str):
                TempTcpStr = Tcp
        else:
            TempTcpStr = "0,0,0,0,0,0"
        # Check if Data is float or string
        if type(Pos) is list:
            if type(Pos[0]) is float:
                TempPos = ["%.4f" % x for x in Pos]

            if isinstance(Pos[0], str):
                TempPos = Pos
            TempPosStr = self.convertStringList(TempPos)

        if isinstance(Pos, str):
            TempPosStr = Pos

        self.sendUrProgram(
            ['''global open=socket_open("192.168.56.103",21,socket_name="socket_write")''', "while(open==False):",
             '''open=socket_open("192.168.56.103",21,socket_name="socket_write")''', "end",
             "set_tcp(p[" + TempTcpStr + "])",
             "global InvA=get_inverse_kin(p[" + TempPosStr + "])",
             '''socket_send_string(to_str(InvA),socket_name="socket_write")'''])

        socket2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        socket2.bind(('', 21))

        socket2.listen(5)
        socket2.settimeout(2)
        try:
            conn, addr = socket2.accept()
            conn.settimeout(2)
        except:
            data = []
            pass

        try:

            data = conn.recv(1024)

        except:
            print(colored("Something went Wrong on this Point", 'red'))

        if len(data) == 0:
            return "No Inverse Kinematic"
        else:
            Temp = data.decode()
            Temp = Temp[1:len(Temp) - 1]
            return Temp
        socket2.close()
        conn.close()

    def CheckPositionList(self, PosList):
        for i in range(len(PosList)):
            self.checkPosition(PosList[i])

    def CheckPositonsFromExcel(self, Path, unit, TCP):
        wb = xlrd.open_workbook(Path)
        sheet = wb.sheet_by_index(0)
        sheetNames = wb.sheet_names()
        PosList = []
        ResultList = []

        for i in range(sheet.nrows):
            print("Calculating Point: " + str(i + 1))

            ResultList.append(self.checkPosition(sheet.cell_value(i, 0), TCP))
            PosList.append(sheet.cell_value(i, 0))

        if (unit == "rad"):
            for i in range(sheet.nrows):
                if ResultList[i] != "No Inverse Kinematic":
                    temp = ResultList[i].split(",")
                    check = 0
                    temp = [float(x) for x in temp]
                    while check < len(temp):
                        if (temp[check] < -math.pi):
                            temp[check] = temp[check] + 2 * math.pi
                        elif (temp[check] > math.pi):
                            temp[check] = temp[check] - 2 * math.pi
                        else:
                            check = check + 1
                    ResultList[i] = self.convertStringList(["%.4f" % x for x in temp])
        if (unit == "deg"):
            for i in range(sheet.nrows):
                if ResultList[i] != "No Inverse Kinematic":
                    temp = ResultList[i].split(",")
                    temp = [math.degrees(float(x)) for x in temp]
                    ResultList[i] = self.convertStringList(["%.4f" % x for x in temp])

        workbook = xlsxwriter.Workbook(Path)
        worksheet = workbook.add_worksheet()
        bold = workbook.add_format({'bold': 1})
        worksheet.write_string(0, 1, "Inverse Kinematik" + " [" + unit + "]", bold)
        worksheet.write_string(0, 0, "Ziel Position [x,y,z,rx,ry,rz]", bold)
        worksheet.set_column(0, 0, 25)
        worksheet.set_column(1, 1, 70)
        cell_format = workbook.add_format()
        cell_format.set_bold()
        cell_format.set_font_color('red')

        c = 0
        for i in range(sheet.nrows):
            if (self.is_number(PosList[i][0])):
                worksheet.write_string(c + 1, 0, PosList[i])
                if (ResultList[i] == "No Inverse Kinematic"):
                    worksheet.write_string(c + 1, 1, ResultList[i], cell_format)
                else:
                    worksheet.write_string(c + 1, 1, ResultList[i])
                c += 1

        workbook.close()

    def is_number(self, s):
        try:
            float(s)
            return True
        except ValueError:
            return False

    def createPlot(self, Xlim, Ylim, Plots):
        fig, ax = plt.subplots(3, 2, sharex=True)

        ax = ax.ravel()
        ax = ax.tolist()
        fig.suptitle("RealTime Plot of UR: " + self.IpAdresse)
        c = 0
        lines = [0, 0, 0, 0, 0, 0]
        names = ["Base", "Shoulder", "Elbow", "Wrist1", "Wrist2", "Wrist3"]
        y_Labels = ["Position", "Velocity", "TcpForce", "Temperatur", "Current", "Voltage", "Moment", "TcpPose"]
        Einheit = [" [°]", " [°/s]", " [N]", " [°C]", " [A]", " [V]", " [Nm]", " [m]"]
        idx = [y_Labels.index(Plots[0])]

        Legend = [y_Labels[0]]
        if (len(Plots) > 1):
            ax_new = [0, 0, 0, 0, 0, 0]
            lines.extend([0, 0, 0, 0, 0, 0])
            idx.append(y_Labels.index(Plots[1]))
            Legend.append(y_Labels[1])

        for ax_i in ax:
            # for ax_i in ax.flat:
            lines[c] = ax_i.plot([], [], color='r', lw=2)
            if (c == 0):
                Legend_L = [lines[c][0]]
            if (len(Plots) > 1):
                ax_new[c] = ax_i.twinx()
                lines[c + 6] = ax_new[c].plot([], [], color='b', lw=2)
                if (c == 0):
                    Legend_L.append(lines[c + 6][0])
                if (c == 1 or c == 3 or c == 5):
                    temp = ax_new[c].set_ylabel(y_Labels[idx[1]] + Einheit[idx[1]])
                    temp.set_color("blue")

            if (c == 0 or c == 2 or c == 4):
                temp = ax_i.set_ylabel(y_Labels[idx[0]] + Einheit[idx[0]])
                temp.set_color("red")

            ax_i.set_title(names[c])
            ax_i.set_ylim(Ylim[0], Ylim[1])
            ax_i.set_xlim(Xlim[0], Xlim[1])
            ax_i.grid()
            ax_i.set_autoscaley_on(True)
            ax_i.autoscale(True, axis='y')
            c += 1
        # fig.legend(handles=Legend_L,     # The line objects
        #      labels=Legend,   # The labels for each line
        #     loc='upper right',   # Position of legend   mode="expand",
        #   bbox_to_anchor=(0.98, 0.96 )
        # )
        if (len(Plots) > 1):
            ax.extend(ax_new)
        q = Process(target=self.PlotProzess, args=(self.sharedData, Xlim, Ylim, lines, ax, fig, idx))
        q.start()

    def PlotProzess(self, DataIn, Xlim, Ylim, lines, axis, fig, idx):

        def run(i, data):
            # update the data
            line = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            for i in range(6):
                c = 0
                for x in idx:
                    data[i + 1 + 6 * c].append(DataIn[i + 6 * x])
                    data[i + 1 + 6 * c] = data[i + 1 + 6 * c][-Xlim[1]:]
                    lines[i + 6 * c][0].set_data(data[0], data[i + 1 + 6 * c])
                    line[i + 6 * c] = lines[i + 6 * c][0]
                    c += 1

            c = 1

            for ax_i in axis:
                ax_i.relim()
                ax_i.autoscale_view()

                # if np.min(data[c])<=line[c-1].axes.get_ylim()[0] or np.max(data[c])>=line[c-1].axes.get_ylim()[1]:
                #   ax_i.set_ylim([np.min(data[c])-100,np.max(data[c])+100])
                #  plt.draw()
                c += 1

            return line

        data = [np.arange(Xlim[0] + 1, Xlim[1] + 1, 1)]
        data.extend(np.zeros(12))
        for i in range(12):
            data[i + 1] = [0] * Xlim[1]

        ani = animation.FuncAnimation(fig,
                                      run,
                                      fargs=(data,),
                                      interval=1,
                                      blit=False)
        plt.show()

    def startUpdateProcess(self):
        p = Process(target=self.Updater, args=(self.sharedData, True))
        p.start()

    def Updater(self, n, Continuously):
        run = True
        while (run):
            self.readData()
            for i in range(6):
                n[i] = self.JointPositions[i]
                n[i + 6] = self.JointVelocity[i]
                n[i + 12] = self.TcpForce[i]
                n[i + 18] = self.MotorTemp[i]
                n[i + 24] = self.JointCurrents[i]
                n[i + 30] = self.JointVoltage[i]
                n[i + 36] = self.TargetJointMoment[i]
                n[i + 42] = self.ActualTcpPos[i]
            run = Continuously

    def formatSharedData(self):
        for i in range(6):
            self.JointPositions[i] = self.sharedData[i]
            self.JointVelocity[i] = self.sharedData[i + 6]
            self.TcpForce[i] = self.sharedData[i + 12]
            self.MotorTemp[i] = self.sharedData[i + 18]
            self.JointCurrents[i] = self.sharedData[i + 24]
            self.JointVoltage[i] = self.sharedData[i + 30]
            self.TargetJointMoment[i] = self.sharedData[i + 36]
            self.ActualTcpPos[i] = self.sharedData[i + 42]


    @TimeMeasure.getTimeStamp
    def readDataWithTime(self):
        self.formatSharedData()
        Data=[self.JointPositions,self.JointVelocity,self.TcpForce,
              self.MotorTemp,self.JointCurrents,self.JointVoltage,
              self.TargetJointMoment,self.ActualTcpPos]
        return self.ActualTcpPos

    def DataWithTime(self,startTime):
        out=self.readDataWithTime()
        dur=out[0]-startTime
        out[0]=dur
        return out


class KalmanFilter(object):

    def __init__(self, process_variance, estimated_measurement_variance):
        self.process_variance = process_variance
        self.estimated_measurement_variance = estimated_measurement_variance
        self.posteri_estimate = 0.0
        self.posteri_error_estimate = 1.0

    def input_latest_noisy_measurement(self, measurement):
        priori_estimate = self.posteri_estimate
        priori_error_estimate = self.posteri_error_estimate + self.process_variance

        blending_factor = priori_error_estimate / (priori_error_estimate + self.estimated_measurement_variance)
        self.posteri_estimate = priori_estimate + blending_factor * (measurement - priori_estimate)
        self.posteri_error_estimate = (1 - blending_factor) * priori_error_estimate

    def get_latest_estimated_measurement(self):
        return self.posteri_estimate




if __name__ == '__main__':

    # ur = UrRobot("192.168.0.101")
    # ur = UrRobot("192.168.237.128")

    ur = UrRobot("192.168.0.105")
    print(ur.IpAdresse)
    ur.connectUr()
    #ur.sendCompleteUrProgram('/home/kfreise/Prototype0.1/RobotScript/test.txt')
    #print(ur.stdOnRobotMsg)
    #print(''.join(completeProgram))


    ###############################################
    #Exapmple for Robot Plot

    #ur.startUpdateProcess()
    #ur.createPlot([0,200],[-100,100],["Position"]) #["Position","Velocity","TcpForce","Temperatur","Current","Voltage","Moment","TcpPose"]
    ###############################################

