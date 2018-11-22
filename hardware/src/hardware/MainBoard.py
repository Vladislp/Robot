import serial
import threading
import time
import subprocess
import rospy


class ComportMainboard(threading.Thread):
    connection = None
    connection_opened = False

    lastSpeed = ''
    lastThrower = ''

    wheelCounter = 0
    throwercounter = 0

    commandTimeout = 50

    def __init__(self):
        threading.Thread.__init__(self)

    def open(self):
        try:
            ports = subprocess.check_output('ls /dev/ttyACM0', shell=True).split('\n')[:-1]
        except:
            print('mainboard: /dev/ttyACM empty')
            return False
        self.connection_opened = False
        for port in ports:  # analyze serial ports
            try:
                while not rospy.is_shutdown():
                    self.connection = serial.Serial(port, baudrate=115200, timeout=0.8, dsrdtr=True)
                    time.sleep(0.5)
                    if self.connection.isOpen():
                        self.connection_opened = True
                        self.connection.flush()
                        break

                print "mainboard: Port opened successfully"
                print(self.connection_opened)
            except Exception as e:
                print(e)
                continue

        return self.connection_opened

    def read(self):
        if self.connection_opened:
            command = ''
            char = ''
            while char != '\n' and char is not None:
                char = self.connection.read()
                command += char
            return command

    def write(self, comm):

        if not self.connection_opened:
            print("Connection is not opened")
            return
        if self.connection is not None:
            #print("Connection is on")
            try:
                self.connection.write(comm + '\n')
                print("Kirjutan {}".format(comm))
            except:
                print('mainboard: err write ' + comm)

    def launch_motor(self, motor_one, motor_two, motor_three, motor_four):
        if self.connection_opened:
            print("Connection is open ")
            self.write("sd:{}:{}:{}:{}\n".format(motor_one, motor_two, motor_three, motor_four))

    def close(self):
        if self.connection is not None and self.connection.isOpen():  # close coil
            try:
                self.connection.close()
                print('mainboard: connection closed')
            except:
                print('mainboard: err connection close')
            self.connection = None

    def run(self):
        if self.open():  # open serial connections
            print('mainboard: opened')
        else:
            print('mainboard: opening failed')
            self.close()
            return

    def set_throw(self, speed):
        print("d{}".format(speed))
        self.write("d:{}".format(speed))

    def set_servo(self, position):
        self.write("sv:{}".format(position))
