import serial


ser = serial.Serial(
    port='COM3',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.EIGHTBITS
)

print("Serial is open: " + str(ser.isOpen()))

def liigu(speed, angle, angularVelocity):
    wheelSpeed0 = str(mainboardSpeedCalc(wheelCalc(0, speed, angle, angularVelocity)))
    wheelSpeed1 = str(mainboardSpeedCalc(wheelCalc(1, speed, angle, angularVelocity)))
    wheelSpeed2 = str(mainboardSpeedCalc(wheelCalc(2, speed, angle, angularVelocity)))

    ser.write("sd:{0}:{1}:{2}:0\n".format(wheelSpeed0, wheelSpeed1, wheelSpeed2).encode())
