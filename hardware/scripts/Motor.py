import numpy as np

def move(com, vel):
    com.write("sd:{}:{}:{}".format(vel[0],vel[1],vel[2]))
    com.Readmsg()

def forward(power):
    vel = [0,-power,power]
    return vel

def backward(power):
    vel = [0,power,-power]
    return vel

def right(power):
    vel = [power,power,power]
    return vel

def left(power):
    vel = [-power,-power,-power]
    return vel

def stop():
    vel = [0,0,0]
    return vel