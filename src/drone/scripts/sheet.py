#!/usr/bin/env python3

from drone.oceanikaAPI import UUV

############## YOUR GLOBAL VARIABLES ##############

var = 0

###################################################


################# YOUR DEFINITIONS ################

def func():
    pass


def wait_3_sec():
    uuv.wait()
    uuv.wait()
    uuv.wait()

###################################################

if __name__ == "__main__":
    uuv = UUV()
    uuv.blink()
    uuv.dive_up()
    uuv.lights_on()
    uuv.wait()
    uuv.wait()
    uuv.lights_off()