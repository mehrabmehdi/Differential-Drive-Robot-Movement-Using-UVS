from ev3dev2.motor import LargeMotor, OUTPUT_C, OUTPUT_D, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.led import Leds
from DDrive import DDrive


base = LargeMotor(OUTPUT_D)
joint = LargeMotor(OUTPUT_C)

drive =DDrive(base, joint)

drive.inv_kin(2,3)
