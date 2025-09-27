#PROGRAM SETTINGS
RUN_AUTON = True

from vex import *

brain = Brain()
controller_1 = Controller(PRIMARY)
FrontR = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
FrontL = Motor(Ports.PORT2, GearSetting.RATIO_18_1, True)
BackL = Motor(Ports.PORT3, GearSetting.RATIO_18_1, True)
BackR = Motor(Ports.PORT4, GearSetting.RATIO_18_1, True)
Roller_Bottom = Motor(Ports.PORT12, GearSetting.RATIO_18_1, False)
Roller_Top = Motor(Ports.PORT20, GearSetting.RATIO_18_1, False)
First_Gauge = Motor(Ports.PORT19, GearSetting.RATIO_18_1, False)
tongue = DigitalOut(brain.three_wire_port.a)

def tt(x):
    global tongue_extended
    tongue.set(x)
    tongue_extended = x

tongue_extended = True
wait(30, MSEC)
atoggle = False

def process(drspeed: int):
    global atoggle
    """Function to handle all user inputs"""
    BackL.spin(FORWARD, speedl, VOLT)
    FrontL.spin(FORWARD, speedl, VOLT)
    BackR.spin(REVERSE, speedr, VOLT)
    FrontR.spin(REVERSE, speedr, VOLT)
    Roller_Bottom.set_velocity(drspeed, PERCENT)
    Roller_Top.set_velocity(drspeed, PERCENT)
    First_Gauge.set_velocity(drspeed, PERCENT)
    if controller_1.buttonL1.pressing(): # lower outtake
        First_Gauge.spin(FORWARD)
        Roller_Bottom.spin(FORWARD)
    elif controller_1.buttonL2.pressing(): #intake
        First_Gauge.spin(REVERSE)
        Roller_Bottom.spin(REVERSE)
    elif controller_1.buttonR1.pressing(): #center outtake
        First_Gauge.spin(FORWARD)
        Roller_Bottom.spin(FORWARD)
        Roller_Top.spin(FORWARD)
    elif controller_1.buttonR2.pressing(): #upper outtake
        First_Gauge.spin(FORWARD)
        Roller_Bottom.spin(FORWARD)
        Roller_Top.spin(REVERSE)
    else:
        Roller_Bottom.stop()
        First_Gauge.stop()
        Roller_Top.stop()
    if controller_1.buttonA.pressing():
        if not atoggle:
            tt(not tongue_extended)
            atoggle = True
    else:
        atoggle = False
def runauton(cmdtuple):
    """
    Accepts a tuple, provided to the function via the topmost entry of the auton list,
    and transforms it into an autonomous sequence of actions, separated by the 5 ms delay
    per iteration every loop.
    The scheme of a cmdtuple:
    (lspeed*, rspeed*, fgspeed, rbspeed, rtspeed, tongue)
    *required
    """
    BackL.spin(FORWARD, cmdtuple[0], VOLT)
    FrontL.spin(FORWARD, cmdtuple[0], VOLT)
    BackR.spin(REVERSE,  cmdtuple[1], VOLT)
    FrontR.spin(REVERSE, cmdtuple[1], VOLT)
    if len(cmdtuple) > 2:
        First_Gauge.set_velocity(cmdtuple[2], PERCENT)
        First_Gauge.spin(FORWARD)
    else:
        First_Gauge.stop()
    if len(cmdtuple) > 3:
        Roller_Bottom.set_velocity(cmdtuple[3], PERCENT)
        Roller_Bottom.spin(FORWARD)
    else:
        Roller_Bottom.stop()
    if len(cmdtuple) > 4:
        Roller_Top.set_velocity(cmdtuple[4], PERCENT)
        Roller_Top.spin(FORWARD)
    else:
        Roller_Top.stop()
    if len(cmdtuple) > 5:
        tt(cmdtuple[5])

def ap(tup, rep):
    for _ in range(rep):
        auton.append(tup)

sensitivity = 2
speedl, speedr, pastl, pastr = 0, 0, 0, 0
auton = []

ap((10, 0, 0, 0, 0, False), 16)
ap((12, 12), 20)
ap((4, 4, -100, -100), 425)
ap((12, 0, 100, -100), 120)
ap((0, 0), 100)
ap((6, 12, 0, 0, 0), 110)
ap((12, 12, 0, 0, 0, True), 40)

while True:
    speedl = (controller_1.axis3.position() + controller_1.axis1.position()) / (16.6 / sensitivity)
    speedr = (controller_1.axis3.position() - controller_1.axis1.position()) / (16.6 / sensitivity)
    if pastl > 0 and speedl == 0:
        pastl = pastl + -1
        BackL.spin(FORWARD, 12, VOLT)
        FrontL.spin(FORWARD, 12, VOLT)
    if pastr > 0 and speedr == 0:
        pastr = pastr + -1
        FrontR.spin(REVERSE, 12, VOLT)
        BackR.spin(REVERSE, 12, VOLT)
    if len(auton) > 0 and RUN_AUTON:
        runauton(auton[0])
        auton.pop(0)
    else:
        process(75)
    pastl = speedl
    pastr = speedr
    wait(5, MSEC)
