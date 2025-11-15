# Library imports
from vex import *
import time
    
SIDE = -1 #-1 for Left, 1 for Right (technically any number besides 1 is left)
# Put left-side (less familiar auton, SIDE = -1) on slot 2

brain = Brain()
controller_1 = Controller(PRIMARY)
FrontR = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
FrontL = Motor(Ports.PORT2, GearSetting.RATIO_18_1, True)
BackL = Motor(Ports.PORT3, GearSetting.RATIO_18_1, True)
BackR = Motor(Ports.PORT4, GearSetting.RATIO_18_1, True)
Roller_Bottom = Motor(Ports.PORT12, GearSetting.RATIO_18_1, False)
Roller_Top = Motor(Ports.PORT20, GearSetting.RATIO_18_1, False)
First_Gauge = Motor(Ports.PORT19, GearSetting.RATIO_18_1, False)
Unjammer = Motor(Ports.PORT17, GearSetting.RATIO_18_1, False)
tongue = DigitalOut(brain.three_wire_port.a)
hood = DigitalOut(brain.three_wire_port.h)
def hh(x):
    global hood_extended
    hood.set(x)
    hood_extended = x
def tt(x):
    global tongue_extended
    tongue.set(x)
    tongue_extended = x

tongue_extended = True
hood_extended = True
atoggle = False
xtoggle = False


def runauton(cmdtuple):
    """
    Accepts a tuple, provided to the function via the topmost entry of the auton list,
    and transforms it into an autonomous sequence of actions, separated by the 5 ms delay
    per iteration every loop.\n
    The scheme of a cmdtuple:
    (dir*, vel*, dist*, fgspeed, rbspeed, rtspeed, tongue)\n
    *required
    """
    BackL.set_velocity(cmdtuple[1], PERCENT)
    FrontL.set_velocity(cmdtuple[1], PERCENT)
    BackR.set_velocity(cmdtuple[1], PERCENT)
    FrontR.set_velocity(cmdtuple[1], PERCENT)
    leftDegree = 0
    rightDegree = 0
    direction = ["f", "b", "l", "r"].index(cmdtuple[0][0].lower())
    if direction == 0:
            leftDegree = cmdtuple[2]
            rightDegree = cmdtuple[2]
    elif direction == 1:
            leftDegree = -cmdtuple[2]
            rightDegree = -cmdtuple[2]
    elif direction == 2:
            leftDegree = -cmdtuple[2]
            rightDegree = cmdtuple[2]
    elif direction == 3:
            leftDegree = cmdtuple[2]
            rightDegree = -cmdtuple[2]

    if len(cmdtuple) > 3:
        First_Gauge.set_velocity(cmdtuple[3], PERCENT)
        First_Gauge.spin(FORWARD)
        Unjammer.set_velocity(cmdtuple[3], PERCENT)
        Unjammer.spin(REVERSE)
    else:
        First_Gauge.stop()
        Unjammer.stop()
    if len(cmdtuple) > 4:
        Roller_Bottom.set_velocity(cmdtuple[4], PERCENT)
        Roller_Bottom.spin(FORWARD)
    else:
        Roller_Bottom.stop()
    if len(cmdtuple) > 5:
        Roller_Top.set_velocity(cmdtuple[5], PERCENT)
        Roller_Top.spin(FORWARD)
    else:
        Roller_Top.stop()
    if len(cmdtuple) > 6:
        tt(cmdtuple[6])
    if len(cmdtuple) > 7:
        hh(cmdtuple[7])
    BackL.spin_for(FORWARD, leftDegree, DEGREES, False)
    FrontL.spin_for(FORWARD, leftDegree, DEGREES, False)
    BackR.spin_for(REVERSE, rightDegree, DEGREES, False)
    FrontR.spin_for(REVERSE, rightDegree, DEGREES, True)
    if len(cmdtuple[0]) == 1:
        time.sleep(0.33)
        

def pistons_detect():
    global atoggle
    if controller_1.buttonA.pressing():
        if not atoggle:
            tt(not tongue_extended)
        atoggle = True
    else:
        atoggle = False
    global xtoggle
    if controller_1.buttonX.pressing():
        if not xtoggle:
            hh(not hood_extended)
        xtoggle = True
    else:
        xtoggle = False

def process(drspeed: int, speedl, speedr):
    """Function to handle all user inputs"""
    Thread(pistons_detect)
    BackL.spin(FORWARD, speedl, VOLT)
    FrontL.spin(FORWARD, speedl, VOLT)
    BackR.spin(REVERSE, speedr, VOLT)
    FrontR.spin(REVERSE, speedr, VOLT)
    Roller_Bottom.set_velocity(drspeed, PERCENT)
    Roller_Top.set_velocity(drspeed * 0.6, PERCENT)
    First_Gauge.set_velocity(drspeed, PERCENT)
    Unjammer.set_velocity(drspeed, PERCENT) 
    if controller_1.buttonL1.pressing(): # lower outtake
        First_Gauge.spin(FORWARD)
        Roller_Bottom.set_velocity(drspeed/2, PERCENT)
        Roller_Bottom.spin(FORWARD)
        Unjammer.spin(REVERSE)
    elif controller_1.buttonL2.pressing(): #intake
        First_Gauge.spin(REVERSE)
        Roller_Bottom.spin(REVERSE)
        Unjammer.spin(FORWARD)
        Roller_Top.spin(FORWARD)
    elif controller_1.buttonR2.pressing(): #upper outtake
        First_Gauge.spin(FORWARD)
        Roller_Bottom.set_velocity(drspeed/1.5, PERCENT)
        Roller_Bottom.spin(REVERSE)
        Roller_Top.spin(FORWARD)
        Unjammer.spin(REVERSE)
    elif controller_1.buttonR1.pressing(): #center outtake
        First_Gauge.spin(FORWARD)
        Roller_Bottom.spin(REVERSE)
        Roller_Top.spin(REVERSE)
        Unjammer.spin(REVERSE)
    else:
        Roller_Bottom.stop()
        First_Gauge.stop()
        Roller_Top.stop()
        Unjammer.stop()


def autonomous():
    global SIDE
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
    #remember, the param is (direction, velocity, distance, first gauge & unjammer speed, bottom roller speed, top roller speed, tongue, hood)
    if SIDE == 1:
        runauton(("r", 40, 40))
        runauton(("f", 40, 585, 75, -95, 95))
        time.sleep(0.2)
        runauton(("b", 20, 70))
        runauton(("l", 40, 135))
        runauton(("f", 30, 330, 50, 20, -30))
        time.sleep(0.1)
        runauton(("f", 0, 0, -50, 20))
        time.sleep(0.1)
        runauton(("b", 0, 0, 50, 20, -30))
        time.sleep(0.2)
        runauton(("l", 40, 20))
        runauton(("b", 70, 1185))
        runauton(("r", 50, 90))
        runauton(("f", 90, 460))
        runauton(("b", 10, 10))
        runauton(("f", 0, 0, 75, -75, 100, False, True))
    else:
        runauton(("l", 40, 40))
        runauton(("f", 25, 585, -75, -75))
        #runauton(("b", 25, 90))haha
        runauton(("r", 40, 120))
        runauton(("f", 30, 240, 70, -20, -20))
        time.sleep(3)
        runauton(("f", 0, 0, -70))
        runauton(("b", 40, 190))
        runauton(("r", 30, 300))
        runauton(("f", 50, 700, 0, 0, 0))
        runauton(("l", 90, 90))
        runauton(("f", 100, 445, -75, -75, 0, True))
def temp():
    while True:
        motortemps = [round(BackL.temperature()), round(FrontL.temperature()), round(BackR.temperature()), round(FrontR.temperature())]
        controller_1.screen.print(motortemps[0], motortemps[1], motortemps[2], motortemps[3])
        controller_1.screen.new_line()
        if 55 in motortemps:
            controller_1.rumble("...")
        time.sleep(2)

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("driver control")
    sensitivity = 2
    speedl, speedr, pastl, pastr = 0, 0, 0, 0
    Thread(temp)
    while True:
        speedl = (controller_1.axis3.position() + controller_1.axis1.position()) / (16.6 / sensitivity)
        speedr = (controller_1.axis3.position() - controller_1.axis1.position()) / (16.6 / sensitivity)
        
        brain.screen.print(speedl)
        if pastl > 0 and speedl == 0:
            pastl = pastl + -1
            BackL.spin(FORWARD, 12, VOLT)
            FrontL.spin(FORWARD, 12, VOLT)
        if pastr > 0 and speedr == 0:
            pastr = pastr + -1
            FrontR.spin(REVERSE, 12, VOLT)
            BackR.spin(REVERSE, 12, VOLT)
        else:
            process(75, speedl, speedr)
        pastl = speedl
        pastr = speedr
        wait(5, MSEC)

# create competition instance
comp = Competition(user_control, autonomous)

# actions to do when the program starts
brain.screen.clear_screen()
brain.screen.print("""
  _   _                               _         _                      
 | \\ | |_   _  __ _ ___   _ __   ___ | |_    __| |_ __ _   _  __ _ ___ 
 |  \\| | | | |/ _` / __| | '_ \\ / _ \\| __|  / _` | '__| | | |/ _` / __|
 | |\\  | |_| | (_| \\__ \\ | | | | (_) | |_  | (_| | |  | |_| | (_| \\__ \\
 |_| \\_|\\__,_|\\__, |___/ |_| |_|\\___/ \\__|  \\__,_|_|   \\__,_|\\__, |___/
              |___/                                          |___/
""")