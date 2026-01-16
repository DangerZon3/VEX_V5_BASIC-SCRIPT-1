from vex import *
import time, math

brain = Brain()
controller_1 = Controller(PRIMARY)
FrontR = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
FrontL = Motor(Ports.PORT2, GearSetting.RATIO_18_1, True)
BackL = Motor(Ports.PORT3, GearSetting.RATIO_18_1, True)
BackR = Motor(Ports.PORT4, GearSetting.RATIO_18_1, False)
Roller_Bottom = Motor(Ports.PORT12, GearSetting.RATIO_18_1, False)
Roller_Top = Motor(Ports.PORT20, GearSetting.RATIO_18_1, False)
First_Gauge = Motor(Ports.PORT19, GearSetting.RATIO_18_1, False)
Unjammer = Motor(Ports.PORT17, GearSetting.RATIO_18_1, False)
Inertia = Inertial(Ports.PORT13)

# Customizable Constants
GEAR_RATIO = 5/3 #External gear ratio (motor to wheel)
WHEEL_DIAMETER = 3.25 #Wheel diameter in inches
L_MOTORS = [FrontL, BackL] #All motors on the left
R_MOTORS = [FrontR, BackR] #All motors on the right
MOTOR_DEGREES_PER_RIGHT_TURN = 163.5 #Motor rotation degrees required for robot to turn 90 degrees
# ------------------------------------------------------------------------------

MDPRT = MOTOR_DEGREES_PER_RIGHT_TURN
motors = L_MOTORS + R_MOTORS

dist_per_deg = WHEEL_DIAMETER * math.pi * GEAR_RATIO / 360 #Distance robot travels in one motor degree
posx = 0
posy = 0

def screen_status(txt, row=1):
    controller_1.screen.clear_row(row)
    controller_1.screen.set_cursor(row,1)
    controller_1.screen.print(txt)

def enum(iterable): #enumerate() disallowed in VEX Python
    index = 0
    for item in iterable:
        yield index, item
        index += 1

class Action: #One unit of actions run during auton
    def __init__(self, action, degrees, velocity=50):
        self.action = action
        self.degrees = degrees
        self.velocity = velocity

    def execute(self):
        for motor in motors:
            motor.set_velocity(self.velocity, PERCENT)
        if self.action.upper() in ["F", "FWD","FORWARD"]:
            for i, motor in enum(motors):
                if i == len(motors) - 1:
                    motor.spin_for(FORWARD, self.degrees, DEGREES, True)
                else:
                    motor.spin_for(FORWARD, self.degrees, DEGREES, False)
        elif self.action.upper() in ["B", "BWD", "BACKWARD"]:
            for i, motor in enum(motors):
                if i == len(motors) - 1:
                    motor.spin_for(REVERSE, self.degrees, DEGREES, True)
                else:
                    motor.spin_for(REVERSE, self.degrees, DEGREES, False)
        elif self.action.upper() in ["L", "LEFT"]:
            for l_motor in L_MOTORS:
                l_motor.spin_for(REVERSE, self.degrees, DEGREES, False)
            for i, r_motor in enum(R_MOTORS):
                if i == len(R_MOTORS) - 1:
                    r_motor.spin_for(FORWARD, self.degrees, DEGREES, True)
                else:
                    r_motor.spin_for(FORWARD, self.degrees, DEGREES, False)
        elif self.action.upper() in ["R", "RIGHT"]:
            for l_motor in L_MOTORS:
                l_motor.spin_for(FORWARD, self.degrees, DEGREES, False)
            for i, r_motor in enum(R_MOTORS):
                if i == len(R_MOTORS) - 1:
                    r_motor.spin_for(REVERSE, self.degrees, DEGREES, True)
                else:
                    r_motor.spin_for(REVERSE, self.degrees, DEGREES, False)

Inertia.calibrate() #Calibrate Inertial sensor
while Inertia.is_calibrating():
    screen_status("Calibrating")
    time.sleep(0.1)
screen_status("Calibration Complete")

auton_actions = [
    Action("F", 360, 50),
    Action("R", MDPRT * 4, 10),
]

while True:
    pastdeg = FrontL.position(DEGREES)
    heading = round(Inertia.heading(), 1)
    print("Inertia Heading: ", heading)
    print("Coordinates:" , posx, posy)
    if len(auton_actions) == 0:
        break
    auton_actions[0].execute()
    auton_actions.pop(0)
    difference = FrontL.position(DEGREES) - pastdeg
    distance = difference * dist_per_deg
    posx += distance * math.sin(math.radians(heading))
    posy += distance * math.cos(math.radians(heading))
