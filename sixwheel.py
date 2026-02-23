#   Module:       main.py
#   Author:       lsong
#   Created:      11/26/2025, 9:11:35 PM
#   Description:  V5 project

from vex import *

#--------- Program Settings ---------

SIDE = "RIGHT" #"LEFT", "RIGHT", or "SKILLS"

brain=Brain()
motorLeft_1 = Motor(Ports.PORT1, GearSetting.RATIO_6_1, False)
motorLeft_2 = Motor(Ports.PORT2, GearSetting.RATIO_6_1, False)
motorLeft_3 = Motor(Ports.PORT3, GearSetting.RATIO_6_1, False)
motorRight_1 = Motor(Ports.PORT5, GearSetting.RATIO_6_1, True)
motorRight_2 = Motor(Ports.PORT6, GearSetting.RATIO_6_1, True)
motorRight_3 = Motor(Ports.PORT7, GearSetting.RATIO_6_1, True)

flapper = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)
intakeMotor = Motor(Ports.PORT11, GearSetting.RATIO_18_1, False)
secondMotor = Motor(Ports.PORT12, GearSetting.RATIO_18_1, False)

tongue = DigitalOut(brain.three_wire_port.a)
aligner = DigitalOut(brain.three_wire_port.b)
descorer = DigitalOut(brain.three_wire_port.c) #also stopper

controller_1 = Controller(PRIMARY)
controller_2 = Controller(PARTNER)

L_MOTORS = [motorLeft_1, motorLeft_2, motorLeft_3] #All motors on the left
R_MOTORS = [motorRight_1, motorRight_2, motorRight_3] #All motors on the right

#---------- Main Program ------------
motors = L_MOTORS + R_MOTORS
sensitivity = 2

def enum(iterable): #enumerate() disallowed in VEX Python
    index = 0
    for item in iterable:
        yield index, item
        index += 1

class Action: #One unit of action run during auton
    def __init__(self, action, degrees, velocity=50, special=0):
        self.action = action
        self.degrees = degrees
        self.velocity = velocity
        self.special = special

    def execute(self):
        if self.special == 1: #intake
            intakeMotor.spin(FORWARD)
            secondMotor.spin(FORWARD)
        elif self.special == -1: #lower
            intakeMotor.spin(REVERSE)
            secondMotor.spin(REVERSE)
        elif self.special == 2: #center
            intakeMotor.spin(FORWARD)
            secondMotor.spin(FORWARD)
            flapper.spin(FORWARD)
        elif self.special == 3: #upper
            intakeMotor.spin(FORWARD)
            secondMotor.spin(FORWARD)
            flapper.spin(REVERSE)
        elif self.special == 4:
            ds()
        elif self.special == 5:
            pistons()
        for motor in motors:
            motor.set_velocity(self.velocity, PERCENT)
        if self.action.upper() in ["F", "FWD","FORWARD"]:
            for i, motor in enum(motors):
                motor.spin_for(FORWARD, self.degrees, DEGREES, i == len(motors) - 1)
        elif self.action.upper() in ["B", "BWD", "BACKWARD"]:
            for i, motor in enum(motors):
                motor.spin_for(REVERSE, self.degrees, DEGREES, i == len(motors) - 1)
        elif self.action.upper() in ["L", "LEFT"]:
            for l_motor in L_MOTORS:
                l_motor.spin_for(REVERSE, self.degrees, DEGREES, False)
            for i, r_motor in enum(R_MOTORS):
                r_motor.spin_for(FORWARD, self.degrees, DEGREES, i == len(R_MOTORS) - 1)

        elif self.action.upper() in ["R", "RIGHT"]:
            for l_motor in L_MOTORS:
                l_motor.spin_for(FORWARD, self.degrees, DEGREES, False)
            for i, r_motor in enum(R_MOTORS):
                r_motor.spin_for(REVERSE, self.degrees, DEGREES, i == len(R_MOTORS) - 1)
        elif self.action.upper() in ["W", "WAIT"]:
            sleep(self.degrees) #degrees used as milliseconds here
        intakeMotor.stop()
        secondMotor.stop()
        flapper.stop()

def joyStick():
    speedl = (controller_1.axis3.position() + controller_1.axis1.position()) / (16.6 / sensitivity)
    speedr = (controller_1.axis3.position() - controller_1.axis1.position()) / (16.6 / sensitivity)
    
    motorLeft_1.spin(FORWARD, speedl, VOLT)
    motorLeft_2.spin(FORWARD, speedl, VOLT)
    motorLeft_3.spin(FORWARD, speedl, VOLT)
    motorRight_1.spin(FORWARD, speedr, VOLT)
    motorRight_2.spin(FORWARD, speedr, VOLT)
    motorRight_3.spin(FORWARD, speedr, VOLT)
    wait(10, MSEC)

intakeMotor.set_velocity(70, PERCENT)
secondMotor.set_velocity(70, PERCENT)
flapper.set_velocity(70, PERCENT)

def temp():
    t = 105
    while True:
        motortemps = [round(motor.temperature()) for motor in motors]
        controller_1.screen.clear_screen()
        controller_1.screen.set_cursor(1, 1)
        controller_1.screen.print("L:", motortemps[0], motortemps[1], motortemps[2])
        controller_1.screen.set_cursor(2, 1)
        controller_1.screen.print("R:", motortemps[3], motortemps[4], motortemps[5])
        controller_1.screen.set_cursor(3, 1)
        controller_1.screen.print("Time remaining:", t)
        if 55 in motortemps:
            controller_1.rumble("...")
        sleep(1000)
        if comp.is_driver_control() and comp.is_enabled() and t > 0:
            t -= 1
            if t == 20:
                controller_1.rumble("-")
            if t == 10 or t == 5:
                controller_1.rumble("--")

def pistons():
    tongue.set(not tongue.value())
    aligner.set(not aligner.value())

def ds():
    descorer.set(not descorer.value())

def user_control():
    aligner.set(not aligner.value())
    did_intake = False
    controller_1.axis3.changed(joyStick)
    controller_1.axis1.changed(joyStick)
    controller_1.buttonA.pressed(pistons)
    controller_1.buttonX.pressed(ds)
    while True:
        if controller_1.buttonR1.pressing(): #center
            intakeMotor.spin(FORWARD)
            secondMotor.spin(FORWARD)
            flapper.spin(FORWARD)
        elif controller_1.buttonR2.pressing(): #upper
            intakeMotor.spin(FORWARD)
            secondMotor.spin(FORWARD)
            flapper.spin(REVERSE)
        elif controller_1.buttonL1.pressing(): #lower
            intakeMotor.spin(REVERSE)
            secondMotor.spin(REVERSE)
        elif controller_1.buttonL2.pressing(): #intake
            intakeMotor.spin(FORWARD)
            secondMotor.spin(FORWARD)
        else:
            did_intake = False
            intakeMotor.stop()
            secondMotor.stop()
            flapper.stop()
        if not did_intake and (controller_1.buttonR1.pressing() or controller_1.buttonR2.pressing()):
            intakeMotor.spin(REVERSE)
            secondMotor.spin(REVERSE)
            did_intake = True
        """
        if flapper.torque() > 0.5:
            flapper.stop()
        if secondMotor.torque() > 0.5:
            psecondMotor.stop()
        if intakeMotor.torque() > 0.2:
            intakeMotor.stop()
        """
        wait(20, MSEC)

SPECIAL_auton_actions = [Action("F", 150, 10)]

RIGHT_auton_actions = [
    Action("F", 450, 25),
    Action("R", 150, 15),
    Action("F", 800, 15, 2),
    Action("L", 350, 15, 2),
    Action("F", 600, 15),
    Action("W", 2250, 0, -1),
    Action("B", 2200, 25),
    Action("L", 600, 15, 5),
    Action("F", 575, 30),
    Action("W", 1500, 0, 1),
    Action("W", 0, 0, 5),
    Action("W", 0, 0, 4),
    Action("B", 1250, 30, -1),
    Action("W", 3000, 0 , 3),
]

LEFT_auton_actions = []
for action in RIGHT_auton_actions:
    if action.action == "L":
        LEFT_auton_actions.append(Action("R", action.degrees, action.velocity, action.special))
    if action.action == "R":
        LEFT_auton_actions.append(Action("L", action.degrees, action.velocity, action.special))
    else:
        LEFT_auton_actions.append(Action(action.action, action.degrees, action.velocity, action.special))

SKILLS_auton_actions = [
    Action("F", 100, 10),
    Action("B", 1300, 100),
]

SUPER_SKILLS_auton_actions = RIGHT_auton_actions + [
    Action("F", 300, 25),
    Action("R", 150, 15),
    Action("F", 500, 15, 2),
    Action("F", 200, 25)
]

def autonomous():
    aligner.set(not aligner.value())
    if SIDE == "RIGHT":
        auton_actions = RIGHT_auton_actions.copy()
    elif SIDE == "LEFT":
        auton_actions = LEFT_auton_actions.copy()
    elif SIDE == "SKILLS":
        auton_actions = SKILLS_auton_actions.copy()
        for motor in motors:
            motor.set_max_torque(100, PERCENT)
    elif SIDE == "SPECIAL":
        auton_actions = SPECIAL_auton_actions.copy()
    while True:
        if len(auton_actions) != 0:
            auton_actions[0].execute()
            auton_actions.pop(0)
        wait(20, MSEC)

brain.screen.clear_screen()
brain.screen.print("""
  _   _                               _         _                      
 | \\ | |_   _  __ _ ___   _ __   ___ | |_    __| |_ __ _   _  __ _ ___ 
 |  \\| | | | |/ _` / __| | '_ \\ / _ \\| __|  / _` | '__| | | |/ _` / __|
 | |\\  | |_| | (_| \\__ \\ | | | | (_) | |_  | (_| | |  | |_| | (_| \\__ \\
 |_| \\_|\\__,_|\\__, |___/ |_| |_|\\___/ \\__|  \\__,_|_|   \\__,_|\\__, |___/
              |___/                                          |___/
""")
Thread(temp)
comp = Competition(user_control, autonomous)
