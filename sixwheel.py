# ---------------------------------------------------------------------------- #
#                                                                              #
#   Module:       main.py                                                      #
#   Author:       lsong                                                        #
#   Created:      11/26/2025, 9:11:35 PM                                       #
#   Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

# Brain should be defined by default
brain=Brain()

motorLeft_1 = Motor(Ports.PORT1, GearSetting.RATIO_6_1, False)
motorLeft_2 = Motor(Ports.PORT2, GearSetting.RATIO_6_1, False)
motorLeft_3 = Motor(Ports.PORT3, GearSetting.RATIO_6_1, False)
motorRight_1 = Motor(Ports.PORT5, GearSetting.RATIO_6_1, True)
motorRight_2 = Motor(Ports.PORT6, GearSetting.RATIO_6_1, True)
motorRight_3 = Motor(Ports.PORT7, GearSetting.RATIO_6_1, True)

flapper = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)
intakeMotor = Motor(Ports.PORT11, GearSetting.RATIO_18_1, False)
secondMotor = Motor(Ports.PORT12, GearSetting.RATIO_18_1, True)

controller_1 = Controller()

L_MOTORS = [motorLeft_1, motorLeft_2, motorLeft_3] #All motors on the left
R_MOTORS = [motorRight_1, motorRight_2, motorRight_3] #All motors on the right
motors = L_MOTORS + R_MOTORS
sensitivity = 2

def enum(iterable): #enumerate() disallowed in VEX Python
    index = 0
    for item in iterable:
        yield index, item
        index += 1

class Action: #One unit of action run during auton
    def __init__(self, action, degrees, velocity=50, i_o_id=0):
        self.action = action
        self.degrees = degrees
        self.velocity = velocity
        self.i_o_id = i_o_id

    def execute(self):
        if self.i_o_id == 1:
            intakeMotor.spin(FORWARD)
            secondMotor.spin(FORWARD)
        elif self.i_o_id == -1:
            intakeMotor.spin(REVERSE)
            secondMotor.spin(REVERSE)
        elif self.i_o_id == 2:
            intakeMotor.spin(FORWARD)
            secondMotor.spin(FORWARD)
            flapper.spin(FORWARD)
        elif self.i_o_id == 3:
            intakeMotor.spin(FORWARD)
            secondMotor.spin(FORWARD)
            flapper.spin(REVERSE)
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

auton_actions = [
    Action("F", 450, 25),
    Action("L", 175, 15),
    Action("F", 700, 15, 2),
    Action("L", 525, 15),
    Action("B", 700, 15, 2),
    Action("B", 30, 1, 2)
]

intakeMotor.set_velocity(75, PERCENT)
secondMotor.set_velocity(75, PERCENT)
flapper.set_velocity(50, PERCENT)

def temp():
    while True:
        motortemps = [round(motorLeft_1.temperature()), round(motorLeft_2.temperature()), round(motorLeft_3.temperature()), round(motorRight_1.temperature()), round(motorRight_2.temperature()), round(motorRight_3.temperature())]
        controller_1.screen.clear_screen()
        controller_1.screen.print(motortemps[0], motortemps[1], motortemps[2])
        controller_1.screen.new_line()
        controller_1.screen.print(motortemps[3], motortemps[4], motortemps[5])
        controller_1.screen.new_line()
        if 55 in motortemps:
            controller_1.rumble("...")
        time.sleep(2)

def user_control():
    controller_1.axis3.changed(joyStick)
    controller_1.axis1.changed(joyStick)
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
        elif controller_1.buttonX.pressing(): #manual flapper
            flapper.spin(FORWARD)
        elif controller_1.buttonB.pressing(): #manual flapper reverse
            flapper.spin(REVERSE)
        elif controller_1.buttonY.pressing(): #manual intake
            intakeMotor.spin(FORWARD)
            secondMotor.spin(FORWARD)
        elif controller_1.buttonA.pressing(): #manual intake reverse
            intakeMotor.spin(REVERSE)
            secondMotor.spin(REVERSE)
        else:
            intakeMotor.stop()
            secondMotor.stop()
            flapper.stop()
        if flapper.torque() > 0.5:
            pass #flapper.stop()
        if secondMotor.torque() > 0.5:
            pass #secondMotor.stop()
        if intakeMotor.torque() > 0.2:
            pass #intakeMotor.stop()
        wait(20, MSEC)

def autonomous():
    while True:
        if len(auton_actions) != 0:
            auton_actions[0].execute()
            print(auton_actions[0].i_o_id, auton_actions[0].action)
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
comp = Competition(user_control, autonomous)