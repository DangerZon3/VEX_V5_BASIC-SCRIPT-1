from vex import *
import math

#--------- Program Settings ---------
SIDE = "LEFT" #"LEFT", "RIGHT", or "SKILLS"
DEBUG = True

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
inertial = Inertial(Ports.PORT9)
bdist = Distance(Ports.PORT19)
fdist = Distance(Ports.PORT20)

controller_1 = Controller(PRIMARY)
controller_2 = Controller(PARTNER)

L_MOTORS = [motorLeft_1, motorLeft_2, motorLeft_3] #All motors on the left
R_MOTORS = [motorRight_1, motorRight_2, motorRight_3] #All motors on the right

#---------- Main Program ------------
motors = L_MOTORS + R_MOTORS
posx = 0
posy = 0
sensitivity = 2

intakeMotor.set_velocity(80, PERCENT)
secondMotor.set_velocity(80, PERCENT)
flapper.set_velocity(50, PERCENT)


def enum(iterable): #enumerate() disallowed in VEX Python
    index = 0
    for item in iterable:
        yield index, item
        index += 1

def screen_status(txt: str, row=1):
    controller_1.screen.clear_row(row)
    controller_1.screen.set_cursor(row,1)
    controller_1.screen.print(txt)

def convert(x: float):
    if -1 <= x < -0.75:
        return 0.25 * x - 0.75
    elif -0.75 <= x < -0.25:
        return 1.75 * x + 0.375
    elif -0.25 <= x < 0.25:
        return 0.25 * x
    elif 0.25 <= x < 0.75:
        return 1.75 * x - 0.375
    else:
        return 0.25 * x + 0.75

def joyStick2():
    speedl = (convert(controller_1.axis3.position() / 100) + convert(controller_1.axis1.position() / 100)) / (0.166 / sensitivity)
    speedr = (convert(controller_1.axis3.position() / 100) - convert(controller_1.axis1.position() / 100)) / (0.166 / sensitivity)

    motorLeft_1.spin(FORWARD, speedl, VOLT)
    motorLeft_2.spin(FORWARD, speedl, VOLT)
    motorLeft_3.spin(FORWARD, speedl, VOLT)
    motorRight_1.spin(FORWARD, speedr, VOLT)
    motorRight_2.spin(FORWARD, speedr, VOLT)
    motorRight_3.spin(FORWARD, speedr, VOLT)
    wait(10, MSEC)

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

def temp():
    t = 105
    while True:
        motortemps = [round(motor.temperature()) for motor in motors + [intakeMotor, secondMotor, flapper]]
        controller_1.screen.clear_screen()
        controller_1.screen.set_cursor(1, 1)
        controller_1.screen.print("L:", motortemps[0], motortemps[1], motortemps[2])
        controller_1.screen.set_cursor(2, 1)
        controller_1.screen.print("R:", motortemps[3], motortemps[4], motortemps[5])
        controller_1.screen.set_cursor(3, 1)
        controller_1.screen.print("I:", motortemps[6], motortemps[7], motortemps[8])
        if 55 in motortemps:
            controller_1.rumble("...")
        sleep(1000)
        if comp.is_driver_control() and comp.is_enabled() and t > 0:
            t -= 1
            if t == 20:
                controller_1.rumble("-")
            if t == 10 or t == 5:
                controller_1.rumble("--")

def it(intake):
    for i, vel in enum(intake):
        [intakeMotor, secondMotor, flapper][i].set_velocity(85 * math.fabs(vel), PERCENT)
        [intakeMotor, secondMotor, flapper][i].spin(FORWARD if vel > 0 else REVERSE)

def pistons():
    aligner.set(tongue.value())
    tongue.set(not tongue.value())

def ds():
    descorer.set(not descorer.value())

INTAKE = (1, 1)
LOWER = (-1, -1)
CENTER = (1, 1, -0.5)
UPPER = (1, 1, -1)

class PID:
    def __init__(self, accel: float, inertia: float, drag: float, max_integral: float = 50):
        self.kp = accel
        self.ki = inertia
        self.kd = drag
        self.max_integral = max_integral
        self.previous_error = 0
        self.integral = 0
    
    def calculate(self, error, dt=0.01):
        p_term = self.kp * error
        self.integral += error * dt
        self.integral = max(-self.max_integral, min(self.max_integral, self.integral))
        i_term = self.ki * self.integral
        d_term = self.kd * (error - self.previous_error) / dt if dt > 0 else 0
        self.previous_error = error
        #print("P:", round(p_term, 2), "I:", round(i_term, 2), "D:", round(d_term, 2))
        return p_term + i_term + d_term

    def reset(self):
        self.previous_error = 0
        self.integral = 0

def PIDdrive(target_distance: float, max_voltage: float = 10,
             acceleration: float = 0.04, inertia: float = 0.006, drag: float = 0.00125,
             correction_rate: float = 0.005, timeout: int = 2000, sensor = bdist, debug: bool = False):
    drivePID = PID(acceleration, inertia, drag)
    start_time = brain.timer.time(MSEC)
    start_rotation = inertial.rotation()
    start_distance = sensor.object_distance(MM)
    current_distance = start_distance
    while brain.timer.time(MSEC) - start_time < timeout:
        if sensor.object_distance() != 9999:
            current_distance = sensor.object_distance()
        if sensor == bdist:
            error = target_distance + start_distance - current_distance
        elif sensor == fdist:
            error = target_distance - start_distance + current_distance
        if math.fabs(error) <= 8:
            break
        pid_output = drivePID.calculate(error)
        motor_voltage = max(-max_voltage, min(max_voltage, pid_output))
        if math.fabs(motor_voltage) < 0.3:
            break
        correction = correction_rate * (inertial.rotation() - start_rotation)
        if debug:
            print(bdist.object_distance(), fdist.object_distance())
        for motor in [motorLeft_1, motorLeft_2, motorLeft_3]:
            motor.spin(FORWARD, motor_voltage - correction, VOLT)
        for motor in [motorRight_1, motorRight_2, motorRight_3]:
            motor.spin(FORWARD, motor_voltage + correction, VOLT)
        wait(10, MSEC)
        for motor in [motorLeft_1, motorLeft_2, motorLeft_3, motorRight_1, motorRight_2, motorRight_3]:
            motor.stop()

def PIDrotate(target_degrees: float, max_voltage: float = 10,
              acceleration: float = 0.36, inertia: float = 0.003, drag: float = 0.0002,
              timeout: int = 2000, debug: bool = False):
    rotatePID = PID(acceleration, inertia, drag)
    start_time = brain.timer.time(MSEC)
    start_rotation = inertial.rotation()
    while brain.timer.time(MSEC) - start_time < timeout:
        error = target_degrees + start_rotation - inertial.rotation()
        if math.fabs(error) <= 5:
            break
        pid_output = rotatePID.calculate(error)
        motor_voltage = max(-max_voltage, min(max_voltage, pid_output))
        if debug:
            print(error, motor_voltage)
        for l_motor in [motorLeft_1, motorLeft_2, motorLeft_3]:
            l_motor.spin(FORWARD, motor_voltage, VOLT)
        for r_motor in [motorRight_1, motorRight_2, motorRight_3]:
            r_motor.spin(REVERSE, motor_voltage, VOLT)
        wait(10, MSEC)
        for motor in [motorLeft_1, motorLeft_2, motorLeft_3, motorRight_1, motorRight_2, motorRight_3]:
            motor.stop()

class ActionDeprecated: #One unit of action run during auton
    def __init__(self, action: str, degrees: float, velocity: float = 50, intake: tuple = (), pistons: tuple = (0, 0), stopIntake: bool = True):
        self.action = action
        self.degrees = degrees
        self.velocity = velocity
        self.intake = intake
        self.pistons = pistons
        self.stopIntake = stopIntake
    def execute(self):
        it(self.intake)
        if self.pistons[0]:
            pistons()
        if self.pistons[1]:
            ds()
        for motor in motors:
            motor.set_velocity(self.velocity, PERCENT)
        if self.action.upper() in ["F", "FWD","FORWARD"]:
            for i, motor in enum(motors):
                motor.spin_for(FORWARD, self.degrees, DEGREES, i == len(motors) - 1)
        elif self.action.upper() in ["FT","FORWARD TIME"]:
            for i, motor in enum(motors):
                motor.spin(FORWARD, self.velocity, PERCENT)
            wait(self.degrees, MSEC) #use degrees as milliseconds here
            for i, motor in enum(motors):
                motor.stop()
        elif self.action.upper() in ["B", "BWD", "BACKWARD"]:
            for i, motor in enum(motors):
                motor.spin_for(REVERSE, self.degrees, DEGREES, i == len(motors) - 1)
        elif self.action.upper() in ["BT","BACKWARD TIME"]:
            for i, motor in enum(motors):
                motor.spin(REVERSE, self.velocity, PERCENT)
            wait(self.degrees, MSEC) #use degrees as milliseconds here
            for i, motor in enum(motors):
                motor.stop()
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
        if self.stopIntake:
            intakeMotor.stop()
            secondMotor.stop()
            flapper.stop()

class Action: #One unit of action run during auton
    def __init__(self, action: str, amount: float, max_voltage: float = 10,
                 correction_rate: float = 0.7, timeout: int = 2000,
                 intake: tuple = (), pistons: tuple = (0, 0), sensor = bdist):
        self.action = action
        self.amount = amount
        self.max_voltage = max_voltage
        self.correction_rate = correction_rate
        self.timeout = timeout
        self.intake = intake
        self.pistons = pistons
        self.sensor = sensor
    def execute(self):
        it(self.intake)
        if self.pistons[0]:
            pistons()
        if self.pistons[1]:
            ds()
        if self.action.upper() in ["M", "MOVE"]:
            PIDdrive(self.amount, max_voltage=10,
                     correction_rate=self.correction_rate, timeout=self.timeout, sensor=self.sensor, debug=DEBUG)
        elif self.action.upper() in ["T", "TURN"]:
            PIDrotate(self.amount, max_voltage=10,
                      timeout=self.timeout)
        elif self.action.upper() in ["W", "WAIT"]:
            sleep(self.amount)
        for motor in motors:
            motor.stop()
        for intakemotor in [intakeMotor, secondMotor]:
            intakemotor.set_velocity(80, PERCENT)
            intakemotor.stop()
        flapper.set_velocity(50, PERCENT)
        flapper.stop()

def user_control():
    did_intake = False
    controller_1.axis3.changed(joyStick2)
    controller_1.axis1.changed(joyStick2)
    controller_1.buttonA.pressed(pistons)
    controller_1.buttonX.pressed(ds)
    while True:
        if controller_1.buttonR1.pressing():
            it(CENTER)
        elif controller_1.buttonR2.pressing():
            it(UPPER)
        elif controller_1.buttonL1.pressing():
            it(LOWER)
        elif controller_1.buttonL2.pressing():
            it(INTAKE)
        else:
            did_intake = False
            intakeMotor.stop()
            secondMotor.stop()
            flapper.stop()
        if not did_intake and (controller_1.buttonR1.pressing() or controller_1.buttonR2.pressing()):
            intakeMotor.spin(REVERSE)
            secondMotor.spin(REVERSE)
            did_intake = True
        wait(20, MSEC)

r_auton = [
    Action("M", 600, intake=INTAKE),
    Action("T", 45),
    Action("M", 300, max_voltage=6, intake=INTAKE),
    Action("T", -90),
]

#l_auton = [
   #Action("M", 415),
   #Action("T", 45),
   #Action("M", 375, max_voltage=6),
    #Action("T", -90, intake=INTAKE),
    #ActionDeprecated("F", 1225, 20, intake=INTAKE),
    #Action("W", 100),
    #ActionDeprecated("B", 300, 10),
    #Action("T", -85),
    #ActionDeprecated("B", 725, 20),
    #Action("W", 750, intake=CENTER, pistons=(0, 1)),
    #ActionDeprecated("F", 1000, 20, pistons=(0, 1)),
    #Action("M", 1000, max_voltage=10, sensor=fdist),
#]
l_auton = [
    #Action("T", 20, max_voltage=5),
    Action("M", 300, max_voltage=7, intake=INTAKE),
    ActionDeprecated("F", 1000, 25, intake=INTAKE),
    Action("W", 100),
    ActionDeprecated("B", 350, 15),
    Action("T", -105, max_voltage=8),
    ActionDeprecated("B", 785, 20),
    Action("W", 1000, intake=CENTER, pistons=(0, 1)),
    ActionDeprecated("F", 60, 25, pistons=(0, 1)),
    Action("T", 15, max_voltage=12),
    ActionDeprecated("F", 1800, 25, pistons=(1, 1)),
    Action("T", -55, max_voltage=10),
    ActionDeprecated("F", 900, 40),
    Action("W", 1000, intake=INTAKE),
    Action("M", -650, max_voltage=10, sensor=fdist, pistons=(1, 0)),
    Action("W", 2000, intake=UPPER)
]

auton = l_auton if SIDE == "LEFT" else r_auton

def autonomous():
    while len(auton) > 0:
        auton.pop(0).execute()
        wait(10, MSEC)

inertial.calibrate() #Calibrate Inertial sensor
while inertial.is_calibrating():
    screen_status("Calibrating")
    wait(100)
screen_status("Calibration Complete")
while True:
    start_distance = bdist.object_distance()
    print("Starting back distance:", start_distance)
    if start_distance < 5000:
        break
    wait(500, MSEC)

brain.screen.clear_screen()
Thread(temp)
comp = Competition(user_control, autonomous)