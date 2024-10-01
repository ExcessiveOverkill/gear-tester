import odrive
import time
import csv
from tkinter import *
import random

programStatus="uncalibrated"
requestedProgramStatus="none"
calibrated=False
servosEnabled=False
maTorque=0
mbTorque=0
error=[]

settings={}
with open('tireTesterSettings.csv', mode='r') as csvfile:
    reader = csv.reader(csvfile, delimiter=':', dialect="unix")
    settings = {rows[0]:rows[1] for rows in reader}

NmperA=float(settings["torque_constant"])
frictionData=[int(settings["calibration_steps"])]
maxTorque=int(settings["max_torque"])
maxSpeed=int(settings["max_speed"])
try:
    servos = odrive.find_any(timeout=3)
    if servos.vbus_voltage < 5:
        programStatus="disconnected"
except:
    programStatus="disconnected"
root = Tk()
root.title("Tire Tester")
#root.iconbitmap('logo.ico')
servoFrame = Frame(root, height=650, width=250, highlightbackground="black", highlightthickness=1)
servoFrame.grid()
servoFrame.grid_propagate(0)

UIframe = Frame(root)
UIframe.grid(column=1, row=0)

controlFrame = Frame(UIframe, height=50, width=600, highlightbackground="black", highlightthickness=1)
controlFrame.grid(column=0, row=0)
controlFrame.grid_propagate(0)

calibrationFrame = Frame(UIframe, height=80, width=600, highlightbackground="black", highlightthickness=1)
calibrationFrame.grid(column=0, row=1)
calibrationFrame.grid_propagate(0)

staticFrictionFrame = Frame(UIframe, height=90, width=600, highlightbackground="black", highlightthickness=1)
staticFrictionFrame.grid(column=0, row=4)
staticFrictionFrame.grid_propagate(0)

kineticFrictionFrame = Frame(UIframe, height=90, width=600, highlightbackground="black", highlightthickness=1)
kineticFrictionFrame.grid(column=0, row=4)
kineticFrictionFrame.grid_propagate(0)

wearFrame = Frame(UIframe, height=130, width=600, highlightbackground="black", highlightthickness=1)
wearFrame.grid(column=0, row=3)
wearFrame.grid_propagate(0)


#Status/Control Section
statusText  = StringVar()
status = Label(controlFrame, anchor="w", textvariable=statusText, width=30, relief=SUNKEN)
statusText.set("This will show the status")
status.grid(padx=3, pady=1)

desc = Entry(controlFrame, relief=SUNKEN, width=35)
desc.grid(column=0, row=1, padx=3, pady=1)

mode = IntVar()

staticFrictionDoneDistVar = StringVar()
staticFrictionTorqueRampRateVar = StringVar()
staticFrictionMaxTorqueVar = StringVar()

kineticFrictionMaxTorqueVar = StringVar()
kineticFrictionVelRampRateVar = StringVar()
kineticFrictionMaxVelVar = StringVar()

wearSpeedVar = StringVar()
wearMaxTorqueVar = StringVar()

#User input variables

staticFrictionDoneDistVar.set(settings["static_friction_done_dist"])
staticFrictionTorqueRampRateVar.set(settings["static_friction_torque_ramp_rate"])
staticFrictionMaxTorqueVar.set(settings["static_friction_max_torque"])

kineticFrictionMaxTorqueVar.set(settings["kinetic_friction_max_torque"])
kineticFrictionVelRampRateVar.set(settings["kinetic_friction_vel_ramp_rate"])
kineticFrictionMaxVelVar.set(settings["kinetic_friction_max_vel"])

wearSpeedVar.set(settings["wear_speed"])
wearMaxTorqueVar.set(settings["wear_max_torque"])



def Start():    #start selected test
    global programStatus
    global requestedProgramStatus
    if(mode.get()==0):
        requestedProgramStatus="calibration"
    if(mode.get()==1):
        requestedProgramStatus="staticFriction"
    if(mode.get()==2):
        requestedProgramStatus="kineticFriction"
    if(mode.get()==3):
        requestedProgramStatus="wear"
    saveSettings()
    

startBtn  = Button(controlFrame, text="START", bg="green", pady=4, width=12, command=Start)
startBtn.grid(column=1, row=0, pady=2, padx=7, rowspan=2)

def Stop():     #stop current action
    global programStatus
    global requestedProgramStatus
    if (programStatus != "idle"):
        requestedProgramStatus="stop"
    return

stopBtn  = Button(controlFrame, text="STOP", bg="red", pady=4, width=12, command=Stop)
stopBtn.grid(column=2, row=0, pady=2, padx=7, rowspan=2)

#Calibration Section

calibrateRbtn = Radiobutton(calibrationFrame, text="Calibration", variable=mode, value=0)
calibrateRbtn.grid(padx=2, pady=2, sticky="nw")

albl = Label(calibrationFrame, text="Motor A")
albl.grid(column=1, row=1, padx=6)

blbl = Label(calibrationFrame, text="Motor B")
blbl.grid(column=1, row=2, padx=6)

encoderlbl = Label(calibrationFrame, text="Encoder Offset")
encoderlbl.grid(column=2, row=0, padx=8)

frictionlbl = Label(calibrationFrame, text="Friction Offset")
frictionlbl.grid(column=3, row=0, padx=8)

aEncoderOffset = StringVar()
aeOffset = Label(calibrationFrame, textvariable=aEncoderOffset)
aeOffset.grid(column=2, row=1)

bEncoderOffset = StringVar()
beOffset = Label(calibrationFrame, textvariable=bEncoderOffset)
beOffset.grid(column=2, row=2)

aFrictionOffset = StringVar()
afOffset = Label(calibrationFrame, textvariable=aFrictionOffset)
afOffset.grid(column=3, row=1)

bFrictionOffset = StringVar()
bfOffset = Label(calibrationFrame, textvariable=bFrictionOffset)
bfOffset.grid(column=3, row=2)


#Static Friction Test Section
staticRbtn = Radiobutton(staticFrictionFrame, text="Static Friction", variable=mode, value=1)
staticRbtn.grid(columnspan=3, padx=2, pady=2, sticky="nw")

staticTorqueEnt = Entry(staticFrictionFrame, relief=SUNKEN, width=5, justify=CENTER, textvariable=staticFrictionMaxTorqueVar)
staticTorqueEnt.grid(column=1, row=2, padx=2, pady=2, sticky="e")

staticTorquelbl = Label(staticFrictionFrame, text="Max Torque (Nm)")
staticTorquelbl.grid(column=2, row=2, padx=2, sticky="w")

staticRampEnt = Entry(staticFrictionFrame, relief=SUNKEN, width=5, justify=CENTER, textvariable=staticFrictionTorqueRampRateVar)
staticRampEnt.grid(column=4, row=1, padx=2, pady=2, sticky="e")

staticRamplbl = Label(staticFrictionFrame, text="Ramp (Nm/s)")
staticRamplbl.grid(column=5, row=1, padx=2, sticky="w")

staticDistanceEnt = Entry(staticFrictionFrame, relief=SUNKEN, width=5, justify=CENTER, textvariable=staticFrictionDoneDistVar)
staticDistanceEnt.grid(column=4, row=2, padx=2, pady=2, sticky="e")

staticDistancelbl = Label(staticFrictionFrame, text="Slip Distance (turns)")
staticDistancelbl.grid(column=5, row=2, padx=2, sticky="w")

staticSpaceA = Frame(staticFrictionFrame, width=25)
staticSpaceA.grid_propagate(0)
staticSpaceA.grid(rowspan=2, column=0, row=1)

staticSpaceB = Frame(staticFrictionFrame, width=42)
staticSpaceB.grid_propagate(0)
staticSpaceB.grid(rowspan=2, column=3, row=1)


#Kinetic Friction Test Section
kineticRbtn = Radiobutton(kineticFrictionFrame, text="Kinetic Friction", variable=mode, value=1)
kineticRbtn.grid(columnspan=3, padx=2, pady=2, sticky="nw")

kineticTorqueEnt = Entry(kineticFrictionFrame, relief=SUNKEN, width=5, justify=CENTER, textvariable=kineticFrictionMaxTorqueVar)
kineticTorqueEnt.grid(column=1, row=2, padx=2, pady=2, sticky="e")

kineticTorquelbl = Label(kineticFrictionFrame, text="Max Torque (Nm)")
kineticTorquelbl.grid(column=2, row=2, padx=2, sticky="w")

kineticRampEnt = Entry(kineticFrictionFrame, relief=SUNKEN, width=5, justify=CENTER, textvariable=kineticFrictionVelRampRateVar)
kineticRampEnt.grid(column=4, row=1, padx=2, pady=2, sticky="e")

kineticRamplbl = Label(kineticFrictionFrame, text="Vel Ramp (rpm/s)")
kineticRamplbl.grid(column=5, row=1, padx=2, sticky="w")

kineticSpaceA = Frame(kineticFrictionFrame, width=25)
kineticSpaceA.grid_propagate(0)
kineticSpaceA.grid(rowspan=2, column=0, row=1)

kineticSpaceB = Frame(kineticFrictionFrame, width=42)
kineticSpaceB.grid_propagate(0)
kineticSpaceB.grid(rowspan=2, column=3, row=1)


#Wear Test Section
wearRbtn = Radiobutton(wearFrame, text="Wear", variable=mode, value=2)
wearRbtn.grid(columnspan=3, padx=2, pady=2, sticky="w")

wearSpeedEnt = Entry(wearFrame, relief=SUNKEN, width=5, justify=CENTER, textvariable=wearSpeedVar)
wearSpeedEnt.grid(column=1, row=1, padx=2, pady=2, sticky="e")

wearSpeedlbl = Label(wearFrame, text="Speed (rpm)")
wearSpeedlbl.grid(column=2, row=1, padx=2, sticky="w")

wearTorqueEnt = Entry(wearFrame, relief=SUNKEN, width=5, justify=CENTER, textvariable=wearMaxTorqueVar)
wearTorqueEnt.grid(column=1, row=2, padx=2, pady=2, sticky="e")

wearTorquelbl = Label(wearFrame, text="Max Torque (Nm)")
wearTorquelbl.grid(column=2, row=2, padx=2, sticky="w")

wearSpaceA = Frame(wearFrame, width=25)
wearSpaceA.grid_propagate(0)
wearSpaceA.grid(rowspan=3, column=0, row=1)

wearSpaceB = Frame(wearFrame, width=20)
wearSpaceB.grid_propagate(0)
wearSpaceB.grid(rowspan=3, column=3, row=1)


Aspeed = DoubleVar()
Bspeed = DoubleVar()
Atorque = DoubleVar()
Btorque = DoubleVar()

SpeedInfolbl = Label(servoFrame, text="Speed (rpm)")
SpeedInfolbl.grid(columnspan=2, column=1, row=0, padx=2)

TorqueInfolbl = Label(servoFrame, text="Torque (Nm)")
TorqueInfolbl.grid(columnspan=2, column=5, row=0, padx=2)

# servoAspeedlbl = Label(servoFrame, text="A")
# servoAspeedlbl.grid(column=1, row=1, padx=5, sticky="e")

# servoBspeedlbl = Label(servoFrame, text="B")
# servoBspeedlbl.grid(column=2, row=1, padx=5, sticky="w")

# servoAtorquelbl = Label(servoFrame, text="A")
# servoAtorquelbl.grid(column=5, row=1, padx=5, sticky="e")

# servoBtorquelbl = Label(servoFrame, text="B")
# servoBtorquelbl.grid(column=6, row=1, padx=5, sticky="w")

AspeedScl = Scale(servoFrame, variable=Aspeed, from_=maxSpeed, to=-maxSpeed, orient=VERTICAL, showvalue=0, sliderlength=10, length=600, width=40, takefocus=0, troughcolor="green", border=4, resolution=.1, tickinterval=int(settings["speed_scale_divs"]), state=DISABLED)
AspeedScl.grid(columnspan=2, column=0, row=2, sticky="e")

# BspeedScl = Scale(servoFrame, variable=Bspeed, from_=maxSpeed, to=-maxSpeed, orient=VERTICAL, showvalue=0, sliderlength=4, length=400, width=15, takefocus=0, troughcolor="green", border=1, resolution=.1, state=DISABLED)
# BspeedScl.grid(column=2, row=2, sticky="w")

AtorqueScl = Scale(servoFrame, variable=Atorque, from_=maxTorque, to=-maxTorque, orient=VERTICAL, showvalue=0, sliderlength=10, length=600, width=40, takefocus=0, troughcolor="red", border=4, resolution=.01, tickinterval=10, state=DISABLED)
AtorqueScl.grid(columnspan=2, column=4, row=2, sticky="e")

# BtorqueScl = Scale(servoFrame, variable=Btorque, from_=maxTorque, to=-maxTorque, orient=VERTICAL, showvalue=0, sliderlength=4, length=400, width=15, takefocus=0, troughcolor="red", border=1, resolution=.01, state=DISABLED)
# BtorqueScl.grid(column=6, row=2, sticky="w")

servoSpaceA = Frame(servoFrame, width=20)
servoSpaceA.grid_propagate(0)
servoSpaceA.grid(rowspan=3, column=3, row=0)

servoSpaceB = Frame(servoFrame, width=15)
servoSpaceB.grid_propagate(0)
servoSpaceB.grid(column=0, row=1)

servoSpaceC = Frame(servoFrame, width=15)
servoSpaceC.grid_propagate(0)
servoSpaceC.grid(column=4, row=1)

AspeedFilter=[0.0]
BspeedFilter=[0.0]
for i in range(int(settings["speed_filter_size"])):     #speed filter
    AspeedFilter.append(0)
    BspeedFilter.append(0)

def updateSpeed():      #filter and update UI speed readouts
    global error
    global requestedProgramStatus
    try:
        As = servos.axis0.encoder.vel_estimate*60
        Bs = servos.axis1.encoder.vel_estimate*60
    except:
        requestedProgramStatus="stop"
        error.append("Could not read motor speed(Odrive fail)")
        
    if(abs(As)<=int(settings["max_speed"])):
        AspeedFilter.pop(0)
        AspeedFilter.append(As)
    
    if(abs(Bs)<=int(settings["max_speed"])):
        BspeedFilter.pop(0)
        BspeedFilter.append(Bs)
    
    Aspeed.set(sum(AspeedFilter)/len(AspeedFilter))
    Bspeed.set(sum(BspeedFilter)/len(BspeedFilter))
    
def updateTorque():     #update UI torque readouts
    if(servosEnabled):
        Atorque.set(measureTorque("A", 0))
        Btorque.set(measureTorque("B", 0))
    else:
        Atorque.set(0)
        Btorque.set(0)

def backgroundUpdate():     #background update(UI/error checking/com checking)
    global programStatus  
    waitUntil = time.time()
    while (time.time() < waitUntil+.01 and programStatus != "stop"):    #small delay to keep usb bandwidth lower
        x=1
    if(programStatus != "disconnected"):
        #print("Vbus is: " + str(getOdriveBusVoltage()))
        updateSpeed()
        updateTorque()
    root.update()

def wait(seconds):  #simple wait function with needed background updates
    waitUntil = time.time()
    while (time.time() < waitUntil+seconds and requestedProgramStatus != "stop"):
        backgroundUpdate()
    return

def AservoAcc(period):      #returns acceleration of servo "A"
    waitUntil = time.time()
    Ta = servos.axis0.encoder.vel_estimate
    while (time.time() < waitUntil+period and programStatus != "stop"):
        x=1
    acc = ((servos.axis0.encoder.vel_estimate)-Ta)/period
    #print(acc)
    return acc

def setTorque(config, torque, direction):   #set torque to be output (internally friction compensated)
    global error
    global requestedProgramStatus
    #convert & offset torque (odrive converts torque to correct current)

    try:
        if(config=="A"):
            if(direction==3):
                if(servos.axis0.encoder.vel_estimate>0):
                    torque *= -1
                direction = 0
            servos.axis0.controller.input_torque = (torque - frictionOffset("A", direction))
            #print("set torque A to: " + str(servos.axis0.controller.input_torque))
        if(config=="B"):
            if(direction==3):
                if(servos.axis1.encoder.vel_estimate>0):
                    torque *= -1
                direction = 0
            servos.axis1.controller.input_torque = (torque - frictionOffset("B", direction))
            #print("set torque B to: " + str(servos.axis1.controller.input_torque))
    except:
        requestedProgramStatus="stop"
        error.append("Could not set motor torque(Odrive error)")

AtorqueFilter=[0.0]
BtorqueFilter=[0.0]
for i in range(int(settings["torque_filter_size"])):    #make filter for torque averaging
    AtorqueFilter.append(0)
    BtorqueFilter.append(0)

def measureTorque(config, direction):   #return filtered & compensated torque output from servo
    global error
    global requestedProgramStatus
    try:
        Ac = servos.axis0.motor.current_control.Iq_measured ###get currents from odrive###
        Bc = servos.axis1.motor.current_control.Iq_measured
    except:
        requestedProgramStatus="stop"
        error.append("Could not measure motor torque(Odrive error)")
        return 0
    Ac = Ac*NmperA + frictionOffset("A", direction)
    Bc = Bc*NmperA + frictionOffset("B", direction)
    AtorqueFilter.pop(0)
    BtorqueFilter.pop(0)
    AtorqueFilter.append(Ac)
    BtorqueFilter.append(Bc)
    Ac = sum(AtorqueFilter)/len(AtorqueFilter)
    Bc = sum(BtorqueFilter)/len(BtorqueFilter)
    #convert & offset torque
    if(config=="A"):
        Ac = sum(AtorqueFilter)/len(AtorqueFilter)
        return Ac
        
    if(config=="B"):
        Bc = sum(BtorqueFilter)/len(BtorqueFilter)
        return Bc

def frictionOffset(config, direction=0):    #return friction offset of servo at current speed
    global error
    global requestedProgramStatus
    if(calibrated == False or direction == 2):
        return 0
    if(config == "A"):
        speed=servos.axis0.encoder.vel_estimate*60
        motorNum=1
    if(config == "B"):
        speed=servos.axis1.encoder.vel_estimate*60
        motorNum=2

    #handle overspeeds
    if(speed > maxSpeed):
        speed = maxSpeed - .001
    if(speed < -maxSpeed):
        speed = -maxSpeed + .001
    

    try:
        offsetNum = (int(speed / float(settings["max_speed"]) * float(frictionData[0]))) + frictionData[0]
        #if (abs(speed)%frictionData[0]==0):
        #    return frictionData[offsetNum][motorNum]
        if (offsetNum<frictionData[0]):
            return ((abs(speed)%frictionData[0])/frictionData[0]) * (frictionData[offsetNum][motorNum]-frictionData[offsetNum+1][motorNum]) + frictionData[offsetNum][motorNum]#abs(speed)
        if (offsetNum>frictionData[0]):
            return ((speed%frictionData[0])/frictionData[0]) * (frictionData[offsetNum][motorNum]+abs(frictionData[offsetNum-1][motorNum])) + frictionData[offsetNum][motorNum]
        if (offsetNum==frictionData[0]):
            if(direction==1):
                return frictionData[frictionData[0]+2][motorNum]
            if(direction==-1):
                return -frictionData[frictionData[0]+1][motorNum]
            if(direction==0):
                if(speed<0):
                    return frictionData[frictionData[0]+1][motorNum]
                if(speed>0):
                    return frictionData[frictionData[0]][motorNum]
                if(speed==0):
                    return 0
        return frictionData[offsetNum][motorNum]
            
    except:
        requestedProgramStatus="stop"
        error.append("Could not calculate friction offset(math error)")
        return 0

def setSpeed(config, speed):    #directly set servo speed
    global error
    global requestedProgramStatus
    try:
        if (config=="A"):
            servos.axis0.controller.input_vel=speed/60
        if (config=="B"):
            servos.axis1.controller.input_vel=speed/60
    except:
        requestedProgramStatus="stop"
        error.append("Could not set motor speed(Odrive fail)")

posOffset = 0
def resetPosDifference():   #set position difference of gears to zero
    global error
    global requestedProgramStatus
    global posOffset
    try:
        posOffset = servos.axis0.encoder.pos_estimate - servos.axis1.encoder.pos_estimate
    except:
        requestedProgramStatus="stop"
        error.append("Could not set position offset(Odrive fail)")

def posDifference():    #return position difference of gears
    global posOffset
    global error
    global requestedProgramStatus
    try:
        return servos.axis0.encoder.pos_estimate - servos.axis1.encoder.pos_estimate - posOffset
    except:
        requestedProgramStatus="stop"
        error.append("Could not measure position difference(Odrive fail)")
        return 0

def enableServos():     #enable servo power
    global servosEnabled
    global error
    global requestedProgramStatus
    try:
        servos.axis0.controller.config.control_mode=2
        servos.axis1.controller.config.control_mode=2
        servos.axis0.controller.input_vel=0
        servos.axis1.controller.input_vel=0
        servos.axis0.controller.input_torque=0
        servos.axis1.controller.input_torque=0
        servos.axis0.requested_state=8
        servos.axis1.requested_state=8
        x=0
        while((servos.axis0.current_state != 8 or servos.axis1.current_state != 8) and requestedProgramStatus!="stop"):
                wait(.1)
                x+=1
                if(x>=5):
                    requestedProgramStatus = "stop"
                    error.append("Could not enable motors(Odrive fail)")
    except:
        requestedProgramStatus = "stop"
        disableServos()
        error.append("Could not enable motors(Odrive fail)")
    servosEnabled = True
        
def getOdriveBusVoltage():  #return Odrive input voltage
    global error
    try:
        return servos.vbus_voltage
    except:
        requestedProgramStatus = "stop"
        disableServos()
        error.append("Could not read bus voltage(Odrive fail)")

def disableServos():    #disable servo power
    global servosEnabled
    global error
    global requestedProgramStatus
    try:
        servos.axis0.requested_state=1
        servos.axis1.requested_state=1
        servos.axis0.controller.input_torque=0
        servos.axis1.controller.input_torque=0
        servos.axis0.controller.input_vel=0
        servos.axis1.controller.input_vel=0
    except:
        requestedProgramStatus = "rebootOdrive"
        error.append("Could not disable motors(Odirve fail), try reboot")
    servosEnabled = False

def orientServos(pos):    #orient servo A to input position(encoder counts), servo B will stay synced
    global error
    global requestedProgramStatus
    try:
        servos.axis0.requested_state=1
        servos.axis1.requested_state=1
        limA = servos.axis0.controller.config.vel_limit
        limB = servos.axis1.controller.config.vel_limit
        servos.axis0.controller.config.input_mode=1
        servos.axis1.controller.config.input_mode=1
        servos.axis0.controller.config.vel_limit = .5
        servos.axis1.controller.config.vel_limit = .5
        

        pos += .23  #offest so pos 0 is horizontal visually


        #calculate needed position move
        orientOffset = pos - (servos.axis0.encoder.pos_cpr_counts/servos.axis0.encoder.config.cpr)

        #set servos to orient position
        servos.axis0.controller.config.control_mode=3
        servos.axis1.controller.config.control_mode=3
        servos.axis0.requested_state=8
        servos.axis1.requested_state=8
        servos.axis0.controller.input_pos += orientOffset
        servos.axis1.controller.input_pos += orientOffset

        wait(2)
        servos.axis0.controller.config.vel_limit = limA
        servos.axis1.controller.config.vel_limit = limB
    except:
        requestedProgramStatus="stop"
        error.append("Could not orient servos(Odrive fail)")

aEncoderOffset.set("? counts")
bEncoderOffset.set("? counts")
aFrictionOffset.set("? Nm")
bFrictionOffset.set("? Nm")

def calibrationMode():      #Run calibration procedure to find self friction offsets
    global error
    global programStatus
    global requestedProgramStatus
    global calibrated
    if(requestedProgramStatus == "calibration" and (programStatus=="idle" or programStatus=="uncalibrated")):
        requestedProgramStatus = ""
        programStatus = "calibration"
        statusText.set("Finding Index")
        print("Vbus is: " + str(getOdriveBusVoltage()))
        if(getOdriveBusVoltage() < 50):
            requestedProgramStatus = "none"
            programStatus = "uncalibrated"
            error.append("Vbus is too low (below 50v), variac should be 65%")
            print("error added")
            statusText.set("Error: vbus <50v, variac should be 65%")
            return
        try:
            x=0
            while((servos.axis0.encoder.vel_estimate != 0 or servos.axis1.encoder.vel_estimate != 0) and requestedProgramStatus=="" and programStatus=="calibration"):
                servos.axis0.requested_state=1
                servos.axis1.requested_state=1
                wait(.5)
                x+=1
                if(x>=6):
                    requestedProgramStatus = "stop"
                    error.append("Could not stop motors for calibration")
                    statusText.set("Error: servos not stopped")
                    return
            x=0
            while((servos.axis0.encoder.is_ready == False or servos.axis1.encoder.is_ready == False) and requestedProgramStatus=="" and programStatus=="calibration"):
                servos.axis0.requested_state=6
                servos.axis1.requested_state=6
                wait(4)
                if(x>1):
                    servos.axis0.requested_state=7
                    servos.axis1.requested_state=7
                    wait(1)
                    servos.axis0.requested_state=1
                    servos.axis1.requested_state=1
                    wait(1)
                    servos.axis0.requested_state=6
                    servos.axis1.requested_state=6
                if(x>=5):
                    requestedProgramStatus = "stop"
                    error.append("Could not find motor index")
                    statusText.set("Error: index not found")
                    return
                x+=1
        except:
            requestedProgramStatus = "stop"
            error.append("Could not find motor index(Odrive fail)")
            return
        wait(1)
        enableServos()
        calibrated=False
        frictionData.clear()
        frictionData.append(int(settings["calibration_steps"]))
        for i in range(frictionData[0]):
            value=[(-maxSpeed/frictionData[0])*(frictionData[0]-i)]
            frictionData.append(value)
        for i in range(frictionData[0]):
            value=[(maxSpeed/frictionData[0])*(i+1)]
            frictionData.append(value)
    if programStatus == "calibration" and requestedProgramStatus=="":
        statusText.set("Calibrating Friction Offsets")
        servos.axis0.controller.config.control_mode=2   #set both servos to velocity mode
        servos.axis1.controller.config.control_mode=2
        servos.axis0.controller.config.input_mode=2     #set to velocity ramp mode for smoother speed
        servos.axis1.controller.config.input_mode=2
        servos.axis0.controller.config.vel_ramp_rate=5
        servos.axis1.controller.config.vel_ramp_rate=5
        servos.axis0.controller.config.anticogging.anticogging_enabled=False
        servos.axis1.controller.config.anticogging.anticogging_enabled=False
        servos.axis0.controller.config.vel_limit = maxSpeed
        servos.axis1.controller.config.vel_limit = maxSpeed
        setSpeed("A", frictionData[1][0])
        setSpeed("B", frictionData[1][0])
        wait(int(settings["servo_warmup_time"]))                                 #motor warm up delay
        for i in range(len(frictionData)-1):
            setSpeed("A", frictionData[i+1][0])
            setSpeed("B", frictionData[i+1][0])
            wait(float(settings["calibration_settling_time"]))
            maTorque = measureTorque("A", 0)*1
            mbTorque = measureTorque("B", 0)*1
            frictionData[i+1].append(maTorque)
            frictionData[i+1].append(mbTorque)
            if (requestedProgramStatus=="stop"):
                disableServos()
                return
        setSpeed("A", 0)
        setSpeed("B", 0)
        wait(1)
        orientServos(0)
        disableServos()
        aEncoderOffset.set(str(servos.axis0.encoder.config.index_offset) + " counts")
        bEncoderOffset.set(str(servos.axis1.encoder.config.index_offset) + " counts")
        aFrictionOffset.set(str(round(frictionData[frictionData[0]+1][1], 2)) + " Nm")
        bFrictionOffset.set(str(round(frictionData[frictionData[0]+1][2], 2)) + " Nm")
        calibrated=True
        programStatus = "idle"
        return
    if programStatus=="calibration" and requestedProgramStatus!="":
        disableServos()
        programStatus = "uncalibrated"
        statusText.set("error: task interrupted(calibration)")

def saveResults(results):   #save results to a .csv file
    global error
    fileName = "results/" + str(results[2][1]) + " " + str(results[1][1]).replace(":", "_")[3:] + ".csv"
    try:
        with open(fileName, 'x') as f:
            writer = csv.writer(f, dialect="unix")
            for k in results:
                writer.writerow(k)
    except:
        statusText.set("error: results file already exists")
        error.append("CSV file already exits and cannot be overwritten")

def staticMode():    #Run static friction test 
    global error
    global programStatus
    global requestedProgramStatus
    if(requestedProgramStatus=="staticFriction" and programStatus=="idle"):
        requestedProgramStatus=""
        programStatus="staticFriction"
        statusText.set("Preparing Static Friction Test")
        enableServos()
        try:
            servos.axis0.controller.config.control_mode=2   #set servo to velocity mode
            servos.axis1.controller.config.control_mode=2
            servos.axis0.controller.config.input_mode=2     #set to velocity ramp mode for smoother speed
            servos.axis1.controller.config.input_mode=2
            servos.axis0.controller.config.vel_ramp_rate=.2  #set slow ramp rate
            servos.axis1.controller.config.vel_ramp_rate=.2
            servos.axis1.controller.input_vel=float(breakSpeedVar.get())/60 #set both servos to running speed
            servos.axis0.controller.input_vel=float(breakSpeedVar.get())/60
        except:
            requestedProgramStatus="stop"
            programStatus="idle"
            disableServos()
            error.append("Could not prepare motors for break test(Odrive fail)")
            return
        wait(float(breakSpeedVar.get())/60/.2 + 2)                         #wait for servos to reach set speed
        if(requestedProgramStatus=="stop"):
            programStatus="idle"
            statusText.set("error: task interrupted")
            return
        resetPosDifference()
        results = []
        results.append(["Test Result", "none"])
        results.append(["Timestamp", time.asctime( time.localtime(time.time()) )])
        results.append(["Test Type", "Breaking Torque"])
        results.append(["Gear Description", desc.get()])
        results.append(["Running Speed", breakSpeedVar.get()])
        results.append(["Maximum Test Torque", breakTorqueVar.get()])
        results.append(["Torque Ramp Rate", breakTorqueRampVar.get()])
        results.append(["Test Fail Distance", breakDistanceVar.get()])
        results.append([])

        if(breakIncludeFriction.get()==1):
            results.append(["Time Since Start (s)", "Torque (Nm)", "Position Difference (turns)", "Friction (Nm)"])
        else:
            results.append(["Time Since Start (s)", "Torque (Nm)", "Position Difference (turns)"])

        try:
            servos.axis1.controller.config.control_mode=1       #set servo to begin applying max torque at specific ramp rate
            servos.axis1.controller.config.input_mode=6
            servos.axis1.controller.config.torque_ramp_rate=float(breakTorqueRampVar.get())/60
            setTorque("B", -float(breakTorqueVar.get()), 1)
        except:
            requestedProgramStatus="stop"
            programStatus="idle"
            disableServos()
            error.append("Could not start break test(Odrive fail)")
            return
        posDiff=posDifference()
        #print(posDiff)
        testOver=False
        testEndTime = time.time() + float(breakTorqueVar.get())/(float(breakTorqueRampVar.get())/60) + 3
        testStartTime = time.time()
        friction=(measureTorque("A", 1)-measureTorque("B", 1))
        torque=-measureTorque("B", 1)
        while(programStatus=="breakingTorque" and requestedProgramStatus==""):
            statusText.set("Running Break Test: " + str(round(torque, 2)) + "Nm")

            if(breakIncludeFriction.get()==1):      #set up header in results
                results.append([round(time.time()-testStartTime, 2), round(torque, 2), round(posDiff, 4), round(friction, 3)])
            else:
                results.append([round(time.time()-testStartTime, 2), round(torque, 2), round(posDiff, 4)])

            waitUntil = time.time() + (1/float(settings["live_sample_rate_hz"]))

            while (time.time() < waitUntil and requestedProgramStatus != "stop" and posDiff<float(breakDistanceVar.get())):
                posDiff=posDifference()
                #print(posDiff)
                torque=-measureTorque("B", 1)
                friction=(measureTorque("A", 1)-measureTorque("B", 1))
                backgroundUpdate()
            
            if(posDiff>float(breakDistanceVar.get())):
                statusText.set("Gear Failed at " + str(round(torque, 2)) + "Nm")
                results[0][1]="Gear Failed at " + str(round(torque, 2)) + "Nm"
                testOver=True
            if(testEndTime<time.time()):
                statusText.set("Gear Passed Test up to " + str(round(torque, 2)) + "Nm")
                results[0][1]="Gear Passed Test up to " + str(round(torque, 2)) + "Nm"
                testOver=True
            if(requestedProgramStatus=="stop"):
                statusText.set("error: task interrupted")
                results[0][1]="Test Aborted"
                testOver=True
            if(testOver):
                disableServos()
                saveResults(results)
                requestedProgramStatus=""
                programStatus="idle"
    
def wearMode():     #Run longterm wear test
    global error
    global programStatus
    global requestedProgramStatus
    if(requestedProgramStatus=="wear" and programStatus=="idle"):   #start test
        requestedProgramStatus=""
        programStatus="wear"
        enableServos()
        try:
            servos.axis0.controller.config.control_mode=2   #set both servos to velocity mode
            servos.axis1.controller.config.control_mode=2
            servos.axis0.controller.config.input_mode=2     #set to velocity ramp mode for smoother speed
            servos.axis1.controller.config.input_mode=2
            servos.axis0.controller.config.vel_ramp_rate=2  #set medium ramp rate
            servos.axis1.controller.config.vel_ramp_rate=2
        except:
            requestedProgramStatus="stop"
            programStatus="idle"
            disableServos()
            error.append("Could not prepare motors for wear test(Odrive fail)")
            return
        if(requestedProgramStatus=="stop"):
            programStatus="idle"
            statusText.set("error: task interrupted")
            return
        results = []
        results.append(["Test Result", "none"])
        results.append(["Timestamp", time.asctime( time.localtime(time.time()) )])
        results.append(["Test Type", "Wear"])
        results.append(["Gear Description", desc.get()])
        results.append(["Running Speed", wearSpeedVar.get()])
        results.append(["Running Torque", wearTorqueVar.get()])
        results.append(["Testing Torque", wearTestTorqueVar.get()])
        results.append(["Backlash Test Interval", wearTimeVar.get()])
        results.append(["Max Test Length", wearFrequencyVar.get()])
        results.append(["Test Fail Distance", wearDistanceVar.get()])
        results.append([])

        if(wearIncludeFriction.get()==1):
            results.append(["Time Since Start (min)", "Backlash (turns)", "Friction (Nm)"])
        else:
            results.append(["Time Since Start (min)", "Backlash (turns)"])
        
        testOver=False
        testEndTime = time.time() + float(wearTimeVar.get())*3600
        testStartTime = time.time()
        friction=(measureTorque("A", 1)+measureTorque("B", 1))
        backlash=0.0
        posDiff=0
        frictionSampleTime = time.time() + 1/float(settings["live_sample_rate_hz"])
        resetPosDifference()
        while(programStatus=="wear" and requestedProgramStatus==""):
            nextSampleTime = time.time() + float(wearFrequencyVar.get())*60 #set time for next backlash sample
            statusText.set("Current Backlash: " + str(round(backlash, 5)) + "  T-" + str(round(nextSampleTime-time.time(), 1)))

            if(wearIncludeFriction.get()==0):
                results.append([round((time.time()-testStartTime)/60, 1), round(backlash, 3)])

            #sample gear backlash
            try:
                #stop servos
                print("doing backlash sample")
                servos.axis0.controller.input_vel=0
                servos.axis1.controller.input_vel=0
                wait((float(wearSpeedVar.get())/60)/2 + .5) #wait for servos to stop
                #orient both servos
                orientServos(0)
                #set servo A position mode
                servos.axis0.controller.config.control_mode=3
                servos.axis0.controller.config.input_mode=1     
                servos.axis0.controller.input_pos = servos.axis0.encoder.pos_estimate
                #set servo B to torque mode
                servos.axis1.controller.config.control_mode=1
                servos.axis1.controller.config.input_mode=6
                servos.axis1.controller.config.torque_ramp_rate=2
                
                wait(1)    #allow time to settle

                #measure backlash
                setTorque("B", float(wearTestTorqueVar.get()), 1)
                servos.axis0.controller.config.torque_ramp_rate=2
                servos.axis1.controller.config.torque_ramp_rate=2
                wait(float(wearTestTorqueVar.get())/servos.axis1.controller.config.torque_ramp_rate + 2)
                posA=posDifference()
                setTorque("B", -float(wearTestTorqueVar.get()), -1)
                wait(2*float(wearTestTorqueVar.get())/servos.axis1.controller.config.torque_ramp_rate + 2)
                posB=posDifference()
                backlash=float(abs(posA-posB))
                
                if(posDiff<float(wearDistanceVar.get()) and backlash<float(wearDistanceVar.get())):
                    
                    #start servos in constant vel
                    print("Putting servos into vel mode")
                    disableServos()
                    enableServos()
                    servos.axis0.controller.input_vel=0
                    servos.axis1.controller.input_vel=0
                    servos.axis0.controller.config.control_mode=2   #set both servos to velocity mode
                    servos.axis1.controller.config.control_mode=2
                    servos.axis0.controller.config.input_mode=2     #set to velocity ramp mode for smoother speed
                    servos.axis1.controller.config.input_mode=2
                    servos.axis0.controller.config.vel_ramp_rate=2  #set medium ramp rate
                    servos.axis1.controller.config.vel_ramp_rate=2
                    
                    servos.axis0.controller.input_vel=float(wearSpeedVar.get())/60     #set to wear test speed
                    servos.axis1.controller.input_vel=float(wearSpeedVar.get())/60
                    print("Ramping up to test speed")
                    wait((float(wearSpeedVar.get())/60/servos.axis0.controller.config.vel_ramp_rate)+.5)
                    print("At test speed")
                    servos.axis1.controller.config.control_mode=1       #set servo to begin applying running torque
                    servos.axis1.controller.config.input_mode=6
                    servos.axis1.controller.config.torque_ramp_rate=2
                    setTorque("B", -float(wearTorqueVar.get()), 0)
                    print("Ramping up to torque")
                    

            except:
                requestedProgramStatus="stop"
                programStatus="idle"
                disableServos()
                error.append("Could not run backlash sample(Odrive fail)")
                return

            

            while (time.time() < nextSampleTime and requestedProgramStatus != "stop" and posDiff<float(wearDistanceVar.get()) and backlash<float(wearDistanceVar.get())):
                statusText.set("Current Backlash: " + str(round(backlash, 5)) + "  T-" + str(round(nextSampleTime-time.time(), 1)))
                friction=(measureTorque("A", 1)+measureTorque("B", 1))
                posDiff=posDifference()
                
                if(wearIncludeFriction.get()==1 and frictionSampleTime<time.time()):
                    frictionSampleTime = time.time() + 1/float(settings["live_sample_rate_hz"])
                    print("Updating Results")
                    results.append([round((time.time()-testStartTime)/60, 3), round(backlash, 3), round(friction, 3)])
                backgroundUpdate()
            
            if(posDiff>float(wearDistanceVar.get()) or backlash>float(wearDistanceVar.get())):
                statusText.set("Gear Failed at " + str(round((time.time()-testStartTime)/3600, 2)) + "Hours")
                results[0][1]="Gear Failed at " + str(round((time.time()-testStartTime)/3600, 2)) + "Hours"
                testOver=True
            if(testEndTime<time.time()):
                statusText.set("Gear Passed Wear Test")
                results[0][1]="Gear Passed Wear Test"
                testOver=True
            if(requestedProgramStatus=="stop"):
                statusText.set("error: task interrupted")
                results[0][1]="Test Aborted"
                testOver=True
            if(testOver):
                disableServos()
                setTorque("A", 0, 0)
                setTorque("B", 0, 0)
                saveResults(results)
                requestedProgramStatus=""
                programStatus="idle"

def frictionMode():     #Run friction test at range of speeds and torques
    global error
    global programStatus
    global requestedProgramStatus
    if(requestedProgramStatus=="friction" and programStatus=="idle"):
        requestedProgramStatus=""
        programStatus="friction"
        enableServos()
        if True:
            servos.axis0.controller.config.control_mode=2   #set both servos to velocity mode
            servos.axis1.controller.config.control_mode=2
            servos.axis0.controller.config.input_mode=2     #set to velocity ramp mode for smoother speed
            servos.axis1.controller.config.input_mode=2
            velStepSize = (float(frictionSpeedStopVar.get())/60-float(frictionSpeedStartVar.get())/60)/float(frictionSpeedStepsVar.get())
            torqueStepSize = (float(frictionTorqueStopVar.get())-float(frictionTorqueStartVar.get()))/float(frictionTorqueStepsVar.get())
            servos.axis0.controller.config.vel_ramp_rate=2      #set all ramp rates to match sample period
            servos.axis1.controller.config.vel_ramp_rate=2
            servos.axis0.controller.config.torque_ramp_rate=2
            servos.axis1.controller.config.torque_ramp_rate=2

        #except:
         #   requestedProgramStatus="stop"
        #    programStatus="idle"
        #    disableServos()
        #    error.append("Could not prepare motors for friction test(Odrive fail)")
        #    return
        if(requestedProgramStatus=="stop"):
            programStatus="idle"
            statusText.set("error: task interrupted")
            return
        results = []
        results.append(["Test Result", "none"])
        results.append(["Timestamp", time.asctime( time.localtime(time.time()) )])
        results.append(["Test Type", "Friction"])
        results.append(["Gear Description", desc.get()])
        results.append(["Speed Start", frictionSpeedStartVar.get()])
        results.append(["Speed Stop", frictionSpeedStopVar.get()])
        results.append(["Speed Steps", frictionSpeedStepsVar.get()])
        results.append(["Torque Start", frictionTorqueStartVar.get()])
        results.append(["Torque Stop", frictionTorqueStopVar.get()])
        results.append(["Torque Steps", frictionTorqueStepsVar.get()])
        results.append(["Test Period", frictionTestPeriodVar.get()])
        results.append(["Test Fail Distance", frictionDistanceVar.get()])
        results.append([])

        results.append(["Time Since Start (s)", "Gear A Torque (Nm)", "Gear B Torque (Nm)", "Friction (Nm)", "Friction %"])
        
        testOver=False
        testStartTime = time.time()
        posDiff=0
        resetPosDifference()
        for torqueNum in range(int(frictionTorqueStepsVar.get())):
            for velNum in range(int(frictionSpeedStepsVar.get())):
                if(programStatus=="friction" and requestedProgramStatus==""):
                    if True:
                        step=str((torqueNum)*int(frictionSpeedStepsVar.get())+(velNum)) + "/" + str(int(frictionTorqueStepsVar.get())*int(frictionSpeedStepsVar.get()))
                        statusText.set("Testing Friction " + step)
                        print(step)
                        vel=round((float(frictionSpeedStartVar.get())/60) + velNum*velStepSize, 3)
                        torque=round((float(frictionTorqueStartVar.get())) + torqueNum*torqueStepSize, 2)
                        print(torque)
                        setTorque("B", 0, 1)
                        wait((float(frictionTestPeriodVar.get())*.1))
                        servos.axis1.controller.config.control_mode=2
                        servos.axis0.controller.input_vel=vel
                        servos.axis1.controller.input_vel=vel
                        if(velNum==0 and torqueNum!=0):
                            wait(int(frictionSpeedStepsVar.get())*float(velStepSize)/servos.axis0.controller.config.vel_ramp_rate)
                        wait((float(frictionTestPeriodVar.get())*.2))
                        servos.axis1.controller.config.control_mode=1
                        setTorque("B", -torque, 1)
                        wait((float(frictionTestPeriodVar.get())*.7))
                        At=measureTorque("A", 1)
                        Bt=measureTorque("B", 1)
                        results.append([round((time.time()-testStartTime), 1), round(At, 3), round(Bt, 3), round(At-Bt, 3), round(100*(At-Bt)/At, 1)])
                        posDiff=posDifference()
                        if(posDiff>float(frictionDistanceVar.get())):
                            statusText.set("Gear Failed at step " + step)
                            results[0][1]="Gear Failed at step " + step
                            testOver=True
                        if(requestedProgramStatus=="stop"):
                            statusText.set("error: task interrupted")
                            results[0][1]="Test Aborted"
                            testOver=True
                        if(testOver):
                            disableServos()
                            saveResults(results)
                            requestedProgramStatus=""
                            programStatus="idle"
                            return
                    #except:
                    #    requestedProgramStatus="stop"
                    #    programStatus="idle"
                    #    disableServos()
                    #    error.append("Could not run friction test(Odrive fail)")
                    #    return
        statusText.set("Friction Test Passed " + step)
        results[0][1]="Friction Test Passed " + step
        disableServos()
        saveResults(results)
        requestedProgramStatus=""
        programStatus="idle"
        return

def saveSettings():     #Save user settings to settings file
    settings["static_friction_done_dist"] = staticFrictionDoneDistVar.get()
    settings["static_friction_max_torque"] = staticFrictionMaxTorqueVar.get()
    settings["static_friction_torque_ramp_rate"] = staticFrictionTorqueRampRateVar.get()

    settings["kinetic_friction_max_torque"] = kineticFrictionMaxTorqueVar.get()
    settings["kinetic_friction_vel_ramp_rate"] = kineticFrictionVelRampRateVar.get()
    settings["kinetic_friction_max_vel_speed"] = kineticFrictionMaxVelVar.get()

    settings["wear_speed"] = wearSpeedVar.get()
    settings["wear_max_torque"] = wearMaxTorqueVar.get()

    with open('tireTesterSettings.csv', 'w') as f:  
        writer = csv.writer(f, delimiter=':', dialect="unix")
        for k, v in settings.items():
            writer.writerow([k, v])


if(programStatus=="disconnected"):  #show error if no odrive found
    statusText.set("Odrive not found, please restart")
else:
    statusText.set("Calibrate Before Use!")
while(requestedProgramStatus != "shutdown"):
    if(error):  #print out errors if there are any
        print("__________New errors__________")
        for i in error:
            print(i)
        error.clear()
        print("\n\n")
    if True:
        if(programStatus != "disconnected"):
            calibrationMode()
            #staticMode()
            # kineticMode()
            # wearMode()
        if programStatus == "idle":
            calibrateRbtn['state'] = NORMAL
            staticRbtn['state'] = NORMAL
            kineticRbtn['state'] = NORMAL
            wearRbtn['state'] = NORMAL

        if programStatus != "idle":
            calibrateRbtn['state'] = DISABLED
            staticRbtn['state'] = DISABLED
            kineticRbtn['state'] = NORMAL
            wearRbtn['state'] = DISABLED
        
        backgroundUpdate()
    #except:
    #    requestedProgramStatus="shutdown"
saveSettings()


