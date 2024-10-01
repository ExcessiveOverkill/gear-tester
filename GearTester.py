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
with open('settings1.csv', mode='r') as csvfile:
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
root.title("Gear Tester")
root.iconbitmap('logo.ico')
servoFrame = Frame(root, height=456, width=210, highlightbackground="black", highlightthickness=1)
servoFrame.grid()
servoFrame.grid_propagate(0)

UIframe = Frame(root)
UIframe.grid(column=1, row=0)

controlFrame = Frame(UIframe, height=50, width=450, highlightbackground="black", highlightthickness=1)
controlFrame.grid(column=0, row=0)
controlFrame.grid_propagate(0)

calibrationFrame = Frame(UIframe, height=80, width=450, highlightbackground="black", highlightthickness=1)
calibrationFrame.grid(column=0, row=1)
calibrationFrame.grid_propagate(0)

breakFrame = Frame(UIframe, height=106, width=450, highlightbackground="black", highlightthickness=1)
breakFrame.grid(column=0, row=2)
breakFrame.grid_propagate(0)

wearFrame = Frame(UIframe, height=130, width=450, highlightbackground="black", highlightthickness=1)
wearFrame.grid(column=0, row=3)
wearFrame.grid_propagate(0)

frictionFrame = Frame(UIframe, height=90, width=450, highlightbackground="black", highlightthickness=1)
frictionFrame.grid(column=0, row=4)
frictionFrame.grid_propagate(0)

#Status/Control Section
statusText  = StringVar()
status = Label(controlFrame, anchor="w", textvariable=statusText, width=30, relief=SUNKEN)
statusText.set("This will show the status")
status.grid(padx=3, pady=1)

desc = Entry(controlFrame, relief=SUNKEN, width=35)
desc.grid(column=0, row=1, padx=3, pady=1)

mode = IntVar()

breakSpeedVar = StringVar()
breakTorqueVar = StringVar()
breakTorqueRampVar = StringVar()
breakDistanceVar = StringVar()

wearSpeedVar = StringVar()
wearTorqueVar = StringVar()
wearTimeVar = StringVar()
wearDistanceVar = StringVar()
wearTestTorqueVar = StringVar()
wearFrequencyVar = StringVar()

frictionTestPeriodVar = StringVar()
frictionDistanceVar = StringVar()
frictionSpeedStartVar = StringVar()
frictionSpeedStopVar = StringVar()
frictionSpeedStepsVar = StringVar()
frictionTorqueStartVar = StringVar()
frictionTorqueStopVar = StringVar()
frictionTorqueStepsVar = StringVar()

#User input variables
breakSpeedVar.set(settings["breaking_torque_speed"])
breakTorqueVar.set(settings["breaking_torque_max_torque"])
breakTorqueRampVar.set(settings["breaking_torque_ramp"])
breakDistanceVar.set(settings["breaking_torque_detection_distance"])

wearSpeedVar.set(settings["wear_speed"])
wearTorqueVar.set(settings["wear_torque"])
wearTimeVar.set(settings["wear_max_time"])
wearDistanceVar.set(settings["wear_fail_distance"])
wearTestTorqueVar.set(settings["wear_test_torque"])
wearFrequencyVar.set(settings["wear_test_frequency"])

frictionTestPeriodVar.set(settings["friction_test_period"])
frictionDistanceVar.set(settings["friction_fail_distance"])
frictionSpeedStartVar.set(settings["friction_speed_start"])
frictionSpeedStopVar.set(settings["friction_speed_stop"])
frictionSpeedStepsVar.set(settings["friction_speed_steps"])
frictionTorqueStartVar.set(settings["friction_torque_start"])
frictionTorqueStopVar.set(settings["friction_torque_stop"])
frictionTorqueStepsVar.set(settings["friction_torque_steps"])



def Start():    #start selected test
    global programStatus
    global requestedProgramStatus
    if(mode.get()==0):
        requestedProgramStatus="calibration"
    if(mode.get()==1):
        requestedProgramStatus="breakingTorque"
        #requestedProgramStatus="randomWheel"
    if(mode.get()==2):
        requestedProgramStatus="wear"
    if(mode.get()==3):
        requestedProgramStatus="friction"
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

#Breaking Torque Test Section
breakRbtn = Radiobutton(breakFrame, text="Breaking Torque", variable=mode, value=1)
breakRbtn.grid(columnspan=3, padx=2, pady=2, sticky="nw")

breakIncludeFriction  = IntVar()
breakCkbtn = Checkbutton(breakFrame, text="Include Friction", variable=breakIncludeFriction, onvalue=1, offvalue=0)
breakCkbtn.grid(columnspan=3, column=0, row=4, sticky="nw")
if(settings["breaking_torque_include_friction"]=="True"):
    breakCkbtn.select()

breakSpeedEnt = Entry(breakFrame, relief=SUNKEN, width=5, justify=CENTER, textvariable=breakSpeedVar)
breakSpeedEnt.grid(column=1, row=1, padx=2, pady=2, sticky="e")

breakSpeedlbl = Label(breakFrame, text="Speed (rpm)")
breakSpeedlbl.grid(column=2, row=1, padx=2, sticky="w")

breakTorqueEnt = Entry(breakFrame, relief=SUNKEN, width=5, justify=CENTER, textvariable=breakTorqueVar)
breakTorqueEnt.grid(column=1, row=2, padx=2, pady=2, sticky="e")

breakTorquelbl = Label(breakFrame, text="Max Torque (Nm)")
breakTorquelbl.grid(column=2, row=2, padx=2, sticky="w")

breakRampEnt = Entry(breakFrame, relief=SUNKEN, width=5, justify=CENTER, textvariable=breakTorqueRampVar)
breakRampEnt.grid(column=4, row=1, padx=2, pady=2, sticky="e")

breakRamplbl = Label(breakFrame, text="Ramp (Nm/min)")
breakRamplbl.grid(column=5, row=1, padx=2, sticky="w")

breakDistanceEnt = Entry(breakFrame, relief=SUNKEN, width=5, justify=CENTER, textvariable=breakDistanceVar)
breakDistanceEnt.grid(column=4, row=2, padx=2, pady=2, sticky="e")

breakDistancelbl = Label(breakFrame, text="Fail Distance (turns)")
breakDistancelbl.grid(column=5, row=2, padx=2, sticky="w")

breakSpaceA = Frame(breakFrame, width=25)
breakSpaceA.grid_propagate(0)
breakSpaceA.grid(rowspan=2, column=0, row=1)

breakSpaceB = Frame(breakFrame, width=42)
breakSpaceB.grid_propagate(0)
breakSpaceB.grid(rowspan=2, column=3, row=1)

#Wear Test Section
wearRbtn = Radiobutton(wearFrame, text="Wear", variable=mode, value=2)
wearRbtn.grid(columnspan=3, padx=2, pady=2, sticky="w")

wearIncludeFriction  = IntVar()
wearCkbtn = Checkbutton(wearFrame, text="Include Friction", variable=wearIncludeFriction, onvalue=1, offvalue=0)
wearCkbtn.grid(columnspan=3, column=0, row=4, sticky="w")
if(settings["wear_include_friction"]=="True"):
    wearCkbtn.select()

wearSpeedEnt = Entry(wearFrame, relief=SUNKEN, width=5, justify=CENTER, textvariable=wearSpeedVar)
wearSpeedEnt.grid(column=1, row=1, padx=2, pady=2, sticky="e")

wearSpeedlbl = Label(wearFrame, text="Speed (rpm)")
wearSpeedlbl.grid(column=2, row=1, padx=2, sticky="w")

wearTorqueEnt = Entry(wearFrame, relief=SUNKEN, width=5, justify=CENTER, textvariable=wearTorqueVar)
wearTorqueEnt.grid(column=1, row=2, padx=2, pady=2, sticky="e")

wearTorquelbl = Label(wearFrame, text="Running Torque (Nm)")
wearTorquelbl.grid(column=2, row=2, padx=2, sticky="w")

wearTimeEnt = Entry(wearFrame, relief=SUNKEN, width=5, justify=CENTER, textvariable=wearTimeVar)
wearTimeEnt.grid(column=1, row=3, padx=2, pady=2, sticky="e")

wearTimelbl = Label(wearFrame, text="Test Length (Hours)")
wearTimelbl.grid(column=2, row=3, padx=2, sticky="w")

wearDistanceEnt = Entry(wearFrame, relief=SUNKEN, width=5, justify=CENTER, textvariable=wearDistanceVar)
wearDistanceEnt.grid(column=4, row=1, padx=2, pady=2, sticky="e")

wearDistancelbl = Label(wearFrame, text="Fail Distance (turns)")
wearDistancelbl.grid(column=5, row=1, padx=2, sticky="w")

wearTestTorqueEnt = Entry(wearFrame, relief=SUNKEN, width=5, justify=CENTER, textvariable=wearTestTorqueVar)
wearTestTorqueEnt.grid(column=4, row=2, padx=2, pady=2, sticky="e")

wearTestTorquelbl = Label(wearFrame, text="Testing Torque (Nm)")
wearTestTorquelbl.grid(column=5, row=2, padx=2, sticky="w")

wearTestFrequencyEnt = Entry(wearFrame, relief=SUNKEN, width=5, justify=CENTER, textvariable=wearFrequencyVar)
wearTestFrequencyEnt.grid(column=4, row=3, padx=2, pady=2, sticky="e")

wearTestFrequencylbl = Label(wearFrame, text="Testing Frequency (Minutes)")
wearTestFrequencylbl.grid(column=5, row=3, padx=2, sticky="w")

wearSpaceA = Frame(wearFrame, width=25)
wearSpaceA.grid_propagate(0)
wearSpaceA.grid(rowspan=3, column=0, row=1)

wearSpaceB = Frame(wearFrame, width=20)
wearSpaceB.grid_propagate(0)
wearSpaceB.grid(rowspan=3, column=3, row=1)

#Friction Test Section
frictionRbtn = Radiobutton(frictionFrame, text="Friction", variable=mode, value=3)
frictionRbtn.grid(columnspan=3, padx=2, pady=2, sticky="w")

frictionPeriodEnt = Entry(frictionFrame, relief=SUNKEN, width=5, justify=CENTER, textvariable=frictionTestPeriodVar)
frictionPeriodEnt.grid(column=1, row=1, padx=2, pady=2, sticky="e")

frictionPeriodlbl = Label(frictionFrame, text="Period (seconds)")
frictionPeriodlbl.grid(column=2, row=1, padx=2, sticky="w")

frictionDistanceEnt = Entry(frictionFrame, relief=SUNKEN, width=5, justify=CENTER, textvariable=frictionDistanceVar)
frictionDistanceEnt.grid(column=1, row=2, padx=2, pady=2, sticky="e")

frictionDistancelbl = Label(frictionFrame, text="Fail Distance (turns)")
frictionDistancelbl.grid(column=2, row=2, padx=2, sticky="w")

frictionSpeedlbl = Label(frictionFrame, text="Speed (rpm)")
frictionSpeedlbl.grid(column=4, row=1, padx=2, sticky="e")

frictionTorquelbl = Label(frictionFrame, text="Torque (Nm)")
frictionTorquelbl.grid(column=4, row=2, padx=2, sticky="e")

frictionStartlbl = Label(frictionFrame, text="Start")
frictionStartlbl.grid(column=5, row=0, padx=2, sticky="s")

frictionStoplbl = Label(frictionFrame, text="Stop")
frictionStoplbl.grid(column=6, row=0, padx=2, sticky="s")

frictionStepslbl = Label(frictionFrame, text="Steps")
frictionStepslbl.grid(column=7, row=0, padx=2, sticky="s")

frictionStartSpeedEnt = Entry(frictionFrame, relief=SUNKEN, width=5, justify=CENTER, textvariable=frictionSpeedStartVar)
frictionStartSpeedEnt.grid(column=5, row=1, padx=2, pady=2)

frictionStartTorqueEnt = Entry(frictionFrame, relief=SUNKEN, width=5, justify=CENTER, textvariable=frictionTorqueStartVar)
frictionStartTorqueEnt.grid(column=5, row=2, padx=2, pady=2)

frictionStopSpeedEnt = Entry(frictionFrame, relief=SUNKEN, width=5, justify=CENTER, textvariable=frictionSpeedStopVar)
frictionStopSpeedEnt.grid(column=6, row=1, padx=2, pady=2)

frictionStopTorqueEnt = Entry(frictionFrame, relief=SUNKEN, width=5, justify=CENTER, textvariable=frictionTorqueStopVar)
frictionStopTorqueEnt.grid(column=6, row=2, padx=2, pady=2)

frictionStepsSpeedEnt = Entry(frictionFrame, relief=SUNKEN, width=5, justify=CENTER, textvariable=frictionSpeedStepsVar)
frictionStepsSpeedEnt.grid(column=7, row=1, padx=2, pady=2)

frictionStepsTorqueEnt = Entry(frictionFrame, relief=SUNKEN, width=5, justify=CENTER, textvariable=frictionTorqueStepsVar)
frictionStepsTorqueEnt.grid(column=7, row=2, padx=2, pady=2)


frictionSpaceA = Frame(frictionFrame, width=25)
frictionSpaceA.grid_propagate(0)
frictionSpaceA.grid(rowspan=2, column=0, row=1)

frictionSpaceB = Frame(frictionFrame, width=25)
frictionSpaceB.grid_propagate(0)
frictionSpaceB.grid(rowspan=2, column=3, row=1)


Aspeed = DoubleVar()
Bspeed = DoubleVar()
Atorque = DoubleVar()
Btorque = DoubleVar()

SpeedInfolbl = Label(servoFrame, text="Speed (rpm)")
SpeedInfolbl.grid(columnspan=2, column=1, row=0, padx=2)

TorqueInfolbl = Label(servoFrame, text="Torque (Nm)")
TorqueInfolbl.grid(columnspan=2, column=5, row=0, padx=2)

servoAspeedlbl = Label(servoFrame, text="A")
servoAspeedlbl.grid(column=1, row=1, padx=5, sticky="e")

servoBspeedlbl = Label(servoFrame, text="B")
servoBspeedlbl.grid(column=2, row=1, padx=5, sticky="w")

servoAtorquelbl = Label(servoFrame, text="A")
servoAtorquelbl.grid(column=5, row=1, padx=5, sticky="e")

servoBtorquelbl = Label(servoFrame, text="B")
servoBtorquelbl.grid(column=6, row=1, padx=5, sticky="w")

AspeedScl = Scale(servoFrame, variable=Aspeed, from_=maxSpeed, to=-maxSpeed, orient=VERTICAL, showvalue=0, sliderlength=4, length=400, width=15, takefocus=0, troughcolor="green", border=1, resolution=.1, tickinterval=int(settings["speed_scale_divs"]), state=DISABLED)
AspeedScl.grid(columnspan=2, column=0, row=2, sticky="e")

BspeedScl = Scale(servoFrame, variable=Bspeed, from_=maxSpeed, to=-maxSpeed, orient=VERTICAL, showvalue=0, sliderlength=4, length=400, width=15, takefocus=0, troughcolor="green", border=1, resolution=.1, state=DISABLED)
BspeedScl.grid(column=2, row=2, sticky="w")

AtorqueScl = Scale(servoFrame, variable=Atorque, from_=maxTorque, to=-maxTorque, orient=VERTICAL, showvalue=0, sliderlength=4, length=400, width=15, takefocus=0, troughcolor="red", border=1, resolution=.01, tickinterval=10, state=DISABLED)
AtorqueScl.grid(columnspan=2, column=4, row=2, sticky="e")

BtorqueScl = Scale(servoFrame, variable=Btorque, from_=maxTorque, to=-maxTorque, orient=VERTICAL, showvalue=0, sliderlength=4, length=400, width=15, takefocus=0, troughcolor="red", border=1, resolution=.01, state=DISABLED)
BtorqueScl.grid(column=6, row=2, sticky="w")

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

def breakMode():    #Run max torque breaking test 
    global error
    global programStatus
    global requestedProgramStatus
    if(requestedProgramStatus=="breakingTorque" and programStatus=="idle"):
        requestedProgramStatus=""
        programStatus="breakingTorque"
        statusText.set("Preparing Break Test")
        enableServos()
        try:
            servos.axis0.controller.config.control_mode=2   #set both servos to velocity mode
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
    settings["breaking_torque_speed"] = breakSpeedVar.get()
    settings["breaking_torque_max_torque"] = breakTorqueVar.get()
    settings["breaking_torque_ramp"] = breakTorqueRampVar.get()
    settings["breaking_torque_detection_distance"] = breakDistanceVar.get()

    settings["wear_speed"] = wearSpeedVar.get()
    settings["wear_torque"] = wearTorqueVar.get()
    settings["wear_max_time"] = wearTimeVar.get()
    settings["wear_fail_distance"] = wearDistanceVar.get()
    settings["wear_test_torque"] = wearTestTorqueVar.get()
    settings["wear_test_frequency"] = wearFrequencyVar.get()

    settings["friction_test_period"] = frictionTestPeriodVar.get()
    settings["friction_fail_distance"] = frictionDistanceVar.get()
    settings["friction_speed_start"] = frictionSpeedStartVar.get()
    settings["friction_speed_stop"] = frictionSpeedStopVar.get()
    settings["friction_speed_steps"] = frictionSpeedStepsVar.get()
    settings["friction_torque_start"] = frictionTorqueStartVar.get()
    settings["friction_torque_stop"] = frictionTorqueStopVar.get()
    settings["friction_torque_steps"] = frictionTorqueStepsVar.get()

    with open('settings1.csv', 'w') as f:  
        writer = csv.writer(f, delimiter=':', dialect="unix")
        for k, v in settings.items():
            writer.writerow([k, v])

def gameWheelTarget():  #calculate the target position for the "random" game wheel
    #uses the desc text box as input
    #if no mode is selected, it will find all slots(if multiple) that contain the input then choose one at random
    minSlotOffset = -(.2/32)
    maxSlotOffset = (.2/32)
    targetPos = 0
    luck = 0
    input = desc.get()
    if input == "":
        print("No input command from user")
        return 24/32
    if input.count("luck ") > 0:
        #input.replace("luck ", "")
        input = input[5:]
        luck = int(input)
        print("input after luck: " + input)
        chanceMap = [   #chance layout
            [["$1", 1], ["$5", 0], ["$10", 0], ["$100", 0], ["$1000", 0]],     #luck level 0
            [["$1", 1000], ["$5", 500], ["$10", 100], ["$100", 10], ["$1000", 1]],     #luck level 1
            [["$1", 500], ["$5", 500], ["$10", 100], ["$100", 50], ["$1000", 10]],     #luck level 2
            [["$1", 1], ["$5", 1], ["$10", 1], ["$100", 1], ["$1000", 1]],     #luck level 3
            [["$1", 0], ["$5", 0], ["$10", 0], ["$100", 0], ["$1000", 1]],     #luck level 4
        ]
        choices = []
        weights = []
        for chances in chanceMap[luck]:
            choices.append(chances[0])
            weights.append(chances[1])
        input = str(random.choices(choices, weights = weights, k = 1))[2:-2]
        print("input after random choice: " + input)
    targetMap = [   #wheel target map
        ["$1"],   #slot 0
        ["$5"],   #slot 1
        ["$10"],   #slot 2
        ["$5"],   #slot 3
        ["$1"],   #slot 4
        ["$5"],   #slot 5
        ["$1"],   #slot 6
        ["$100"],   #slot 7
        ["$1"],   #slot 8
        ["$5"],   #slot 9
        ["$10"],   #slot 10
        ["$100"],   #slot 11
        ["$1"],   #slot 12
        ["$5"],   #slot 13
        ["$1"],   #slot 14
        ["$5"],   #slot 15
        ["$1"],   #slot 16
        ["$10"],   #slot 17
        ["$1", "C"],   #slot 18
        ["$100"],   #slot 19
        ["$1"],   #slot 20
        ["$5"],   #slot 21 
        ["$10"],   #slot 22
        ["$1"],   #slot 23
        ["$1000", "A"],   #slot 24
        ["$1"],   #slot 25
        ["$5"],   #slot 26
        ["$1"],   #slot 27
        ["$5"],   #slot 28
        ["$10"],   #slot 29
        ["$5", "B"],   #slot 30
        ["$100"],   #slot 31
    ]
    slotSize = 1 / len(targetMap)
    slotOptions = []
    if input.count("nearmiss ") > 0:
        input = input[9:]
        print("input after nearmiss: " + input)
        if random.randint(0, 1) == 1:
            targetPos += slotSize
        else:
            targetPos -= slotSize

    targetPos += (random.randint(0, 100) / 100) * (maxSlotOffset - minSlotOffset) + minSlotOffset   #apply a random position offset within the slot to appear more random

    for slot in enumerate(targetMap):
        if input in slot[1]:
            slotOptions.append(slot[0])
    print("slotOptions: " + str(slotOptions))
    targetPos += random.choice(slotOptions) * slotSize   #calculate rotary position from selected slot
    return targetPos

def randomGameWheel():  #game wheel that you manually spin and "randomly" slows and stops
    global error
    global programStatus
    global requestedProgramStatus
    autoStart = False
    minSpeedRpm = 10    #minimum speed the wheel must be spun up to
    accReleasedThreshold = 0    #wheel acceleration to detect it is no long being spun by someone
    targetTurnPos = 0   #angle(in turns) that the wheel should stop at
    spinDownTime = .1   #how long in seconds/rpm it takes for the wheel to stop(ideal value)
    stopped = True

    if(requestedProgramStatus=="randomWheel" and programStatus=="idle"):
        requestedProgramStatus=""
        programStatus="randomWheel"
        enableServos()
        
        
        if(requestedProgramStatus=="stop"):
            programStatus="idle"
            statusText.set("error: task interrupted")
            return
        
        
        turnPos = servos.axis0.encoder.pos_cpr_counts/servos.axis0.encoder.config.cpr
        absPos = servos.axis0.encoder.pos_estimate
        turnOffset = (turnPos+1) - (absPos % 1.0)
        turnOffset %= 1.0
        #print("turn offset: " + str(turnOffset))
        #print("absPos: " + str(absPos))
        #print("turnPos " + str(turnPos))
        while(requestedProgramStatus != "stop"):
            if(stopped):
                try:
                    statusText.set("Spin wheel to start!")
                    servos.axis1.requested_state = 1    #disable servo B
                    servos.axis0.controller.config.vel_limit = maxSpeed    #limit servo driven velocity
                    servos.axis0.controller.config.input_mode=1     #set to input passthrough(no smoothing)
                except:
                    requestedProgramStatus="stop"
                    programStatus="idle"
                    disableServos()
                    error.append("Could not prepare motors for random wheel(Odrive fail)")
                    return
            backgroundUpdate()
            wait(.2)
            if not autoStart:
                stopPos = servos.axis0.encoder.pos_estimate
                servos.axis0.controller.input_pos = stopPos     #set to current position
                servos.axis0.controller.config.control_mode = 3   #set servo to position mode
                while(requestedProgramStatus != "stop" and stopped and abs(stopPos - servos.axis0.encoder.pos_estimate) < .005):    #hold wheel in place until touched to prevent drifting from set position
                    backgroundUpdate()
            else:
                wait(2)
                servos.axis0.controller.config.control_mode = 2
                servos.axis0.controller.config.input_mode = 2
                servos.axis0.controller.config.vel_ramp_rate = 5
                servos.axis0.controller.input_vel = -minSpeedRpm / 60
                wait(servos.axis0.controller.input_vel / servos.axis0.controller.config.vel_ramp_rate + .5)
                turnPos = servos.axis0.encoder.pos_cpr_counts/servos.axis0.encoder.config.cpr
                while turnPos < .9:
                    turnPos = servos.axis0.encoder.pos_cpr_counts/servos.axis0.encoder.config.cpr
                    backgroundUpdate()


            servos.axis0.controller.config.control_mode = 1   #set servo to torque mode
            servos.axis0.controller.config.input_mode = 1

            #wait for wheel to be spun, resist speed in wrong direction
            while(requestedProgramStatus != "stop" and stopped):
                backgroundUpdate()
                if(servos.axis0.encoder.vel_estimate*60>1):
                    setTorque("A", servos.axis0.encoder.vel_estimate*60, 3)
                else:
                    setTorque("A", 0, 2)
                if(servos.axis0.encoder.vel_estimate*60 <= -minSpeedRpm and AservoAcc(.1) >= accReleasedThreshold or autoStart):
                    targetTurnPos = gameWheelTarget() #get new target position
                    print(targetTurnPos)
                    stopped = False

            if(not stopped and requestedProgramStatus != "stop"):
                statusText.set("Wheel spun, slowing down")
                servos.axis0.controller.config.control_mode = 2
                servos.axis0.controller.config.input_mode = 1
                servos.axis0.controller.config.vel_ramp_rate = .1
                servos.axis0.controller.input_vel = servos.axis0.encoder.vel_estimate
                servos.axis0.controller.config.input_mode = 2
                servos.axis0.controller.input_vel = 0

                speed = servos.axis0.encoder.vel_estimate
                turnPos = servos.axis0.encoder.pos_cpr_counts/servos.axis0.encoder.config.cpr
                absPos = servos.axis0.encoder.pos_estimate
                #turnOffset = turnPos - absPos % 1
                idealEndPos = absPos - (speed * 60 * spinDownTime) * (.5 * speed)      #calculate ideal end position
                idealEndTurnPos = ((idealEndPos % 1) + 1+turnOffset) % 1      #calculate ideal end angle(in turns)
                idealVsTargetDiff = idealEndTurnPos - targetTurnPos     #find difference between ideal and target end position angle
                #print("turn offset: " + str(turnOffset))
                #print("idealEndTurnPos: " + str(idealEndTurnPos))
                #print("absPos: " + str(absPos))
                #print("turnPos " + str(turnPos))
                if(idealVsTargetDiff > 0):      #shift ideal to match target
                    targetEndPos = idealEndPos - idealVsTargetDiff
                else:
                    targetEndPos = idealEndPos + idealVsTargetDiff
                targetEndPos = idealEndPos - idealVsTargetDiff
                #print("targetEndPos: " + str(targetEndPos))
                while(not stopped):
                    speed = servos.axis0.encoder.vel_estimate
                    absPos = servos.axis0.encoder.pos_estimate
                    #turnPos = (servos.axis0.encoder.pos_cpr_counts / servos.axis0.encoder.config.cpr)
                    if(speed == 0 or (targetEndPos - absPos) >= 0 or requestedProgramStatus == "stop"):
                        stopped = True
                    else:
                        rampRate = abs((.5 * speed * speed) / (targetEndPos - absPos))
                        #print("Distance to go: " + str(targetEndPos - absPos))
                        servos.axis0.controller.config.vel_ramp_rate = rampRate
                    backgroundUpdate()
        if(requestedProgramStatus=="stop" or autoStart):
            statusText.set("error: task interrupted")
            requestedProgramStatus=""
            programStatus="idle"
            disableServos()

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
            breakMode()
            wearMode()
            frictionMode()
            randomGameWheel()
        if programStatus == "idle":
            calibrateRbtn['state'] = NORMAL
            breakRbtn['state'] = NORMAL
            wearRbtn['state'] = NORMAL
            frictionRbtn['state'] = NORMAL

        if programStatus != "idle":
            calibrateRbtn['state'] = DISABLED
            breakRbtn['state'] = DISABLED
            wearRbtn['state'] = DISABLED
            frictionRbtn['state'] = DISABLED
        
        backgroundUpdate()
    #except:
    #    requestedProgramStatus="shutdown"
saveSettings()


