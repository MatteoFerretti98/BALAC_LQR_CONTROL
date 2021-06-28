from m5stack import *
from m5ui import *
from uiflow import *
import time
from math import *
from easyIO import *
import i2c_bus
import machine 
import imu

lcd.setRotation(0)

N_CAL1 = 100
N_CAL2 = 100

standing = False

counter = 0

time0 = 0
time1 = 1

counterOverPwr = 0
maxOvp = 20

power = float()
powerR = float()
powerL = float()
yawPower = float()

varAng = float()
varOmg = float()
varSpd = float()
varDst = float()
varIang = float()

gyroXoffset = float()
gyroYoffset = float()
gyroZoffset = float()
accXoffset = float()

gyroXdata = float()
gyroYdata = float()
gyroZdata = float()
accXdata = float()
accZdata = float()

aveAccX = 0.0
aveAccZ = 0.0
aveAbsOmg = 0.0

interval =0.01*1000

Kang = float()
komg = float()
KIang = float()
Kyaw = float()
Kdst = float()
Kspd = float()

maxPwr = int()
yawAngle = 0.0

moveDestination = float()
moveTarget = float()

moveRate = 0.0
moveStep = 0.2*0.01

fbBalance = 0
motorDeadBand = 0
mechFactR = float()
mechFactL = float()

motorRDir = 0
motorLDir = 0

spinContinuous = False
spinDest = float()
spinTarget = float()
spinFact = 1.0
spinStep = 0.0

ipowerL = 0
ipowerR = 0
motorLdir = 0
motorRdir = 0

punchPwr = int()
punchPwr2 = int()
punchDur = int()
punchCountL = 0
punchCountR = 0
demoMode = 0

def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

def drvMotor(ch,sp):
    i2c0.write_mem_data(ch, sp, i2c_bus.INT8LE)

def drvMotorL(pwm):
    drvMotor(0,int(constrain(pwm,-127,127)))

def drvMotorR(pwm):
    drvMotor(1,int(constrain(-pwm,-127,127)))

def resetMotor():
    global counterOverPwr
    drvMotorR(0)
    drvMotorL(0)
    counterOverPwr = 0

def imuInit():
    lcd.clear()
    global imu0
    GYRO_FS_SEL_250DPS = const(0b00000000)
    ACCEL_FS_SEL_4G = const(0b00001000)
    imu0 = imu.IMU(accel_fs=ACCEL_FS_SEL_4G, gyro_fs=GYRO_FS_SEL_250DPS)
    lcd.print('MPU6886 OK',0,0)

def resetVar():
    global power,moveTarget,moveRate,spinContinuous,spinDest,spinTarget,spinStep,yawAngle,varOmg,varDst,varSpd,varIang,varAng
    power = 0.0
    moveTarget = 0.0
    moveRate = 0.0
    spinContinuous = False
    spinDest = 0.0
    spinTarget = 0.0
    spinStep = 0.0
    yawAngle = 0.0
    varAng = 0.0
    varOmg = 0.0
    varDst = 0.0
    varSpd = 0.0
    varIang = 0.0

def resetPara():
    global Kang,Komg,KIang,Kyaw,Kdst,Kspd,mechFactL,mechFactR,punchPwr,punchDur,fbBalance,motorDeadband,maxPwr,PunchPwr2 
    Kang = 37.0 
    Komg = 0.84
    KIang = 800.0 
    Kyaw = 4.0
    Kdst = 85.0
    Kspd = 2.7
    mechFactL = 0.45 
    mechFactR = 0.45
    punchPwr = 20
    punchDur = 1
    fbBalance = -3
    motorDeadband = 10
    maxPwr = 120
    PunchPwr2 = max(punchPwr,motorDeadband)


def readGyro():
    global gyroYdata,gyroZdata,gyroXdata,accXdata,accZdata, imu0
    gX = imu0.gyro[0] 
    gY = imu0.gyro[1] 
    gZ = imu0.gyro[2] 
    aX = imu0.acceleration[0]
    aY = imu0.acceleration[1]
    aZ = imu0.acceleration[2]
    gyroYdata = gX
    gyroZdata = -gY
    gyroXdata = -gZ
    accXdata = aZ
    accZdata = aY
    
def getGyro():
    global varOmg, yawAngle, varAng, gyroZoffset, gyroYoffset, accXoffset
    readGyro()
    varOmg = gyroYdata-gyroYoffset
    yawAngle += (gyroZdata-gyroZoffset)*0.01
    varAng +=(varOmg+((accXdata-accXoffset)*57.3-varAng)*0.1)*0.01
    
def calDelay(n):
    for i in range(n-1):
        getGyro()
        wait_ms(9)
        
def calib1():
    global gyroYoffset
    calDelay(30)
    lcd.clear()
    lcd.print('            Posiziona disteso...')
    wait_ms(1000)
    lcd.print('     Cal-1')
    M5Led.on()
    gyroYoffset = 0.0
    for i in range(N_CAL1-1):
        readGyro()
        gyroYoffset += gyroYdata
        wait_ms(9)
    gyroYoffset /= float(N_CAL1)
    
    M5Led.off()
    lcd.clear()
            
        
def calib2():
    global accXoffset,gyroZoffset
    lcd.clear()
    resetVar()
    resetMotor()
    lcd.print('    Cal-2')
    M5Led.on()
    accXoffset = 0.0
    gyroZoffset = 0.0
    for i in range(N_CAL2-1):
        readGyro()
        accXoffset += accXdata
        gyroZoffset += gyroZdata
        wait_ms(9)
    accXoffset /= float(N_CAL2)
    gyroZoffset /= float(N_CAL2)
    M5Led.off()
    lcd.clear()
    

def startDemo():
    global moveRate,spinContinuous,spinStep 
    moveRate = 1.0
    spinContinuous = True
    spinStep = -40.0*0.01
    
def setMode(mode):
    global demoMode
    lcd.clear()
    if mode:
        demoMode = ++demoMode%2
    if demoMode==0:
        lcd.print('Stand')
    elif demoMode == 1:
        lcd.print("Demo")
        
def checkButtonP():
    if btnB.isPressed():
        calib1()
    elif btnA.isPressed():
        setMode(True)
        
def dispBat():
    lcd.clear()
    lcd.print("Bat%: ",0,0,0xffffff)
    lcd.print((map_value((axp.getBatVoltage()), 3.7, 4.1, 0, 100)), 80, 0, 0xffffff)
    
def drive():
    global moveRate,spinFact,powerR,powerL,spinContinuous,spinTarget,spinStep,spinDest
    global moveTarget,moveStep,fbBalance,varSpd,Kspd,varDst,Kdst,varIang,KIang,varAng,Kang,power,Komg,varOmg,Kyaw
    global counterOverPwr,maxOvp,maxPwr,mechFactL,mechFactR,motorDeadband,punchPwr2,motorLdir,motorRdir,punchCountL,punchCountR,punchDur
    if abs(moveRate)>0.1:
        spinFact = constrain(-(powerR+powerL)/10.0,-1.0,1.0)#moving
    else:
        spinFact = 1.0#standing
    if spinContinuous:
        spinTarget += spinStep*spinFact
    else:
        if spinTarget < spinDest:
            spinTarget += spinStep
        if spinTarget > spinDest:
            spinTarget -= spinStep
            
    
    moveTarget += moveStep*(moveRate+float(fbBalance)/100.0)
    varSpd += power*0.01
    varDst += Kdst*(varSpd*0.01-moveTarget)
    varIang += KIang*varAng*0.01
    power = varIang+varDst+(Kspd*varSpd)+(Kang*varAng)+(Komg*varOmg)
    
    if abs(power)> 1000.0:
        counterOverPwr += 1
    else:
        counterOverPwr = 0
    if counterOverPwr > maxOvp:
        return
    
    power = constrain(power,-maxPwr,maxPwr)
    yawPower = (yawAngle-spinTarget)*Kyaw
    powerR = power-yawPower
    powerL = power+yawPower

    ipowerL = int(constrain(powerL*mechFactL,-maxPwr,maxPwr))
    mdbn = int(-motorDeadband)#motordeadband
    pp2n = int(-punchPwr2)

    if ipowerL >0:
        if motorLdir == 1:
            punchCountL = constrain(punchCountL+1,0,100)
        else:
            punchCountL = 0
            motorLdir = 1
        if (punchCountL < punchDur):
            drvMotorL(max(ipowerL,punchPwr2))
        else:
            drvMotorL(max(ipowerL,motorDeadband))
    elif ipowerL < 0:
        if motorLdir == -1:
            punchCountL = constrain(punchCountL+1,0,100)
        else:
            punchCountL = 0
            motorLdir = -1
        if (punchCountL < punchDur):
            drvMotorL(min(ipowerL,punchPwr2))
        else:
            drvMotorL(min(ipowerL,motorDeadband))
    else: 
        drvMotorL(0)
        motorLdir = 0
    
    ipowerR = int(constrain(powerR*0.45,-120,120))

    if ipowerR >0:
        if motorRdir == 1:
            punchCountR = constrain(punchCountL+1,0,100)
        else:
            punchCountR = 0
            motorRdir = 1
        if (punchCountL < punchDur):
            drvMotorR(max(ipowerR,punchPwr2))
        else:
            drvMotorR(max(ipowerR,motorDeadband))
    elif ipowerR < 0:
        if motorRdir == -1:
            punchCountR = constrain(punchCountR+1,0,100)
        else:
            punchCountR = 0
            motorRdir = -1
        if (punchCountR < punchDur):
            drvMotorR(min(ipowerR,punchPwr2))
        else:
            drvMotorR(min(ipowerR,motorDeadband))
    else: 
        drvMotorR(0)
        motorRdir = 0
    
##setup
i2c0 = i2c_bus.easyI2C((0, 26), 0x38, freq=400000)
imuInit()
lcd.clear()
resetMotor()
resetPara()
resetVar()
calib1()
    
    
while True:
    checkButtonP()
    getGyro()
    if not standing:
        lcd.print('   Posiziona verticale')
        wait_ms(1000)
        lcd.print(".")
        wait_ms(1000)
        lcd.print(".")
        wait_ms(1000)
    
        calib2()

        standing = True
    else:
        if abs(varAng)>30.0 or counterOverPwr>maxOvp:
            resetMotor()
            resetVar()
            standing = False
            setMode(False)
        else:
          drive()
    counter +=1
    if counter >= 100:
        counter = 0
        dispBat()
    while time1-time0 < interval:
        time1 = time.ticks_ms()
    time0 = time1