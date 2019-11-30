#変更不要########################

#ライブラリのインポート
import RPi.GPIO as GPIO
import time
import numpy as np
import os

GPIO.setmode( GPIO.BCM )

#駆動モータIOピン定義
PWMA=21
Ain1=26
Ain2=20

#モータ動作許可IOピン定義
STBY=16

#操舵モータIOピン定義
PWMB=6
Bin1=19
Bin2=13

#駆動操舵モータ初期設定
m_list=[PWMA,Ain1,Ain2,STBY,PWMB,Bin1,Bin2]
GPIO.setup(m_list, GPIO.OUT, initial=GPIO.LOW )

#超音波センサ中央(C)定義
trig1=23
echo1=8
#超音波センサ左前(LF)定義
trig2=15
echo2=9
#超音波センサ右前(RF)定義
trig3=18
echo3=7
#超音波センサ左横(LS)定義
trig4=14
echo4=11
#超音波センサ右横(RS)定義
trig5=24
echo5=1
#超音波センサ初期設定
t_list=[trig1,trig2,trig3,trig4,trig5]
GPIO.setup(t_list,GPIO.OUT,initial=GPIO.LOW)
e_list=[echo1,echo2,echo3,echo4,echo5]
GPIO.setup(e_list,GPIO.IN)

#駆動モータ制御関数
def Accel(Duty):
    if Duty >= 0:
        GPIO.output( STBY, 1 )
        GPIO.output( Ain1, 0 )
        GPIO.output( Ain2, 1 )
        p_A.ChangeDutyCycle(Duty)
    else:
        Duty = abs(Duty)
        GPIO.output( STBY, 1 )
        GPIO.output( Ain1, 1 )
        GPIO.output( Ain2, 0 )
        p_A.ChangeDutyCycle(Duty)

#操舵モータ制御関数
def Steer(Duty):
    if Duty >= 0:
        GPIO.output( STBY, 1 )
        GPIO.output( Bin1, 0 )
        GPIO.output( Bin2, 1 )
        p_B.ChangeDutyCycle(Duty)
    else:
        Duty = abs(Duty)
        GPIO.output( STBY, 1 )
        GPIO.output( Bin1, 1 )
        GPIO.output( Bin2, 0 )
        p_B.ChangeDutyCycle(Duty)
        
#障害物センサ測定関数
def Mesure(trig,echo):
    sigoff = 0
    sigon = 0
    GPIO.output(trig,GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig,GPIO.LOW)
    while(GPIO.input(echo)==GPIO.LOW):
        sigoff=time.time()
    while(GPIO.input(echo)==GPIO.HIGH):
        sigon=time.time()
    return round(((sigon - sigoff)*34000/2))

print("set ini para")
################################


#For escape from sensing roop
STOPTIME = 7
STOPDIST = 10


#Waittime for drive
standby_wait = 0.01
normal_wait = 0.02
escape_wait = 1.5


#Sensor value for steer judgement 
dist_Cshort = 80
dist_short = 30
dist_middle = 40


#Drive DUTY
drive_weak = 88
drive_medium = 90
drive_strong = 100
drive_turnC = 0
drive_goR = -25
drive_goL = 25
drive_turnR = -60
drive_turnL = 60
drive_turnRR = -100
drive_turnLL = 100


#PWM Settings
p_A = GPIO.PWM(PWMA, 1000);
p_B = GPIO.PWM(PWMB, 1000);


#PWM Initialize
p_A.start(0)
p_B.start(0)
      

count = 0
count1 = 0
print ("start roop")
    


#Get steer value
while True:
    
    #Get sensor value
    get_distC = Mesure(trig1,echo1)
    get_distLF = Mesure(trig2,echo2)
    get_distRF = Mesure(trig3,echo3)
    get_distLS = Mesure(trig4,echo4)
    get_distRS = Mesure(trig5,echo5)

    waittime = 0

    #条件分岐
    if get_distC >= dist_Cshort:

        if get_distLF < dist_middle and get_distRF > dist_middle:
            Steer(drive_turnR)
            Accel(drive_medium)
            waittime = standby_wait
            comment = "turnR,strong,C>Cshort,LF<middle,RF>middle"

        elif get_distLF >= dist_middle and get_distRF <= dist_middle:
            Steer(drive_turnL)
            Accel(drive_medium)
            waittime = standby_wait
            comment = "turnL,strong,C>Cshort,LF>=middle,RF<=middle"


        elif get_distLF <= dist_middle and get_distRF <= dist_middle:
            
            if get_distRF - get_distLF > 5:
                Accel(drive_medium)
                Steer(drive_turnRR)
                waittime = standby_wait
                comment = "turnRR,medium,C>Cshort,LF:RF<middle++RF-LF>5"

            elif get_distLF - get_distRF > 5:
                Accel(drive_medium)
                Steer(drive_turnLL)
                waittime = standby_wait
                comment = "turnLL,medium,C>Cshort,LF&RF<middle++LF-RF>5"

            else:
                Accel(drive_medium)
                Steer(drive_turnC)
                waittime = standby_wait
                comment = "turnC,medium,C>Cshort,LF&RF<middle++LF:RF"

        else:
            if get_distLS - get_distRS > 10:
                Accel(drive_strong)
                Steer(drive_goL)
                waittime = standby_wait
                comment = "goL,strong,C>Cshort,LF&RF>middle,LS-RS>10"

            elif get_distRS - get_distLS > 10:
                Accel(drive_strong)
                Steer(drive_goR)
                waittime = standby_wait
                comment = "goR,strong,C>Cshort,LF&RF>middle,RS-LS>10"
               
            else:
                Accel(drive_strong)
                Steer(drive_turnC)
                waittime = standby_wait
                comment = "turnC,strong,C>Cshort,LF&RF>middle,RS:LS"


    else:
        if get_distLF < dist_short and get_distRF > dist_short:
            Accel(-drive_weak)
            Steer(drive_turnLL)
            time.sleep(escape_wait)
            Accel(drive_weak)
            Steer(drive_turnRR)
            waittime = normal_wait
            comment = "turnLL,-weakC,C<Cshort,LF<short,RF>short"

        elif get_distLF > dist_short and get_distRF < dist_short:
            Accel(-drive_weak)
            Steer(drive_turnRR)
            time.sleep(escape_wait)
            Accel(drive_medium)
            Steer(drive_turnLL)
            waittime = normal_wait
            comment = "turnRR,-weak,C<Cshort,LF>short,RF<short"

        elif get_distLF < dist_short and get_distRF < dist_short:
            if get_distLS >= get_distRS:
                Accel(-drive_weak)
                Steer(drive_turnRR)
                time.sleep(escape_wait)
                Accel(drive_medium)
                Steer(drive_turnLL)
                waittime = normal_wait
                comment = "turnRR,-weak,C<Cshort,LF&RF<short,LS>=RS"
            else:
                Accel(-drive_weak)
                Steer(drive_turnLL)
                time.sleep(escape_wait)
                Accel(drive_medium)
                Steer(drive_turnRR)
                waittime = normal_wait
                comment = "turnRR,-weak,C<Cshort,LF&RF<short,RS>LS"
        
        elif get_distC < dist_short:
            while get_distC > dist_Cshort - 20:
                Accel(-drive_weak)
                Steer(drive_turnC)
                get_distC = Mesure(trig1,echo1)
                time.sleep(normal_wait)
                comment = "turnC,-weak,C<Cshort,C<short"
            waittime = standby_wait
                
        elif get_distRS - get_distLS > 20:
            Accel(drive_weak)
            Steer(drive_turnRR)
            waittime = standby_wait
            comment = "turnRR,medium,C<Cshort,LF&RF<middle,RS>LS"

        elif get_distLS - get_distRS > 20:
            Accel(drive_weak)
            Steer(drive_turnLL)
            waittime = standby_wait
            comment = "turnLL,medium,C<Cshort,LF&RF<middle,LS>RS"
        
        elif get_distLF < get_distRF:
            Accel(drive_medium)
            Steer(drive_turnR)
            waittime = standby_wait
            comment = "turnR,medium,C<Cshort,RF>LF"
            
        elif get_distRF < get_distLF:
            Accel(drive_medium)
            Steer(drive_turnL)
            waittime = standby_wait
            comment = "turnL,medium,C<Cshort,LF>RF"
            
        else:
            Accel(drive_medium)
            Steer(drive_turnC)
            comment = "turnC,middum,C<Cshort,RF:LF"
            waittime = standby_wait


    #Escape Loop
    if get_distLS <= STOPDIST and get_distRS <= STOPDIST:
         count = count + 1
    else:
         count = 0
         
    if count == STOPTIME:
        print("escaped roop")
        break#抜ける

    #Waittime
    time.sleep(waittime)


    #Visualize measured value
    count1 = count1 + 1
    if count1 == 1:
        print("Measured value  C:",get_distC,", LF:",get_distLF,", RF:",get_distRF,", LS:",get_distLS,", RS:",get_distRS, ", :: ",comment)
        count1 = 0


#Shutdown
p_A.stop()
p_B.stop()
GPIO.cleanup()
print("End")