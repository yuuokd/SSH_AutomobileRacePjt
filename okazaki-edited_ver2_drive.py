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
STOPTIME = 4
STOPDIST = 10

waittime = 0.01
escape_waittime = 1.0

#センサ距離Center
dist_Cshort = 40

#センサ距離
dist_short = 25
dist_middle = 40


#駆動DUTY
drive_weak = 70
drive_medium = 85
drive_strong = 100
drive_turnC = 0
drive_turnR = -70
drive_turnL = 70
drive_turnRR = -100
drive_turnLL = 100


#正規化処理 -get_distを100以上切り捨て
def rounddown(get_dist):
	if get_dist >100:
		get_dist =100
	return get_dist

#方向制御処理
def SteerContl(now_position,rudder):
    if now_position >> rudder:
        while now_position <= rudder:
            now_position = now_position - 10
            Steer(now_position)
            time.sleep(waittime)
    elif now_position << rudder:
        while now_position >= rudder:
            now_position = now_position + 10
            Steer(now_position)
            time.sleep(waittime)

	
#PWM設定
p_A = GPIO.PWM(PWMA, 1000);
p_B = GPIO.PWM(PWMB, 1000);

#PWM初期化
p_A.start(0)
p_B.start(0)
      

count = 0
count1 = 0
print ("start roop")
	

#動作プログラム
while True :
    
    #距離計測と正規化処理
    get_distC = rounddown(Mesure(trig1,echo1))
    get_distLF = rounddown(Mesure(trig2,echo2))
    get_distRF = rounddown(Mesure(trig3,echo3))
    get_distLS = rounddown(Mesure(trig4,echo4))
    get_distRS = rounddown(Mesure(trig5,echo5))

 
    if get_distC >= dist_Cshort:
        if get_distLF < dist_middle and get_distRF > dist_middle:
           Accel(drive_medium)
           Steer(drive_turnR)
           comment = "右旋回" 
        elif get_distLF >= dist_middle and get_distRF <= dist_middle:
           Accel(drive_medium)
           Steer(drive_turnL)
           comment = "左旋回"

        elif get_distLF < dist_middle and get_distRF < dist_middle:
            if (get_distLF - get_distRF) > 5:
                Accel(drive_medium)
                Steer(drive_turnLL)
                time.sleep(escape_waittime)
                comment = "左旋回"
            elif(get_distRF - get_distLF) > 5:
                Accel(drive_medium)
                Steer(drive_turnRR)
                time.sleep(escape_waittime)
                comment = "右旋回"
            else:
                Accel(drive_medium)
                Steer(drive_turnC)
                comment = "直進中"
        else:
            if get_distLS > dist_middle and get_distRS < dist_middle:
                while(True):
                   Accel(drive_weak)
                   Steer(drive_turnLL)
                   comment = "左避け2"
                   get_distC = rounddown(Mesure(trig1,echo1))
                   get_distLF = rounddown(Mesure(trig2,echo2))
                   get_distRF = rounddown(Mesure(trig3,echo3))
                   get_distLS = rounddown(Mesure(trig4,echo4))
                   get_distRS = rounddown(Mesure(trig5,echo5))
                   time.sleep(waittime)
                   if get_distC < dist_short or get_distLF < dist_short or get_distRF < dist_short or get_distLS < 100:
                       break

            elif get_distLS < dist_middle and get_distRS > dist_middle:
                while(True): 
                   Accel(drive_weak)
                   Steer(drive_turnRR)
                   comment = "右避け2"
                   get_distC = rounddown(Mesure(trig1,echo1))
                   get_distLF = rounddown(Mesure(trig2,echo2))
                   get_distRF = rounddown(Mesure(trig3,echo3))
                   get_distLS = rounddown(Mesure(trig4,echo4))
                   get_distRS = rounddown(Mesure(trig5,echo5))
                   time.sleep(waittime)
                   if get_distC < dist_short or get_distLF < dist_short or get_distRF < dist_short or get_distRS < 100:
                       break
               
            else:
                Accel(drive_strong)
                Steer(drive_turnC)
                comment = "直進中"
       
    else:
        if get_distLF < dist_short and get_distRF > dist_short:
            Accel(-drive_weak)
            Steer(drive_turnLL)
            time.sleep(escape_waittime)
            comment = "左バック"
        elif get_distLF > dist_short and get_distRF < dist_short:
            Accel(-drive_weak)
            Steer(drive_turnRR)
            time.sleep(escape_waittime)
            comment = "右バック"
        else:
            Accel(-drive_weak)
            Steer(-drive_turnR)
            comment = "バック"
            time.sleep(escape_waittime)

#ループ抜ける用       
    if get_distLS <= STOPDIST and get_distRS <= STOPDIST:
         count = count + 1
    else:
         count = 0
         
    if count == STOPTIME:
        print("escape roop")
        break#抜ける

#測定値表示用
    time.sleep(waittime)

    count1 = count1 + 1
    if count1 == 1:
        print("測定値 C:",get_distC,", LF:",get_distLF,", RF:",get_distRF,", LS:",get_distLS,", RS:",get_distRS,comment)
        count1 = 0
 

#while抜けた後の停止処理
p_A.stop()
p_B.stop()
GPIO.cleanup()
print("End.")