import RPi.GPIO as GPIO
import time
import os

GPIO.setmode( GPIO.BCM )
#駆動モータIO定義
PWMA=21
Ain1=26
Ain2=20
#モータ動作許可IO定義
STBY=16
#操舵モータIO定義
PWMB=6
Bin1=19
Bin2=13
#まとめて初期設定
m_list=[PWMA,Ain1,Ain2,STBY,PWMB,Bin1,Bin2]
GPIO.setup(m_list, GPIO.OUT, initial=GPIO.LOW )

#超音波センサ１中央定義
trig1=23
echo1=8
#超音波センサ２左前定義
trig2=15
echo2=9
#超音波センサ3右前定義
trig3=18
echo3=7
#超音波センサ4左横定義
trig4=14
echo4=11
#超音波センサ5右横定義
trig5=24
echo5=1
#まとめて初期設定
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
#For escape from sensing roop
STOPTIME = 2
STOPDIS = 10

waittime = 0.01

#センサ距離Center
Cshort = 40
Cmiddle = 60
Clong = 100

#センサ距離
short = 40
middle = 60
middle2 = 80
long = 100

#駆動DUTY
weak = 60
medium = 80
strong = 100
turnC = 0
turnR = -100
turnL = 100

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
    distance1 = Mesure(trig1,echo1)
    distance2 = Mesure(trig2,echo2)
    distance3 = Mesure(trig3,echo3)
    distance4 = Mesure(trig4,echo4)
    distance5 = Mesure(trig5,echo5)
 
    if distance1 >= Cshort:
        if distance2 <= short and distance3 >= short:
           Accel(medium)
           Steer(turnR)
           comment = "右旋回" 
        elif distance2 > short and distance3 < short:
           Accel(medium)
           Steer(turnL)
           comment = "左旋回"

        elif distance2 < short and distance3 < short:
            if (distance2 - distance3)>10:
                Accel(medium)
                Steer(turnL)
                comment = "左旋回"
            elif(distance3 - distance2) > 10:
                Accel(medium)
                Steer(turnR)
                comment = "右旋回"
            else:
                Accel(medium)
                Steer(turnC)
                comment = "直進中"
        else:
            if distance4 > middle2 and distance5 < middle2:
                while(True): 
                   Accel(weak)
                   Steer(turnL)
                   comment = "左避け2"
                   distance4 = Mesure(trig4,echo4)
                   distance1 = Mesure(trig1,echo1)
                   distance2 = Mesure(trig2,echo2)
                   distance3 = Mesure(trig3,echo3)
                   time.sleep(0.05)
                   if distance1 < short or distance2 < short or distance3 < short or distance4 < 100:
                       break
                                             
            elif distance4 < middle2 and distance5 > middle2:
                while(True): 
                   Accel(weak)
                   Steer(turnR)
                   comment = "右避け2"
                   distance5 = Mesure(trig5,echo5)
                   distance1 = Mesure(trig1,echo1)
                   distance2 = Mesure(trig2,echo2)
                   distance3 = Mesure(trig3,echo3)
                   time.sleep(0.05)
                   if distance1 < short or distance2 < short or distance3 < short or distance5 < 100:
                       break
               
            else:
                Accel(strong)
                Steer(turnC)
                comment = "直進中"
       
    else:
        if distance2 < middle and distance3 > middle:
            Accel(-weak)
            Steer(turnL)
            time.sleep(0.2)
            comment = "左バック"
        elif distance2 > middle and distance3 < middle:
            Accel(-weak)
            Steer(turnR)
            time.sleep(0.2)
            comment = "右バック"
        else:
            Accel(-weak)
            Steer(turnC)
            comment = "バック"
            time.sleep(0.2)

#ループ抜ける用       
    if distance4 <= STOPDIS and distance5 <= STOPDIS:
         count = count + 1
    else:
         count = 0
         
    if count == STOPTIME:
        print("escape roop")
        break#抜ける

#測定値表示用
    count1 = count1 + 1
    time.sleep(waittime)
    
    if count1 == 1:
        print("測定値 C:",distance1,", L:",distance2,", R:",distance3,", SL:",distance4,", SR:",distance5,comment)
        count1 = 0
 

#while抜けた後の停止処理
p_A.stop()
p_B.stop()
GPIO.cleanup()
print("End!!")