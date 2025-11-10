import gamepad 
from cyberpi import* 
import time
onfloor = int(0)
onbase = int(0)


while True:
    #Code to use joystick to control servo
    raw = gamepad.get_joystick('Lx')
    value = int((raw+100)* 180 /200)
    raw2 = gamepad.get_joystick('Rx')
    value2 = int((raw2+100)* 180 /200)
    if gamepad.is_key_pressed('Up'):
        #Code for Omni Wheel Movement
        mbot2.drive_power(100,0)
        mbot2.motor_set(-200,'M1')
        mbot2.motor_set(0,'M2') #M3
    elif gamepad.is_key_pressed('Down'): 
        #Code for Omni Wheel Movement
        mbot2.drive_power(-100,0)
        mbot2.motor_set(200,'M1')
        mbot2.motor_set(0,'M2') #M3
        
        mbot2.motor_set(0,'M2')
    elif gamepad.is_key_pressed('Left'): 
        #Code for Omni Wheel Movement
        mbot2.drive_power(-5,0)
        mbot2.motor_set(200,'M1')
        mbot2.motor_set(-100,'M2') #M3
    elif gamepad.is_key_pressed('Right'): 
        #Code for Omni Wheel Movement
        mbot2.drive_power(-100,0)
        mbot2.motor_set(-7,'M1')
        mbot2.motor_set(100,'M2') #M3
    elif gamepad.is_key_pressed('L1'): 
        #Code for Omni Wheel Movement
        mbot2.drive_power(-100,0)
        mbot2.motor_set(-200,'M1')
        mbot2.motor_set(-100,'M2') #M3
        #spinleft
    elif gamepad.is_key_pressed('R1'): 
        #Code for Omni Wheel Movement
        mbot2.drive_power(100,0)
        mbot2.motor_set(100,'M1')
        mbot2.motor_set(100,'M2') #M3
        #spinright  
    elif gamepad.is_key_pressed('N1'): 
        if onbase == 0:
            #Code for to use N1 button Mutiple times to set Servo difference angle
            mbot2.servo_set(65,'s2')
            time.sleep(1)
            mbot2.servo_set(0,'s4')
            onbase =1
        elif onbase == 1:
            mbot2.servo_set(80,'s2')
            time.sleep(1)
            mbot2.servo_set(3,'s4')
            onbase = 2
        else:
            mbot2.servo_set(35,'s4') #0
            time.sleep(2)
            mbot2.servo_set(80,'s2')
            onbase = 0  
    elif gamepad.is_key_pressed('N2'):
        if onfloor != 1:
            #Code for to use N1 button Mutiple times to set Servo difference angle   
            mbot2.servo_set(10,'s4')
            time.sleep(1)
            mbot2.servo_set(3,'s2')
            onfloor = 1
        else:
            mbot2.servo_set(130,'s4') #0
            time.sleep(2)
            mbot2.servo_set(70,'s2')
            onfloor = 0
    elif gamepad.is_key_pressed('R2'): 
        #Code to set servo open gripper
        mbot2.servo_set(90,'s3')    
    elif gamepad.is_key_pressed('L2'): 
        #Code to set servo close gripper
        mbot2.servo_set(0,'s3')     
    else:
        #Code to set motor to stop when not pressing
        mbot2.motor_set(0,'M1')
        mbot2.EM_stop()
        mbot2.motor_set(0,'M2')
        mbot2.servo_set(value,'s4')
        mbot2.servo_set(value2,'s2')
        #Code to set servo to the joystick angle 
    
    