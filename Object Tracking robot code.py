#Importing all packages

#RPI.GPIO for using GPIO pins as inputs and outputs on the Pi
#cv2 used for Computer Vision tasks of the project
#numpy to use arrays to achieve computer vision tasks
#pygame to build a graphical user interface on the PiTFT and display camera feed on it
#os used to execute system commands like shutdown etc
#gpio zero is imported to make use of built in functions to use with ultrasonic sensor

import RPi.GPIO as GPIO 
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import time
import pygame
from pygame.locals import *
#import cv2
#import numpy as np
import os
import sys
from gpiozero import DistanceSensor
from subprocess import call

#INITIALIZATIONS
#====================================================================================================

#declare variable that is an object with parameters regarding distance of the sensor
ultrasonic_front = DistanceSensor(echo=27, trigger=4)


#display the camera feed and pygame screen on the PiTFT
#framebuffer set to 0 since PiTFT is configured as 0 frame buffer when no external display is connected
os.putenv('SDL_VIDEODRIVER','fbcon')
os.putenv('SDL_FBDEV','/dev/fb0')


#initializing the variables for the object mid point as 0
x_mid =0
y_mid = 0

#initialization of kernels to perform dilation and closing
#This step smoothens the boundaries and contours of the red object
#this is an optional step. can be skipped for easier understanding and implementation
kernelOpen=np.ones((5,5))
kernelClose=np.ones((20,20))

#This thresh is basically the tolerence within which the object and frame centers would be considered aligned
thresh = 30
#the frequency of PWM signal generated and supplied to the motor driver
freq = 100


#This is the base duty cycle given to the motors set at 50%.
#it is changed to float because it works to mitigate the error so it can be computed in decimals as well
baseSpeed = float(50)
#this is the maximum duty cycle that will be added or subtracted from the base duty cycle based on the error generated 
pwmBound = float(40)

#this is the maximum error that can occur in terms of pixel differences
cameraBound = float(200)
#we find the coefficient for proprotional controller. This decides how much of the 40% duty cycle to add or subtract from the base sppeed
kp = pwmBound/cameraBound

#initialize the error to 0
error = 0

#GPIO INITIALIZATION
#=============================================================================
#PWM AND MOTORS INITIALIZATION
#Pins initialization
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP) #pull up
GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_UP) #pull up
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP) #pull up
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP) #pull up
GPIO.setup(13, GPIO.IN, pull_up_down=GPIO.PUD_UP) #pull up
GPIO.setup(4, GPIO.IN, pull_up_down=GPIO.PUD_UP) #pull up

GPIO.setup(5, GPIO.OUT)
GPIO.setup(6, GPIO.OUT)
GPIO.setup(26, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)

#DISABLE MOVEMENTS AT THE START
GPIO.output(5, False)
GPIO.output(6, False)
GPIO.output(26, False)
GPIO.output(21, False)
GPIO.output(16, False)
GPIO.output(20, False)

#===============================================================================


#CALLBACK FUNCTION DEFINE
#===============================================================================

# Pin 26 is set to give PWM signal to the right side motor whereas pin 21 will give a PWM signal to the let side motor. They are fed to the motor driver
right_pwm26 = GPIO.PWM(26, freq)
left_pwm21 = GPIO.PWM(21, freq)

#start the motors at a 0 duty cycle initially
right_pwm26.start(0)
left_pwm21.start(0)
#frequency and duty cycle initialized for motor operation


#This function is executed when button 17 is pressed. It quits the program when button 17 is pressed
def GPIO17_callback(channel):
    global robot_loop
    robot_loop = False
    left_pwm21.ChangeDutyCycle(0)
    right_pwm26.ChangeDutyCycle(0)
    print('should quit')
    
    print ("Button 17 pressed.")
#command to initialize the execution of GPIO17_callback function when button 17 is pressed
GPIO.add_event_detect(17, GPIO.FALLING, callback = GPIO17_callback, bouncetime = 300)


#We also need the functionality of shutting down the Pi and the program by a button press. When button 22 is pressed it shuts down the program and the Pi
def GPIO22_callback(channel):
    #global robot_loop
    robot_loop = False
    left_pwm21.ChangeDutyCycle(0)
    right_pwm26.ChangeDutyCycle(0)
    call("sudo shutdown -h now", shell=True)
    print('should shutdown')
    
    print ("Button 22 pressed.")

#command to initialize the execution of GPIO22_callback when button 22 is pressed
GPIO.add_event_detect(17, GPIO.FALLING, callback = GPIO17_callback, bouncetime = 300)

#=============================================================================================



#DEFINING FUNCTIONS TO MOVE THE ROBOT IN GIVEN DIRECTIONS
#==============================================================================================

#Defining functions to move the motors in response to PID


#FUNCTION TO MAKE THE ROBOT MOVE FORWARD. THE GPIO PINS ARE SET TO HIGH AND LOW ACCORDINGLY TO ACHIVE FORWARD MOTION    
def forward(rightPwm, leftPwm):
    right_pwm26.ChangeDutyCycle(rightPwm)
    GPIO.output(6, GPIO.HIGH)
    GPIO.output(5, GPIO.LOW)
    left_pwm21.ChangeDutyCycle(leftPwm)
    GPIO.output(20, GPIO.HIGH)
    GPIO.output(16, GPIO.LOW)

#FUNCTION TO MAKE THE ROBOT MOVE BACKWARD. THE GPIO PINS ARE SET TO HIGH AND LOW ACCORDINGLY TO ACHIVE BACKWARD MOTION
def backward(rightPwm, leftPwm):
    right_pwm26.ChangeDutyCycle(rightPwm)
    GPIO.output(6, GPIO.LOW)
    GPIO.output(5, GPIO.HIGH)
    left_pwm21.ChangeDutyCycle(leftPwm)
    GPIO.output(20, GPIO.LOW)
    GPIO.output(16, GPIO.HIGH)
#THIS FUNCTIONS SETS THE ROBOT TO REST. CHANGES THE DUTY CYCLE TO 0
def pwmStop():
    right_pwm26.ChangeDutyCycle(0)
    left_pwm21.ChangeDutyCycle(0)
    

#===========================================================================================


#INITIALIZING THE FRAME RESOLUTION OF THE PI CAMERA
new_width = 320 #640  # Replace with your desired width
new_height = 240 #480



#capturing the frame from Pi camera which is established as camera 0 or the default camera
cap = cv.VideoCapture(0, cv.CAP_V4L) #0 stands for webcam 0

#condition to exit the program if no camera is detected
if not cap:
   print("!!! Failed VideoCapture: unable to open device 0")
   sys.exit(1)
#============================================================================================



#PYGAME BUTTON AND DISPLAY INITIALIZATION
#============================================================================================
#pygame initialization
pygame.init()

#display set at the same size as the camera frame
screen = pygame.display.set_mode([320,240])

# Colors
white = (255, 255, 255)
black = (0, 0, 0)
green = (0, 255, 0)
red = (255, 0, 0)


#font and button size of the start button
font = pygame.font.Font(None, 18)
button_radius = 35
button1_x, button2_x, button3_x = new_width // 2,  new_width // 2, new_width // 2
button_y = new_height // 2

#robot_loop runs after the start button is pressed. hence it is currently false
robot_loop = False

# Main loop - This loops detects if Start button is pressed. Until then the robot does nothing. When the start button is pressed, the program goes out of this loop and into the robot_loop
first_button = True
#==============================================================================================



#Start of the loop which detects if the start button is pressed
#the robot program edoes not start until the start button is pressed and this while loop ends
while first_button:
    #This loop only checks in every run if the start button is pressed. If the button is pressed the first button will set False to get out of this loop and the next loop which is the robot program loop will start
    for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    x, y = event.pos
                    start_dist1 = ((x - button1_x) ** 2 + (y - button_y) ** 2) ** 0.5
                    #this condition cheks if the press is at a distance within the button radius
                    if start_dist1 <= button_radius:
                        #if the start button is pressed, the robot loop is set to start whereas this loop will end
                        robot_loop = True
                        first_button = False

    #These are generic process codes to execute pygame screen display
    screen.fill(white)
    #Draw the start button circle of the initlaized raidus and green in colour
    pygame.draw.circle(screen, green, (button1_x, button_y), button_radius)
    #the text the button should have and finally blit the text rect object with the text iteself
    text1 = font.render("Start", True, white)
    text1_rect = text1.get_rect(center=(button1_x, button_y))
    screen.blit(text1, text1_rect)
    
    #show the display
    pygame.display.flip()
#=========================================================================================================



#Here the robot loop actually starts.
#=============================================================================================================
i=0
while robot_loop:
    #frame variable stored the arrays of pixel intesities from the video capture
    misc, frame = cap.read()
    #this is just a counter of the frames recorded
    i += 1
    print(i)

    #we resize the frame to 230x240p and rotate it by 180 degrees since the camera was installed upside down due to assembly limitations
    frame = cv.resize(frame, (new_width, new_height))
    frame = cv.rotate(frame, cv.ROTATE_180)
    
    #CONVERT FRAME FROM BGR TO HSV
    hsv_frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    #red color
    #these are the highest and possible RGB values for different shades of red
    low_red = np.array([161,155,84])
    high_red = np.array([179,255,255])

    #create a mask for pixels that have their values within the given ranges
    red_mask = cv.inRange(hsv_frame, low_red, high_red)
    #performing morpohlogical filtering on the red objects to smoothen the boundaries
    maskOpen=cv.morphologyEx(red_mask,cv.MORPH_OPEN,kernelOpen) #this is opening
    maskClose=cv.morphologyEx(maskOpen,cv.MORPH_CLOSE,kernelClose) #we close the contours here
    contours, hierarchy = cv.findContours(red_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    #we sort the contours in order of area. The biggest contour which will be our red object wil only be detected further
    contours = sorted(contours, key=lambda x:cv.contourArea(x), reverse=True)

    #Now we create a bounding box around the contour and the coordinates of the box are x, y, x+w, y+h
    for cnt in contours:
        (x,y,w,h) = cv.boundingRect(cnt)
        #creates a rectangle
        cv.rectangle(frame,(x,y) , (x+w, y+h) , (0,255,0), 2)

        #THIS IS IMPORTANT!

        #THIS COMPUTES THE CENTER COORDINATE OF THE BOUNDING BOX OF THE UNIQUE RED OBJECT
        x_mid = int((x + x+ w)/2)
        y_mid = int((y+y+h)/2)

        break

    #HERE WE COMPUTE THE DEVIATION BETWEEN OBJECT CENTER AND FRAME CENTER BOTH IN X AND Y DIFFERENCE
    x_diff = x_mid - frame.shape[1]//2
    y_diff = y_mid - frame.shape[0]//2
    
    #store the distane from the nearest object from the ultrasonic sensor
    distance = ultrasonic_front.distance
    
    
   #This condition says that if there are no contours which means no red object detected then do the following
    if len(contours) == 0:
        #print on the screen that no unqiue object is detected
        cv.putText(frame, str('no unique id') , (20,150) , cv.FONT_HERSHEY_COMPLEX, 1.0, (0,255,0), thickness=2)
        #set the duty cycles of both motors to 0 which means to stop the robot movement
        pwmStop()

        #If there is a contour, then do the following
    else:
        #print x_diff and y_diff on the screen
        cv.putText(frame, str(f'x diff is {x_diff} , y_diff is {y_diff}') , (20,150) , cv.FONT_HERSHEY_COMPLEX, 1.0, (0,255,0), thickness=2)
        cv.circle(frame, (x_mid,y_mid) , 4, (0,255,0) , thickness=-1) #thickness =-1 if you want to fill the circle
        cv.circle(frame, (frame.shape[1]//2,frame.shape[0]//2) , 7, (0,255,0) , thickness = -1)
        
        
        #Implementation of PID!!!

        #We compute error by combining the error in x and y directions and taking the diagonal/hyptoneuse
        error = result = (x_diff**2 + y_diff**2)**0.5
        #output the desired duty cycle by multiplying kp with the error.
        #this basically tells how much 
        pwmOut = abs(kp*error)
        area = (x+w)*(y+h)


        #This condition checks that if the sensor detects an object within 5 cm to it but that object is not the red object which means its an obstacle then it will back up and change it direction to avoid obstacle avoidance
        if distance < 0.05 and area < 76800:
            backward(70,70)
            time.sleep(0.7)
            forward(30,70)
            time.sleep(0.7)
            forward(70,30)
            time.sleep(0.3)
            forward(70,70)
            time.sleep(0.7)
            pwmStop()

        #For other conditions, do this routine
        else:
            #Conditions for the cases where x_diff is greater than the tolerence of 30. x_diff>thresh means the object is on the right of the robot
            if x_diff > thresh:
                #if y_diff is greater than thresh it means the red object is too close so the robot moves backward and towards left so it aligns with object in both x and y directions
                if y_diff > thresh:
                    cv.putText(frame, str('element on the right & move back pls') , (20,20) , cv.FONT_HERSHEY_COMPLEX, 0.5, (0,255,0), thickness=2)
                    
                    #MOVE backward AND RIGHT
                    backward((baseSpeed+pwmOut) , (baseSpeed-pwmOut))
                    

                #if y_diff < thresh this means the object is far from the robot so the robot moves forward and to the right to align with the object that was on the right    
                elif y_diff < (-1*thresh):
                    cv.putText(frame, str('element on the right & move fwd pls') , (20,20) , cv.FONT_HERSHEY_COMPLEX, 0.5, (0,255,0), thickness=2)
                    #the pwmOut duty cycle is subtracted from the right motor and added to the left motor because we need to steer to the right hence the left motor should rotate faster
                    forward((baseSpeed-pwmOut) , (baseSpeed+pwmOut))
                    
                #if y_diff is within the tolerence then the robot does not need to move forward or backward. it just needs to move to the right
                elif (-1*thresh) <= y_diff <= thresh:
                    cv.putText(frame, str('element on the right. donot go fwd/back') , (20,20) , cv.FONT_HERSHEY_COMPLEX, 0.5, (0,255,0), thickness=2)
                    forward(0 , pwmOut)


            #if the x_diff < thresh this means the object is on the left of the robot
            elif x_diff < (-1*thresh):
                #means the object is too close hence move bacakwards
                if y_diff > thresh:
                    cv.putText(frame, str('element on the left & move back pls') , (20,20) , cv.FONT_HERSHEY_COMPLEX, 0.5, (0,255,0), thickness=2)
                    
                    #MOVE backward AND right 
                    backward((baseSpeed-pwmOut) , (baseSpeed+pwmOut))
                    
                #move forward and to the left
                elif y_diff < (-1*thresh):
                    cv.putText(frame, str('element on the left & move fwd pls') , (20,20) , cv.FONT_HERSHEY_COMPLEX, 0.5, (0,255,0), thickness=2)
                    
                    #MOVE forward AND LEFT
                    forward((baseSpeed+pwmOut) , (baseSpeed-pwmOut))
                
                #only steer to the left hence just the right motor is given duty cycle
                elif (-1*thresh) <= y_diff <= thresh:
                    cv.putText(frame, str('element on the left. donot go fwd/back') , (20,20) , cv.FONT_HERSHEY_COMPLEX, 0.5, (0,255,0), thickness=2)

                    forward((pwmOut) , 0)

            #if x_diff is within the thresh then no need to steer to the left or right
            elif (-1*thresh) <= x_diff <= thresh:
                if y_diff > thresh:
                    #move backward since the object is too close
                    cv.putText(frame, str('move back. dont move left/right') , (20,20) , cv.FONT_HERSHEY_COMPLEX, 0.5, (0,255,0), thickness=2)
                    
                    backward((baseSpeed+pwmOut) , (baseSpeed+pwmOut))
                    
                #move forward since the object is far from the robot
                elif y_diff < (-1*thresh):
                    cv.putText(frame, str('move fdw. dont move left/right') , (20,20) , cv.FONT_HERSHEY_COMPLEX, 0.5, (0,255,0), thickness=2)
                    
                    forward((baseSpeed+pwmOut) , (baseSpeed+pwmOut))

                    
                #if both x and y difference are within threshold this means the object is centered hence the robot should come to rest
                elif (-1*thresh) <= y_diff <= thresh:
                    cv.putText(frame, str('centered. dont move') , (20,20) , cv.FONT_HERSHEY_COMPLEX, 0.5, (0,255,0), thickness=2)
                    pwmStop()
                    #stop_flag = True
    
    #display the camera frame on the pitft
    pygame.display.flip()

    #just rotating the frame to show right display on the PiTFT. Transformation on the frame from numpy array to pygame object.
    frame = np.rot90(frame)
    frame = pygame.surfarray.make_surface(frame)
    frame = pygame.transform.flip(frame,False, True)
    screen.blit(frame, (0,0))
    pygame.display.update()
    #cv.imshow('RoboCar View' , frame)

#Quit the program if escape key is pressed. However, this was not used since we were using physical buttons and we didn't have a keyboard connected to the autonomous robot
    key=cv.waitKey(1)
    if key==27:
            
        break

#quit pygame, bring the motors to rest when the program is quit and clean up any signals on the GPIOs
cap.release()
pygame.quit()
pwmStop()
GPIO.cleanup()

#Finally destroy all windows generated by cv2 video capture
cv.destroyAllWindows()

