# Meca470_UBR_Project

Team #6

Members: Jarod Mica, Alec Mitchell, Bryce Gregory

## Introduction
The original purpose of this project was to design a robot capable of facing and stocking shelves in a retail setting.  Using the UBR_1 robot, this was to be achieved by interfacing between a ROS environment and a chosen simulation environment, Coppelia Sim.  However, due to limited documentation on the UBR_1 and time constraints, this proved to be much more difficult than what had been planned and only simple communication could be achieved from ROS to the UBR in Copellia.

## Software
- CoppeliaSim
- Virtual Box
- ROS (melodic)

## Coding
The coding done for the project was all done in a virtual maching running ROS Melodic.  The main functionality of it is publishing data to the port Coppelia is subscribed to in order to move the chosen joint of the UBR.  Since the code for moving a joint is the same using the Remote API functionality of Copellia Sim, the following code block shows how it was coded for the shoulder_lift_joint chosen.  Below it is a process on how the code works from the block provided.

Since the Remote API can be configured by referencing Coppelia's user manual, the details for the files needed won't be discussed.  The only part that will be discussed is what is important in the code below. https://www.coppeliarobotics.com/helpFiles/en/legacyRemoteApiOverview.htm

```
#!/usr/bin/env python3
import rospy
import sim
import simConst
import sys
import time

#-----Try to connect---------------
def connect():
    sim.simxFinish(-1)
    your_IP='192.168.0.165'
    clientID = sim.simxStart(your_IP,19999,True,True,10000,5)
    if clientID != -1:
        print("Connected to remote API server")
    else:
        print("Not connected to remote API server")
        sys.exit ("could not connect")
    while not rospy.is_shutdown():
        ask_shoulder = 'Enter shoulder lift:'
        err_code = sim.simxStartSimulation(clientID,sim.simx_opmode_oneshot)
        #Ask
        sim.simxAddStatusbarMessage(clientID,ask_shoulder,sim.simx_opmode_oneshot)
        #Loop to check for valid input
        while True:
            try:
                move_shoulder = float(input('Enter shoulder lift: \n')); #Take movement input
            except ValueError:
                print("Invalid input, enter again:")
                #better try again... Return to the start of the loop
                continue
            else:
                #valid input
                break
        #Move
        print ("Now moving {} rad/s\n".format(move_shoulder))
        sim.simxAddStatusbarMessage(clientID,"Now moving {} rad/s\n".format(move_shoulder),sim.simx_opmode_oneshot)
        err_code,j1 = sim.simxGetObjectHandle(clientID,"shoulder_lift_joint",sim.simx_opmode_blocking)
        
        err_code = sim.simxSetJointTargetVelocity (clientID, j1, move_shoulder,sim.simx_opmode_streaming) # set the postion of J1
        time.sleep(1) # Move for 1 second since rad/s
        
        move_shoulder = 0 # Stop arm movement
        err_code = sim.simxSetJointTargetVelocity (clientID, j1, move_shoulder,sim.simx_opmode_streaming) # set the postion of J1
        #sim.simxGetPingTime(clientID)    
        #sim.simxFinish(clientID)
           
if __name__ == '__main__':
    try: 
        connect()
    except rospy.ROSInterruptException:
        pass
```

## Breakdown


```
#!/usr/bin/env python3
import rospy
import sim
import simConst
import sys
import time
```

The first thing that had to be included were all of the used imports.  Import for sim and simConst were found in the Copellia installation files which must be moved into the folder running our connect.py program.

```
if __name__ == '__main__':
    try: 
        connect()
    except rospy.ROSInterruptException:
        pass
```

Then a main function calling to the function connect() had to be created for the program to run.  

```
#-----Try to connect---------------
def connect():
    sim.simxFinish(-1)
    your_IP='192.168.0.165'
    clientID = sim.simxStart(your_IP,19999,True,True,10000,5)
    if clientID != -1:
        print("Connected to remote API server")
    else:
        print("Not connected to remote API server")
        sys.exit ("could not connect")
```

This part of the code is what connects our ROS virtual machine to Coppelia Sim.  First a fuction connect() was created. The variable your_IP is the Ip4 address of the host machine and the 19999 is the port that Coppelia is specified to listen to.  This must also be specified in Coppelia Sim and is done by pasting simRemoteApi.start(19999) into the sysCall_init function of a Coppelia sim non-threaded child script.  If it connects, the client ID will be anything except -1. If not, it fails and exits the program.

```
while not rospy.is_shutdown():
        ask_shoulder = 'Enter shoulder lift:'
        err_code = sim.simxStartSimulation(clientID,sim.simx_opmode_oneshot)
        #Ask
        sim.simxAddStatusbarMessage(clientID,ask_shoulder,sim.simx_opmode_oneshot)
        #Loop to check for valid input
        while True:
            try:
                move_shoulder = float(input('Enter shoulder lift: \n')); #Take movement input
            except ValueError:
                print("Invalid input, enter again:")
                #better try again... Return to the start of the loop
                continue
            else:
                #valid input
                break
```

The next part of the function is to check for a valid input.  As we are looking for a numeric value, anything that is not a float or integer will break the code and cause the program to fail.  This is a simple while loop implemented to check for such input and will only break the loop when it's valid.  

```
        #Move
        print ("Now moving {} rad/s\n".format(move_shoulder))
        sim.simxAddStatusbarMessage(clientID,"Now moving {} rad/s\n".format(move_shoulder),sim.simx_opmode_oneshot)
        err_code,j1 = sim.simxGetObjectHandle(clientID,"shoulder_lift_joint",sim.simx_opmode_blocking)
        
        err_code = sim.simxSetJointTargetVelocity (clientID, j1, move_shoulder,sim.simx_opmode_streaming) # set the postion of J1
        time.sleep(1) # Move for 1 second since rad/s
        
        move_shoulder = 0 # Stop arm movement
        err_code = sim.simxSetJointTargetVelocity (clientID, j1, move_shoulder,sim.simx_opmode_streaming) # set the postion of J1
        #sim.simxGetPingTime(clientID)    
        #sim.simxFinish(clientID)
```

The last part now just passes the valid input into a function that will update the new position of the chosen joint.  These are functions provided by Copellia Sim and located in the sim.pyc file. 

```
sim.simxAddStatusbarMessage(clientID,"Now moving {} rad/s\n".format(move_shoulder),sim.simx_opmode_oneshot)
```

This code just outputs a message in the status bar of Coppelia Sim.

```
err_code,j1 = sim.simxGetObjectHandle(clientID,"shoulder_lift_joint",sim.simx_opmode_blocking)
```

This code looks for a joint name in Coppelia Sim.  Here it looks for "shoulder_lift_joint" and stores the handle name into j1.

```
err_code = sim.simxSetJointTargetVelocity (clientID, j1, move_shoulder,sim.simx_opmode_streaming) # set the postion of J1
        time.sleep(1) # Move for 1 second since rad/s
```

Now the input gets passed into Coppelia sim by calling this function and moves it for 1 second since the movement specified is in rad/s.  This means if an input of 1 is specified, it'll move a total of 1 rad.

```
        move_shoulder = 0 # Stop arm movement
        err_code = sim.simxSetJointTargetVelocity (clientID, j1, move_shoulder,sim.simx_opmode_streaming) # set the postion of J1
        #sim.simxGetPingTime(clientID)    
        #sim.simxFinish(clientID)
```

The last bit is to stop the movement of the arm.  Since the input changes the velocity to some chosen rad/s, the move_shoulder variable is set back to zero and passed back into Coppelia to stop the movement of the arm.

## 
