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