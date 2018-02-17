#Basic information: when you write time.sleep(x) the program will stop x seconds.
#Before running THIS code...
    #Requirements: Using MacOsX download APM Planner; if Windows, download MissionPlanner. 
    #Step 1: Open a terminal and write the following: 
        #dronekit-sitl copter --home=20.737641,-103.457018,1357,0
    #Step 1 explanation: This will allow you to run dronekit and set your home location to one of the fields in the campus.
    #Step 2: Open a different terminal and write the following:
        #mavproxy.py --master tcp:127.0.0.1:5760 --out udp:127.0.0.1:14551 --out udp:YOUR IP:14550
    #Step 2 explanation: This will connect your dronekit with your drone, allowing to test the code in APM/Mission Planner.

#HECHO POR LUIS CONRADO ALCALA BECERRA

# Lines 17 to 20 are the ones that will allow us to use all the libraries that Python or ,in this case...
#... Dronekit have available to make easier our work.

#A library is a group of codes already pre-written in Python or some apps that you can download (Dronekit).
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import Tkinter as tk
import dronekit_sitl 

sitl=dronekit_sitl.start_default(20.737641,-103.457018)
connection_string = sitl.connection_string

#Lines 27-60 are a function that can be called later. This function is made to arm our drone (turn off the...
#...security that we usually did by pressing the lighting button in the top or our drone). The value in line...
#... 27 that is between parenthesis is our Target Altitude, value which we will write when the function is...
#... called.

def arm_and_takeoff(TargetAltitude):
    #Vehicle Connection
    print ("Executing Takeoff")

    #Lines 32-34 stand for the waiting proccess when the drone is not ready to be armed.
    while not drone.is_armable:
        print ("Vehicle is not armable, waiting...")
        time.sleep(1)

    #Lines 38-40 are activated when our drone is ready to be armed and we define the drone mode to GUIDED.
    #Guided Mode is used when we are controlling our drone telemetrically, like we are specting for in this code.
    print ("Ready to arm")
    drone.mode = VehicleMode("GUIDED")
    drone.armed = True

    #Lines 43-45: Its work is to wait the drone to be armed and tell the user that he needs to wait.
    while not drone.armed:
        print("Waiting for arming...")
        time.sleep(1)

    #Lines 48-49: When the drone is armed it will start to lift up with the objetive of reaching the Target Altitude.
    print("Ready for takeoff, taking off...")
    drone.simple_takeoff(TargetAltitude)

    #Lines 52-55: Tell the user which is the altitude of the drone.
    while True:
        Altitude = drone.location.global_relative_frame.alt
        print("Altitud", Altitude)
        time.sleep(1)
        
    #Lines 58-60: When Target Altitud is reached or almost reached (at least 95%) all the function will end.
        if Altitude >= TargetAltitude * 0.95:
            print("ALtitude has been reached")
            break

#This function, as "arm_and_takeoff" is used to call some commands later, in this case they are used to... 
#...change the drone position in 3 vectors, x, y and z.
def set_velocity_body(vehicle, vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

#This function is coordinated with "set_velocity_body", this is the one that reads the key that is being pressed.
#This part of the code is the one that puts the drone into movement with the keys.
def key(event):
    if event.char == event.keysym: #-- standard keys
        if event.keysym == 'r':
            print ("Returning home")
            drone.mode = VehicleMode("RTL")      
    else: 
        if event.keysym == 'Up': set_velocity_body(drone, 5,0,0)
        elif event.keysym == 'Down':set_velocity_body(drone, -5,0,0)
        elif event.keysym == 'Left':set_velocity_body(drone, 0,-5,0)
        elif event.keysym == 'Right':set_velocity_body(drone, 0,5,0)
        
drone = connect('127.0.0.1:14551' , wait_ready=True)

# Take off to 10 m altitude
arm_and_takeoff(10)
 
# Read the keyboard with tkinter: a little white board appears and you have to select the square to click the keys so they...
#... start reading the code and the movement.
root = tk.Tk()
print(">> Control the drone with the arrow keys. Press r for RTL mode")
root.bind_all('<Key>', key)
root.mainloop()

Retrieving the voltage of the battery from APM/Mission Planner and showing it to the user.
DroneBattery= drone.battery.voltage
print ('Drone Battery:', DroneBattery, 'V')

#Line 102: Exiting the code.
drone.close()

sitl.stop()