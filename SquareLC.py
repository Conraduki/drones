#La mayoria del codigo fue tomado de -python.dronekit.io- utilizado con fines educativos y siendo modificado para las... 
#... necesidades del programa mostrado en la parte inferior. 

#Basic information: when you write time.sleep(x) the program will stop x seconds.
#Before running THIS code...
	#Requirements: Using MacOsX download APM Planner; if Windows, download MissionPlanner. 
	#Step 1: Open a terminal and write the following: 
		#dronekit-sitl copter --home=20.737641,-103.457018,1357,200
	#Step 1 explanation: This will allow you to run dronekit and set your home location to one of the fields in the campus.
	#Step 2: Open a different terminal and write the following:
		#mavproxy.py --master tcp:127.0.0.1:5760 --out udp:127.0.0.1:14551 --out udp:YOUR IP:14550
	#Step 2 explanation: This will connect your dronekit with your drone, allowing to test the code in APM/Mission Planner.

#HECHO POR LUIS CONRADO ALCALA BECERRA

# Lines 20 and 21 are the ones that will allow us to use all the libraries that Python or ,in this case...
#... Dronekit have available to make easier our work.

#A library is a group of codes already pre-written in Python or some apps that you can download (Dronekit).
from  dronekit import connect, VehicleMode, LocationGlobalRelative
import time

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

#Line 63: Vehicle Connection using the values previously written in our second terminal.
drone = connect('127.0.0.1:14551' , wait_ready=True)

#Line 66: We are calling the function defined in lines x-x with a Target Altitude of 20
arm_and_takeoff(20)

#Line 69: Setting drone speed in m/s
drone.airspeed = 10

#Lines 73-75: Defining the 3 waypoints to complete the mission. Coordinates in Latitude,Longitude,RelativeAltitude.
#Latitude and Longitude retrieved from Google Maps.
FirstWaypoint = LocationGlobalRelative (20.737602, -103.456557, 20)
SecondWaypoint = LocationGlobalRelative (20.737197, -103.456588, 20)
ThirdWaypoint = LocationGlobalRelative (20.737234, -103.457055, 20)

#Lines 79-89: Directing the drone to the 3 waypoints previously defined using a simple_goto command, which allow us...
#... to change the position (Lat,Lon,Alt) of our drone.
print("Flying to the first waypoint")
drone.simple_goto(FirstWaypoint)
time.sleep(18)

print("Flying to the second waypoint")
drone.simple_goto(SecondWaypoint)
time.sleep(18)

print("Flying to the third waypoint")
drone.simple_goto(ThirdWaypoint)
time.sleep(18)

#Lines 92-93: Retrieving the voltage of the battery from APM/Mission Planner and showing it to the user.
DroneBattery= drone.battery.voltage
print ('Drone Battery:', DroneBattery, 'V')

#Lines 96-97: Changing the Drone mode to RTL (Return To Landing), that send us to the point were the drone took off.
print ("Returning home")
drone.mode = VehicleMode("RTL")

#Line 100: Exiting the code.
drone.close()