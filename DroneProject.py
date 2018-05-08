import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
#import RPi.GPIO as GPIO
import dronekit_sitl 
from pymavlink import mavutil

#Arm and Takeoff.

sitl=dronekit_sitl.start_default(20.737641,-103.457018)
connection_string = sitl.connection_string

def arm_and_takeoff(TargetAltitude):
	#Vehicle Connection
	print ("Executing Takeoff")

	while not drone.is_armable:
		print ("Vehicle is not armable, waiting...")
		time.sleep(1)

	print ("Ready to arm")
	drone.mode = VehicleMode("GUIDED")
	drone.armed = True

	while not drone.armed:
		print("Waiting for arming...")
		time.sleep(1)

	print("Ready for takeoff, taking off...")
	drone.simple_takeoff(TargetAltitude)

	while True:
		Altitude = drone.location.global_relative_frame.alt
		print("Altitud", Altitude)
		time.sleep(1)

		if Altitude >= TargetAltitude * 0.95:
			print("ALtitude has been reached")
			break

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

def condition_yaw(vehicle, heading, relative=False):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = drone.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    drone.send_mavlink(msg)

#Make parallel
def getdistance():
	return 130
"""	try:
      	GPIO.setmode(GPIO.BOARD)

      	PIN_TRIGGER = 23
      	PIN_ECHO = 24

      	GPIO.setup(PIN_TRIGGER, GPIO.OUT)
      	GPIO.setup(PIN_ECHO, GPIO.IN)

      	GPIO.output(PIN_TRIGGER, GPIO.LOW)

      	print "Waiting for sensor to settle"

      	time.sleep(1)

      	print "Calculating distance"

      	GPIO.output(PIN_TRIGGER, GPIO.HIGH)

      	time.sleep(0.000001)

      	GPIO.output(PIN_TRIGGER, GPIO.LOW)

      	while GPIO.input(PIN_ECHO)==0:
            pulse_start_time = time.time()
      	while GPIO.input(PIN_ECHO)==1:
            pulse_end_time = time.time()

      	pulse_duration = pulse_end_time - pulse_start_time
      	distance = round(pulse_duration * 17150, 2)
      	print "Distance:",distance,"cm"
      	return distance
	finally:
      	GPIO.cleanup()
"""

def scan():
	for i in range (0,10):
		distance=getdistance()
		face=0
		condition_yaw(drone,0)
		time.sleep(2)

		if distance<220:
			condition_yaw(drone,45)
			time.sleep(2)
			distance=getdistance()
			print ("45")
			if distance<220:
				condition_yaw(drone,90)
				time.sleep(2)
				distance=getdistance()
				print("90")
				if distance<220:
					condition_yaw(drone,315)
					time.sleep(5)
					distance=getdistance()
					print("315")
					if distance<220:
						condition_yaw(drone,270)
						time.sleep(2)
						distance=getdistance()
						print("270")
						if distance<220:
							drone.mode = VehicleMode("LAND")
		set_velocity_body(drone,0.5,0,0)
		time.sleep(1)

"""		if distance<220 and face=0
			condition_yaw(15)
			face=15
			set_velocity_body(drone,.5,0,0)
			distance=getdistance()
			elif
				set_velocity_body(drone,.5,0,0)
				distance=getdistance()

		if distance<220 and face=15
			condition_yaw(30)
			face=30
			set_velocity_body(drone,.5,0,0)
			distance=getdistance()
			elif
				set_velocity_body(drone,.5,0,0)
				distance=getdistance()

		if distance<220 and face=30
			condition_yaw(45)
			face=45
			set_velocity_body(drone,.5,0,0)
			distance=getdistance()
			elif
				set_velocity_body(drone,.5,0,0)
				distance=getdistance()

		if distance<220 and face=45
			condition_yaw(60)
			face=60
			set_velocity_body(drone,.5,0,0)
			distance=getdistance()
			elif
				set_velocity_body(drone,.5,0,0)
				distance=getdistance()

		if distance<220 and face=60
			condition_yaw(75)
			face=75
			set_velocity_body(drone,.5,0,0)
			distance=getdistance()
			elif
				set_velocity_body(drone,.5,0,0)
				distance=getdistance()

		if distance<220 and face=75
			condition_yaw(90)
			face=90
			set_velocity_body(drone,.5,0,0)
			distance=getdistance()
			elif
				set_velocity_body(drone,.5,0,0)
				distance=getdistance()

		if distance<220 and face=90
			condition_yaw(345)
			face=345
			set_velocity_body(drone,.5,0,0)
			distance=getdistance()
			elif
				set_velocity_body(drone,.5,0,0)
				distance=getdistance()

		if distance<220 and face=345
			condition_yaw(330)
			face=330
			set_velocity_body(drone,.5,0,0)
			distance=getdistance()
			elif
				set_velocity_body(drone,.5,0,0)
				distance=getdistance()

		if distance<220 and face=330
			condition_yaw(315)
			face=315
			set_velocity_body(drone,.5,0,0)
			distance=getdistance()
			elif
				set_velocity_body(drone,.5,0,0)
				distance=getdistance()

		if distance<220 and face=315
			condition_yaw(300)
			face=300
			set_velocity_body(drone,.5,0,0)
			distance=getdistance()
			elif
				set_velocity_body(drone,.5,0,0)
				distance=getdistance()

		if distance<220 and face=300
			condition_yaw(285)
			face=285
			set_velocity_body(drone,.5,0,0)
			distance=getdistance()
			elif
				set_velocity_body(drone,.5,0,0)
				distance=getdistance()

		if distance<220 and face=285
			condition_yaw(270)
			face=270
			set_velocity_body(drone,.5,0,0)
			distance=getdistance()
			elif
				set_velocity_body(drone,.5,0,0)
				distance=getdistance()

		if distance<220 and face=270
			condition_yaw(15)
			face=15
			set_velocity_body(drone,.5,0,0)
			distance=getdistance()
			elif
				set_velocity_body(drone,.5,0,0)
				distance=getdistance()

		if distance>220
			condition_yaw(0)
			face=0
			set_velocity_body(drone,.5,0,0)
			distance=getdistance()
"""


drone = connect('127.0.0.1:14551' , wait_ready=True)

arm_and_takeoff(2)

drone.airspeed = .5


scan()