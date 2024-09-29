import krpc


conn = krpc.connect()

vessel = conn.space_center.active_vessel

def Thrust_to_weight():
    Total_thrust = sum(engine.thrust for engine in vessel.parts.engines)
    mass_kg = vessel.mass
    Local_gravity = vessel.orbit.body.surface_gravity
    
    twr = Total_thrust / (mass_kg * Local_gravity)
    return twr



def adjust_throttle(twr, velocity_x, velocity_y, velocity_z):
    target_vel_Z = 0
    target_twr = 1.00

    velocity_err = target_vel_Z - velocity_z
    
    if(velocity_z < -50):
        vessel.control.throttle = 0
        print("throttle cut", "X-Axis Velocity (m/s):", velocity_x , "Z-Axis Velocity (m/s):", velocity_z , "Y-Axis Velocity (m/s):", velocity_y, "TWR", twr)
        

    elif(velocity_z > target_vel_Z and twr < target_twr ):
        vessel.control.throttle += 0.01
        print("X-Axis Velocity (m/s):", velocity_x , "Z-Axis Velocity (m/s):", velocity_z , "Y-Axis Velocity (m/s):", velocity_y, "TWR", twr)

    elif(velocity_z > target_vel_Z and twr > target_twr ):
            vessel.control.throttle += 0.01
            print("X-Axis Velocity (m/s):", velocity_x , "Z-Axis Velocity (m/s):", velocity_z , "Y-Axis Velocity (m/s):", velocity_y, "TWR", twr)
            if(velocity_z < 10 or velocity_z > -10):
                Fine_throttle(twr, velocity_x, velocity_y, velocity_z, target_vel_Z, target_twr, velocity_err)
                print("X-Axis Velocity (m/s):", velocity_x , "Z-Axis Velocity (m/s):", velocity_z , "Y-Axis Velocity (m/s):", velocity_y, "TWR", twr)

    elif(velocity_z <  target_vel_Z and twr > target_twr):
        vessel.control.throttle -= 0.01
        print("X-Axis Velocity (m/s):", velocity_x , "Z-Axis Velocity (m/s):", velocity_z , "Y-Axis Velocity (m/s):", velocity_y, "TWR", twr)


    

def Fine_throttle(twr, velocity_x, velocity_y, velocity_z, target_vel_Z, target_twr, velocity_err):
    if(velocity_z > target_vel_Z):
        vessel.control.throttle -= 0.000000000000000010
        print("X-Axis Velocity (m/s):", velocity_x , "Z-Axis Velocity (m/s):", velocity_z , "Y-Axis Velocity (m/s):", velocity_y, "TWR", twr)

    
    elif(velocity_z < target_vel_Z ):
        vessel.control.throttle += 0.000000000000000010
        print("X-Axis Velocity (m/s):", velocity_x , "Z-Axis Velocity (m/s):", velocity_z , "Y-Axis Velocity (m/s):", velocity_y, "TWR", twr)

    if(velocity_z == target_vel_Z ):
        vessel.control.throttle = vessel.control.throttle
        print("X-Axis Velocity (m/s):", velocity_x , "Z-Axis Velocity (m/s):", velocity_z , "Y-Axis Velocity (m/s):", velocity_y, "TWR", twr)

while True:

    flight = vessel.flight(vessel.orbit.body.reference_frame)


    velocity_x, velocity_y, velocity_z = flight.velocity
    TWR = Thrust_to_weight()


    if(flight.surface_altitude >= 1500 and flight.surface_altitude <= 5000):
        adjust_throttle(TWR, velocity_x, velocity_y, velocity_z)
    else:
        print("X-Axis Velocity (m/s):", velocity_x , "Z-Axis Velocity (m/s):", velocity_z , "Y-Axis Velocity (m/s):", velocity_y, "TWR", TWR)
