import krpc
import time

connection = krpc.connect()
vessel = connection.space_center.active_vessel
vessel.control.throttle = 1
vessel.control.activate_next_stage()

# Define the desired altitude range for hovering
hover_altitude_lower = 500  # Lower altitude threshold
hover_altitude_upper = 1500  # Upper altitude threshold
target_velocity = 0  # Target vertical velocity for hovering

def Thrust_to_weight():
    Total_thrust = sum(engine.thrust for engine in vessel.parts.engines)
    mass_kg = vessel.mass
    Local_gravity = vessel.orbit.body.surface_gravity
    
    twr = Total_thrust / (mass_kg * Local_gravity)
    return twr

def adjust_throttle(target_velocity):
    velocity = vessel.flight(vessel.orbit.body.reference_frame).vertical_speed
    velocity_error = target_velocity - velocity

    print("Current Vertical Velocity:", velocity)
    print("Velocity Error:", velocity_error)

    # Adjust throttle based on velocity error
    if abs(velocity_error) > 0.1:  # Threshold for significant velocity error
        if velocity_error > 0:  # Need to reduce velocity
            vessel.control.throttle -= 0.05
        else:  # Need to increase velocity
            vessel.control.throttle += 0.05

while True:
    altitude = vessel.flight().surface_altitude

    if hover_altitude_lower <= altitude <= hover_altitude_upper:
        adjust_throttle(target_velocity)
    else:
        vessel.control.throttle = 1  # Reset throttle outside the desired altitude range

    twr = Thrust_to_weight()
    print(f"Thrust To Weight Ratio {twr}, Current Altitude: {altitude}")
        
    time.sleep(0.5)
