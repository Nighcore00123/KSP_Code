import time
import krpc
import math


conn = krpc.connect()

vessel = conn.space_center.active_vessel

body = conn.space_center.bodies[vessel.orbit.body.name]

# Exponential Variables
Soothing_rate = 0.2
Exponential_Rate = 3

# get Mu
mu = 3.5316e12
Radius = 600000


# Capture the start time
start_time = time.perf_counter()

# Define the threshold in seconds
threshold = 0.05

# Threshold height
height_threshold_1 = 10000

#Boolean
checker = True


#Target degree for 10000
Degree_Target_for_10000m = 45

# Headoing Target
Heading_Target = 90

# Define a function to calculate the target pitch angle based on height
def calculate_target_degree(height, max_height, max_degree):
    return 90 - ((90 - max_degree) * (height / max_height))

# Initialize previous values for debugging
previous_pitch = None
previous_time = start_time


# Loop to continuously check elapsed time
while checker == True:

    # Get the pitch of the vessel
    Vessel_yaw = vessel.flight().pitch

    # Get the current time
    current_time = time.perf_counter()
    
    # Get the current mass of the rocket
    mass = vessel.mass

    # get the current vessel thrust
    thrust = vessel.thrust

     # Get the current vessel drag as a tuple
    drag_tuple = vessel.flight().drag

    # Calculate the magnitude of the drag vector
    drag = math.sqrt(drag_tuple[0]**2 + drag_tuple[1]**2 + drag_tuple[2]**2)

    # get the height of the current vessel
    height = vessel.flight().mean_altitude

    # Get the current velocity of the vessel as a tuple
    current_velocity_tuple = vessel.flight().velocity
    
    # Calculate the magnitude of the velocity vector
    current_velocity = math.sqrt(current_velocity_tuple[0]**2 + current_velocity_tuple[1]**2 + current_velocity_tuple[2]**2)

    # Calculate Distance From center of body
    Distance = height + Radius

    # Calculate Gravitational froce
    G = mu / Distance ** 2

    # Vessel Heading
    Vessel_Heading = vessel.flight().heading

    elapsed_time = current_time - start_time

    if (height < 10000):
        get_Heading = vessel.flight().heading
        if(get_Heading > 90 and time.time() - start_time < 3):
            vessel.control.yaw = 0.02

        if (elapsed_time >= threshold and height > 100):

            vessel.control.sas = False

            # Calculating the Acceleration of rocket
            Acceleration_Squared = thrust - drag - (mass * G)
            Acceleration_Squared = Acceleration_Squared / mass

            if Acceleration_Squared < 0:
                Acceleration_Squared = 1

            Acceleration = math.sqrt(Acceleration_Squared)
            
            # Caulating the Time to Reach Height Threshold 
            Time_Squared = (0.5 * height_threshold_1) / Acceleration
            if Time_Squared < 0:
                print(f"Time squared to threshold is negative: {Time_Squared:.2e}")
                Time_Squared = float('inf')
            else:
                Time = math.sqrt(Time_Squared)

            Distance_To_threshold = height_threshold_1 - height

            Time_to_height = 2 * Distance_To_threshold  / Acceleration_Squared

            Time_to_height = math.sqrt(max (Time_to_height, 0))

            # Calculating Target Degree for 10000M
            Target_degree = calculate_target_degree(height, height_threshold_1, Degree_Target_for_10000m)

            # Calculating the Velocity at Height Threshold 
            Velocity = current_velocity + (Acceleration_Squared * Time)

            # Calculate the pitch difference
            Yaw_Difference = Vessel_yaw - Target_degree

            # Calculate Heading Difference
            Heading_Difference = Vessel_Heading - Heading_Target

            # Exponential yaw command
            Yaw_command = Soothing_rate * (Yaw_Difference ** Exponential_Rate)
            Yaw_command = max(-1, min(1, Yaw_command))  # Ensure yaw command is between -1 and 1

            Pitch_Command = Soothing_rate * (Heading_Difference ** Exponential_Rate)
            Pitch_Command = max(-1, min(1, Pitch_Command))  # Ensure pitch command is between -1 and 1
            start_time = time.time()
            if (0.0 < Heading_Difference <= 0.30):
                    Soothing_rate = 0.0025
                    Exponential_Rate = 2
                    Pitch_Command = Soothing_rate * (abs(Heading_Difference) ** Exponential_Rate)
                    Pitch_Command = max(-1, min(1, Pitch_Command))  # Ensure pitch command is between -1 and 1
                    vessel.control.pitch = -Pitch_Command # minus command for positiv difference

            if (0.30 < Heading_Difference <= 0.5):
                    Soothing_rate = 0.005
                    Exponential_Rate = 2
                    Pitch_Command = Soothing_rate * (abs(Heading_Difference) ** Exponential_Rate)
                    Pitch_Command = max(-1, min(1, Pitch_Command))  # Ensure pitch command is between -1 and 1
                    vessel.control.pitch = -Pitch_Command

            if (0.5 < Heading_Difference <= 1):
                    Soothing_rate = 0.0075
                    Exponential_Rate = 2
                    Pitch_Command = Soothing_rate * (abs(Heading_Difference) ** Exponential_Rate)
                    Pitch_Command = max(-1, min(1, Pitch_Command))  # Ensure pitch command is between -1 and 1
                    vessel.control.pitch = -Pitch_Command
                
            if (Heading_Difference < 0):
                    Soothing_rate = 0.005
                    Exponential_Rate = 4
                    Pitch_Command = Soothing_rate * (abs(Heading_Difference) ** Exponential_Rate)
                    Pitch_Command = max(-1, min(1, Pitch_Command))  # Ensure pitch command is between -1 and 1
                    vessel.control.pitch = Pitch_Command # positiv command for negative difference

            if (-1 > Heading_Difference <= -0.50):
                    Soothing_rate = 0.0075
                    Exponential_Rate = 4
                    Pitch_Command = Soothing_rate * (abs(Heading_Difference) ** Exponential_Rate)
                    Pitch_Command = max(-1, min(1, Pitch_Command))  # Ensure pitch command is between -1 and 1
                    vessel.control.pitch = Pitch_Command # positiv command for negative difference

            if (-2 > Heading_Difference <= -1):
                    Soothing_rate = 0.01
                    Exponential_Rate = 4
                    Pitch_Command = Soothing_rate * (abs(Heading_Difference) ** Exponential_Rate)
                    Pitch_Command = max(-1, min(1, Pitch_Command))  # Ensure pitch command is between -1 and 1
                    vessel.control.pitch = Pitch_Command # positiv command for negative difference
            
            if (vessel.flight().heading > 100 or vessel.flight().heading < 80):
                    vessel.control.pitch = 0
            start_time = current_time
            time.sleep(0.1)

            while (vessel.control.abort == True):
                    #vessel.control.pitch = 0
                    vessel.control.yaw = 0.03
                    vessel.control.sas = True
                    time.sleep(5)
                    if(vessel.flight().speed < 220):
                        vessel.control.activate_next_stage()
                        checker = False

                    print("ABORT !!!")
            
            print(f"Elapsed Time: {elapsed_time:.2f} | Vessel_Heading: {Vessel_Heading:.2f} | Vessel Heading Difference {Heading_Difference:.2f} | Vessel Control {vessel.control.pitch:.2f} | Target Degree: {Target_degree:.2f} ")
            print(f"")    
            # else:
            #     vessel.control.yaw = 0.05