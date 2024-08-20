import time
import krpc
import math

conn = krpc.connect()

vessel = conn.space_center.active_vessel

body = conn.space_center.bodies[vessel.orbit.body.name]

# Define a function to calculate the target pitch angle based on height
def calculate_target_degree(height, max_height, max_degree):
    degree = 90 - ((90 - max_degree) * (height / max_height))
    degree = max(min(degree, 90), max_degree)  # Constrain the output
    return degree

def smooth_transition(current_height, threshold_1, threshold_2, degree_1, degree_2):
    if current_height <= threshold_1:
        return degree_1
    elif current_height >= threshold_2:
        return degree_2
    else:
        # Linearly interpolate between degree_1 and degree_2
        factor = (current_height - threshold_1) / (threshold_2 - threshold_1)
        return degree_1 + factor * (degree_2 - degree_1)

def Negative_Yaw(Yaw_Difference):

    if (Yaw_Difference < 0):
        Soothing_rate = 0.075
        Exponential_Rate = 1
        Yaw_command = Soothing_rate * (Yaw_Difference ** Exponential_Rate)
        Yaw_command = max(-1, min(1, Yaw_command))  # Ensure yaw command is between -1 and 1
        vessel.control.yaw = Yaw_command

    elif (-0.003 > Yaw_Difference <= -0.005):
        Soothing_rate = 0.1
        Exponential_Rate = 1
        Yaw_command = Soothing_rate * (Yaw_Difference ** Exponential_Rate)
        Yaw_command = max(-1, min(1, Yaw_command))  # Ensure yaw command is between -1 and 1
        vessel.control.yaw = Yaw_command

    elif (-0.005 > Yaw_Difference <= -0.01):
        # Changing soothing rate
        Soothing_rate = 0.150
        Exponential_Rate = 1
        Yaw_command = Soothing_rate * (Yaw_Difference ** Exponential_Rate)
        Yaw_command = max(-1, min(1, Yaw_command))  # Ensure yaw command is between -1 and 1
        vessel.control.yaw = Yaw_command

    elif (Yaw_Difference < -0.01):
        Soothing_rate = 0.2
        Exponential_Rate = 0.5
        Yaw_command = Soothing_rate * (Yaw_Difference ** Exponential_Rate)
        Yaw_command = max(-1, min(1, Yaw_command))  # Ensure yaw command is between -1 and 1
        vessel.control.yaw = Yaw_command

def Positive_Yaw (Yaw_difference):
    
    if (0.000 < Yaw_Difference <= 0.003):
        # Changing soothing rate
        Soothing_rate = 0.05
        Exponential_Rate = 1
        Yaw_command = Soothing_rate * (Yaw_Difference ** Exponential_Rate)
        Yaw_command = max(-1, min(1, Yaw_command))  # Ensure yaw command is between -1 and 1
        vessel.control.yaw = Yaw_command

    elif (0.003 < Yaw_Difference <= 0.005):
        # Changing soothing rate
        Soothing_rate = 0.075
        Exponential_Rate = 1
        Yaw_command = Soothing_rate * (Yaw_Difference ** Exponential_Rate)
        Yaw_command = max(-1, min(1, Yaw_command))  # Ensure yaw command is between -1 and 1
        vessel.control.yaw = Yaw_command
                
    elif (0.005 < Yaw_Difference <= 0.01):
        # Changing soothing rate
        Soothing_rate = 0.1
        Exponential_Rate = 1
        Yaw_command = Soothing_rate * (Yaw_Difference ** Exponential_Rate)
        Yaw_command = max(-1, min(1, Yaw_command))  # Ensure yaw command is between -1 and 1
        vessel.control.yaw = Yaw_command

    elif (Yaw_Difference > 0.01):
        Soothing_rate = 0.2
        Exponential_Rate = 0.5
        Yaw_command = Soothing_rate * (Yaw_Difference ** Exponential_Rate)
        Yaw_command = max(-1, min(1, Yaw_command))  # Ensure yaw command is between -1 and 1
        vessel.control.yaw = Yaw_command


def Positive_Pitch(Heading_Difference):
        
    if (Heading_Difference < 0):
        Soothing_rate = 0.0075
        Exponential_Rate = 0.5
        Pitch_Command = Soothing_rate * (abs(Heading_Difference) ** Exponential_Rate)
        Pitch_Command = max(-1, min(1, Pitch_Command))  # Ensure pitch command is between -1 and 1
        vessel.control.pitch = Pitch_Command # positive pitch for negative Heading difference

    elif (-1 < Heading_Difference <= -0.50):
        Soothing_rate = 0.01
        Exponential_Rate = 0.5
        Pitch_Command = Soothing_rate * (abs(Heading_Difference) ** Exponential_Rate)
        Pitch_Command = max(-1, min(1, Pitch_Command))  # Ensure pitch command is between -1 and 1
        vessel.control.pitch = Pitch_Command # positive pitch for negative Heading difference

    elif (-1.50 < Heading_Difference <= -1):
        Soothing_rate = 0.0150
        Exponential_Rate = 0.8
        Pitch_Command = Soothing_rate * (abs(Heading_Difference) ** Exponential_Rate)
        Pitch_Command = max(-1, min(1, Pitch_Command))  # Ensure pitch command is between -1 and 1
        vessel.control.pitch = Pitch_Command # positive pitch for negative Heading difference

    elif (Heading_Difference < - 3):
        Soothing_rate = 0.02
        Exponential_Rate = 1
        Pitch_Command = Soothing_rate * (abs(Heading_Difference) ** Exponential_Rate)
        Pitch_Command = max(-1, min(1, Pitch_Command))  # Ensure pitch command is between -1 and 1
        vessel.control.pitch = Pitch_Command # positive pitch for negative Heading difference


def Negative_Pitch(Heading_Difference):
    if (0 < Heading_Difference < 0.50):
        Soothing_rate = 0.01
        Exponential_Rate = 0.5
        Pitch_Command = Soothing_rate * (abs(Heading_Difference) ** Exponential_Rate)
        Pitch_Command = max(-1, min(1, Pitch_Command))  # Ensure pitch command is between -1 and 1
        vessel.control.pitch = -Pitch_Command # negative pitch for positive Heading difference

    elif (0.50 < Heading_Difference < 1):
        Soothing_rate = 0.0125
        Exponential_Rate = 0.5
        Pitch_Command = Soothing_rate * (abs(Heading_Difference) ** Exponential_Rate)
        Pitch_Command = max(-1, min(1, Pitch_Command))  # Ensure pitch command is between -1 and 1
        vessel.control.pitch = -Pitch_Command # negative pitch for positive Heading difference

    elif (1 < Heading_Difference < 1.5):
        Soothing_rate = 0.0150
        Exponential_Rate = 0.8
        Pitch_Command = Soothing_rate * (abs(Heading_Difference) ** Exponential_Rate)
        Pitch_Command = max(-1, min(1, Pitch_Command))  # Ensure pitch command is between -1 and 1
        vessel.control.pitch = -Pitch_Command # negative pitch for positive Heading difference
    
    elif(1.5 < Heading_Difference < 3):
        Soothing_rate = 0.0175
        Exponential_Rate = 1
        Pitch_Command = Soothing_rate * (abs(Heading_Difference) ** Exponential_Rate)
        Pitch_Command = max(-1, min(1, Pitch_Command))  # Ensure pitch command is between -1 and 1
        vessel.control.pitch = -Pitch_Command # negative pitch for positive Heading difference

def Negative_Roll (Vessel_Roll):
    if (90 < Vessel_Roll):
        Soothing_rate = 0.0025
        Exponential_Rate = 0.5
        Roll_Command = Soothing_rate * (abs(Vessel_Roll) ** Exponential_Rate)
        Roll_Command = max(-1, min(1, Roll_Command))  # Ensure pitch command is between -1 and 1
        vessel.control.roll = -Roll_Command

    elif (90 < Vessel_Roll > 90.50):
        Soothing_rate = 0.005
        Exponential_Rate = 0.5
        Roll_Command = Soothing_rate * (abs(Vessel_Roll) ** Exponential_Rate)
        Roll_Command = max(-1, min(1, Roll_Command))  # Ensure pitch command is between -1 and 1
        vessel.control.roll = -Roll_Command

    elif (90.5 < Vessel_Roll > 91):
        Soothing_rate = 0.0075
        Exponential_Rate = 0.5
        Roll_Command = Soothing_rate * (abs(Vessel_Roll) ** Exponential_Rate)
        Roll_Command = max(-1, min(1, Roll_Command))  # Ensure pitch command is between -1 and 1
        vessel.control.roll = -Roll_Command

    elif(91 < Vessel_Roll):
        Soothing_rate = 0.01
        Exponential_Rate = 1
        Roll_Command = Soothing_rate * (abs(Vessel_Roll) ** Exponential_Rate)
        Roll_Command = max(-1, min(1, Roll_Command))  # Ensure pitch command is between -1 and 1
        vessel.control.roll = -Roll_Command


def Positive_Roll(Vessel_Roll):
    if (Vessel_Roll < 90):
        Soothing_rate = 0.0025
        Exponential_Rate = 0.5
        Roll_Command = Soothing_rate * (abs(Vessel_Roll) ** Exponential_Rate)
        Roll_Command = max(-1, min(1, Roll_Command))  # Ensure pitch command is between -1 and 1
        vessel.control.roll = Roll_Command
    
    elif (89.50 < Vessel_Roll < 90):
        Soothing_rate = 0.005
        Exponential_Rate = 0.5
        Roll_Command = Soothing_rate * (abs(Vessel_Roll) ** Exponential_Rate)
        Roll_Command = max(-1, min(1, Roll_Command))  # Ensure pitch command is between -1 and 1
        vessel.control.roll = Roll_Command

    elif (89 < Vessel_Roll < 89.50):
        Soothing_rate = 0.0075
        Exponential_Rate = 0.5
        Roll_Command = Soothing_rate * (abs(Vessel_Roll) ** Exponential_Rate)
        Roll_Command = max(-1, min(1, Roll_Command))  # Ensure pitch command is between -1 and 1
        vessel.control.roll = Roll_Command
    
    elif (Vessel_Roll < 89 ):
        Soothing_rate = 0.01
        Exponential_Rate = 1
        Roll_Command = Soothing_rate * (abs(Vessel_Roll) ** Exponential_Rate)
        Roll_Command = max(-1, min(1, Roll_Command))  # Ensure pitch command is between -1 and 1
        vessel.control.roll = Roll_Command

# get Mu
mu = 3.5316e12
Radius = 600000


# Capture the start time
start_time = time.perf_counter()

# Define the threshold in seconds
threshold = 0.05

# Threshold height
height_threshold_1 = 10000
height_threhsold_2 = 15000

#Boolean
checker = True


#Target degree for 10000
Degree_Target_for_10000m = 45

Degree_Target_for_20000m = 40

# Headoing Target
Heading_Target = 90

# Roll Target
Roll_Target = 90



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

    # Get Vessel Roll
    Vessel_Roll = vessel.flight().roll

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

    if (height < height_threshold_1):
        if(Vessel_Heading > 180):
            vessel.control.yaw = 0.02
        else:
            vessel.control.yaw = 0

        if (height > 150):

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

            if (Yaw_Difference < 0.00):
                Negative_Yaw (Yaw_Difference)

            elif (Yaw_Difference > 0.00):
                Positive_Yaw (Yaw_Difference)
            
            if (Heading_Difference < 0.00):
                Positive_Pitch(Heading_Difference)
            
            elif (Heading_Difference > 0.00):
                Negative_Pitch(Heading_Difference)

            if(vessel.flight().pitch < 88):
                if (Vessel_Roll < 90):
                    Positive_Roll(Vessel_Roll)

                elif (Vessel_Roll > 90):
                    Negative_Roll(Vessel_Roll)
                elif (Vessel_Roll == 90):
                    vessel.control.roll = 0
            print(f"Target Degree: {Target_degree:.2f} | Vessels Pitch: {vessel.flight().pitch:.3f} | Target Difference: {Yaw_Difference:.2f} | Yaw Control: {vessel.control.yaw:.2f} | Vessel Heading {Vessel_Heading:.2f} | Target Heading: {Heading_Target:.2f} | Heading Difference: {Heading_Difference:.2f} | Pitch Control: {vessel.control.pitch:.2f} | Vessels Roll: {Vessel_Roll:.2f} | Roll Command: {vessel.control.roll:.2f} | Vessel Height {vessel.flight().mean_altitude:.2f} | ")
        


    
    if(height_threshold_1 < height < height_threhsold_2):
            
            
            vessel.control.sas = False

            Target_degree = smooth_transition(height, height_threshold_1, height_threhsold_2, Degree_Target_for_10000m, Degree_Target_for_20000m)

            # Calculating the Acceleration of rocket
            Acceleration_Squared = thrust - drag - (mass * G)
            Acceleration_Squared = Acceleration_Squared / mass

            if Acceleration_Squared < 0:
                Acceleration_Squared = 1

            Acceleration = math.sqrt(Acceleration_Squared)
            
            # Caulating the Time to Reach Height Threshold 
            Time_Squared = (0.5 * height_threhsold_2) / Acceleration
            if Time_Squared < 0:
                print(f"Time squared to threshold is negative: {Time_Squared:.2e}")
                Time_Squared = float('inf')
            else:
                Time = math.sqrt(Time_Squared)

            Distance_To_threshold = height_threhsold_2 - height

            Time_to_height = 2 * Distance_To_threshold  / Acceleration_Squared

            Time_to_height = math.sqrt(max (Time_to_height, 0))

            # Calculating the Velocity at Height Threshold 
            Velocity = current_velocity + (Acceleration_Squared * Time)

            # Calculate the pitch difference
            Yaw_Difference = Vessel_yaw - Target_degree

            # Calculate Heading Difference
            Heading_Difference = Vessel_Heading - Heading_Target

            if (Yaw_Difference < 0.00):
                Negative_Yaw (Yaw_Difference)

            elif (Yaw_Difference > 0.00):
                Positive_Yaw (Yaw_Difference)
            
            if (Heading_Difference < 0.00):
                Positive_Pitch(Heading_Difference)
            
            elif (Heading_Difference > 0.00):
                Negative_Pitch(Heading_Difference)

            if (Vessel_Roll < 90):
                Positive_Roll(Vessel_Roll)

            elif (Vessel_Roll > 90):
                Negative_Roll(Vessel_Roll)
            
            elif (Vessel_Roll == 90):
                vessel.control.roll = 0

            print(f"Target Degree: {Target_degree:.2f} | Vessels Pitch: {vessel.flight().pitch:.3f} | Target Difference: {Yaw_Difference:.2f} | Yaw Control: {vessel.control.yaw:.2f} | Vessel Heading {Vessel_Heading:.2f} | Target Heading: {Heading_Target:.2f} | Heading Difference: {Heading_Difference:.2f} | Pitch Control: {vessel.control.pitch:.2f} | Vessels Roll: {Vessel_Roll:.2f} | Roll Command: {vessel.control.roll:.2f} | Vessel Height {vessel.flight().mean_altitude:.2f} | ")
        


else:
    checker = False
time.sleep(0.1)