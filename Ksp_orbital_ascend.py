import time
import krpc
import math

conn = krpc.connect()
vessel = conn.space_center.active_vessel
body = conn.space_center.bodies[vessel.orbit.body.name]

# function to calculate the target pitch angle based on height
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
    
# function to calculate degree per second For Roll
def Rate_of_Change_Roll(Current_Heading, Previous_heading, elapsed_time):
    Distance = Current_Heading - Previous_heading
    Degrees_Per_Second = abs(Distance / elapsed_time)
    return Degrees_Per_Second

# function to calculate degree per second For Pitch
def Rate_of_Change_Pitch(Current_Heading, Previous_heading, elapsed_time):
    Distance = Previous_heading - Current_Heading
    Degrees_Per_Second = abs(Distance / elapsed_time)
    return Degrees_Per_Second

# function to calculate degree per second For Yaw
def Rate_of_Change_Yaw(Current_Heading, Previous_heading):
    Distance = Previous_heading - Current_Heading
    Time = 0.5
    Degrees_Per_Second = Distance / Time
    return Degrees_Per_Second

def Phase_one_Negative_Yaw(Yaw_Difference):

    if (Yaw_Difference < -0.00005):
        Soothing_rate = 0.025
        Exponential_Rate = 2
        Yaw_command = Soothing_rate * (abs(Yaw_Difference ** Exponential_Rate))
        Yaw_command = max(-1, min(1, Yaw_command))  # Ensure yaw command is between -1 and 1
        vessel.control.yaw = -Yaw_command

def Phase_One_Positive_Yaw (Yaw_Difference):
    if (0.00005 < Yaw_Difference):
        # Changing soothing rate
        Soothing_rate = 0.050
        Exponential_Rate = 2
        Yaw_command = Soothing_rate * (abs(Yaw_Difference ** Exponential_Rate))
        Yaw_command = max(-1, min(1, Yaw_command))  # Ensure yaw command is between -1 and 1
        vessel.control.yaw = Yaw_command

def Positive_Pitch(Heading_Difference, Heading_Rate_Of_Change):
        
    if (Heading_Difference < 0.0000):
        Soothing_rate = 0.0050
        Exponential_Rate = 2
        Pitch_Command = Soothing_rate * (abs(Heading_Difference) ** Exponential_Rate)
        Pitch_Command = max(-1, min(1, Pitch_Command))  # Ensure pitch command is between -1 and 1
        vessel.control.pitch = Pitch_Command # positive pitch for negative Heading difference

    elif (Heading_Difference <= -0.00005):
        Soothing_rate = 0.0075
        Exponential_Rate = 2
        Pitch_Command = Soothing_rate * (abs(Heading_Difference) ** Exponential_Rate)
        Pitch_Command = max(-1, min(1, Pitch_Command))  # Ensure pitch command is between -1 and 1
        vessel.control.pitch = Pitch_Command # positive pitch for negative Heading difference
        
def Negative_Pitch(Heading_Difference, Heading_Rate_Of_Change):

    if (0.00000 < Heading_Difference < 0.00005):
        Soothing_rate = 0.001
        Exponential_Rate = 2
        Pitch_Command = Soothing_rate * (abs(Heading_Difference) ** Exponential_Rate)
        Pitch_Command = max(-1, min(1, Pitch_Command))  # Ensure pitch command is between -1 and 1
        vessel.control.pitch = -Pitch_Command # negative pitch for positive Heading difference

    elif (0.0005 <= Heading_Difference):
        Soothing_rate = 0.0125
        Exponential_Rate = 2
        Pitch_Command = Soothing_rate * (abs(Heading_Difference) ** Exponential_Rate)
        Pitch_Command = max(-1, min(1, Pitch_Command))  # Ensure pitch command is between -1 and 1
        vessel.control.pitch = -Pitch_Command  # negative pitch for positive Heading difference

def Negative_Roll (Roll_Difference, Roll_Rate_Of_Change):

    if (0.0000 < Roll_Difference < 0.0005):
        Soothing_rate = 0.000050
        Exponential_Rate = 2.5
        Roll_Command = Soothing_rate * (abs(Roll_Difference) ** Exponential_Rate)
        Roll_Command = max(-1, min(1, Roll_Command))  # Ensure pitch command is between -1 and 1
        vessel.control.roll = -Roll_Command

    elif (0.0010 <= Roll_Difference):
        Soothing_rate = 0.000125
        Exponential_Rate = 2.5
        Roll_Command = Soothing_rate * (abs(Roll_Difference) ** Exponential_Rate)
        Roll_Command = max(-1, min(1, Roll_Command))  # Ensure pitch command is between -1 and 1
        vessel.control.roll = -Roll_Command

def Positive_Roll(Roll_Difference, Roll_Rate_Of_Change):

    if (Roll_Difference < 0.0000):
        Soothing_rate = 0.000050
        Exponential_Rate = 2.5
        Roll_Command = Soothing_rate * (abs(Roll_Difference) ** Exponential_Rate)
        Roll_Command = max(-1, min(1, Roll_Command))  # Ensure pitch command is between -1 and 1
        vessel.control.roll = Roll_Command
    
    elif (-0.0010 < Roll_Difference):
        Soothing_rate = 0.000125
        Exponential_Rate = 2.5
        Roll_Command = Soothing_rate * (abs(Roll_Difference) ** Exponential_Rate)
        Roll_Command = max(-1, min(1, Roll_Command))  # Ensure pitch command is between -1 and 1
        vessel.control.roll = +Roll_Command

def Phase_two_positive_yaw(Yaw_Difference):
    if (0.00005 < Yaw_Difference):
        # Changing soothing rate
        Soothing_rate = 0.025
        Exponential_Rate = 2
        Yaw_command = Soothing_rate * (abs(Yaw_Difference ** Exponential_Rate))
        Yaw_command = max(-1, min(1, Yaw_command))  # Ensure yaw command is between -1 and 1
        vessel.control.yaw = Yaw_command#

def  Phase_Two_Negative_Yaw(Yaw_Difference):
    if (Yaw_Difference < -0.00005):
        Soothing_rate = 0.025
        Exponential_Rate = 2
        Yaw_command = Soothing_rate * (abs(Yaw_Difference ** Exponential_Rate))
        Yaw_command = max(-1, min(1, Yaw_command))  # Ensure yaw command is between -1 and 1
        vessel.control.yaw = -Yaw_command

# get Mu
mu = 3.5316e12
Radius = 600000

# Capture the start time
start_time = time.perf_counter()

# Define the threshold in seconds
threshold = 0.05

# Threshold height
height_threshold_1 = 10000
height_threhsold_2 = 20000
height_threhsold_3 = 40000
height_threhsold_4 = 75000

#Boolean
checker = True

#Target degree for 10000
Degree_Target_for_10000m = 50
Degree_Target_for_20000m = 40
Degree_Target_for_40000m = 20
Degree_Target_for_75000m = 5

# Headoing Target
Heading_Target = 90

# Roll Target
Roll_Target = 90

#degree per second target
Degree_per_second_Target = 0.0008


# Initialize previous values for debugging
previous_pitch = None
previous_vessel_roll = vessel.flight().roll
previous_vessel_pitch = vessel.flight().pitch
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
    
    Roll_Difference = Vessel_Roll - Roll_Target

    degrees_per_Seconds_Roll = Rate_of_Change_Roll(Vessel_Roll, previous_vessel_roll, elapsed_time)

    Degrees_per_second_pitch = Rate_of_Change_Pitch(Vessel_Heading, previous_vessel_pitch, elapsed_time)

    previous_vessel_roll = Vessel_Roll

    previous_vessel_pitch = Vessel_Heading

    if (height < height_threshold_1):
        if (height > 150):

            vessel.control.sas = False
            vessel.control.rcs = False

            if (Yaw_Difference < 0.00):
                Phase_one_Negative_Yaw(Yaw_Difference)

            elif (Yaw_Difference > 0.00):
                Phase_One_Positive_Yaw (Yaw_Difference)
                
                

            if(vessel.flight().pitch < 89):
                if (Heading_Difference < 0.00):
                    Positive_Pitch(Heading_Difference, Degrees_per_second_pitch)
                elif (0.00 < Heading_Difference):
                    Negative_Pitch(Heading_Difference, Degrees_per_second_pitch)
                    
                if (Roll_Difference < 0.000):
                    Positive_Roll(Roll_Difference, degrees_per_Seconds_Roll)

                elif (Roll_Difference > 0.000):
                    Negative_Roll(Roll_Difference, degrees_per_Seconds_Roll)

            print(f"Target Degree: {Target_degree:.5f} | Vessels Pitch: {vessel.flight().pitch:.5f} | Target Difference: {Yaw_Difference:.5f} | Yaw Control: {vessel.control.yaw:.5f} | Vessel Heading {Vessel_Heading:.5f} | Target Heading: {Heading_Target:.5f} | Heading Difference: {Heading_Difference:.5f} | Pitch Control: {vessel.control.pitch:.5f} | Roll Diference: {Roll_Difference:.5f} | Roll Command: {vessel.control.roll:.5f} | Rate of Change Roll: {degrees_per_Seconds_Roll:.8f} | Rate of Change pitch: {Degrees_per_second_pitch:.8f} | Vessel Height {vessel.flight().mean_altitude:.5f} | ")

    elif(height_threshold_1 < height < height_threhsold_2):

        Target_degree = smooth_transition(height, height_threshold_1, height_threhsold_2, Degree_Target_for_10000m, Degree_Target_for_20000m)      

        Yaw_Difference = Vessel_yaw - Target_degree
        if (Yaw_Difference < 0.0000):
            Phase_Two_Negative_Yaw(Yaw_Difference)

        elif (Yaw_Difference > 0.0000):
            Phase_two_positive_yaw(Yaw_Difference)
                
        if (Heading_Difference < 0.0000):
            Positive_Pitch(Heading_Difference, Degrees_per_second_pitch)
                
        elif (Heading_Difference > 0.0000):
            Negative_Pitch(Heading_Difference, Degrees_per_second_pitch)

        if (Roll_Difference < 0.0000):
            Positive_Roll(Roll_Difference, degrees_per_Seconds_Roll)

        elif (Roll_Difference > 0.0000):
            Negative_Roll(Roll_Difference, degrees_per_Seconds_Roll)

        print(f"Target Degree: {Target_degree:.5f} | Vessels Pitch: {vessel.flight().pitch:.5f} | Target Difference: {Yaw_Difference:.5f} | Yaw Control: {vessel.control.yaw:.5f} | Vessel Heading {Vessel_Heading:.5f} | Target Heading: {Heading_Target:.5f} | Heading Difference: {Heading_Difference:.5f} | Pitch Control: {vessel.control.pitch:.5f} | Roll Diference: {Roll_Difference:.5f} | Roll Command: {vessel.control.roll:.5f} | Rate of Change Roll: {degrees_per_Seconds_Roll:.8f} | Rate of Change pitch: {Degrees_per_second_pitch:.6f} | Vessel Height {vessel.flight().mean_altitude:.5f} | ")

    elif (height_threhsold_2 < height < height_threhsold_3):
        Target_degree = smooth_transition(height, height_threhsold_2, height_threhsold_3, Degree_Target_for_20000m, Degree_Target_for_40000m)

        Yaw_Difference = Vessel_yaw - Target_degree

        if (Yaw_Difference < 0.0000):
            Phase_Two_Negative_Yaw(Yaw_Difference)

        elif (Yaw_Difference > 0.0000):
            Phase_two_positive_yaw(Yaw_Difference)
                
        if (Heading_Difference < 0.0000):
            Positive_Pitch(Heading_Difference, Degrees_per_second_pitch)
                
        elif (Heading_Difference > 0.0000):
            Negative_Pitch(Heading_Difference, Degrees_per_second_pitch)

        if (Roll_Difference < 0.0000):
            Positive_Roll(Roll_Difference, degrees_per_Seconds_Roll)

        elif (Roll_Difference > 0.0000):
            Negative_Roll(Roll_Difference, degrees_per_Seconds_Roll)

        print(f"Target Degree: {Target_degree:.5f} | Vessels Pitch: {vessel.flight().pitch:.5f} | Target Difference: {Yaw_Difference:.5f} | Yaw Control: {vessel.control.yaw:.5f} | Vessel Heading {Vessel_Heading:.5f} | Target Heading: {Heading_Target:.5f} | Heading Difference: {Heading_Difference:.5f} | Pitch Control: {vessel.control.pitch:.5f} | Roll Diference: {Roll_Difference:.5f} | Roll Command: {vessel.control.roll:.5f} | Rate of Change Roll: {degrees_per_Seconds_Roll:.8f} | Rate of Change pitch: {Degrees_per_second_pitch:.6f} | Vessel Height {vessel.flight().mean_altitude:.5f} | ")

    if(height >= 13000):
        vessel.control.rcs = True

time.sleep(0.1)
