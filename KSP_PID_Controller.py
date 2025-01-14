import math
import time
import matplotlib.pyplot as plt
import krpc

#Global variables
conn = krpc.connect()
vessel = conn.space_center.active_vessel

mu = 3.5316e12
Radius = 600000

#------Important Functions------
def calculate_target_degree(height, max_Height, max_Degree):
    degree = 90 - ((90 - max_Degree) * (height / max_Height))
    degree = max(min(degree, 90), max_Degree)  # Constrain the output
    return degree

def smooth_transition(current_Height, threshold_1, threshold_2, degree_1, degree_2):
    if current_Height <= threshold_1:
        return degree_1
    elif current_Height >= threshold_2:
        return degree_2
    else:
        # Linearly interpolate between degree_1 and degree_2
        factor = (current_Height - threshold_1) / (threshold_2 - threshold_1)
        return degree_1 + factor * (degree_2 - degree_1)

def pitch_pid (kp, ki, kd, pitch_target, current_time, previous_time, previous_error, initial_integral_value, max_integral):
    passed_time = current_time - previous_time
    pitch_error = pitch_target - vessel.flight().pitch

    proportional_calculation = kp * pitch_error
    if vessel.flight().pitch == 0.00:
        initial_integral_value = 0
    else:
        initial_integral_value += pitch_error * passed_time

    initial_integral_value = max(-max_integral, min(max_integral, initial_integral_value))
    integral_calculation = ki * initial_integral_value
    derivative_calculation = kd * (pitch_error - previous_error) / passed_time

    PID_output = proportional_calculation + integral_calculation + derivative_calculation
    pitch_output = max(-1, min(1, PID_output))

    vessel.control.yaw = -pitch_output

    return current_time, pitch_error, initial_integral_value

def roll_pid(kp, ki, kd, roll_target, current_time, previous_time, previous_error, initial_Integral_value, max_integral):
    passed_time = current_time - previous_time
    roll_error = roll_target - vessel.flight().roll


    proportional_calculation = kp * roll_error
    if vessel.flight().roll == 0.00:
        initial_Integral_value = 0
    else:
        initial_Integral_value += roll_error * passed_time


    initial_Integral_value = max(-max_integral, min(max_integral, initial_Integral_value))
    integral_calculation = ki * initial_Integral_value

    derivative_calculation = kd * (roll_error - previous_error) / passed_time

    PID_output = proportional_calculation + integral_calculation + derivative_calculation

    roll_output = max(-1, min(1, PID_output))

    #print("time", current_time, "pid output", PID_output)

    vessel.control.roll = roll_output

    return current_time, roll_error, initial_Integral_value

def heading_pid(kp, ki, kd, heading_target, current_time, previous_time, previous_error, initial_Integral_value, max_integral):
    passed_time = current_time - previous_time
    heading_error = heading_target - vessel.flight().heading

    proportional_Calculation = kp * heading_error

    if vessel.flight().heading == 0.00:
        initial_Integral_value = 0
    else:
            initial_Integral_value += heading_error * passed_time

    initial_Integral_value = max(-max_integral, min(max_integral, initial_Integral_value))
    integral_calculation = ki * initial_Integral_value

    derivative_calculation = kd * (heading_error - previous_error) / passed_time

    PID_output = proportional_Calculation + integral_calculation + derivative_calculation
    yaw_output = max(-1, min(1, PID_output))

    vessel.control.pitch = yaw_output

    return current_time, heading_error, initial_Integral_value

def orbital_insertion_burn_calculation():
    gravitational_constant_kerbin = 6.67 * 10 ** -11
    kerbin_radius = 6.0 * 10 ** 5
    kerbin_Mass = 5.3 * 10 ** 22

    total_Radius = kerbin_radius + vessel.orbit.apoapsis_altitude

    deltaV_Requirement = (gravitational_constant_kerbin * kerbin_Mass) / total_Radius

    deltaV_Requirement = math.sqrt(deltaV_Requirement)

    deltaV_Requirement = deltaV_Requirement - vessel.orbit.speed

    return deltaV_Requirement

def sub_orbital_phase():
    # ----------------Heading PID Controller Variables------------
    # 0.17092437744140623 - 0.1833892822265625 = tu = 0.01
    # ku = 0.6
    # tu = 0.00001 ms
    # kp = 0.3
    # ki = 0.0000072
    # kd = 0.00000055
    # current kp = 0.1
    # current ki = 0.00000325
    # current kd = 0.00000260

    heading_kp = 0.04
    heading_ki = 0.000001625
    heading_kd = 0.00000740

    heading_initial_integral_value = 0
    heading_previous_time = time.perf_counter()
    heading_previous_error = 0

    # --------------Pitch PID Controller Variables
    # ku = 0.6
    # tu = 0.0001 ms
    # ki = 0.000072
    # kp = 0.0000045
    # current kp = 0.15
    # current ki = 0.000036
    # current kd = 0.000016

    pitch_kp = 0.15
    pitch_ki = 0.000036
    pitch_kd = 0.000016

    pitch_initial_integral_value = 0
    pitch_previous_time = time.perf_counter()
    pitch_previous_error = 0

    # ---------------Roll PID Controller Variables
    # 0.2312003271484372 - 0.3228308813476559 = tu = 0.01 / 1000 = 0.00001

    # ku = 0.1
    # tu = 0.09
    # ki = 0.0013333333333
    # kp =
    # current kp =
    # current ki =
    # current kd =

    roll_kp = 0.01
    roll_ki = 0.00033335
    roll_kd = 0.00000560

    roll_initial_integral_value = 0
    roll_previous_time = time.perf_counter()
    roll_previous_error = 0
    checker = True

    heading_Target = 90.0000  # target heading  aka my set point for pitch
    roll_Target = 90.0000  # target roll  aka my set point for roll

    degree_Target_for_10000m = 60
    degree_Target_for_20000m = 50
    degree_Target_for_40000m = 30
    degree_Target_for_75000m = 10

    height_threshold_1 = 10000
    height_threshold_2 = 20000
    height_threshold_3 = 40000
    height_threshold_4 = 70000



    max_integral = 10

    while checker:
        vessel_height = vessel.flight().mean_altitude
        heading_current_time = time.perf_counter()
        pitch_current_time = time.perf_counter()
        roll_current_time = time.perf_counter()
        pitch_target_degree = calculate_target_degree(vessel_height, height_threshold_1, degree_Target_for_10000m)

        if vessel_height < height_threshold_1:
            if vessel_height > 200:
                vessel.control.sas = False
                vessel.control.rcs = False


                pitch_previous_time, pitch_previous_error, pitch_initial_integral_value = pitch_pid(pitch_kp,
                                                                                                        pitch_ki,
                                                                                                        pitch_kd,
                                                                                                        pitch_target_degree,
                                                                                                        pitch_current_time,
                                                                                                        pitch_previous_time,
                                                                                                        pitch_previous_error,
                                                                                                        pitch_initial_integral_value,
                                                                                                        max_integral)

                if vessel.flight().pitch < 89:

                    heading_previous_time, heading_previous_error, heading_initial_integral_value = heading_pid(heading_kp,heading_ki,
                                                                                                                    heading_kd,
                                                                                                                    heading_Target,
                                                                                                                    heading_current_time,
                                                                                                                    heading_previous_time,
                                                                                                                    heading_previous_error,
                                                                                                        heading_initial_integral_value,
                                                                                                                    max_integral)
                    roll_previous_time, roll_previous_error, roll_initial_integral_value = roll_pid(roll_kp,
                                                                                                        roll_ki,
                                                                                                        roll_kd,
                                                                                                        roll_Target,
                                                                                                        roll_current_time,
                                                                                                        roll_previous_time,
                                                                                                        roll_previous_error,
                                                                                                        roll_initial_integral_value,
                                                                                                        max_integral)

        if height_threshold_1 <= vessel_height < height_threshold_2:
            vessel.control.rcs = True
            pitch_target_degree = smooth_transition(vessel_height, height_threshold_1, height_threshold_2,
                                                                                degree_Target_for_10000m,
                                                                                degree_Target_for_20000m)


            pitch_previous_time, pitch_previous_error, pitch_initial_integral_value = pitch_pid(pitch_kp, pitch_ki,
                                                                                                    pitch_kd,
                                                                                                    pitch_target_degree,
                                                                                                    pitch_current_time,
                                                                                                    pitch_previous_time,
                                                                                                    pitch_previous_error,
                                                                                                    pitch_initial_integral_value,
                                                                                                    max_integral)

            heading_previous_time, heading_previous_error, heading_initial_integral_value = heading_pid(heading_kp,
                                                                                                            heading_ki,
                                                                                                            heading_kd,
                                                                                                            heading_Target,
                                                                                                            heading_current_time,
                                                                                                            heading_previous_time,
                                                                                                            heading_previous_error,
                                                                                                            heading_initial_integral_value,
                                                                                                            max_integral)

            roll_previous_time, roll_previous_error, roll_initial_integral_value = roll_pid(roll_kp, roll_ki,
                                                                                                roll_kd,
                                                                                                roll_Target,
                                                                                                roll_current_time,
                                                                                                roll_previous_time,
                                                                                                roll_previous_error,
                                                                                                roll_initial_integral_value,
                                                                                                max_integral)
        if height_threshold_2 <= vessel_height < height_threshold_3:
            pitch_target_degree = smooth_transition(vessel_height, height_threshold_2, height_threshold_3,
                                                    degree_Target_for_20000m,
                                                    degree_Target_for_40000m)

            pitch_previous_time, pitch_previous_error, pitch_initial_integral_value = pitch_pid(pitch_kp, pitch_ki,
                                                                                                    pitch_kd,
                                                                                                    pitch_target_degree,
                                                                                                    pitch_current_time,
                                                                                                    pitch_previous_time,
                                                                                                    pitch_previous_error,
                                                                                                    pitch_initial_integral_value,
                                                                                                    max_integral)

            heading_previous_time, heading_previous_error, heading_initial_integral_value = heading_pid(heading_kp,
                                                                                                            heading_ki,
                                                                                                            heading_kd,
                                                                                                            heading_Target,
                                                                                                            heading_current_time,
                                                                                                            heading_previous_time,
                                                                                                            heading_previous_error,
                                                                                                            heading_initial_integral_value,
                                                                                                            max_integral)

            roll_previous_time, roll_previous_error, roll_initial_integral_value = roll_pid(roll_kp, roll_ki,
                                                                                                roll_kd,
                                                                                                roll_Target,
                                                                                                roll_current_time,
                                                                                                roll_previous_time,
                                                                                                roll_previous_error,
                                                                                                roll_initial_integral_value,
                                                                                                max_integral)
        if height_threshold_3 <= vessel_height < height_threshold_4:
            pitch_target_degree = smooth_transition(vessel_height, height_threshold_3, height_threshold_4,
                                                    degree_Target_for_40000m,
                                                    degree_Target_for_75000m)

            pitch_previous_time, pitch_previous_error, pitch_initial_integral_value = pitch_pid(pitch_kp, pitch_ki,
                                                                                                    pitch_kd,
                                                                                                    pitch_target_degree,
                                                                                                    pitch_current_time,
                                                                                                    pitch_previous_time,
                                                                                                    pitch_previous_error,
                                                                                                    pitch_initial_integral_value,
                                                                                                    max_integral)

            heading_previous_time, heading_previous_error, heading_initial_integral_value = heading_pid(heading_kp,
                                                                                                            heading_ki,
                                                                                                            heading_kd,
                                                                                                            heading_Target,
                                                                                                            heading_current_time,
                                                                                                            heading_previous_time,
                                                                                                            heading_previous_error,
                                                                                                            heading_initial_integral_value,
                                                                                                            max_integral)

            roll_previous_time, roll_previous_error, roll_initial_integral_value = roll_pid(roll_kp, roll_ki,
                                                                                                roll_kd,
                                                                                                roll_Target,
                                                                                                roll_current_time,
                                                                                                roll_previous_time,
                                                                                                roll_previous_error,
                                                                                                roll_initial_integral_value,
                                                                                                max_integral)
        if vessel.orbit.apoapsis_altitude >= 75000:
            vessel.control.throttle = 0

        if vessel.orbit.apoapsis_altitude >= 75000 and vessel_height >= 70000:
            checker = False
        time.sleep(0.01)
    return True

def orbital_insertion_burn_start():#
    print("test")




def coasting_phase():
    checker = False

    pitch_target_degree = 0
    heading_Target = 90.000
    roll_Target  = 90.000

    roll_kp = 0.01
    roll_ki = 0.000333335
    roll_kd = 0.00000040

    pitch_kp = 0.075
    pitch_ki = 0.000018
    pitch_kd = 0.000016

    heading_kp = 0.06
    heading_ki = 0.000001625
    heading_kd = 0.0000032

    heading_previous_time = time.perf_counter()
    roll_previous_time = time.perf_counter()
    pitch_previous_time = time.perf_counter()

    burn_complete = False



    pitch_previous_error = 0.0
    roll_previous_error = 0.0
    heading_previous_error = 0.0

    heading_initial_integral_value = 0.0
    pitch_initial_integral_value = 0.0
    roll_initial_integral_value = 0.0

    max_integral = 10


    while not checker:
        if vessel.flight().mean_altitude >= 70000:
            deltaV_required = orbital_insertion_burn_calculation()
            pitch_current_time = time.perf_counter()
            heading_current_time = time.perf_counter()
            roll_current_time = time.perf_counter()
            burn_Counter = time.perf_counter()

            acceleration = vessel.available_thrust / vessel.mass

            burn_Time = deltaV_required / acceleration

            half_burn_time = burn_Time / 2

            time_To_Apoapsis = vessel.orbit.time_to_apoapsis





            pitch_previous_time, pitch_previous_error, pitch_initial_integral_value = pitch_pid(pitch_kp, pitch_ki,
                                                                                                    pitch_kd,
                                                                                                    pitch_target_degree,
                                                                                                    pitch_current_time,
                                                                                                    pitch_previous_time,
                                                                                                    pitch_previous_error,
                                                                                                    pitch_initial_integral_value,
                                                                                                    max_integral)

            heading_previous_time, heading_previous_error, heading_initial_integral_value = heading_pid(heading_kp,
                                                                                                            heading_ki,
                                                                                                            heading_kd,
                                                                                                            heading_Target,
                                                                                                            heading_current_time,
                                                                                                            heading_previous_time,
                                                                                                            heading_previous_error,
                                                                                                            heading_initial_integral_value,
                                                                                                            max_integral)

            roll_previous_time, roll_previous_error, roll_initial_integral_value = roll_pid(roll_kp, roll_ki, roll_kd,
                                                                                                roll_Target,
                                                                                                roll_current_time,
                                                                                                roll_previous_time,
                                                                                                roll_previous_error,
                                                                                                roll_initial_integral_value,
                                                                                                max_integral)
            if time_To_Apoapsis > half_burn_time:
                pass
    return

# if time to apoapsis is smaller or equal to burn time
#   while perioapsis is smaller than apoapsis :
#       vessel thrust on

            #if time_To_Apoapsis > burn_Time or burn_Time <= 0:
             #   vessel.control.throttle = 0

            #if time_To_Apoapsis <= burn_Time:
             #   vessel.control.throttle = 1
              #  print(vessel.orbit.periapsis_altitude)







        # For demonstration, limit the loop duration
        #if len(timestamps) > 500000:
         #   checker = False

    # Plot the PID outputs
#    plt.figure(figsize=(10, 6))
#    timestamps = [t - timestamps[0] for t in timestamps]  # Normalize timestamps to start at 0
#    plt.plot(timestamps, pid_outputs, label="PID Output")
#    plt.xlabel("Time (s)")
#    plt.ylabel("PID Output")
#    plt.title("PID Output vs. Time")
#    plt.legend()
#    plt.grid(True)
#    plt.show()
#    time.sleep(0.01)


#--------Main Function-------
def main():
    sub_Orbital_Phase_complete = False
    sub_Orbital_Coasting_Phase = False

    print(sub_Orbital_Phase_complete, sub_Orbital_Coasting_Phase)

    while sub_Orbital_Phase_complete == False or sub_Orbital_Coasting_Phase == False:

        if not sub_Orbital_Phase_complete:
            sub_Orbital_Phase_complete = sub_orbital_phase()
            print(sub_Orbital_Phase_complete, sub_Orbital_Coasting_Phase)

        print(sub_Orbital_Phase_complete, sub_Orbital_Coasting_Phase)

        if sub_Orbital_Coasting_Phase == False and sub_Orbital_Phase_complete == True:
            sub_Orbital_Coasting_Phase = coasting_phase()
            print(sub_Orbital_Phase_complete, sub_Orbital_Coasting_Phase)





if __name__ == '__main__':
    main()