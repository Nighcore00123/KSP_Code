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
# noinspection DuplicatedCode
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
# noinspection DuplicatedCode
def pitch_pid (kp, ki, kd, pitch_target, current_time, previous_time, previous_error, initial_integral_value, max_integral):
    passed_time = current_time - previous_time
    pitch_error = pitch_target - vessel.flight().pitch

    proportional_calculation = kp * pitch_error
    if pitch_error == 0.0000:
        initial_integral_value = 0
    else:
        initial_integral_value += pitch_error * passed_time

    initial_integral_value = max(-max_integral, min(max_integral, initial_integral_value))
    integral_calculation = ki * initial_integral_value
    if passed_time > 1:
        derivative_calculation = kd * (pitch_error - previous_error) / passed_time
    else:
        derivative_calculation = 0

    PID_output = proportional_calculation + integral_calculation + derivative_calculation
    pitch_output = max(-1, min(1, PID_output))


    vessel.control.yaw = -pitch_output

    return current_time, pitch_error, initial_integral_value
# noinspection DuplicatedCode
def roll_pid(kp, ki, kd, roll_target, current_time, previous_time, previous_error, initial_integral_value, max_integral):
    passed_time = current_time - previous_time
    roll_error = roll_target - vessel.flight().roll


    proportional_calculation = kp * roll_error
    if roll_error == 0.0000:
        initial_integral_value = 0
    else:
        initial_integral_value += roll_error * passed_time


    initial_integral_value = max(-max_integral, min(max_integral, initial_integral_value))
    integral_calculation = ki * initial_integral_value

    derivative_calculation = kd * (roll_error - previous_error) / passed_time

    PID_output = proportional_calculation + integral_calculation + derivative_calculation

    roll_output = max(-1, min(1, PID_output))

    vessel.control.roll = roll_output

    return current_time, roll_error, initial_integral_value
# noinspection DuplicatedCode
def heading_pid(kp, ki, kd, heading_target, current_time, previous_time, previous_error, initial_integral_value, max_integral):
    passed_time = current_time - previous_time
    heading_error = heading_target - vessel.flight().heading

    proportional_Calculation = kp * heading_error

    if heading_error == 0.0000:
        initial_integral_value = 0
    else:
            initial_integral_value += heading_error * passed_time

    initial_integral_value = max(-max_integral, min(max_integral, initial_integral_value))
    integral_calculation = ki * initial_integral_value

    if passed_time > 2:
        derivative_calculation = kd * (heading_error - previous_error) / passed_time
    else:
        derivative_calculation = 0

    PID_output = proportional_Calculation + integral_calculation + derivative_calculation
    yaw_output = max(-1, min(1, PID_output))

    print("PID Output: ", PID_output, " Proportional Calculations: ", proportional_Calculation, "Integral Calculations: ", integral_calculation, "Derivative Calculations: ", derivative_calculation)

    vessel.control.pitch = yaw_output


    return current_time, heading_error, initial_integral_value

def  orbital_insertion_burn_calculation():
    gravitational_constant_kerbin = 6.67 * 10 ** -11
    kerbin_radius = 6.0 * 10 ** 5
    kerbin_Mass = 5.3 * 10 ** 22

    total_Radius = kerbin_radius + vessel.orbit.apoapsis_altitude

    deltaV_Requirement = (gravitational_constant_kerbin * kerbin_Mass) / total_Radius

    deltaV_Requirement = math.sqrt(deltaV_Requirement)

    deltaV_Requirement = deltaV_Requirement - vessel.orbit.speed

    return deltaV_Requirement

def sub_orbital_phase(roll_previous_error, heading_previous_error, pitch_previous_error):
    # ----------------Heading PID Controller Variables------------
    # 0.17092437744140623 - 0.1833892822265625 = tu = 0.01
    # ku = 0.6
    # tu = 0.00001 ms
    # kp = 0.3
    # ki = 0.0000072
    # kd = 0.00000055
    # current kp = 0.03
    # current ki = 0.0000075
    # current kd = 0.00000888

    heading_kp = 0.01
    heading_ki = 0.000000005
    heading_kd = 0.0000055

    heading_initial_integral_value = 0



    # --------------Pitch PID Controller Variables
    # ku = 0.6
    # tu = 0.0001 ms
    # ki = 0.000072
    # kp = 0.0000045
    # current kp = 0.15
    # current ki = 0.000036
    # current kd = 0.000016

    pitch_kp = 0.10
    pitch_ki = 0.0000036
    pitch_kd = 0.000088

    pitch_initial_integral_value = 0



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

    checker = True

    heading_Target = 90.000# target heading  aka my set point for pitch
    roll_Target = 90.000  # target roll  aka my set point for roll

    degree_Target_for_10000m = 50
    degree_Target_for_20000m = 40
    degree_Target_for_40000m = 20
    degree_Target_for_75000m = 0

    height_threshold_1 = 10000
    height_threshold_2 = 20000
    height_threshold_3 = 40000
    height_threshold_4 = 70000

    max_integral = 0.05

    previous_vessel_thrust = vessel.available_thrust / vessel.mass

    while checker:
        vessel_height = vessel.flight().mean_altitude
        roll_current_time = time.perf_counter()
        pitch_target_degree = calculate_target_degree(vessel_height, height_threshold_1, degree_Target_for_10000m)

        current_vessel_thrust = vessel.available_thrust / vessel.mass

        if current_vessel_thrust < previous_vessel_thrust:
            vessel.control.activate_next_stage()
            previous_vessel_thrust = current_vessel_thrust


        if vessel_height < height_threshold_1:
            if vessel_height > 200:
                vessel.control.sas = False
                vessel.control.rcs = False


                pitch_previous_time, pitch_previous_error, pitch_initial_integral_value = pitch_pid(pitch_kp,
                                                                                                        pitch_ki,
                                                                                                        pitch_kd,
                                                                                                        pitch_target_degree,
                                                                                                        roll_current_time,
                                                                                                        roll_previous_time,
                                                                                                        pitch_previous_error,
                                                                                                        pitch_initial_integral_value,
                                                                                                        max_integral)

                if vessel.flight().pitch < 89:

                    heading_previous_time, heading_previous_error, heading_initial_integral_value = heading_pid(heading_kp,heading_ki,
                                                                                                                    heading_kd,
                                                                                                                    heading_Target,
                                                                                                                    roll_current_time,
                                                                                                                    roll_previous_time,
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
            # noinspection DuplicatedCode#
            pitch_target_degree = smooth_transition(vessel_height, height_threshold_1, height_threshold_2,
                                                    degree_Target_for_10000m,
                                                    degree_Target_for_20000m)

            pitch_previous_time, pitch_previous_error, pitch_initial_integral_value = pitch_pid(pitch_kp, pitch_ki,
                                                                                                    pitch_kd,
                                                                                                    pitch_target_degree,
                                                                                                roll_current_time,
                                                                                                roll_previous_time,
                                                                                                    pitch_previous_error,
                                                                                                    pitch_initial_integral_value,
                                                                                                    max_integral)

            heading_previous_time, heading_previous_error, heading_initial_integral_value = heading_pid(heading_kp,
                                                                                                            heading_ki,
                                                                                                            heading_kd,
                                                                                                            heading_Target,
                                                                                                        roll_current_time,
                                                                                                        roll_previous_time,
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
        # noinspection DuplicatedCode
        if height_threshold_2 <= vessel_height < height_threshold_3:
            pitch_target_degree = smooth_transition(vessel_height, height_threshold_2, height_threshold_3,
                                                    degree_Target_for_20000m,
                                                    degree_Target_for_40000m)

            pitch_previous_time, pitch_previous_error, pitch_initial_integral_value = pitch_pid(pitch_kp, pitch_ki,
                                                                                                    pitch_kd,
                                                                                                    pitch_target_degree,
                                                                                                roll_current_time,
                                                                                                roll_previous_time,
                                                                                                    pitch_previous_error,
                                                                                                    pitch_initial_integral_value,
                                                                                                    max_integral)

            heading_previous_time, heading_previous_error, heading_initial_integral_value = heading_pid(heading_kp,
                                                                                                            heading_ki,
                                                                                                            heading_kd,
                                                                                                            heading_Target,
                                                                                                        roll_current_time,
                                                                                                        roll_previous_time,
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
        # noinspection DuplicatedCode
        if height_threshold_3 <= vessel_height < height_threshold_4:
            pitch_target_degree = smooth_transition(vessel_height, height_threshold_3, height_threshold_4,
                                                    degree_Target_for_40000m,
                                                    degree_Target_for_75000m)

            pitch_previous_time, pitch_previous_error, pitch_initial_integral_value = pitch_pid(pitch_kp, pitch_ki,
                                                                                                    pitch_kd,
                                                                                                    pitch_target_degree,
                                                                                                roll_current_time,
                                                                                                roll_previous_time,
                                                                                                    pitch_previous_error,
                                                                                                    pitch_initial_integral_value,
                                                                                                    max_integral)

            heading_previous_time, heading_previous_error, heading_initial_integral_value = heading_pid(heading_kp,
                                                                                                            heading_ki,
                                                                                                            heading_kd,
                                                                                                            heading_Target,
                                                                                                        roll_current_time,
                                                                                                        roll_previous_time,
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

        if vessel.orbit.apoapsis_altitude < 75000:
            vessel.control.throttle = 1

        if vessel.orbit.apoapsis_altitude >= 75000 and vessel_height >= 70000:
            checker = False
    return True, roll_previous_error, heading_previous_error, pitch_previous_error

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


            acceleration = vessel.available_thrust / vessel.mass

            burn_Time = deltaV_required / acceleration

            half_burn_time = burn_Time / 2

            time_To_Apoapsis = vessel.orbit.time_to_apoapsis

            # noinspection DuplicatedCode
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
            if time_To_Apoapsis < half_burn_time:
                checker = True
    return True




#--------Main Function-------
def main():
    launch_Timer_start = time.perf_counter()

    launch_sequence_complete = False
    sub_Orbital_Phase_complete = False
    sub_Orbital_Coasting_Phase = False

    roll_previous_error = 0
    heading_previous_error = 0
    pitch_previous_error = 0

    print(sub_Orbital_Phase_complete, sub_Orbital_Coasting_Phase)

    time_counter = 5

    while not launch_sequence_complete:
        launch_timer_end = time.perf_counter()

        if launch_timer_end - launch_Timer_start >= 1 and time_counter != 0:
            time_counter -= 1
            print("Launching in: ", time_counter)

            launch_Timer_start = time.perf_counter()

        if time_counter == 0:
            vessel.control.throttle = 1
            time.sleep(0.5)
            vessel.control.activate_next_stage()
            vessel.control.sas = True
            vessel.control.rcs = True
            time.sleep(0.001)
            launch_sequence_complete = True

    while sub_Orbital_Phase_complete == False or sub_Orbital_Coasting_Phase == False and launch_sequence_complete == True:

        if not sub_Orbital_Phase_complete:
            sub_Orbital_Phase_complete, roll_previous_error, heading_previous_error, pitch_previous_error = sub_orbital_phase(roll_previous_error, heading_previous_error, pitch_previous_error)
            print(sub_Orbital_Phase_complete, sub_Orbital_Coasting_Phase)


        if sub_Orbital_Coasting_Phase == False and sub_Orbital_Phase_complete == True:
            sub_Orbital_Coasting_Phase = coasting_phase()
            print(sub_Orbital_Phase_complete, sub_Orbital_Coasting_Phase)



if __name__ == '__main__':
    main()

    # if time to apoapsis is smaller or equal to burn time
    #   while perioapsis is smaller than apoapsis :
    #       vessel thrust on

    # if time_To_Apoapsis > burn_Time or burn_Time <= 0:
    #   vessel.control.throttle = 0

    # if time_To_Apoapsis <= burn_Time:
    #   vessel.control.throttle = 1
    #  print(vessel.orbit.periapsis_altitude)

    # For demonstration, limit the loop duration
    # if len(timestamps) > 500000:
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