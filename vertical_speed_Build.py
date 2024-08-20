import krpc
import time


conn = krpc.connect()

vessel = conn.space_center.active_vessel

flight = vessel.flight(vessel.orbit.body.reference_frame)

temp_alt = vessel.flight().surface_altitude

start_time = time.time()

while True:
    current_time = time.time()
    elapsed_time = current_time - start_time

    if(elapsed_time >= 2):
        print(elapsed_time)
        start_time = current_time

