import time
import krpc
import math

conn = krpc.connect()
vessel = conn.space_center.active_vessel
body = conn.space_center.bodies[vessel.orbit.body.name]