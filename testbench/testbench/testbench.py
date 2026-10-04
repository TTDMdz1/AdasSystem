import socket
import struct
import time

CPP_HOST = "127.0.0.1"
CPP_PORT = 9000
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0.1)

print("=" * 60)
print("  AUTOMATED SIL TESTBENCH (PYTHON -> C++ CORE)")
print("  Scenario: Lead Vehicle Emergency Braking")
print("=" * 60)

SENSOR_FMT = "<IBff"
ACTUATOR_FMT = "<IBff"

distance = 50.0        
relative_speed = 0.0   
sim_time = 0.0
dt = 0.02              

aeb_triggered = False

for step in range(250): 
    sim_time += dt

    if sim_time >= 1.5 and not aeb_triggered:
        relative_speed = -18.0 
    
    distance += relative_speed * dt
    if distance < 0:
        distance = 0.0

    tx_data = struct.pack(SENSOR_FMT, 0x110, 8, distance, relative_speed)
    sock.sendto(tx_data, (CPP_HOST, CPP_PORT))

    try:
        rx_data, _ = sock.recvfrom(1024)
        can_id, state, throttle, brake = struct.unpack(ACTUATOR_FMT, rx_data)
        
        state_str = ["STANDBY", "ACC_ACTIVE", "AEB_TRIGGERED", "FAULT_SAFE"][state]
        
        print(f"[{sim_time:4.2f}s] Dist: {distance:5.1f}m | RelSpd: {relative_speed:5.1f}m/s "
              f"| C++ State: {state_str:<13} | Brake: {brake:5.1f}%")

        if state == 2: 
            aeb_triggered = True

        if brake > 0.0:
            decel = (brake / 100.0) * 10.0 
            relative_speed += decel * dt
            if relative_speed > 0.0 and distance < 50.0:
                relative_speed = 0.0

    except socket.timeout:
        print(f"[{sim_time:4.2f}s] Timeout: No response received from C++!")

    time.sleep(dt)

print("-" * 60)
if aeb_triggered and distance > 0.0:
    print(">> TEST RESULT: [PASSED] - Thuat toan C++ kich hoat AEB kip thoi!")
else:
    print(">> TEST RESULT: [FAILED] - Xay ra va cham hoac khong phanh!")
print("-" * 60)