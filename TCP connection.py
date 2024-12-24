import time
import socket
import math  # Import math module for pi constant

def send_to_robot(angles):
    SERVER_IP = "192.168.1.159"
    SERVER_PORT = 5001
    
    # Convert angles from radians to degrees
    angles_degrees = [angle * (180 / math.pi) for angle in angles]
    for angles in angles_degrees:
        print(angles)
    t1, t2, t3, t4, t5, t6 = angles_degrees
    MESSAGE = f"set_angles({t1}, {t2}, {t3}, {t4}, {t5}, {t6}, 300)"
    print(MESSAGE)
    send_tcp_packet(SERVER_IP, SERVER_PORT, MESSAGE)

def send_tcp_packet(server_ip, server_port, message):
    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((server_ip, server_port))
        print(f"Connected to {server_ip}:{server_port}")
        client_socket.sendall(message.encode('utf-8'))
        print(f"Sent: {message}")
        response = client_socket.recv(1024).decode('utf-8')
        print(f"Received: {response}")
    except socket.error as e:
        print(f"Error: {e}")
    finally:
        client_socket.close()
        print("Connection closed.")

def send_angle_array(angle_array, wait_time):
    for angles in angle_array:
        send_to_robot(angles)
        time.sleep(wait_time)

# Example usage with angles in radians
ikForPath = [
[1.300674, -2.603063, -1.460636, -0.678099, 1.561670, -1.869992],
[1.291044, -2.568303, -1.546043, -0.627363, 1.561387, -1.879618],
[1.280747, -2.536329, -1.627090, -0.578192, 1.561086, -1.889911],
[1.269717, -2.506930, -1.704320, -0.530252, 1.560764, -1.900937],
[1.257880, -2.479972, -1.778142, -0.483268, 1.560420, -1.912770],
[1.245150, -2.455374, -1.848865, -0.437008, 1.560052, -1.925495],
[1.284117, -2.440552, -1.892967, -0.408125, 1.561184, -1.886542],
[1.324448, -2.427265, -1.934102, -0.380640, 1.562371, -1.846227],
[1.313472, -2.406661, -2.002874, -0.332378, 1.562047, -1.857199],
[1.301538, -2.388655, -2.069216, -0.283936, 1.561695, -1.869128],
[1.258229, -2.400101, -2.026962, -0.314323, 1.560430, -1.912421],
[1.216625, -2.413175, -1.981870, -0.345884, 1.559233, -1.954010],
[1.176734, -2.427924, -1.933957, -0.378564, 1.558104, -1.993888],
[1.138543, -2.444390, -1.883217, -0.412333, 1.557042, -2.032067],
[1.120289, -2.424909, -1.944637, -0.370139, 1.556541, -2.050317]
]

send_angle_array(ikForPath, 2)  # Send angles with a 2-second wait between each set
