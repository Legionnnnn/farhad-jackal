import serial
import time
import sys
import struct
import threading

def open_serial(port='/dev/ttyACM0', baudrate=9600, timeout=1):
    try:
        ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)  # Attendre l'initialisation de l'Arduino
        print("Serial port opened successfully.")
        return ser
    except serial.SerialException as e:
        print(f"Connection error on Serial port: {e}")
        return None

def send_command(ser, command):
    if ser and ser.is_open:
        ser.write((command + '\n').encode())  # Ajouter '\r' pour correspondre au protocole Arduino
        print(f"Sent: {command}")


def send_command_binary(ser, command):
    "double command"
    if ser and ser.is_open:
        command_bytes = struct.pack('<d', command)
        ser.write(command_bytes)
        print(f"Sent binary command: {command}")


def read_response(ser):
    if ser and ser.is_open:
        response = ser.readline().decode().strip()
        if response:
            pass
            print(f"Received: {response}")
        return int(response)
    return None

def read_response_binary(ser, end_char=b'\n'):
    if ser and ser.is_open:
        response = bytearray()
        while True:
            byte = ser.read(1)
            if not byte:
                break  # Timeout or no data
            if byte == end_char:
                break
            response += byte
        if response:
            if len(response) == 8:
                value = struct.unpack('<d', response)[0]
                print(f"Received binary response: {value}")
                return value
            else:
                print("Invalid binary response length.")
                return None
        else:
            print("No binary response received.")
    return None

def reader_thread(ser):
    while True:
        response = read_response_binary(ser)
        if response is not None:
            pass
            # print(f"Thread received: {response}")
        time.sleep(0.1)  # Adjust sleep time as needed

def main():

    ser = open_serial(port = f'/dev/ttyUSB0', baudrate=57600, timeout=1)
    if not ser:
        return
    
    # Thread
    thread = threading.Thread(target=reader_thread, args=(ser,))
    thread.daemon = True  # Daemonize thread
    thread.start()

    try :
        dt = 0.1  # Intervalle de temps en secondes
        i = 0
        while True:
            
            #send_command(ser, str(i))  # Send a numeric command

            #send_command_binary(ser, 200) #  Header

            send_command_binary(ser, i+0.5)
            #send_command(ser, ",")
            send_command_binary(ser, 3.14)  # Send a float command

            send_command(ser, "\n")  # Send a newline to indicate end of command

            # read_response_binary(ser)  # Read a binary response
            i+=1
            if i > 100:
                i = 0
            
            time.sleep(dt)
    
    except KeyboardInterrupt :
        print("Keyboard interrupt received. Exiting...")

    finally :
        print("End of program.")

        if ser:
            ser.close()
            print("Port série fermé.")
        


if __name__ == "__main__":
    main()