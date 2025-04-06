import sys
import os
import serial
import struct
import zlib
import time
import subprocess
import enum

# Configuration
PORTS = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3",
         "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3"]
BAUDRATE_APP = 115200
BAUDRATE_BL = 460800
BAUDRATES = [115200, 230400, 460800, 921600]
debug_print = False
chunk_size = 512
kill_screen = True

cwd = os.getcwd()
FIRMWARE_FILE = cwd + "/" + sys.argv[1]  # Firmware file


class State(enum.Enum):
    HANDSHAKE = 1
    SIZE = 2
    FILE = 3
    CRC = 4
    FINISHED = 5


def kill_screen_on_uart(port):
    try:
        subprocess.run(["pkill", "-f", f"SCREEN {port}"], check=True)
        if debug_print:
            print(f"Killed screen session on {port}")
    except subprocess.CalledProcessError:
        if debug_print:
            print(f"No screen session found on {port}")


def read_message(ser, size, blocking=False):
    try:
        if blocking:
            ser.timeout = None
        else:
            ser.timeout = 0.2
        if size == 0:
            response = ser.readline()
        else:
            response = ser.readline(size)
        if response:
            decoded = response.decode(errors="ignore").strip()
            if debug_print:
                print(f"\nSTM32: {decoded}\n")
            return decoded
    except serial.Timeout:
        ser.timeout = 0.2
        return None
    ser.timeout = 0.2
    return None


def open_serial(port, baudrate=115200):

    while True:
        try:
            with serial.Serial(port, baudrate) as ser:
                return ser
        except serial.SerialException as e:
            print(e)
            exit(1)


def get_port():
    port_index = 0
    screen_killed = {}
    for port in PORTS:
        screen_killed[port] = False

    available_ports = []
    port = None

    while True:
        try:
            with serial.Serial(PORTS[port_index]):
                available_ports.append(PORTS[port_index])
                port = PORTS[port_index]
                port_index += 1
        except serial.SerialException as e:
            if e.errno == 16 and not screen_killed[PORTS[port_index]]:
                screen_killed[PORTS[port_index]] = True
                kill_screen_on_uart(PORTS[port_index])
            elif e.errno == 2 and port_index < len(PORTS)-1:
                port_index += 1
            elif e.errno == 2 and len(available_ports) == 0:
                print(f"[Errno {e.errno}] could not find a valid port")
                exit(1)

        if port_index == len(PORTS)-1:
            break

    # TODO: Allow for multiple units to get flashed?
    if len(available_ports) > 1:
        print("Multiple connections found:")
        for i in range(1, len(available_ports)+1):
            print(f"{i}: {available_ports[i-1]}")

        ans = None
        while ans is None:
            ans = input("\nWhat port do you want to use?\n> ")
            if ans.isdigit() and 1 <= int(ans) <= len(available_ports):
                port = available_ports[int(ans)-1]
            else:
                ans = None
                print("\nInvalid choice, these connections where found:\n")
                for i in range(1, len(available_ports)+1):
                    print(f"{i}: {available_ports[i-1]}")

    return port


def read_binary():
    try:
        with open(FIRMWARE_FILE, "rb") as f:
            return f.read()
    except FileNotFoundError:
        print("Error: Firmware file not found!")
        sys.exit(1)


def send_firmware():

    port = get_port()

    state = State.HANDSHAKE
    firmware_data = read_binary()

    with open_serial(port, BAUDRATE_APP) as ser:

        # TODO: Test different baud rates here before sending flash command
        # Maybe can just send the flash command with different bauds and
        # see if one responds

        # Sending a return to clear any commands not parsed by the application
        ser.write(b"\r")
        time.sleep(0.1)
        ser.reset_input_buffer()
        # Sends a flash command to the application,
        # making it jump to bootloader
        ser.write(b"flash\r")

    # Attempting to flash
    with open_serial(port, BAUDRATE_BL) as ser:
        print()
        print("///////////////////////////////////////////////////")
        print(f"// Firmware file: {sys.argv[1]}")
        print(f"// Firmware size: {len(firmware_data)} bytes")
        print("///////////////////////////////////////////////////\n")
        print("Restart device to initiate update")
        print("Waiting for handshake..")
        ser.timeout = 0.1

        firmware_size_bytes = struct.pack(">I", len(firmware_data))

        crc = zlib.crc32(firmware_data) & 0xFFFFFFFF
        crc_bytes = struct.pack(">I", crc)

        ser.reset_input_buffer()

        while True:
            time.sleep(0.1)
            match state:
                case State.HANDSHAKE:
                    # TODO: Try to send "1234" with different baud rates
                    # If response is also "1234" baud is matching, can proceed
                    ser.write(b"U")
                    ser.reset_input_buffer()
                    msg = read_message(ser, 0)
                    if msg and "ACK" in msg:
                        ser.write(b"A")
                        ser.reset_input_buffer()
                        msg = read_message(ser, 3)
                        if msg and "ACK" in msg:
                            print("Handshake completed, sending size..")
                            state = State.SIZE

                case State.SIZE:
                    ser.write(bytearray(firmware_size_bytes))
                    ser.reset_input_buffer()
                    msg = read_message(ser, len(str(len(firmware_data))))
                    if msg and str(len(firmware_data)) in msg:
                        state = State.FILE
                    elif msg and "FTL" in msg:
                        print("Firmware size too large, exiting.")
                        return
                    else:
                        time.sleep(0.1)

                # TODO: Require ACK for each chunk
                # make it retry after a small delay
                #
                # bootloader also needs to reset if no data is received
                # within an acceptable time frame

                case State.FILE:
                    for i in range(0, len(firmware_data), chunk_size):
                        chunk = firmware_data[i: i + chunk_size]
                        ser.write(bytearray(chunk))
                        print(f"Sent {i + len(chunk)
                                      } / {len(firmware_data)
                                           } bytes", end="\r")
                    msg = read_message(ser, 3)
                    if msg and "ACK" in msg:
                        print()
                        print("Binary transfer completed!")
                        state = State.CRC

                case State.CRC:
                    ser.write(bytearray(crc_bytes))
                    msg = read_message(ser, len(bytearray(crc)))
                    if msg and str(crc) in msg:
                        print("CRC transfer completed!")
                        state = State.FINISHED

                case State.FINISHED:
                    msg = read_message(ser, 0)
                    if msg:
                        if "ACK" in msg:
                            break
                        print(msg)

        if debug_print:
            while True:
                msg = read_message(ser, 0)


if __name__ == "__main__":
    send_firmware()
