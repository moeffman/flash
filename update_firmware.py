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


def open_serial(baudrate):
    port_index = 0
    screen_killed = False
    while True:
        try:
            with serial.Serial(PORTS[port_index], baudrate) as ser:
                return ser
        except serial.SerialException as e:
            if e.errno == 16 and not screen_killed:
                screen_killed = True
                kill_screen_on_uart(PORTS[port_index])
            elif e.errno == 2 and port_index < len(PORTS)-1:
                port_index = port_index + 1
            else:
                if e.errno == 2:
                    print(
                        f"[Errno {e.errno}] could not open port: "
                        + "No such file or directory")
                else:
                    print(e)
                exit(1)


def send_firmware():

    try:
        with open(FIRMWARE_FILE, "rb") as f:
            firmware_data = f.read()
    except FileNotFoundError:
        print("Error: Firmware file not found!")
        sys.exit(1)

    state = State.HANDSHAKE

    # TODO: Before actually sending the flash message and proceeding,
    # check if more than one device is connected, and let the user
    # choose what device to flash to

    # Sends a command to the application, sending it to bootloader
    with open_serial(BAUDRATE_APP) as ser:
        # TODO: Test different baud rates here before sending flash command
        # Maybe can just send the flash command with different bauds and
        # see if one responds
        ser.write(b"\rflash\r")

    # Attempting to flash
    with open_serial(BAUDRATE_BL) as ser:
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

        for i in range(1, 1000):
            ser.timeout = 0
            if not ser.readline():
                break

        while True:
            time.sleep(0.1)
            match state:
                case State.HANDSHAKE:
                    # TODO: Try to send "1234" with different baud rates
                    # If response is also "1234" baud is matching, can proceed
                    ser.write(b"U")
                    msg = read_message(ser, 0)
                    if msg and "ACK" in msg:
                        ser.write(b"A")
                        msg = read_message(ser, 3)
                        if msg and "ACK" in msg:
                            print("Handshake completed, sending size..")
                            state = State.SIZE

                case State.SIZE:
                    ser.write(bytearray(firmware_size_bytes))
                    msg = read_message(ser, len(str(len(firmware_data))))
                    if msg and str(len(firmware_data)) in msg:
                        state = State.FILE
                    elif msg and "FTL" in msg:
                        print("Firmware size too large, exiting.")
                        return
                    else:
                        time.sleep(0.1)

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
