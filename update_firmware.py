import sys
import os
import serial
import serial_asyncio
import asyncio
import struct
import zlib
import subprocess
import time
import random

# Configuration
PORTS = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3",
         "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3"]
BAUDRATE_APPLICATION = 115200
BAUDRATE_BOOTLOADER = 460800
chunk_size = 256
kill_screen = True

debug_print = False
chaos_mode = False

cwd = os.getcwd()
FIRMWARE_FILE = cwd + "/" + sys.argv[1]  # Firmware file

# little-endian: seq (2 bytes), len (2 bytes), crc (4 bytes)
PACKET_HEADER_FORMAT = "<HHI"
PACKET_HEADER_SIZE = struct.calcsize(PACKET_HEADER_FORMAT)
PACKET_DATA_SIZE = 256


class Packet:
    def __init__(self, seq, data: bytes):
        self.seq = seq
        self.len = len(data)
        self.data = data
        head = struct.pack("<HH", self.seq, self.len)
        crc_input = head + data
        pad_len = (4 - (len(crc_input) % 4)) % 4
        padded = crc_input + b'\xFF' * pad_len
        self.crc = zlib.crc32(padded) & 0xFFFFFFFF

    def encode(self) -> bytes:
        if chaos_mode:
            data = maybe_corrupt(self.data)
        else:
            data = self.data
        header = struct.pack(PACKET_HEADER_FORMAT,
                             self.seq, self.len, self.crc)
        return header + data

    @classmethod
    def decode(cls, raw: bytes):
        if len(raw) < PACKET_HEADER_SIZE:
            raise ValueError("Header mismatch")
        seq, length, crc = struct.unpack(
            PACKET_HEADER_FORMAT, raw[:PACKET_HEADER_SIZE])
        data = raw[PACKET_HEADER_SIZE:PACKET_HEADER_SIZE + length]
        if chaos_mode:
            data = maybe_corrupt(data)
        if len(data) < length:
            raise ValueError("Length mismatch")
        crc_input = raw[0:2] + raw[2:4] + data
        pad_len = (4 - (len(crc_input) % 4)) % 4
        padded = crc_input + b'\xFF' * pad_len
        if (zlib.crc32(padded) & 0xFFFFFFFF) != crc and seq != 0:
            raise ValueError("CRC mismatch")
        return cls(seq, data)


class BTP(asyncio.Protocol):
    def __init__(self):
        self.buffer = bytearray()
        self.ser = None
        self.seq = 0
        self.firmware_data = read_binary()
        self.firmware_size = len(self.firmware_data)
        self.chunk_size = chunk_size
        self.chunk_index = 0
        self.start = time.perf_counter()

    def connection_made(self, ser):
        self.ser = ser

    def data_received(self, data):
        self.buffer.extend(data)
        while len(self.buffer) >= PACKET_HEADER_SIZE:
            # Peek header to check if we have enough for full message
            seq, length, crc = struct.unpack(
                PACKET_HEADER_FORMAT, self.buffer[:PACKET_HEADER_SIZE])
            total_len = PACKET_HEADER_SIZE + length

            if length > PACKET_DATA_SIZE:
                self.buffer.clear()
                continue

            if len(self.buffer) < total_len:
                return  # wait for more data

            raw_packet = self.buffer[:total_len]
            self.buffer = self.buffer[total_len:]

            try:
                packet = Packet.decode(raw_packet)
                if debug_print:
                    print(f"Received msg: seq={packet.seq}, len={
                          packet.len}, data={packet.data}")
                self.respond(packet)
            except ValueError as e:
                print(f"Error decoding message: {e}")

    def respond(self, packet):

        if packet.seq == 0:
            print(packet.data.decode('ascii'), end="")

        elif packet.seq == 1 and packet.data == b"update":
            print("Connection established, sending handshake..")
            self.send_packet(2, b"ACK")

        elif packet.seq == 3 and packet.data == b"ACK":
            print("Handshake completed, erasing old flash and sending size..")
            payload = struct.pack("<I", self.firmware_size)
            self.send_packet(4, payload)

        elif packet.seq == 5 and packet.data == b"ACK":
            print("Size verified, writing firmware..")
            chunk = self.firmware_data[:min(self.firmware_size, chunk_size)]
            self.send_packet(100, chunk)

        elif 100 <= packet.seq <= (99 + int(self.firmware_size / chunk_size)) and packet.data == b"ACK":

            offset = packet.seq - 99

            remaining_bytes = self.firmware_size - offset

            if remaining_bytes > 0:
                chunk_size_to_send = min(chunk_size, remaining_bytes)
                chunk = self.firmware_data[offset *
                                           chunk_size:(offset * chunk_size) + chunk_size_to_send]

                self.send_packet(packet.seq + 1, bytearray(chunk))
                print(f"{(packet.seq-99)*chunk_size + len(chunk)
                         } / {len(self.firmware_data)
                              } bytes written..", end="\r")

        elif packet.seq == (100 + int(self.firmware_size /
                                      chunk_size)) and packet.data == b"ACK":
            print("\nFlash successfully written!")
            pad_len = (4 - (len(self.firmware_data) % 4)) % 4
            padded = self.firmware_data + b'\xFF' * pad_len
            payload = struct.pack("<I", zlib.crc32(padded) & 0xFFFFFFFF)
            self.send_packet(65534, payload)

        elif packet.seq == 65535 and packet.data == b"ACK":
            print(f"Update took {
                  time.perf_counter() - self.start:.2f} seconds")
            exit(1)

        else:
            print("No match, ask for re-transfer")
            print(str(packet.data))

    def send_packet(self, seq, data: bytes):
        packet = Packet(seq, data)
        encoded = packet.encode()
        if debug_print:
            print(f"Sending: seq={packet.seq}, len={
                  packet.len}, data={packet.data}")
        self.ser.write(encoded)


def maybe_corrupt(data):
    if random.random() < 0.1:  # 10% chance to corrupt
        data = bytearray(data)
        data[random.randint(0, len(data)-1)] ^= 0xFF
    return data


def kill_screen_on_uart(port):
    try:
        subprocess.run(["pkill", "-f", f"SCREEN {port}"], check=True)
        if debug_print:
            print(f"Killed screen session on {port}")
    except subprocess.CalledProcessError:
        if debug_print:
            print(f"No screen session found on {port}")


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


async def main(port):

    port = get_port()
    loop = asyncio.get_running_loop()
    await serial_asyncio.create_serial_connection(
        loop, BTP, port, baudrate=BAUDRATE_BOOTLOADER
    )
    await asyncio.sleep(60)


def send_firmware():

    port = get_port()

    with serial.Serial(port, BAUDRATE_APPLICATION) as ser:
        ser.write(b"\rflash\r")

    print()
    print("///////////////////////////////////////////////////")
    print(f"// Firmware file: {sys.argv[1]}")
    print(f"// Firmware size: {len(read_binary())} bytes")
    print("///////////////////////////////////////////////////\n")

    asyncio.run(main(port))


if __name__ == "__main__":
    send_firmware()
