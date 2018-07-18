#!/usr/bin/env python3

import os
import subprocess
import sys
import signal
import socket
import sys
import threading
import time

import RPi.GPIO as GPIO

def cleanup():
    command = "pkill wpantund"
    process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
    process.wait()

def check():
    command = "wpanctl get Thread:Leader:Address"
    process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
    out, err = process.communicate()
    errno = process.returncode
    #errno = process.wait()
    print(out)
    if errno != 0:
        print("error LeaderAddrGet")
        return False

    command = "wpanctl get NCP:State"
    process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
    out, err = process.communicate()
    errno = process.returncode
    print(out)
    if errno != 0:
        print("error getNCPState")
        return False

    if 'NCP:State = "associated"' in out:
        return True
    else:
        print("error currentState")
        return False

def run():
    command = "sudo wpantund -o Config:NCP:SocketPath /dev/ttyAMA0 -o Config:NCP:SocketBaud 115200"
    process = subprocess.Popen(command.split(), stderr=subprocess.PIPE)
    while True:
        line = process.stderr.readline()
        if b"Finished initializing NCP" in line:
            print("finished starting wpantund")
            break
        elif b"Resetting and trying again..." in line:
            print("failed starting wpantund")
            break

    command = "wpanctl set Network:XPANID DEAD00BEEF00CAFE"
    process = subprocess.Popen(command.split())
    process.wait()

    command = "wpanctl set Network:Key 00112233445566778899AABBCCDDEEFF"
    process = subprocess.Popen(command.split())
    process.wait()

    command = "wpanctl config-gateway -d fd11:22::"
    process = subprocess.Popen(command.split())
    process.wait()

    command = "wpanctl form -T router -c 26 OpenThreadDemo"
    process = subprocess.Popen(command.split())
    process.wait()


def signal_handler(signal, frame):
    print("You pressed Ctrl+C! Cleaning up")
    cleanup()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
last_hb_lock = threading.Lock()
last_hb = 0
GREEN_LED_PIN = 22
LED_BLINK_TOLERANCE_SECONDS = 1.6

def blink_led():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(GREEN_LED_PIN, GPIO.OUT, initial=GPIO.LOW)
    count = 0
    while True:
        time.sleep(1.0 / 16.0)
        count = (count + 1) % 16
        hb_time = None
        with last_hb_lock:
            hb_time = last_hb
        if time.time() - hb_time < LED_BLINK_TOLERANCE_SECONDS:
            GPIO.output(GREEN_LED_PIN, GPIO.HIGH)
        elif count < 8:
            GPIO.output(GREEN_LED_PIN, GPIO.LOW)
        else:
            GPIO.output(GREEN_LED_PIN, GPIO.HIGH)

wan_blink_lock = threading.Lock()
wan_blink = b"\x00"

def check_wan():
    global wan_blink
    while True:
        time.sleep(20)
        status = os.system("ping6 -c 1 ipv6.google.com")
        if status == 0:
            # IPv6 Internet connectivity
            with wan_blink_lock:
                wan_blink = b"\x02"
            continue
        status = os.system("ping -c 1 google.com")
        if status == 0:
            # IPv4 Internet connectivity
            with wan_blink_lock:
                wan_blink = b"\x05"
            continue
        # Check interface state
        with open("/sys/class/net/eth0/operstate") as f:
            status = f.read()
        if status == "up":
            # Interface is up, but no connectivity
            with wan_blink_lock:
                wan_blink = b"\x04"
            continue
        # Check if interface exists
        status = os.system("ip link show eth0")
        if status == 0:
            # Interface exists
            with wan_blink_lock:
                wan_blink = b"\x03"
            continue
        # None of these things worked
        with wan_blink_lock:
            wan_blink = b"\x01"


PACKET_TOLERANCE_SECONDS = 60

if __name__ == "__main__":
    blink_thread = threading.Thread(target = blink_led)
    blink_thread.daemon = True
    blink_thread.start()

    wan_thread = threading.Thread(target = check_wan)
    wan_thread.daemon = True
    wan_thread.start()

    while True:
        cleanup()
        run()
        s = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
        s.bind(("", 54321))
        s.settimeout(PACKET_TOLERANCE_SECONDS)
        while True:
            try:
                # Receive heartbeat from MCU
                data, _ = s.recvfrom(4096)
                if len(data) != 89:
                    print(time.time(), "Heartbeat has unexpected length")
                    break
                hb_time = time.time()
                with last_hb_lock:
                    last_hb = hb_time
                print(hb_time, "Received heartbeat from MCU")

                # Send heartbeat to MCU
                with wan_blink_lock:
                    blink_state = wan_blink
                s.sendto(blink_state, ("fdde:ad00:beef:0:e9f0:45bc:c507:6f0b", 12345))
            except socket.timeout:
                print("Timed out waiting for heartbeat")
        s.close()
