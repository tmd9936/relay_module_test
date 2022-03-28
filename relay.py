from ctypes import CDLL, c_int, POINTER
import os, time, sys

def resource_path(relative_path):
    """ Get absolute path to resource, works for dev and for PyInstaller """
    base_path = getattr(sys, '_MEIPASS', os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base_path, relative_path)

if __name__ == "__main__":
    relay_lib = CDLL(resource_path("usb_relay_device.so"))

    init = relay_lib.usb_relay_init()
    device_enum = relay_lib.usb_relay_device_enumerate()
    print(type(device_enum))
    device = relay_lib.usb_relay_device_open(device_enum)

    print(device)

    channel = None

    while True:
        s = input("입력: ")
        if s == "u":
            print("up")
            t = input("시간: ")
            channel = relay_lib.usb_relay_device_close_one_relay_channel(device, c_int(2))
            print(channel)
            channel = relay_lib.usb_relay_device_open_one_relay_channel(device, c_int(1))
            print(channel)

            # time.sleep(int(t))
            # channel = relay_lib.usb_relay_device_close_one_relay_channel(device, c_int(1))
        elif s == "d":
            print("down")
            t = input("시간: ")
            channel = relay_lib.usb_relay_device_close_one_relay_channel(device, c_int(1))
            channel = relay_lib.usb_relay_device_open_one_relay_channel(device, c_int(2))
            # time.sleep(int(t))
            # channel = relay_lib.usb_relay_device_close_one_relay_channel(device, c_int(2))
        elif s == "us":
            print("up stop")
            channel = relay_lib.usb_relay_device_close_one_relay_channel(device, c_int(1))
        elif s == "ds":
            print("down stop")
            channel = relay_lib.usb_relay_device_close_one_relay_channel(device, c_int(2))
        elif s == "e":
            break


    device_close = relay_lib.usb_relay_device_close(device_enum)
    realy_exit = relay_lib.usb_relay_exit()