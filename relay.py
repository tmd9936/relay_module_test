from ctypes import CDLL, c_int, POINTER
import os, time


if __name__ == "__main__":
    so_path = os.path.abspath('./')
    print(so_path)
    relay_lib = CDLL(so_path + "/usb_relay_device.so")

    
    init = relay_lib.usb_relay_init()
    device_enum = relay_lib.usb_relay_device_enumerate()
    print(device_enum)
    device = relay_lib.usb_relay_device_open(device_enum)

    channel = None

    while True:
        s = input("입력: ")
        if s == "u":
            print("up")
            t = input("시간: ")
            channel = relay_lib.usb_relay_device_close_one_relay_channel(device, c_int(2))
            channel = relay_lib.usb_relay_device_open_one_relay_channel(device, c_int(1))
            time.sleep(int(t))
            channel = relay_lib.usb_relay_device_close_one_relay_channel(device, c_int(1))
        elif s == "d":
            print("down")
            t = input("시간: ")
            channel = relay_lib.usb_relay_device_close_one_relay_channel(device, c_int(1))
            channel = relay_lib.usb_relay_device_open_one_relay_channel(device, c_int(2))
            time.sleep(int(t))
            channel = relay_lib.usb_relay_device_close_one_relay_channel(device, c_int(2))
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