# relay_module_test

참고: https://github.com/pavel-a/usb-relay-hid

$ git clone  https://github.com/pavel-a/usb-relay-hid

$ cd usb-relay-hid-master/commandline/makemake

$ make

take usb_relay_device.so and copy to libs/

USB authorization setting

$ sudo gedit /etc/udev/rules.d/50-myusb.rules

```
SUBSYSTEMS=="usb", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="05df", GROUP="users", MODE="0666"
```

$ sudo udevadm control --reload
