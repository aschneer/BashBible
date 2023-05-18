# Serial RS232

First find the USB/Serial adapter device

```bash
# Use less to view the syslog
sudo less /var/log/syslog
# Search and highlight (case-sensitive) for keywords.
# Find a group of lines that discuss a Serial USB device.
# Find the Vendor ID and Product ID and save those numbers.
/USB
/Serial
/idVendor
/idProduct
# Another way to look at the syslog is with 'tail'.
# Do this to get the device information as you plug/unplug it.
# It will populate the syslog with the device info.
sudo tail -f /var/log/syslog
	# -f follows, only goes forward in time
# Print last group of syslog items.
sudo tail /var/log/syslog
# You can also try this.
dmesg | grep -i idVendor
dmesg | grep -i idProduct
```

Configure serial/USB adapter for pl2303

```bash
# load drive
sudo modprobe pl2303
# attach device to driver using idVendor and idProduct values.
echo "067b 23a3" | sudo tee /sys/bus/usb/drivers/pl2303/new_id
	# 067b is vender ID
	# 23a3 is product ID
	# These show up in dmesg when you grep it
# List all ports
ls /dev/tty*
# Find your serial
dmesg | grep tty
	# Identify the port that comes up, let's say it's ttyUSB4
	# It should say "...now attached to tty____"
# Check permissions
ls -l /dev/ttyUSB4
# Make sure port has correct permissions
sudo chmod 666 /dev/ttyUSB0
```

Start the screen session

```bash
# Start serial screen with 115200 baud rate
# (must match baud rate of connected device).
screen /dev/ttyUSB4 115200
```

You can type commands in the screen and press **Enter** to send them to the serial device, but what you type will not be echoed back to you.

Record data from screen session

```bash
# Record screen session to log file in current directory.
# Log file name = screenlog.0.
Ctrl+A Shift+H
```

Exit screen session (leaves session running, just removes you from it)

```bash
Ctrl+A
D
```

Once you exit the screen, you will not be able to open it again with the same command. Instead, you need to do the following.

```bash
# Detach and restart screen.
screen -dr
	# d = detach
	# r = restart
```

Kill screen session (ends session for everyone)

```bash
Ctrl+A
K
```

Screen Troubleshooting

- If you run screen and immediately get `[screen terminated]`, most likely another device or software is already connected to the serial port. Only one device can connect to a serial port at once.
- If the screen opens but it is blank and there is no response when you type and send commands:
	- Another USB/DB9 serial adapter might be connected to the serial lines from a different computer. Remove it.
	- Serial port settings (baud rate, etc.) might not match the device you're connecting to. Sometimes the wrong baud rate will produce garbled text on the screen, and sometimes it will be blank.
	- The serial device is not connected (i.e. wiring issue) such that you are not receiving the signal.

Minicom / Picocom

```bash
sudo minicom -s
picocom
```

Output Serial Data to Text File

```bash
# Continuously record data from serial port.
# Stop with Ctrl+C.
# Only works if serial port is not in use already.
cat /dev/ttyUSB0 > output.txt
```