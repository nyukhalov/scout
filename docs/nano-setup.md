# Setting up Jetson Nano

```bash
wget -O - https://raw.githubusercontent.com/nyukhalov/scout/master/scripts/setup-nano.sh | sudo bash
```

## Connecting a DualShock 4 controller via bluetooth

```bash
# Turn on scanning and and put a controller to the pairing mode
# by pressing and holding Share + PS buttons.
# Wait until the controller is found by the tool.
bluetoothctl scan on

# Copy the device's MAC address.
bluetoothctl devices | grep "Wireless Controller"

# Make sure the controller is in pairing mode, then execute
bluetoothctl pair <MAC_ADDR>
bluetoothctl connect <MAC_ADDR>
bluetoothctl trust <MAC_ADDR>
```

