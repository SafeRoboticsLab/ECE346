import maestro

devices = maestro.getConnectedDevices()

if len(devices) == 0:
    print("No Maestro devices connected")
    exit()

for device in devices:
    print(f"name: {device.getName()} #channels: {device.getNumChannels()}")
    for c in range(device.getNumChannels()):
        channel=device.getChannelSettings(c)
        print(f"#{c} mode: {channel.mode} min: {channel.minimum} max: {channel.maximum} speed: {channel.speed} acceleration: {channel.acceleration}")

# select first Maestro device
device=devices[0]

# change servo 0 position
device.setTarget(0, 5000)