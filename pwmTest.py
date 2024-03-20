from smbus import SMBus

bus = SMBus(1)
device_address = 0x14
channel_on_value_register = 0x20
pulse_width_value = 50
bus.write_word_data(device_address, channel_on_value_register, pulse_width_value)

while True:
    continue


