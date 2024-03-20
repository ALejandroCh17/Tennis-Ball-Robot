from smbus import SMBus
bus = SMBus(1)

# Set timer 0 period to 4095
bus.write_word_data(0x14, 0x44, 2048)
# Set frequency to 50Hz,
freq = 50
# Calculate prescaler
prescaler = int(72000000 / (4095 + 1) / freq) - 1
# Set prescaler
bus.write_word_data(0x14, 0x40, prescaler)

# Set channel 0 to 50% pulse width
bus.write_word_data(0x14, 0x20, 2048)
