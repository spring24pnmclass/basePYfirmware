import RPi.GPIO as GPIO

# Set the GPIO mode (either BOARD or BCM)
GPIO.setmode(GPIO.BCM)

# Specify the pin you want to check
pin_number = 7  # Replace with your desired GPIO pin number

# Get the function of the specified GPIO channel
function = GPIO.gpio_function(pin_number)

# Interpret the result
if function == GPIO.IN:
    print(f"GPIO pin {pin_number} is configured as an input.")
elif function == GPIO.OUT:
    print(f"GPIO pin {pin_number} is configured as an output.")
elif function == GPIO.SPI:
    print(f"GPIO pin {pin_number} is configured for SPI communication.")
elif function == GPIO.I2C:
    print(f"GPIO pin {pin_number} is configured for I2C communication.")
elif function == GPIO.HARD_PWM:
    print(f"GPIO pin {pin_number} is configured for hardware PWM.")
elif function == GPIO.SERIAL:
    print(f"GPIO pin {pin_number} is configured for serial communication.")
else:
    print(f"Function of GPIO pin {pin_number} is unknown or unsupported.")


