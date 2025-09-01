from gpiozero import Device, Button
from gpiozero.pins.lgpio import LGPIOFactory
from signal import pause

Device.pin_factory = LGPIOFactory()

BUTTON_PIN = 4  # or 4, if that’s your wiring
btn = Button(BUTTON_PIN, pull_up=False)  # wire button between pin and GND

btn.when_pressed = lambda: print("Button pressed!")
btn.when_released = lambda: print("Button released!")
print("Waiting for button…")
pause()
