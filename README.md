# Tiny LDR switch

Turns ON when it's dark (user configurable), stays ON for X hours, tuerns OFF and waits till it's dark again. Intended to use with battery, so it's only consumes 5uA.
Load power consumtion obviously varries.



![Schematic](schematic.jpg)

# Manufacturing

![PCB](pcb.jpg)
![Top view](top-view-3d.jpg)
![Bottom view](bottom-view-3d.jpg)

# Code

MCU sleeps all the time. Wakes up via watchdog interrupt every 4 sec to measure light. It powers LDR/R voltage divider via MCU pin, takes ADC measurement and turns off the pin to save power. Compares measured light with the thresholds and goes into ON, OFF or WAIT_FOR_RESET state. Depending on the state it turns load ON/OFF.
A short button press wakes it up from sleep, forces to measure light. 

* If it was in the OFF state - records the new "dark" light threshold and turns the load ON, starts the auto-turn-off timer, goes into ON state.
* If it was in the ON state - turns the load OFF, goes into WAIT_FOR_RESET state.

Long button press, let you set the the auto-turn-off timer duration. Keep the button pressed and watch the LOAD blink. 1 blink == 1 hour, 2 blinks == 2 hours etc. Release the button once the desired number of hours was indicated. 

When the timer is UP, load turns OFF and MCu transitions into the WAIT_FOR_RESET state.

There are two ligh thresholds in the code. 

* "dark" - once it's darker than this, MCU turns the load ON and goes into ON state
* "bright" - once it's brighter than this, MCU goes from  WAIT_FOR_RESET into the OFF state. 

WAIT_FOR_RESET is meant to capture the day time reset, so that the switch only active once per day for X hours.
