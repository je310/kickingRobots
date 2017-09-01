# URobot
Microbot firmware

## ToDo:
- [x] Basic blin + hellow workd woth OpenOCD.
- [x] Compile similar protram with CubeMX headers.
- [x] Write functions to activate the aux power (write up into a particular value).
- [x] Blink a cloured LED
- [x] PWM Dim an LED
- [x] PWM Transmit IR LED (38 KHz) Both ends are actively controlled.
- [x] Make interrupt routine on IR recieve pin. Should trigger when above is active.
- [x] Read ADC value for front + bottom sensors. Light LED's at thresholds.
	-Bottom sensor works, thought front does not. Only one is connected to appropriate pin. 
- [x] Use UART to transmit on the IR LED.
- [x] Look into low power sleep modes. (achieved 500uA ish) (Standby mode is not possible as all oscillators are off and we would need to use a wakeup pin instead, currently they are not connected to anything good, the difference is minor anywhay, 15uA or so in theory) (We should aim for the Potential divider to dominate 200uA)

These may not be possible with the current hardware. 
- [ ] use UART to Recieve on the IR reciever.
- [ ] Port the IR remote library (or find one that already works). (found one is broken due to availablity of the correct timer periphrials.
- [ ] Initiate LED behaviour using keys on a remote.
- [ ] Sleep and intermittanlty check for IR activity, wake on a particular key, sleep on another.

## Hardware Tests
- [x] Solder motors on them.
- [x] Implement phase control.
- [x] Test in isolation the front sensors. 
- [x] Solder a battery on.
- [x] Check charging.
- [x] Check reverse protection. (Requires at least 10 constitution) 
- [x] Check reverse Programming connection. (Requires at least 10 constitution).
  
## Hardware TODO 
- [x] Swap PA10 and PA4, such that IRrec is available with timer1 (for NEC control library) and the hardware UART. PA4 is also an ADC channel, so this is a nice swap. 
- [ ] Try configuring the Upright charging pins to double as shaft. 
- [x] (we are lacking space, so for now we will only increase the resistance) Consider adding a transistor to lower power consumption of potental divider (Only if it is the primary power draw in aggressive sleep. 
