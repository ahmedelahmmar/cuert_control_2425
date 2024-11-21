# Stm32 Based 180 Commutation
[+] Info:
  - Max Electrical cycle frequency => ~ 220 hz
  - Theta mech = 23 * Theta electrical
  - 
[+] Peripherals mapping : 
  -  Timer1 : PWM generation for the commutation
  -  Timer2 : Is used to emulate hall sensors 
  -  Timer3 : Triggers ADC sampling of the throttle (at 200 ms rate)
  -  Timer4 : is already configured to do Input capture on rising edges ,and the callback is ready to be developed..(check the IOC file for the specific pins)
  -   EXTI is used on Rising/Falling Edges of Hall Sensors Inputs to determine current position and take suitable commutation decision
 [+] Notes:
  Timer 2 requires slight configurations for more realistic timing range .... (fancy words .. just set suitable prescalar and ARR ..)
