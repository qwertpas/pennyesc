## PennyESC, a tiny and cheap brushless motor controller

![](pics/initalconcept.jpg)

* Aiming for $10-15 component cost, around 15 mm diameter
* Assembled by JLCPCB using their in-stock parts
* MP6536 3-phase power stage ($3.38)
    * 5-26 V input
    * 5.5 A current limit
    * PWM input for each half-bridge
    * no current sense
    * 5x5 mm
* MA782 magnetic encoder ($4.11)
    * SPI
    * 2x2 mm
* STM32L011 microcontroller ($1.28)
    * 32 MHz clock
    * 3x3 mm
