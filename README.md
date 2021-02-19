# DigitalBird-Camera-Slider
Professional level DIY Camera slider
This repository is for all the software used to drive the Digital bird camera motion control system.

Currently the system consists of 3 parts. The Slider, Pan Tilt Head, Cinematic Turntable.

All parts of the system currently use the aruino based RF-Nano

The repository currently holds four sketches since the Pan Tilt uses two boards with differing software one sketch for the Pan another for the Tilt

The reason we are using two boards to drive the Pan Tilt is a work around for the AsselStepper library which does not allow for acceleration control on more than one stepper at a time

While this may seem inconvenient it has led to a modular system which allows for almost unlimited axes, since each is wireless and has its own controller with its own logic.

The software makes use of the following libraries you will also need to install.

AccelStepper.h    (To control the steppers)

MultiStepper.h    (Part of AccelStepper)

RF24.h            (For the radio wifi)

nRF24L01.h        (Also for the radio)

SPI.h             (Serial Peripheral Interface)

AS5600.h          (For the AS5600 magnetic encoder used on the slider and Turntable)

Nextion.h         (For control of the Nextion display)
