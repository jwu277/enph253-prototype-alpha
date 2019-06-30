# enph253-prototype-alpha
[![Build Status](https://travis-ci.com/jwu277/enph253-prototype-alpha.svg?token=GxxSjGqNnwfCvuxigwZq&branch=master)](https://travis-ci.com/jwu277/enph253-prototype-alpha)

README last update: June 30th (2019)

ENPH 253: Code for Prototype Alpha

Feel free to modify our SW approach/this README as need be.

## Structure
An OOP infrastructure is used to create the HW sensors + actuators in SW.

Concrete components are derived from abstract base classes, depending on their properties.

### Class List
**Concrete**

Sensors:
- `QrdSensor`: QRD Sensor
- `TapeSensor`: the main QRD sensor system to follow tape

Actuators:
- `DriveMotor`: the driving motors
- `DriveSystem`: the driving system, composed of 2 `DriveMotor`s

**Abstract**

Sensors:
- `BaseSensor`
- `AnalogSesnor`

Actuators:
- `BaseActuator`
- `PolarityActuator`
- `PwmActuator`

## Program Flow
`setup()`: SW + HW initialization of actuators, SW initialization of sensors

`loop()`:
- read sensors
- perform SW calculations + update actuators in SW (maybe split this into 2?)
- execute actuators in HW

## Conventions
Feel free to add/change any conventions you wish.
* Use many single-line comments instead of a multi-line comment. This allows for multi-line commenting out a chunk of code.
* lower_underscore_names for fields, variables, and parameters. lower_underscore for non-constructor methods. UpperCamelCase for classes.
* End files with a newline/blank line.
