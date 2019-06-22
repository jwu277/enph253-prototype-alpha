# enph253-prototype-alpha
[![Build Status](https://travis-ci.com/jwu277/enph253-prototype-alpha.svg?token=GxxSjGqNnwfCvuxigwZq&branch=master)](https://travis-ci.com/jwu277/enph253-prototype-alpha)

ENPH 253: Code for Prototype Alpha

Feel free to modify our SW approach/this README as need be.

## Structure
An OOP infrastructure is used to create the HW sensors + actuators in SW.

Concrete components are derived from abstract base classes, depending on their properties.

### Class List
**Concrete**
- `DriveMotor`: The driving motors

**Abstract**
- `BaseActuator`
- `PolarityActuator`
- `PwmActuator`

## Program Flow
`setup()`: SW + HW initialization of components

`loop()`:
- read sensors
- perform SW calculations
- update actuators in SW
- execute actuators in HW

## Conventions
Feel free to add/change any conventions you wish.
* Use many single-line comments instead of a multi-line comment. This allows for multi-line commenting out a chunk of code.
* lower_underscore_names for fields, variables, and parameters. lowerCamelCase for non-constructor methods. UpperCamelCase for classes.
* End files with a newline/blank line.
