# 2k23 Ember (AdvantageKit) Style Guide
Classes will be in ```PascalCase```:
<ul>
* ```public class ClassOne {}```
* ```public class ClassTwo {}```
</ul>
<br/><br/>
Variables will be in ```camelCase```:
<ul>
* ```public String variableOne = "my string";```
* ```public double variableTwo = 0.0;```
<ul/>
<br/><br/>
Constants will be all caps in ```SNAKE_CASE```:
<ul>
* ```public static final String CONSTANT_ONE = "my string"```;
* ```public static final double CONSTANT_TWO = 0.0```;
<ul/>
<br/><br/>
Constants will be located in their respective classes, with very specific exceptions
Only create a new branch if you're making a big change to the code or at a big milestone. For example, when in a competition, use a different branch specifically for that competition so as not to clutter up the main branch with competition code
<br/><br/>
Unless commands need to extend Command, all commands will be inline.
<br/><br/>
Commands that require a single subsystem will be inside that subsystem as factories:
<ul>
* ```public Command myCommand(SubsystemOne subsystem) { return ...}```
<ul/>
<br/><br/>
Commands that require multiple subsystems will be inside ```CompositeCommands``` as factories:
* ```public static final myCommand(SubsystemOne subsystemOne, SubsystemTwo, subsystemTwo) { return ... } ```
Every Subsystem IO will have these standard inputs:
<ul>
* Position
* Velocity
* Voltage
* Current
<ul/>

**(The drive subsystem IO will be slightly different)**

<br/><br/>
Standard Units:
<ul>
* Distance: meters (m)
* Velocity: meters per second (m/s)
* Acceleration: meters per second squared (m/s/s)
* Rotation: Rotation2D object (this is to make unit conversion easier)
<ul/>

**If you have a measurement in a non-standard unit, use the ```Units``` class to convert between them**

<br/><br/>
all CTRE can update frequencies will be 50 Hz with very specific exceptions

<br/><br/>
Closed loop control should generally be performed on RIO but this can be changed if needed
