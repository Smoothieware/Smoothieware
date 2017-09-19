
To use multi-axis and ABC axis
==============================

**In order to use there should be no extruders defined.**
This is being tested in CNC mode, it is not currently known to work for extruders.

The new axis are defined using...

```
# A axis
delta_steps_per_mm                    100     # may be steps per degree for example
delta_step_pin                               xx              # Pin for delta stepper step signal
delta_dir_pin                                xx             # Pin for delta stepper direction
delta_en_pin                                 xx             # Pin for delta enable
delta_current                                1.5              # Z stepper motor current
delta_max_rate                               300.0            # mm/min
delta_acceleration                          500.0            # mm/sec²

# B axis
epsilon_steps_per_mm                    100     # may be steps per degree for example
epsilon_step_pin                              xx              # Pin for delta stepper step signal
epsilon_dir_pin                                xx             # Pin for delta stepper direction
epsilon_en_pin                                xx             # Pin for delta enable
epsilon_current                                1.5              # Z stepper motor current
epsilon_max_rate                               300.0            # mm/min
epsilon_acceleration                          500.0            # mm/sec²

# C axis
zeta_steps_per_mm                    100     # may be steps per degree for example
zeta_step_pin                               xx              # Pin for delta stepper step signal
zeta_dir_pin                                xx             # Pin for delta stepper direction
zeta_en_pin                                 xx             # Pin for delta enable
zeta_current                                1.5              # Z stepper motor current
zeta_max_rate                               300.0            # mm/min
zeta_acceleration                          500.0            # mm/sec²
```

The firmware must be compiled with `make AXIS=6 CNC=1` or `make AXIS=4 CNC=1` depending on how many axis are needed. (Use minimum needed).
**NOTE** that by default only XYZ are considered primary (or cartesian) axis for purposes of junction deviation calculations and distance traveled. (which effects effective feed rate).
Adding `PAXIS=4` to the make would make the first 4 axis primary axis so XYZA would be considered for the Cartesian distance traveled

`get pos` will show the A B C axis positions

`G28.3 A0 B0 C0` will set the axis to the specified values (sort of manual homing)
`G28` or `G28 A0` etc will home the ABC axis if they are defined using the new homing syntax.
(**Note** on a CNC build this would be `G28.2 A0 B0`.)

**NOTE** that the parameters passed into A B or C in a G0 or G1 are raw, they do not go through the arm solution, they are the values that will be passed to the stepper based on steps_per_mm for that axis (although it may be interpreted as steps per degree for instance).

**NOTE** G2/G3 currently do not handle ABC axis correctly, basically it will draw the arc then ABC will move afterwards, which is totally incorrect.

This may not be a complete solution, by default (see PAXIS above) the planner treats  X Y Z as primary axis, A B C are not treated as primary axis, there is no junction deviation applied to them for instance, and if they move a significant number of steps more than X Y Z there may be speed issues.

Note that M907 for setting currents is deprecating the use of E it is now channels XYZABCD matching the associated axis. Use of E will still do what it used to do for backward compatibility but will print out a warning. Saving the M907 with M500 will write it in the new format.

## Other changes...

- Add ability to home ABC axis or E axis (see ConfigSamples/Snippets/abc-endstop.config for example of new endstop syntax required for this)
- Major Change to endstops config syntax. (Old syntax is still supported and preferred unless ABC endstops are needed).
- Add an inverse compensation transform for all ABL types.
- Use inverse transform for resetting axis from actuator position.
- All probes leave compensation transform on now by default.
- Homing will use inverse compensation transform to get actual Z position at the given XY position.
- ? and M114.1 and M114.2 will return corrected position based on compensation transform.

> The reasoning of having reset_axis setting compensated_machine_position and applying inverse transform to get machine_position is that most Z endstops are fixed on the Z actuator and trigger at the same place regardless of where the XY is. So consider a sloped bed when you home in Z you will always be at the same height, but as the bed is sloped wrt to the head you want to calculate the WCS position (returned with M114) to be the actual height above the bed at the current XY. Which will differ depending on where the XY is at the time Z is homed.

These changes only affect systems where ABL is active and a zprobe or home is performed while active.
To probe raw bed heights turn off the ABL first.

- Fix M306 so it does not reset axis position and acts more like M206 which needs a home cycle to take effect.
- Removed ZGrid from build as it has been replaced by delta grid set for square beds.
- Changed G30 Z0 to use G92 to set the global offset.
- Refactor naming of last_milestone in Robot to machine_position.
- Add notion of a homed axis, and M codes to view and clear homing status of an axis.



To upgrade from old master or very old edge to the new motion control firmware
==============================================================================

for builds prior to July 1, 2016 you must do the following....


The following changes must be made to your config

1. ```alpha_max_travel, beta_max_travel, gamma_max_travel``` must be correctly defined for homing to work properly
   they control the maximum distance the axis will move before it gives up finding the home switch. They are currently set to ```alpha_max, beta_max and Gamma_max``` (or 500) if not found in the config file.
NOTE on a delta this needs to be the total hieght (or greater) than yur towers NOT the same as gamma_max.

2. it is best to start from a fresh config

3. you must delete the config-override (M502) as M203 format has changed (M203 sets cartesian max speeds, M203.1 sets actuator max speeds, no longer uses ABC as these are now reserved for future n-axis). (or for a short while just do M500 as the  A B C will be read until it is deprecated, and M500 will save it in the new format)

4. Homing is slightly different, by default it will home X and Y axis at the same time then Z, this can be reversed and have Z home first then X and Y.
   the homing_order setting still works the same way as before.

5. The old extruder syntax is no longer allowed so check your config has the latest extruder config like
   ```
   extruder.hotend.enable                          true             # Whether to activate the extruder module at all. All configuration is ignored if false
   extruder.hotend.steps_per_mm                    710              # Steps per mm for extruder stepper
   extruder.hotend.default_feed_rate               600              # Default rate ( mm/minute ) for moves where only the extruder moves
   etc
   ```
   
   and not the old
   
   ```
   extruder_module_enable                       true
   ```

6. If you use volumetric extrusion (M200 D2.85) then note that unlike the current edge, G1 E5 will extrude 5mm³ not 5mm. Note that in slic3r there is now a smootheware flavor you need to select for gcode generation, which fixes an issue with firmware retract.

7. if you have configured more than 2 extruders you will need to recompile and set ```MAX_ROBOT_ACTUATORS``` accordingly.. https://github.com/Smoothieware/Smoothieware/blob/edge/src/modules/robot/ActuatorCoordinates.h#L17

8. Due to a mistake in the previous versions of the firmware the E direction was reversed, so you must invert your dir pin for your extruders (or reverse the extruder plug) from how they were before.


