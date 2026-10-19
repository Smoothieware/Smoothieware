
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




Filament advance (pressure advance)
===================================

Smoothie can now push the extruder slightly ahead while the print head speeds up and hold it back while it slows
down, which sharpens corners and evens out line width. It is off by default, nothing changes unless you enable it.

```
extruder.hotend.pressure_advance      0.05     # time constant K in seconds, 0 = off
extruder.hotend.pressure_advance_min_reversal  2.5   # smallest backward move (steps) worth a direction change, 0 = never skip
```

It can also be set at run time with `M900 K0.05` (and `M900 R2.5` for the reversal threshold) (add `P1` etc. for another extruder; `M900` alone prints the value) and
saved with `M500`. Retracts, primes and pure extruder moves are never affected. When it is on, the head slows to
the planner's minimum speed where a printed line starts or ends (the same place most slicers already stop).
See `src/modules/robot/FilamentAdvance.h` for how it works.
CNC builds (`make CNC=1`) do not contain this feature at all, so they need no more flash or RAM than before.


Backlash compensation
=====================

Each motor can now be given the slack of its drive train, which the planner adds to the steps of a move that reverses
direction on that motor. Off by default (0 = no compensation), nothing changes unless you set it.

```
alpha_backlash_mm      0.10     # also beta_, gamma_, delta_, epsilon_, zeta_backlash_mm; extruders are ignored
```

At run time `M425 X0.1 Y0.1 Z0 A0 B0` sets it (`M425` alone prints it) and `M500` saves it. The slack is added to the
motor's steps of the reversing move, so that motor runs a little faster during that move; the speed and acceleration
limits take the extra distance into account. The slack steps are taken up first: they are issued at an extra step rate
at the start of the move (`backlash_takeup_rate`, mm/s of motor travel, default the motor's maximum rate; `0` spreads the
slack over the whole move as the first version did), then the motor rejoins the planned profile. The step interrupt does
one more add and compare per moving motor per tick. Not tested on a machine.

The planner queue now stores blocks in a byte arena in the main heap instead of 32 fixed `Block` objects in AHB0: `planner_queue_size` (default 32) is the number of slots, `planner_queue_bytes` (default 0) the arena size in bytes, and `mem` shows its use. 0 means room for 31 blocks in which every motor moves plus the largest record; a value below 3 times the largest record, or too large for the 16-bit offsets or the heap, falls back to the default. Computed (not measured on a board) totals at the default size against upstream: 3 motors -224 B, 4 motors -576 B, 5 motors -928 B, 6 motors -1,280 B; AHB0 use of the queue falls to zero and heap use rises by 1.5 to 2.2 KB. A queue of only short or one-motor blocks needs far less (`planner_queue_bytes`); with compensation on the largest records are bigger, so the default arena holds fewer blocks (27 for advance on a delta printer, 21 with backlash on X Y Z too).

Known limits of backlash compensation (outside review 2026-10-18, see the PR text): after a halt in the middle of a reversal the reported position can differ from the real one by up to the backlash value; `M92`, `G92.4` and manual stepping do not reconcile the stored direction.

- Backlash blocks that carry slack steps are now joined to their neighbours at minimum speed (slower reversals, no step-rate jump); a motor with slack and no own acceleration is capped by the default acceleration; backlash values above 100 mm are ignored; an arena that is too small is replaced by the default size.

- `M425 P0` / `M425 P1` switch all backlash compensation off/on without losing the configured values, and a bare `M425` prints the state and values. homing, probing, jogging and halt forget the tracked motor directions, so the first move after them gets no slack steps (safe but not exact).

- Block records and the step interrupt now carry the generic compensation fields (`shaped_motors`, `shaper_state`, `reversal_tick`, `decel_start_rate_delta`, `takeup_rate`, `takeup_end_tick`) in every build, including `CNC`: the fields are stored only in the 32 B extension record of a compensated moving motor, so a block without compensation is unchanged in size. With compensation off the step output is unchanged.

- A queued block now stores its per-motor axis record only for the motors that move in it, so the average block is smaller than the previous bullet suggests; `mem` prints the largest record size. Blocks are variable-size in the arena (the queue reserves the largest while one is filled).
