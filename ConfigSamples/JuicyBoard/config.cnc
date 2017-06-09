# NOTE Lines must not exceed 132 characters
## Robot module configurations : general handling of movement G-codes and slicing into moves
default_feed_rate                            4000             # Default rate ( mm/minute ) for G1/G2/G3 moves
default_seek_rate                            4000             # Default rate ( mm/minute ) for G0 moves
mm_per_arc_segment                           0.0              # Fixed length for line segments that divide arcs 0 to disable
mm_max_arc_error                             0.01             # The maximum error for line segments that divide arcs 0 to disable
                                                              # note it is invalid for both the above be 0
                                                              # if both are used, will use largest segment length based on radius
#mm_per_line_segment                          5                # Lines can be cut into segments ( not usefull with cartesian
                                                              # coordinates robots ).

# Arm solution configuration : Cartesian robot. Translates mm positions into stepper positions

# Planner module configuration : Look-ahead and acceleration configuration
planner_queue_size                           32               # DO NOT CHANGE THIS UNLESS YOU KNOW EXACTLY WHAT YOU ARE DOING
acceleration                                 3000             # Acceleration in mm/second/second.
#z_acceleration                              500              # Acceleration for Z only moves in mm/s^2, 0 uses acceleration which is the default. DO NOT SET ON A DELTA
junction_deviation                           0.05             # Similar to the old "max_jerk", in millimeters,
                                                              # see https://github.com/grbl/grbl/blob/master/planner.c
                                                              # and https://github.com/grbl/grbl/wiki/Configuring-Grbl-v0.8
                                                              # Lower values mean being more careful, higher values means being
                                                              # faster and have more jerk
#z_junction_deviation                        0.0              # for Z only moves, -1 uses junction_deviation, zero disables junction_deviation on z moves DO NOT SET ON A DELTA
#minimum_planner_speed                       0.0              # sets the minimum planner speed in mm/sec

# Stepper module configuration
microseconds_per_step_pulse                  1                # Duration of step pulses to stepper drivers, in microseconds
base_stepping_frequency                      100000           # Base frequency for stepping

# Cartesian axis speed limits
x_axis_max_speed                             3000             # mm/min
y_axis_max_speed                             3000             # mm/min
z_axis_max_speed                             3000             # mm/min

# Stepper module pins ( ports, and pin numbers, appending "!" to the number will invert a pin )
alpha_slot_num                               6
alpha_steps_per_mm                           80
stepmotor_6_stepres                          16
stepmotor_6_current                          1000
stepmotor_6_decay_mode                       fast
alpha_max_rate                               5000.0           # mm/min

beta_slot_num                                5
beta_steps_per_mm                            80
stepmotor_5_stepres                          16
stepmotor_5_current                          1000
stepmotor_5_decay_mode                       fast
beta_max_rate                                5000.0           # mm/min

gamma_slot_num                               4
gamma_steps_per_mm                           80
stepmotor_4_stepres                          16
stepmotor_4_current                          1000
stepmotor_4_decay_mode                       fast
gamma_max_rate                               5000.0           # mm/min

## System configuration
# Serial communications configuration ( baud rate defaults to 9600 if undefined )
uart0.baud_rate                              115200           # Baud rate for the default hardware serial port
second_usb_serial_enable                     false            # This enables a second usb serial port (to have both pronterface
                                                              # and a terminal connected)
#leds_disable                                true             # disable using leds after config loaded
#play_led_disable                            true             # disable the play led

# Kill button (used to be called pause) maybe assigned to a different pin, set to the onboard pin by default
kill_button_enable                           false            # set to true to enable a kill button
#kill_button_pin                              2.12             # kill button pin. default is same as pause button 2.12 (2.11 is another good choice)

#msd_disable                                 false            # disable the MSD (USB SDCARD) when set to true (needs special binary)
#dfu_enable                                  false            # for linux developers, set to true to enable DFU
#watchdog_timeout                            10               # watchdog timeout in seconds, default is 10, set to 0 to disable the watchdog

# Only needed on a smoothieboard
#currentcontrol_module_enable                 true             #

## Laser module configuration
laser_module_enable                          false            # Whether to activate the laser module at all. All configuration is
                                                              # ignored if false.
#laser_module_pin                             2.5             # this pin will be PWMed to control the laser. Only P2.0 - P2.5, P1.18, P1.20, P1.21, P1.23, P1.24, P1.26, P3.25, P3.26
                                                              # can be used since laser requires hardware PWM
#laser_module_maximum_power                   1.0             # this is the maximum duty cycle that will be applied to the laser
#laser_module_minimum_power                   0.0             # This is a value just below the minimum duty cycle that keeps the laser
                                                              # active without actually burning.
#laser_module_default_power                   0.8             # This is the default laser power that will be used for cuts if a power has not been specified.  The value is a scale between
                                                              # the maximum and minimum power levels specified above
#laser_module_pwm_period                      20              # this sets the pwm frequency as the period in microseconds



## Endstops
endstops_enable                              false             # the endstop module is enabled by default and can be disabled here
#endstops_enable                              true             # the endstop module is enabled by default and can be disabled here
#corexy_homing                               false            # set to true if homing on a hbot or corexy
#alpha_min_endstop                            1.24^            # add a ! to invert if endstop is NO connected to ground
#alpha_max_endstop                            1.25^            # NOTE set to nc if this is not installed
#alpha_homing_direction                       home_to_min      # or set to home_to_max and set alpha_max
#alpha_min                                    0                # this gets loaded after homing when home_to_min is set
#alpha_max                                    200              # this gets loaded after homing when home_to_max is set
#beta_min_endstop                             1.26^            #
#beta_max_endstop                             1.27^            #
#beta_homing_direction                        home_to_min      #
#beta_min                                     0                #
#beta_max                                     200              #
#gamma_min_endstop                            1.28^            #
#gamma_max_endstop                            1.29^            #
#gamma_homing_direction                       home_to_min      #
#gamma_min                                    0                #
#gamma_max                                    200              #

alpha_max_travel                             500              # max travel in mm for alpha/X axis when homing
beta_max_travel                              500              # max travel in mm for beta/Y axis when homing
gamma_max_travel                             500              # max travel in mm for gamma/Z axis when homing

# optional order in which axis will home, default is they all home at the same time,
# if this is set it will force each axis to home one at a time in the specified order
#homing_order                                 XYZ              # x axis followed by y then z last
#move_to_origin_after_home                    false            # move XY to 0,0 after homing

# optional enable limit switches, actions will stop if any enabled limit switch is triggered
#alpha_limit_enable                          false            # set to true to enable X min and max limit switches
#beta_limit_enable                           false            # set to true to enable Y min and max limit switches
#gamma_limit_enable                          false            # set to true to enable Z min and max limit switches

alpha_fast_homing_rate_mm_s                  50               # feedrates in mm/second
beta_fast_homing_rate_mm_s                   50               # "
gamma_fast_homing_rate_mm_s                  4                # "
alpha_slow_homing_rate_mm_s                  25               # "
beta_slow_homing_rate_mm_s                   25               # "
gamma_slow_homing_rate_mm_s                  2                # "

alpha_homing_retract_mm                      5                # distance in mm
beta_homing_retract_mm                       5                # "
gamma_homing_retract_mm                      1                # "

#endstop_debounce_count                       100              # uncomment if you get noise on your endstops, default is 100

## Z-probe
zprobe.enable                                false           # set to true to enable a zprobe
## Panel
panel.enable                                 false             # set to true to enable the panel code

## Network settings
network.enable                               false            # enable the ethernet network services
network.webserver.enable                     true             # enable the webserver
network.telnet.enable                        true             # enable the telnet server
network.ip_address                           auto             # use dhcp to get ip address
# uncomment the 3 below to manually setup ip address
#network.ip_address                           192.168.3.222    # the IP address
#network.ip_mask                              255.255.255.0    # the ip mask
#network.ip_gateway                           192.168.3.1      # the gateway address
#network.mac_override                         xx.xx.xx.xx.xx.xx  # override the mac address, only do this if you have a conflict

switch.spindle.enable                            true             #
switch.spindle.input_on_command                  M3               #
switch.spindle.input_off_command                 M5               #
switch.spindle.output_pin                        2.0              # Here we are using the first big MOSFET
switch.spindle.output_type                       pwm              # pwm output settable with S parameter in the input_on_comand
switch.spindle.max_pwm                           255              # set max pwm for the pin default is 255
