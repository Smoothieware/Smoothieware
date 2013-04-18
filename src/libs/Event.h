// this event called from main loop. It is safe to perform any action in this event
EVENT( ON_MAIN_LOOP             , on_main_loop              )

// this event is called when a Stream receives a line.
// TODO: make redundant, GcodeDispatch or SimpleShell can poll sources when they're able to process a line
EVENT( ON_CONSOLE_LINE_RECEIVED , on_console_line_received  )

// this event fires when we receive a valid gcode command
// modules may register for this, then act on any gcodes they recognise
EVENT( ON_GCODE_RECEIVED        , on_gcode_received         )

// called when we find gcode in the Block Queue
// TODO: supercede with on_action_invoke
EVENT( ON_GCODE_EXECUTE         , on_gcode_execute          )

// used by Laser and Extruder to match their tool to the movement of the Robot
EVENT( ON_SPEED_CHANGE          , on_speed_change           )

EVENT( ON_BLOCK_BEGIN           , on_block_begin            )
EVENT( ON_BLOCK_END             , on_block_end              )

EVENT( ON_CONFIG_RELOAD         , on_config_reload          )

// this event called when we are no longer paused
EVENT( ON_PLAY                  , on_play                   )
// this event called when we pause for some reason
EVENT( ON_PAUSE                 , on_pause                  )

// this event is called in main loop context, and also in busy loops.
// DO NOT add actions from this event!
EVENT( ON_IDLE                  , on_idle                   )

// TODO: revamp config system
// EVENT( ON_CONFIG_VALUE          , on_config_value           )
// EVENT( ON_CONFIG_RETRIEVE       , on_config_retrieve        )
// EVENT( ON_CONFIG_COMPLETE       , on_config_complete        )

// this event called once per second. intended for generating informational and/or debug output eg temperature reports during warmup
EVENT( ON_SECOND_TICK           , on_second_tick            )

// action_invoke is called repeatedly until all data is removed from the action
// once that happens, we move to the next action in the queue
EVENT( ON_ACTION_INVOKE         , on_action_invoke          )

// this event is fired from main loop when there is space in the action queue
// all gcode sources should flush their buffers from this event
EVENT( ON_FREE_ACTION           , on_free_action            )

// TODO: these events added so that GcodeDispatch can properly manage its queue when USBSerial streams attach and detach
// EVENT( ON_STREAM_ADD            , on_stream_add             )
// EVENT( ON_STREAM_REMOVE         , on_stream_remove          )
