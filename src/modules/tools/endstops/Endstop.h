#ifndef _ENDSTOP_H
#define _ENDSTOP_H

#include "Module.h"
#include "Pin.h"

#define endstop_checksum           CHECKSUM("endstop")

// we only have pointers to Endstops so we don't need the header, just predeclare the class
class Endstops;

typedef enum
{
    ENDSTOP_TYPE_UNCOMMITTED,
    ENDSTOP_TYPE_CARTESIAN,
    ENDSTOP_TYPE_ACTUATOR
} Endstop_Types_e;

typedef enum
{
    ENDSTOP_DIR_MIN,
    ENDSTOP_DIR_MAX,
    ENDSTOP_DIR_INDEX
} Endstop_Direction_e;

class Endstop : public Module
{
public:
    // constructor
    Endstop(uint16_t name_checksum);

    // class methods
    void on_module_loaded(void);
    void on_config_reload(void*);

    void poll(void);
    bool asserted();

    uint8_t             axis(void);
    Endstop_Direction_e dir(void);

    /*
     * class variables
     */
    // name
    uint16_t  myname_checksum;
    const char* name;

    Endstops* pool;
    int       pool_index;

    Pin       pin;

    float     position;
    float     stopping_distance;

    Endstop_Types_e type;
    uint8_t         type_index; // may be unnecessary

    // debounce stuff
    int       debounce;
    int       debounce_count;
    int       last_change_count;
    bool      state;
};

#endif /* _ENDSTOP_H */
