#ifndef MODULE_PREVENT_COLD_EXTRUSION_H_
#define MODULE_PREVENT_COLD_EXTRUSION_H_

#define FLAG_ENABLE      0x01
#define FLAG_READ_TEMP   0x02
#define FLAG_TEMP_OK     0x04

class ColdExtrusionPrevention : public Module {
    public:
        ColdExtrusionPrevention();
        void on_module_loaded();
        void on_gcode_execute();
        void on_config_reload(void* argument);
        void on_idle(void* argument);
        void on_second_tick(void* argument);
        void on_gcode_received(void* argument);

        uint8_t get_highest_temperature();

        void set_flag(uint8_t flag, bool enabled);
        bool read_flag(uint8_t flag);

    private:
        uint8_t  flags;
        uint8_t  minExtrusionTemperature;

        uint16_t temperatureController;
};

#endif

