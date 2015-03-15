#ifndef PROTON_MODULE_H
#define PROTON_MODULE_H



//Basic laser control module
class Proton : public Module{
    public:
        Proton();
        virtual ~Proton();

      	void on_module_loaded();
        void on_block_end(void* argument);
        void on_block_begin(void* argument);
        void on_play(void* argument);
        void on_pause(void* argument);
        void on_gcode_execute(void* argument);
        void on_speed_change(void* argument);
        void on_gcode_received(void *);
};
#endif