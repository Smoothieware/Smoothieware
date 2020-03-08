#pragma once

#include <stdint.h>
#include <memory>

#include <map>

namespace stepper_connection_methods {
        enum Connection_Methods {
                UART,
                SPI
        };
}

class DRV8711DRV;
class TMC26X;
class TMC22X;

class StreamOutput;

class StepperDrv{
    public:
        // no destructors
        //virtual ~StepperDrv(){};
        
        uint16_t max_current= 3000;
        using options_t= std::map<char,int>;
        
        virtual void init(uint16_t cs) ;

        virtual void set_current(uint16_t currentma);
        virtual void set_enable(bool enable) ;
        virtual int set_microsteps(int number_of_steps);
        virtual int get_microsteps();
        virtual bool set_options(const options_t& options);
        virtual void set_write_only(bool wo);

        virtual void dump_status(StreamOutput *stream);
        virtual bool set_raw_register(StreamOutput *stream, uint32_t reg, uint32_t val);
        virtual bool check_alarm();
        
        virtual void get_debug_info(StreamOutput *stream);
        
        stepper_connection_methods::Connection_Methods connection_method;
        
    protected:
        bool write_only = false;
};
