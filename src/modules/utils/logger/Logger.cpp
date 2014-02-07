#include "Logger.h"
#include "string.h"
#include "libs/Kernel.h"
#include "platform_memory.h"
#define loglevel_checksum CHECKSUM("loglevel")

Logger::Logger(){}

void Logger::on_module_loaded()
{
    on_config_reload(this);
    if (loglevel_ > 0)
    {
        register_for_event(ON_CONFIG_RELOAD);
        register_for_event(ON_MAIN_LOOP);
        register_for_event(ON_CONFIG_RELOAD);
        first_ = 0; //First log position in log buffer
        last_ = 0; //Last log position in log buffer
    }
    size_ = 0;
}

void Logger::on_config_reload(void* argument)
{
    loglevel_ = DEFAULT_LOGLEVEL;
    loglevel_ = THEKERNEL->config->value( loglevel_checksum )->by_default(loglevel_)->as_int(); // get loglevel parameter from config file
    if (loglevel_)
    {
        logbuffer_ = (char**)AHB0.alloc(MAX_CHAR_LOG * MAX_LOGBUFFER_SIZE * sizeof(char*));
        if(logbuffer_ == NULL)
        {
            THEKERNEL->streams->puts("Not enough memory available for frame buffer");
            return;
        }
        for(int i = 0; i < MAX_LOGBUFFER_SIZE; i++)
        {
            logbuffer_[i] = (char*)AHB0.alloc(MAX_CHAR_LOG * sizeof(char)); // grab some memoery from USB_RAM ...
            if(logbuffer_[i] == NULL)
            {
                THEKERNEL->streams->puts("Not enough memory available for frame buffer");
                return;
            }
        }
    }
    else if ( logbuffer_ != NULL )
        AHB0.dealloc(logbuffer_);
}

// This method is called during Smoothie Off-load, Logbuffer is then flushed to default kernel streams.
void Logger::on_main_loop(void* argument)
{
    while (size_ >0)
    {
        THEKERNEL->streams->puts( logbuffer_[first_] );
        first_ = ( first_ + 1 ) % MAX_LOGBUFFER_SIZE;
        size_--;
    }
}

void Logger::logError(const char* log)
{
    if (loglevel_ > 0)
        logHeader(log, "ERROR   : ");
}

void Logger::logWarning(const char* log)
{
    if (loglevel_ > 1)
        logHeader(log, "WARNING : ");
}

void Logger::logDebug(const char* log)
{
    if (loglevel_ > 2)
        logHeader(log, "DEBUG   : ");
}

void Logger::logHeader(const char* log, const char* header)
{
    if ( size_ != MAX_LOGBUFFER_SIZE )
    {
        size_++;
        if ( size_ < MAX_LOGBUFFER_SIZE )
        {
            strcpy(logbuffer_[last_], header);
            if ( ( strlen(log) + strlen(header) +1 ) <= MAX_CHAR_LOG )
            {
                strcat(logbuffer_[last_], log);
            }
            else
            {
                // The log message is bigger than MAX_CHAR_LOG, so it is truncated
                strncat(logbuffer_[last_], log, MAX_CHAR_LOG - strlen(header) - 6);
                strcat(logbuffer_[last_], "-CUT\n");
            }
        }
        else
        {
            strcpy(logbuffer_[last_], "Log buffer full, please increase MAX_LOGBUFFER_SIZE\n");
        }
        last_ = ( last_ + 1 ) % MAX_LOGBUFFER_SIZE;
    }
}
