/*
      this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
      smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
      smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
      you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>.
*/

/*
 *    Logger Module usage :
 *    In Smoothie firmware, the logger is accessed through THEKERNEL (kernel instance)
 *    Simply call one of the 3 method logDebug(), logWarning() or logError() like that:
 *    THEKERNEL->logger->logWarning("MyLog Message \n");
 *
 *    loglevel parameter can be used in Wmoothie config file accordingly :
 *    loglevel  1
 *    0 : disable, 1 : error, 2 : warning , 3 : debug
 *
 *    DEFAULT_LOGLEVEL    2         Default loglevel
 *    MAX_LOGBUFFER_SIZE  5         log buffer max size (static mem allocation)
 *    MAX_CHAR_LOG        128       max char in log message (static mem allocation)
 */

#ifndef LOGGER_H
#define LOGGER_H

#include "libs/Module.h"

#define MAX_LOGBUFFER_SIZE  3
#define MAX_CHAR_LOG        128
#define DEFAULT_LOGLEVEL    1

class Logger : public Module {
public:
    Logger();

    void on_module_loaded(void);
    void on_config_reload(void*);
    void on_main_loop(void*);

    void logError( const char* );
    void logWarning( const char* );
    void logDebug( const char*);
    void logHeader( const char*, const char* );

private:
//    char logbuffer_[MAX_LOGBUFFER_SIZE][MAX_CHAR_LOG];
    char** logbuffer_;
    int16_t first_;
    int16_t last_;
    int16_t size_;
    int16_t loglevel_;
};

#endif /* LOGGER_H */
