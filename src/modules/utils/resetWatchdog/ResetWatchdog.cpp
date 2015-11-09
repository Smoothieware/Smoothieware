#include "ResetWatchdog.h"
#include "Kernel.h"
#include "SlowTicker.h"
#include "StreamOutputPool.h"
#include "SerialMessage.h"

ResetWatchdog::ResetWatchdog(){
   if(WDT_ReadTimeOutFlag() == SET){
      awaked_from_reset = true;
      WDT_ClrTimeOutFlag();
      connected = false;
   }
}

void ResetWatchdog::on_module_loaded(){
   WDT_Init(WDT_CLKSRC_PCLK,WDT_MODE_RESET);
   WDT_Start(100000);   // in micrseconds
   THEKERNEL->slow_ticker->attach( (unsigned int)11, this, &ResetWatchdog::watchdog_tick ); // readings per second must be appropriate to restart the watchdog before timeout
   this->register_for_event(ON_GCODE_RECEIVED);
}

uint32_t ResetWatchdog::watchdog_tick(uint32_t dummy){
   WDT_Feed();
   return 0;
}

// G-Code can be only received if connected.
// Needed to ensure that the user sees the message
void ResetWatchdog::on_gcode_received( void *argument ) {
   //Gcode *gcode = static_cast<Gcode *>(argument);

   if(!connected){
      connected = true;
      if(awaked_from_reset){
         THEKERNEL->streams->printf("// Booted after internal Reset trigger, because of a firmware crash.\n");
         awaked_from_reset = false;
      }
   }
}

