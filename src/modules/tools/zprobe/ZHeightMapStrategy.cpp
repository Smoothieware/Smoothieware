/*
    Author: Quentin Harley (quentin.harley@gmail.com)
    License: GPL3 or better see <http://www.gnu.org/licenses/>

    Summary
    -------
    Probes twenty-five calculated points on the bed and creates ZHeightMap data of the bed surface.
    Bilinear 
    As the head moves in X and Y it will adjust Z to keep the head tram with the bed.

    Configuration
    -------------
    The strategy must be enabled in the config as well as zprobe.

    leveling-strategy.ZHeightMap-leveling.enable         true

    The bed size limits must be defined, in order for the module to calculate the calibration points

    leveling-strategy.ZHeightMap-leveling.bedx           200
    leveling-strategy.ZHeightMap-leveling.bedy           200

    Usage
    -----
    //TODO: G32 probes the twenty-five probe points and defines the bed ZHeightMap, this will remain in effect until reset or M370
    //TODO: G31 reports the status

    M370 clears the ZHeightMap and the bed leveling is disabled until G32 is run again
    M371 moves the head to the next calibration postion without saving
    M372 move the head to the next calibration postion after saving the current probe point

    M500 saves the probe points and the probe offsets
    M503 displays the current settings
*/

#include "ZHeightMapStrategy.h"
#include "Kernel.h"
#include "Config.h"
#include "Robot.h"
#include "StreamOutputPool.h"
#include "Gcode.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "Conveyor.h"
#include "ZProbe.h"
#include "nuts_bolts.h"

#include <string>
#include <algorithm>
#include <cstdlib>
#include <cmath>

#define bed_x_checksum                CHECKSUM("bed_x")
#define bed_y_checksum                CHECKSUM("bed_y")


ZHeightMapStrategy::ZHeightMapStrategy(ZProbe *zprobe) : LevelingStrategy(zprobe)
{
 /*   for (int i = 0; i < 3; ++i) {
        probe_points[i] = std::make_tuple(NAN, NAN);
    }
    plane = nullptr;*/
}

ZHeightMapStrategy::~ZHeightMapStrategy()
{
   // delete plane;
}

bool ZHeightMapStrategy::handleConfig()
{
    this->bed_x = THEKERNEL->config->value(leveling_strategy_checksum, zheightmap_leveling_checksum, bed_x_checksum)->by_default(200.0F)->as_number();
    this->bed_y = THEKERNEL->config->value(leveling_strategy_checksum, zheightmap_leveling_checksum, bed_y_checksum)->by_default(200.0F)->as_number();

    this->bed_div_x = this->bed_x / 4.0F;    // Find divisors to find the 25 calbration points
    this->bed_div_y = this->bed_y / 4.0F;

    this->bed_level_data.numRows = 5;
    this->bed_level_data.numCols = 5;

    this->bed_level_data.pData = (float*) malloc((this->bed_level_data.numRows) * this->bed_level_data.numCols *sizeof(float));

    int i;
    for (i=0; i<(this->bed_level_data.numRows * this->bed_level_data.numCols); i++){
        this->bed_level_data.pData[i] = 0.0F;        // Clear the grid
    }

    return true;
}

bool ZHeightMapStrategy::handleGcode(Gcode *gcode)
{
    if(gcode->has_m) {
        switch( gcode->m ) {

            // manual bed ZHeightMap calbration: M370 - M375
            // M370: Clear current ZHeightMap for calibration, and move to first position
            case 370: {
                this->homexyz();
                //for (int i=0; i<25; i++){
                //    this->bed_level_data.pData[i] = 0.0F;        // Clear the ZHeightMap
                //}

                this->cal[X_AXIS] = 0;                                              // Clear calibration position
                this->cal[Y_AXIS] = 0;
                this->in_cal = true;                                         // In calbration mode
                this->setAdjustFunction(false); // disable leveling code for caloibration


            }
            return true;
            // M371: Move to next initial position
            case 371: {
                if (in_cal){
                    this->move(this->cal, slow_rate);
                    this->next_cal();
                }

            }
            return true;
            // M372: save current position in ZHeightMap, and move to next initial position
            case 372: {
                if (in_cal){
                    float cartesian[3];
                    int pindex = 0;

                    THEKERNEL->robot->get_axis_position(cartesian);         // get actual position from robot
                    pindex = (int) ((cartesian[X_AXIS]/10.0F)+(cartesian[Y_AXIS]/50.0F));

                    this->move(this->cal, slow_rate);                       // move to the next position
                    this->next_cal();                                       // to not cause damage to machine due to Z-offset

                    this->bed_level_data.pData[pindex] = 0.0F + cartesian[Z_AXIS];  // save the offset

                }                   
            }
            return true;
            // M373: finalize calibration  
            case 373: {
                 this->in_cal = false;

            }
            return true;
            // M374: manually inject calibration - Alphabetical ZHeightMap assignment
            case 374: {
                int i=0,
                    x=0;
                
                if(gcode->has_letter('X')) { // Column nr (X)
                    x = gcode->get_value('X');
                }
                if (x<5){                    // Only for valid rows
                    for (i=0; i<5; i++){
                        if(gcode->has_letter('A'+i)) {
                            this->bed_level_data.pData[i+(x*5)] = gcode->get_value('A'+i);
                        }
                    }
                }
                this->setAdjustFunction(true); // Enable leveling code   
            }
            return true;
            case 376: {
                float target[3];

                for(char letter = 'X'; letter <= 'Z'; letter++) {
                    if( gcode->has_letter(letter) ) {
                         target[letter - 'X'] = gcode->get_value(letter);
                    }
                }
                gcode->stream->printf(" Z0 %1.3f\n",getZOffset(target[0], target[1]));
                
            }
            return true;
            case 500: // M500 saves some volatile settings to config override file
            case 503:  // M503 just prints the settings

                // Bed ZHeightMap data as gcode: Still not working right...
                gcode->stream->printf(";Bed Level settings:\n");
                int x,y;
                for (x=0; x<5; x++){
                    gcode->stream->printf("M374 X%i",x);
                    for (y=0; y<5; y++){
                         gcode->stream->printf(" %c%1.2f", 'A'+y, this->bed_level_data.pData[(x*5)+y]);
                    }
                    gcode->stream->printf("\n"); 
                }  
                gcode->mark_as_taken();
                break;
            
            return true;
        }
    }

    return false;
}

void ZHeightMapStrategy::homexyz()
{
    Gcode gc("G28", &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);
}

void ZHeightMapStrategy::move(float *position, float feed)
{
    char cmd[64];

    // Assemble Gcode to add onto the queue
    snprintf(cmd, sizeof(cmd), "G0 X%1.3f Y%1.3f Z%1.3f F%1.1f", position[0], position[1], position[2], feed * 60); // use specified feedrate (mm/sec)

    Gcode gc(cmd, &(StreamOutput::NullStream));
    THEKERNEL->robot->on_gcode_received(&gc); // send to robot directly
}


void ZHeightMapStrategy::next_cal(void){
    if (this->cal[X_AXIS] == bed_div_x || this->cal[X_AXIS] == bed_div_x * 3){
        this->cal[Y_AXIS] -= bed_div_y;
        if (this->cal[Y_AXIS] < 0){
            this->cal[Y_AXIS] = 0;
            this->cal[X_AXIS] += bed_div_x;
        }
    }
    else {
        this->cal[Y_AXIS] += bed_div_y;
        if (this->cal[Y_AXIS] > bed_y){
            this->cal[X_AXIS] += bed_div_x;
            if (this->cal[X_AXIS] > bed_x){
                this->cal[X_AXIS] = 0;
                this->cal[Y_AXIS] = 0;
            }
            else
                this->cal[Y_AXIS] = bed_y;
        }
    }
}


void ZHeightMapStrategy::setAdjustFunction(bool on)
{
    if(on) {
        // set the adjustZfnc in robot
        THEKERNEL->robot->adjustZfnc= [this](float x, float y) { return this->arm_bilinear_interp(x, y); };
    }else{
        // clear it
        THEKERNEL->robot->adjustZfnc= nullptr;
    }
}

// find the Z offset for the point on the plane at x, y
float ZHeightMapStrategy::getZOffset(float x, float y)
{
    //if(this->plane == nullptr) return NAN;
    return this->arm_bilinear_interp(x, y);
}

float ZHeightMapStrategy::arm_bilinear_interp(float X,
                                 float Y)
{
    int xIndex2, yIndex2;

    float xdiff = X / this->bed_div_x;
    float ydiff = Y / this->bed_div_y;

    float dCartX1, dCartX2;

    int xIndex = (int) xdiff;                  // Get the current sector (X)
    int yIndex = (int) ydiff;                  // Get the current sector (Y)

    // * Care taken for table outside boundary 
    // * Returns zero output when values are outside table boundary 
    if(xIndex < 0 || xIndex > (this->bed_level_data.numRows - 1) || yIndex < 0
       || yIndex > (this->bed_level_data.numCols - 1))
    {
      return (0);
    }

    if (xIndex == (this->bed_level_data.numRows - 1))
        xIndex2 = xIndex;
    else
        xIndex2 = xIndex+1;

    if (yIndex == (this->bed_level_data.numCols - 1))
        yIndex2 = yIndex;
    else
        yIndex2 = yIndex+1;
  
    xdiff -= xIndex;                    // Find floating point
    ydiff -= yIndex;                    // Find floating point

    dCartX1 = (1-xdiff) * this->bed_level_data.pData[(xIndex*this->bed_level_data.numCols)+yIndex] + (xdiff) * this->bed_level_data.pData[(xIndex2)*this->bed_level_data.numCols+yIndex];
    dCartX2 = (1-xdiff) * this->bed_level_data.pData[(xIndex*this->bed_level_data.numCols)+yIndex2] + (xdiff) * this->bed_level_data.pData[(xIndex2)*this->bed_level_data.numCols+yIndex2];

    return ydiff * dCartX2 + (1-ydiff) * dCartX1;    // Calculated Z-delta  

}


