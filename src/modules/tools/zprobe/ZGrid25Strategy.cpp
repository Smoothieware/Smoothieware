/*
    Author: Quentin Harley (quentin.harley@gmail.com)
    License: GPL3 or better see <http://www.gnu.org/licenses/>

    Summary
    -------
    Probes twenty-five calculated points on the bed and creates ZGrid25 data of the bed surface.
    Bilinear 
    As the head moves in X and Y it will adjust Z to keep the head tram with the bed.

    Configuration
    -------------
    The strategy must be enabled in the config as well as zprobe.

    leveling-strategy.ZGrid25-leveling.enable         true

    The bed size limits must be defined, in order for the module to calculate the calibration points

    leveling-strategy.ZGrid25-leveling.bedx           200
    leveling-strategy.ZGrid25-leveling.bedy           200


    The number of divisions for X and Y should be defined

    leveling-strategy.ZGrid25-leveling.rows           7          # X divisions (Default 5)
    leveling-strategy.ZGrid25-leveling.cols           9          # Y divisions (Default 5)


    The probe offset should be defined, default to zero offset

    leveling-strategy.ZGrid25-leveling.probe_offsets  0,0,16.3


    slow feedrate can be defined for probe speed

    leveling-strategy.ZGrid25-leveling.slow_feedrate  100


   
    Usage
    -----
    G32 probes the probe points and defines the bed ZGrid25, this will remain in effect until reset or M370
    G31 reports the status

    M370 clears the ZGrid25 and the bed leveling is disabled until G32 is run again
    M371 moves the head to the next calibration postion without saving
    M372 move the head to the next calibration postion after saving the current probe point to memory
   
    M373 completes calibration and enables the grid

    M

    M500 saves the probe points and the probe offsets
    M503 displays the current settings
*/

#include "ZGrid25Strategy.h"
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
#include "libs/FileStream.h"
#include "nuts_bolts.h"
#include "platform_memory.h"
#include "MemoryPool.h"
//#include "new.h"

#include <string>
#include <algorithm>
#include <cstdlib>
#include <cmath>

#define bed_x_checksum                CHECKSUM("bed_x")
#define bed_y_checksum                CHECKSUM("bed_y")
#define bed_z_checksum                CHECKSUM("bed_z")

#define slow_feedrate_checksum       CHECKSUM("slow_feedrate")
#define probe_offsets_checksum       CHECKSUM("probe_offsets")

#define cols_checksum                CHECKSUM("cols")
#define rows_checksum                CHECKSUM("rows")

#define probe_points                 (this->numRows * this->numCols)



ZGrid25Strategy::ZGrid25Strategy(ZProbe *zprobe) : LevelingStrategy(zprobe)
{
    this->cal[X_AXIS] = 0.0f;
    this->cal[Y_AXIS] = 0.0f;
    this->cal[Z_AXIS] = 30.0f;

    this->in_cal = false;
}

ZGrid25Strategy::~ZGrid25Strategy()
{
    // Free program memory for the pData grid
    if(this->pData != nullptr) AHB0.dealloc(this->pData);
}

bool ZGrid25Strategy::handleConfig()
{
    this->bed_x = THEKERNEL->config->value(leveling_strategy_checksum, ZGrid25_leveling_checksum, bed_x_checksum)->by_default(200.0F)->as_number();
    this->bed_y = THEKERNEL->config->value(leveling_strategy_checksum, ZGrid25_leveling_checksum, bed_y_checksum)->by_default(200.0F)->as_number();
    this->bed_z = THEKERNEL->config->value(leveling_strategy_checksum, ZGrid25_leveling_checksum, bed_z_checksum)->by_default(200.0F)->as_number();

    this->slow_rate = THEKERNEL->config->value(leveling_strategy_checksum, ZGrid25_leveling_checksum, slow_feedrate_checksum)->by_default(20.0F)->as_number();

    this->numRows = THEKERNEL->config->value(leveling_strategy_checksum, ZGrid25_leveling_checksum, rows_checksum)->by_default(5)->as_number();
    this->numCols = THEKERNEL->config->value(leveling_strategy_checksum, ZGrid25_leveling_checksum, cols_checksum)->by_default(5)->as_number();
    
    this->bed_div_x = this->bed_x / float(this->numRows-1);    // Find divisors to find the calbration points
    this->bed_div_y = this->bed_y / float(this->numCols-1);

    // Allocate program memory for the pData grid
    this->pData = (float *)AHB0.alloc(probe_points * sizeof(float));

    // Probe offsets xxx,yyy,zzz
    std::string po = THEKERNEL->config->value(leveling_strategy_checksum, ZGrid25_leveling_checksum, probe_offsets_checksum)->by_default("0,0,0")->as_string();
    this->probe_offsets= parseXYZ(po.c_str());


    int i;
    for (i=0; i<(probe_points); i++){
        this->pData[i] = 0.0F;        // Clear the grid
    }

    if(this->loadGrid()){
        this->setAdjustFunction(true); // Enable leveling code
    }

    return true;
}

bool ZGrid25Strategy::handleGcode(Gcode *gcode)
{
     // G code processing
    if(gcode->has_g) {
        if( gcode->g == 31 ) { // report status

               // Bed ZGrid25 data as gcode:
                gcode->stream->printf(";Bed Level settings:\r\n");
                
                for (int x=0; x<this->numRows; x++){
                    int y;
                    
                    gcode->stream->printf("X%i",x);
                    for (y=0; y<this->numCols; y++){
                         gcode->stream->printf(" %c%1.2f", 'A'+y, this->pData[(x*this->numCols)+y]);
                    }
                    gcode->stream->printf("\r\n"); 
                }
            return true;

        } else if( gcode->g == 32 ) { // three point probe
            // first wait for an empty queue i.e. no moves left
            THEKERNEL->conveyor->wait_for_empty_queue();

            this->setAdjustFunction(false); // Disable leveling code
            if(!doProbing(gcode->stream)) {
                gcode->stream->printf("Probe failed to complete, probe not triggered or other error\n");
            } else {
                this->setAdjustFunction(true); // Enable leveling code
                gcode->stream->printf("Probe completed, bed grid defined\n");
            }
            return true;
        }

    } else if(gcode->has_m) {
        switch( gcode->m ) {

            // manual bed ZGrid25 calbration: M370 - M375
            // M370: Clear current ZGrid25 for calibration, and move to first position
            case 370: {
                this->setAdjustFunction(false); // Disable leveling code

                this->homexyz();
                for (int i=0; i<probe_points; i++){
                    this->pData[i] = 0.0F;        // Clear the ZGrid
                }

                this->cal[X_AXIS] = 0.0f;                                              // Clear calibration position
                this->cal[Y_AXIS] = 0.0f;
                this->in_cal = true;                                         // In calbration mode
                
            }
            return true;
            // M371: Move to next manual calibration position
            case 371: {
                if (in_cal){
                    this->move(this->cal, slow_rate);
                    this->next_cal();
                }

            }
            return true;
            // M372: save current position in ZGrid25, and move to next calibration position
            case 372: {
                if (in_cal){
                    float cartesian[3];
                    int pindex = 0;

                    THEKERNEL->robot->get_axis_position(cartesian);         // get actual position from robot
                    pindex = (int) ((cartesian[X_AXIS]/this->bed_div_x*numRows)+(cartesian[Y_AXIS]/this->bed_div_y));

                    this->move(this->cal, slow_rate);                       // move to the next position
                    this->next_cal();                                       // to not cause damage to machine due to Z-offset

                    this->pData[pindex] = 0.0F + cartesian[Z_AXIS];  // save the offset

                }                   
            }
            return true;
            // M373: finalize calibration  
            case 373: {
                 this->in_cal = false;
                 this->setAdjustFunction(true); // Enable leveling code   

            }
            return true;
 
/*           // M374: manually inject calibration - Alphabetical ZGrid25 assignment  TODO - debug only
            case 374: {
                int i=0,
                    x=0;
                
                if(gcode->has_letter('X')) { // Column nr (X)
                    x = gcode->get_value('X');
                }
                if (x<5){                    // Only for valid rows
                    for (i=0; i<5; i++){
                        if(gcode->has_letter('A'+i)) {
                            this->pData[i+(x*5)] = gcode->get_value('A'+i);
                        }
                    }
                }
                this->setAdjustFunction(true); // Enable leveling code   
            }
            return true;

            case 376: { // Check grid value calculations: TODO - For debug only.
                float target[3];

                for(char letter = 'X'; letter <= 'Z'; letter++) {
                    if( gcode->has_letter(letter) ) {
                         target[letter - 'X'] = gcode->get_value(letter);
                    }
                }
                gcode->stream->printf(" Z0 %1.3f\n",getZOffset(target[0], target[1]));
                
            }
            return true;
*/
            case 565: { // M565: Set Z probe offsets
                float x= 0, y= 0, z= 0;
                if(gcode->has_letter('X')) x = gcode->get_value('X');
                if(gcode->has_letter('Y')) y = gcode->get_value('Y');
                if(gcode->has_letter('Z')) z = gcode->get_value('Z');
                probe_offsets = std::make_tuple(x, y, z);
            }
            return true;

            case 501: // Load grid values
                if(this->loadGrid()){
                    this->setAdjustFunction(true); // Enable leveling code
                }
            return true;

            case 500: // M500 saves some volatile settings to config override file

                this->saveGrid();  // TODO: Move this line to a new gcode for saving grids

            case 503: { // M503 just prints the settings

                float x,y,z;
                gcode->stream->printf(";Probe offsets:\n");
                std::tie(x, y, z) = probe_offsets;
                gcode->stream->printf("M565 X%1.5f Y%1.5f Z%1.5f\n", x, y, z);

                gcode->mark_as_taken();
                break;
            }
            
            return true;
        }
    }

    return false;
}

bool ZGrid25Strategy::saveGrid()
{
    StreamOutput *ZMap_file = new FileStream("/sd/grid25");

    ZMap_file->printf("P%i\n",probe_points);    // Store probe points to prevent loading undefined grid files

    for (int pos = 0; pos < probe_points; pos++){
        ZMap_file->printf("%1.3f\n", this->pData[pos]);
    }
    delete ZMap_file;

    return true;

}

bool ZGrid25Strategy::loadGrid()
{
    char flag[10];	
    int fpoints;
    float val;

    FILE *fd = fopen("/sd/grid25", "r");
    if(fd != NULL) {
        fscanf(fd, "%s\n", flag);
        if (flag[0] == 'P'){
            sscanf(flag, "P%i\n", &fpoints);    // read number of points
            fscanf(fd, "%f\n", &val);          // read first value from file
            
        } else {  // original 25point file
            fpoints = 25;
            sscanf(flag, "%f\n", &val);     // read first value from string
        }

        if (fpoints == probe_points){  // Only read file if grid points match up with file

            this->pData[0] = val;    // Place the first read value in grid
             
            for (int pos = 1; pos < probe_points; pos++){
                fscanf(fd, "%f\n", &val);
                this->pData[pos] = val;
            }

        }


        fclose(fd);

        return true;

    } else {
        return false;
    }

}

bool ZGrid25Strategy::doProbing(StreamOutput *stream)  // probed calibration
{
    // home first
    this->homexyz();

    // deactivate correction during moves
    this->setAdjustFunction(false);

    for (int i=0; i<probe_points; i++){
       this->pData[i] = 0.0F;        // Clear the ZGrid
    }

    stream->printf("*** Ensure probe is attached and press probe when done ***\n");

    this->cal[X_AXIS] = this->bed_x/2.0f;           // Clear calibration position
    this->cal[Y_AXIS] = this->bed_y/2.0f;
    this->cal[Z_AXIS] = this->bed_z/2.0f;           // Position head for probe attachment
    this->move(this->cal, slow_rate);               // Move to probe attachment point

    stream->printf("*** Ensure probe is attached and press probe when done ***\n");

    while(!zprobe->getProbeStatus());// || secwait < 10 )

    this->in_cal = true;                         // In calbration mode
    
    this->cal[X_AXIS] = 0.0f;                    // Clear calibration position
    this->cal[Y_AXIS] = 0.0f;
    this->cal[Z_AXIS] = std::get<Z_AXIS>(this->probe_offsets) + 5.0f;
         
    this->move(this->cal, slow_rate);            // Move to probe start point

    for (int probes = 0; probes <probe_points; probes++){
        int pindex = 0;

        float z = 5.0f - zprobe->probeDistance(this->cal[X_AXIS]-std::get<X_AXIS>(this->probe_offsets),
                                       this->cal[Y_AXIS]-std::get<Y_AXIS>(this->probe_offsets));

        pindex = (int) ((this->cal[X_AXIS]/this->bed_div_x*numRows)+(this->cal[Y_AXIS]/this->bed_div_y));

        if (probes == (probe_points-1)){
            this->cal[X_AXIS] = this->bed_x/2.0f;           // Clear calibration position
            this->cal[Y_AXIS] = this->bed_y/2.0f;
            this->cal[Z_AXIS] = this->bed_z/2.0f;                     // Position head for probe removal
        } else {
            this->next_cal();                        // to not cause damage to machine due to Z-offset
        }
        this->move(this->cal, slow_rate);        // move to the next position
        
        this->pData[pindex]                      // set offset
          = z ;                                  // save the offset
    }
    
    stream->printf("\nDone!  Please remove probe\n");
   
    // activate correction
    this->setAdjustFunction(true);
              
    this->in_cal = false;
    return true;
}


void ZGrid25Strategy::homexyz()
{
    Gcode gc("G28", &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);
}

void ZGrid25Strategy::move(float *position, float feed)
{
    char cmd[64];

    // Assemble Gcode to add onto the queue
    snprintf(cmd, sizeof(cmd), "G0 X%1.3f Y%1.3f Z%1.3f F%1.1f", position[0], position[1], position[2], feed * 60); // use specified feedrate (mm/sec)

    //THEKERNEL->streams->printf("DEBUG: move: %s\n", cmd);

    Gcode gc(cmd, &(StreamOutput::NullStream));
    THEKERNEL->robot->on_gcode_received(&gc); // send to robot directly
}


void ZGrid25Strategy::next_cal(void){
    if (int(this->cal[X_AXIS] / bed_div_x) % 2 != 0){  // Odd row
        this->cal[Y_AXIS] -= bed_div_y;
        if (this->cal[Y_AXIS] < 0){
            this->cal[Y_AXIS] = 0;
            this->cal[X_AXIS] += bed_div_x;
        }
    }
    else {                                          // Even row
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


void ZGrid25Strategy::setAdjustFunction(bool on)
{
    if(on) {
        // set the compensationTransform in robot
        THEKERNEL->robot->compensationTransform= [this](float target[3]) { target[2] += this->getZOffset(target[0], target[1]); };
    }else{
        // clear it
        THEKERNEL->robot->compensationTransform= nullptr;
    }
}


// find the Z offset for the point on the plane at x, y
float ZGrid25Strategy::getZOffset(float X, float Y)
{
    int xIndex2, yIndex2;

    float xdiff = X / this->bed_div_x;
    float ydiff = Y / this->bed_div_y;

    float dCartX1, dCartX2;

    int xIndex = (int) xdiff;                  // Get the current sector (X)
    int yIndex = (int) ydiff;                  // Get the current sector (Y)

    // * Care taken for table outside boundary 
    // * Returns zero output when values are outside table boundary 
    if(xIndex < 0 || xIndex > (this->numRows - 1) || yIndex < 0
       || yIndex > (this->numCols - 1))
    {
      return (0);
    }

    if (xIndex == (this->numRows - 1))
        xIndex2 = xIndex;
    else
        xIndex2 = xIndex+1;

    if (yIndex == (this->numCols - 1))
        yIndex2 = yIndex;
    else
        yIndex2 = yIndex+1;
  
    xdiff -= xIndex;                    // Find floating point
    ydiff -= yIndex;                    // Find floating point

    dCartX1 = (1-xdiff) * this->pData[(xIndex*this->numCols)+yIndex] + (xdiff) * this->pData[(xIndex2)*this->numCols+yIndex];
    dCartX2 = (1-xdiff) * this->pData[(xIndex*this->numCols)+yIndex2] + (xdiff) * this->pData[(xIndex2)*this->numCols+yIndex2];

    return ydiff * dCartX2 + (1-ydiff) * dCartX1;    // Calculated Z-delta  

}

// parse a "X,Y,Z" string return x,y,z tuple
std::tuple<float, float, float> ZGrid25Strategy::parseXYZ(const char *str)
{
    float x = 0, y = 0, z= 0;
    char *p;
    x = strtof(str, &p);
    if(p + 1 < str + strlen(str)) {
        y = strtof(p + 1, &p);
        if(p + 1 < str + strlen(str)) {
            z = strtof(p + 1, nullptr);
        }
    }
    return std::make_tuple(x, y, z);
}

