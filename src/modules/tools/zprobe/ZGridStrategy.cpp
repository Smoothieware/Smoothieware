/*
    Author: Quentin Harley (quentin.harley@gmail.com)
    License: GPL3 or better see <http://www.gnu.org/licenses/>

    Summary
    -------
    Probes user defined amount of calculated points on the bed and creates compensation grid data of the bed surface.
    Bilinear
    As the head moves in X and Y it will adjust Z to keep the head level with the bed.

    Configuration
    -------------
    The strategy must be enabled in the config as well as zprobe.

       leveling-strategy.ZGrid-leveling.enable         true

    The bed size limits must be defined, in order for the module to calculate the calibration points

       leveling-strategy.ZGrid-leveling.bed_x           200
       leveling-strategy.ZGrid-leveling.bed_y           200

    Machine height, used to determine probe attachment point (bed_z / 2)

       leveling-strategy.ZGrid-leveling.bed_z           20

    Configure for Machines with bed 0:0 at center of platform
       leveling-strategy.ZGrid-leveling.bed_zero        false

    configure for Machines with circular beds
       leveling-strategy.ZGrid-leveling.bed_circular    false


    The number of divisions for X and Y should be defined

       leveling-strategy.ZGrid-leveling.rows           7          # X divisions (Default 5)
       leveling-strategy.ZGrid-leveling.cols           9          # Y divisions (Default 5)


    The probe offset should be defined, default to zero offset

       leveling-strategy.ZGrid-leveling.probe_offsets  0,0,16.3

    The machine can be told to wait for probe attachment and confirmation

       leveling-strategy.ZGrid-leveling.wait_for_probe  true


    Slow feedrate can be defined for probe positioning speed.  Note this is not Probing slow rate - it can be set to a fast speed if required.

       leveling-strategy.ZGrid-leveling.slow_feedrate  100         # ZGrid probe positioning feedrate



    Usage
    -----
    G32 probes the probe points and defines the bed ZGrid, this will remain in effect until reset or M370
    G31 reports the status

    M370 clears the ZGrid and the bed leveling is disabled until G32 is run again
    M370 X7 Y9 allocates a new grid size of 7x9 and clears as above

    M371 moves the head to the next calibration postion without saving for manual calibration
    M372 move the head to the next calibration postion after saving the current probe point to memory - manual calbration

    M373 completes calibration and enables the Z compensation grid

    M

    M500 saves the probe points and the probe offsets
    M503 displays the current settings
*/

#include "ZGridStrategy.h"
#include "Kernel.h"
#include "Config.h"
#include "Robot.h"
#include "StreamOutputPool.h"
#include "Gcode.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "EndstopsPublicAccess.h"
#include "Conveyor.h"
#include "ZProbe.h"
#include "libs/FileStream.h"
#include "nuts_bolts.h"
#include "platform_memory.h"
#include "MemoryPool.h"
#include "libs/utils.h"

#include <string>
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <unistd.h>

#define bed_x_checksum               CHECKSUM("bed_x")
#define bed_y_checksum               CHECKSUM("bed_y")
#define bed_z_checksum               CHECKSUM("bed_z")

#define slow_feedrate_checksum       CHECKSUM("slow_feedrate")
#define probe_offsets_checksum       CHECKSUM("probe_offsets")
#define wait_for_probe_checksum      CHECKSUM("wait_for_probe")
#define center_zero_checksum         CHECKSUM("center_zero")
#define circular_bed_checksum        CHECKSUM("circular_bed")
#define cal_offset_x_checksum        CHECKSUM("cal_offset_x")
#define cal_offset_y_checksum        CHECKSUM("cal_offset_y")


#define cols_checksum                CHECKSUM("cols")
#define rows_checksum                CHECKSUM("rows")

#define probe_points                 (this->numRows * this->numCols)



ZGridStrategy::ZGridStrategy(ZProbe *zprobe) : LevelingStrategy(zprobe)
{
    this->cal[X_AXIS] = 0.0f;
    this->cal[Y_AXIS] = 0.0f;
    this->cal[Z_AXIS] = 30.0f;

    this->in_cal = false;
    this->pData = nullptr;
}

ZGridStrategy::~ZGridStrategy()
{
    // Free program memory for the pData grid
    if(this->pData != nullptr) AHB0.dealloc(this->pData);
}

bool ZGridStrategy::handleConfig()
{
    this->bed_x = THEKERNEL->config->value(leveling_strategy_checksum, ZGrid_leveling_checksum, bed_x_checksum)->by_default(200.0F)->as_number();
    this->bed_y = THEKERNEL->config->value(leveling_strategy_checksum, ZGrid_leveling_checksum, bed_y_checksum)->by_default(200.0F)->as_number();
    this->bed_z = THEKERNEL->config->value(leveling_strategy_checksum, ZGrid_leveling_checksum, bed_z_checksum)->by_default(20.0F)->as_number();

    this->slow_rate = THEKERNEL->config->value(leveling_strategy_checksum, ZGrid_leveling_checksum, slow_feedrate_checksum)->by_default(20.0F)->as_number();

    this->numRows = THEKERNEL->config->value(leveling_strategy_checksum, ZGrid_leveling_checksum, rows_checksum)->by_default(5)->as_number();
    this->numCols = THEKERNEL->config->value(leveling_strategy_checksum, ZGrid_leveling_checksum, cols_checksum)->by_default(5)->as_number();

    this->wait_for_probe = THEKERNEL->config->value(leveling_strategy_checksum, ZGrid_leveling_checksum, wait_for_probe_checksum)->by_default(false)->as_bool();

    this->center_zero = THEKERNEL->config->value(leveling_strategy_checksum, ZGrid_leveling_checksum, center_zero_checksum)->by_default(false)->as_bool();
    this->circular_bed = THEKERNEL->config->value(leveling_strategy_checksum, ZGrid_leveling_checksum, circular_bed_checksum)->by_default(false)->as_bool();

    // configures calbration positioning offset.  Defaults to 0 for standard cartesian space machines, and to negative half of the current bed size in X and Y
    this->cal_offset_x = THEKERNEL->config->value(leveling_strategy_checksum, ZGrid_leveling_checksum, cal_offset_x_checksum)->by_default( this->center_zero ? this->bed_x / -2.0F : 0.0F )->as_number();
    this->cal_offset_y = THEKERNEL->config->value(leveling_strategy_checksum, ZGrid_leveling_checksum, cal_offset_y_checksum)->by_default( this->center_zero ? this->bed_y / -2.0F : 0.0F )->as_number();


    // Probe offsets xxx,yyy,zzz
    std::string po = THEKERNEL->config->value(leveling_strategy_checksum, ZGrid_leveling_checksum, probe_offsets_checksum)->by_default("0,0,0")->as_string();
    this->probe_offsets= parseXYZ(po.c_str());

    this->calcConfig();                // Run calculations for Grid size and allocate initial grid memory

    for (int i=0; i<(probe_points); i++){
        this->pData[i] = 0.0F;        // Clear the grid
    }

    return true;
}

void ZGridStrategy::calcConfig()
{
    this->bed_div_x = this->bed_x / float(this->numRows-1);    // Find divisors to calculate the calbration points
    this->bed_div_y = this->bed_y / float(this->numCols-1);

    // Ensure free program memory for the pData grid
    if(this->pData != nullptr) AHB0.dealloc(this->pData);

    // Allocate program memory for the pData grid
    this->pData = (float *)AHB0.alloc(probe_points * sizeof(float));
}

bool ZGridStrategy::handleGcode(Gcode *gcode)
{
    string args = get_arguments(gcode->get_command());

     // G code processing
    if(gcode->has_g) {
        if( gcode->g == 31 ) { // report status

               // Bed ZGrid data as gcode:
                gcode->stream->printf(";Bed Level settings:\r\n");

                for (int x=0; x<this->numRows; x++){
                    gcode->stream->printf("X%i",x);
                    for (int y=0; y<this->numCols; y++){
                         gcode->stream->printf(" %c%1.2f", 'A'+y, this->pData[(x*this->numCols)+y]);
                    }
                    gcode->stream->printf("\r\n");
                }
            return true;

        } else if( gcode->g == 32 ) { //run probe
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

            // manual bed ZGrid calbration: M370 - M375
            // M370: Clear current ZGrid for calibration, and move to first position
            case 370: {
                this->setAdjustFunction(false); // Disable leveling code

                if(gcode->has_letter('X'))  // Rows (X)
                    this->numRows = gcode->get_value('X');
                if(gcode->has_letter('Y'))  // Cols (Y)
                    this->numCols = gcode->get_value('Y');

                this->calcConfig();                // Run calculations for Grid size and allocate grid memory

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
            // M372: save current position in ZGrid, and move to next calibration position
            case 372: {
                if (in_cal){
                    float cartesian[3];
                    int pindex = 0;

                    THEKERNEL->robot->get_axis_position(cartesian);         // get actual position from robot

                    pindex = int(cartesian[X_AXIS]/this->bed_div_x + 0.25)*this->numCols + int(cartesian[Y_AXIS]/this->bed_div_y + 0.25);

                    this->move(this->cal, slow_rate);                       // move to the next position
                    this->next_cal();                                       // to not cause damage to machine due to Z-offset

                    this->pData[pindex] = cartesian[Z_AXIS];  // save the offset

                }
            }
            return true;
            // M373: finalize calibration
            case 373: {
                 // normalize the grid
                 this->normalize_grid();

                 this->in_cal = false;
                 this->setAdjustFunction(true); // Enable leveling code

            }
            return true;

            // M374: Save grid
            case 374:{
                char gridname[5];

                if(gcode->has_letter('S'))  // Custom grid number
                    snprintf(gridname, sizeof(gridname), "S%03.0f", gcode->get_value('S'));
                else
                    gridname[0] = '\0';

                if(this->saveGrid(gridname)) {
                    gcode->stream->printf("Grid saved: Filename: /sd/Zgrid.%s\n",gridname);
                }
                else {
                    gcode->stream->printf("Error: Grid not saved: Filename: /sd/Zgrid.%s\n",gridname);
                }
            }
            return true;

            case 375:{ // Load grid values
                char gridname[5];

                if(gcode->has_letter('S'))  // Custom grid number
                    snprintf(gridname, sizeof(gridname), "S%03.0f", gcode->get_value('S'));
                else
                    gridname[0] = '\0';

                if(this->loadGrid(gridname)) {
                    this->setAdjustFunction(true); // Enable leveling code
                    gcode->stream->printf("Grid loaded: /sd/Zgrid.%s\n",gridname);
                }
                else {
                    gcode->stream->printf("Error: Grid not loaded: /sd/Zgrid.%s\n",gridname);
                }
            }
            return true;

/*          case 376: { // Check grid value calculations: For debug only.
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

            case 500: // M500 saves probe_offsets config override file
                gcode->stream->printf(";Load default grid\nM375\n");


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


bool ZGridStrategy::saveGrid(std::string args)
{
    args = "/sd/Zgrid." + args;
    StreamOutput *ZMap_file = new FileStream(args.c_str());

    ZMap_file->printf("P%i %i %i %1.3f\n", probe_points, this->numRows, this->numCols, getZhomeoffset());    // Store probe points to prevent loading undefined grid files

    for (int pos = 0; pos < probe_points; pos++){
        ZMap_file->printf("%1.3f\n", this->pData[pos]);
    }
    delete ZMap_file;

    return true;

}

bool ZGridStrategy::loadGrid(std::string args)
{
    char flag[20];

    int fpoints, GridX = 5, GridY = 5;   // for 25point file
    float val, GridZ;

    args = "/sd/Zgrid." + args;
    FILE *fd = fopen(args.c_str(), "r");
    if(fd != NULL) {
        fscanf(fd, "%s\n", flag);

        if (flag[0] == 'P'){

            sscanf(flag, "P%i\n", &fpoints);                        // read number of points, and Grid X and Y
            fscanf(fd, "%i %i %f\n", &GridX, &GridY, &GridZ);       // read number of points, and Grid X and Y and ZHoming offset
            fscanf(fd, "%f\n", &val);                               // read first value from file

        } else {  // original 25point file -- Backwards compatibility
            fpoints = 25;
            sscanf(flag, "%f\n", &val);                             // read first value from string
        }

        if (GridX != this->numRows || GridY != this->numCols){
            this->numRows = GridX;                                  // Change Rows and Columns to match the saved data
            this->numCols = GridY;
            this->calcConfig();                                     // Reallocate memory for the grid according to the grid loaded
        }

        this->pData[0] = val;    // Place the first read value in grid

        for (int pos = 1; pos < probe_points; pos++){
            fscanf(fd, "%f\n", &val);
            this->pData[pos] = val;
        }

        fclose(fd);

        this->setZoffset(GridZ);

        return true;

    } else {
        return false;
    }

}

float ZGridStrategy::getZhomeoffset()
{
    void* rd;

    bool ok = PublicData::get_value( endstops_checksum, home_offset_checksum, &rd );

    if (ok) {
      return ((float*)rd)[2];
    }

    return 0;
}

void ZGridStrategy::setZoffset(float zval)
{
    char cmd[64];

    // Assemble Gcode to add onto the queue
    snprintf(cmd, sizeof(cmd), "M206 Z%1.3f", zval); // Send saved Z homing offset

    Gcode gc(cmd, &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);

}

bool ZGridStrategy::doProbing(StreamOutput *stream)  // probed calibration
{
    // home first
    this->homexyz();

    // deactivate correction during moves
    this->setAdjustFunction(false);

    for (int i=0; i<probe_points; i++){
       this->pData[i] = 0.0F;        // Clear the ZGrid
    }

    if (this->wait_for_probe){

        this->cal[X_AXIS] = this->bed_x/2.0f;
        this->cal[Y_AXIS] = this->bed_y/2.0f;
        this->cal[Z_AXIS] = this->bed_z/2.0f;           // Position head for probe attachment
        this->move(this->cal, slow_rate);               // Move to probe attachment point

        stream->printf("*** Ensure probe is attached and press probe when done ***\n");

        while(!zprobe->getProbeStatus()){            // Wait for button press
            THEKERNEL->call_event(ON_IDLE);
        };
    }

    this->in_cal = true;                         // In calbration mode

    this->cal[X_AXIS] = 0.0f;                    // Clear calibration position
    this->cal[Y_AXIS] = 0.0f;
    this->cal[Z_AXIS] = std::get<Z_AXIS>(this->probe_offsets) + zprobe->getProbeHeight();

    this->move(this->cal, slow_rate);            // Move to probe start point

    for (int probes = 0; probes < probe_points; probes++){
        int pindex = 0;

        float z = 5.0f - zprobe->probeDistance((this->cal[X_AXIS] + this->cal_offset_x)-std::get<X_AXIS>(this->probe_offsets),
                                               (this->cal[Y_AXIS] + this->cal_offset_y)-std::get<Y_AXIS>(this->probe_offsets));

        pindex = int(this->cal[X_AXIS]/this->bed_div_x + 0.25)*this->numCols + int(this->cal[Y_AXIS]/this->bed_div_y + 0.25);

        if (probes == (probe_points-1) && this->wait_for_probe){  // Only move to removal position if probe confirmation was selected
            this->cal[X_AXIS] = this->bed_x/2.0f;                 // Else machine will return to first probe position when done
            this->cal[Y_AXIS] = this->bed_y/2.0f;
            this->cal[Z_AXIS] = this->bed_z/2.0f;                 // Position head for probe removal
        } else {
            this->next_cal();                                     // to not cause damage to machine due to Z-offset
        }
        this->move(this->cal, slow_rate);                         // move to the next position

        this->pData[pindex] = z ;                                 // save the offset
    }

    stream->printf("\nCalibration done.  Please remove probe\n");

    // activate correction
    this->normalize_grid();
    this->setAdjustFunction(true);

    this->in_cal = false;

    return true;
}


void ZGridStrategy::normalize_grid()
{
    float min = 100.0F,    // set large start value
          norm_offset = 0;

    // find minimum value in offset grid
    for (int i = 0; i < probe_points; i++)
        if (this->pData[i] < min)
          min = this->pData[i];

    // creates addition offset to set minimum value to zero.
    norm_offset = -min;

    // adds the offset to create a table of deltas, normalzed to minimum zero
    for (int i = 0; i < probe_points; i++)
        this->pData[i] += norm_offset;

   // add the offset to the current Z homing offset to preserve full probed offset.
   this->setZoffset(getZhomeoffset() + norm_offset);
}

void ZGridStrategy::homexyz()
{
    Gcode gc("G28", &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);
}

void ZGridStrategy::move(float *position, float feed)
{
    char cmd[64];

    // Assemble Gcode to add onto the queue.  Also translate the position for non standard cartesian spaces (cal_offset)
    snprintf(cmd, sizeof(cmd), "G0 X%1.3f Y%1.3f Z%1.3f F%1.1f", position[0] + this->cal_offset_x, position[1] + this->cal_offset_y, position[2], feed * 60); // use specified feedrate (mm/sec)

    THEKERNEL->streams->printf("DEBUG: move: %s cent: %i\n", cmd, this->center_zero);

    Gcode gc(cmd, &(StreamOutput::NullStream));
    THEKERNEL->robot->on_gcode_received(&gc); // send to robot directly
}


void ZGridStrategy::next_cal(void){
    if (int(this->cal[X_AXIS] / this->bed_div_x) % 2 != 0){  // Odd row
        this->cal[Y_AXIS] -= this->bed_div_y;
        if (this->cal[Y_AXIS] < 0.0F){

            THEKERNEL->streams->printf("DEBUG: Y (%f) < cond (%f)\n",this->cal[Y_AXIS], 0.0F);

            this->cal[X_AXIS] += bed_div_x;
            if (this->cal[X_AXIS] > this->bed_x){
                this->cal[X_AXIS] = 0.0F;
                this->cal[Y_AXIS] = 0.0F;
            }
            else
                this->cal[Y_AXIS] = 0.0F;
        }
    }
    else {                                          // Even row (0 is an even row - starting point)
        this->cal[Y_AXIS] += bed_div_y;
      if (this->cal[Y_AXIS] > this->bed_y){

            THEKERNEL->streams->printf("DEBUG: Y (%f) > cond (%f)\n",this->cal[Y_AXIS], this->bed_y);

            this->cal[X_AXIS] += bed_div_x;
            if (this->cal[X_AXIS] > bed_x){
                this->cal[X_AXIS] = 0.0F;
                this->cal[Y_AXIS] = 0.0F;
            }
            else
                this->cal[Y_AXIS] = this->bed_y;
        }
    }
}


void ZGridStrategy::setAdjustFunction(bool on)
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
float ZGridStrategy::getZOffset(float X, float Y)
{
    int xIndex2, yIndex2;

    // Subtract calibration offsets as applicable
    X -= this->cal_offset_x;
    Y -= this->cal_offset_y;

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
std::tuple<float, float, float> ZGridStrategy::parseXYZ(const char *str)
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

