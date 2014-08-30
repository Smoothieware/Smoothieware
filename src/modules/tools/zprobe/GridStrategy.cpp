/*
    Author: Quentin Harley (quentin.harley@gmail.com)
    License: GPL3 or better see <http://www.gnu.org/licenses/>

    Summary
    -------
    Probes twenty-five calculated points on the bed and creates grid data of the bed surface.
    Bilinear 
    As the head moves in X and Y it will adjust Z to keep the head tram with the bed.

    Configuration
    -------------
    The strategy must be enabled in the config as well as zprobe.

    leveling-strategy.grid-leveling.enable         true

    The bed size limits must be defined, in order for the module to calculate the calibration points

    leveling-strategy.grid-leveling.bedx           200
    leveling-strategy.grid-leveling.bedy           200

    Usage
    -----
    //TODO: G32 probes the twenty-five probe points and defines the bed grid, this will remain in effect until reset or M370
    //TODO: G31 reports the status

    M370 clears the grid and the bed leveling is disabled until G32 is run again
    M371 moves the head to the next calibration postion without saving
    M372 move the head to the next calibration postion after saving the current probe point

    M500 saves the probe points and the probe offsets
    M503 displays the current settings
*/

#include "GridStrategy.h"
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

#define bedx_checksum                CHECKSUM("bedx")
#define bedy_checksum                CHECKSUM("bedy")

GridStrategy::GridStrategy(ZProbe *zprobe) : LevelingStrategy(zprobe)
{
 /*   for (int i = 0; i < 3; ++i) {
        probe_points[i] = std::make_tuple(NAN, NAN);
    }
    plane = nullptr;*/
}

GridStrategy::~GridStrategy()
{
   // delete plane;
}

bool GridStrategy::handleConfig()
{
/*    // format is xxx,yyy for the probe points
    std::string p1 = THEKERNEL->config->value(leveling_strategy_checksum, grid_strategy_checksum, bedx_checksum)->by_default("")->as_string();
    std::string p2 = THEKERNEL->config->value(leveling_strategy_checksum, grid_strategy_checksum, bedy_checksum)->by_default("")->as_string();
    std::string p3 = THEKERNEL->config->value(leveling_strategy_checksum, grid_strategy_checksum, probe_point_3_checksum)->by_default("")->as_string();
    if(!p1.empty()) probe_points[0] = parseXY(p1.c_str());
    if(!p2.empty()) probe_points[1] = parseXY(p2.c_str());
    if(!p3.empty()) probe_points[2] = parseXY(p3.c_str());

    // Probe offsets xxx,yyy,zzz
    std::string po = THEKERNEL->config->value(leveling_strategy_checksum, three_point_leveling_strategy_checksum, probe_offsets_checksum)->by_default("0,0,0")->as_string();
    this->probe_offsets= parseXYZ(po.c_str());

    this->home= THEKERNEL->config->value(leveling_strategy_checksum, three_point_leveling_strategy_checksum, home_checksum)->by_default(true)->as_bool();
    this->tolerance= THEKERNEL->config->value(leveling_strategy_checksum, three_point_leveling_strategy_checksum, tolerance_checksum)->by_default(0.03F)->as_number();
    this->save= THEKERNEL->config->value(leveling_strategy_checksum, three_point_leveling_strategy_checksum, save_plane_checksum)->by_default(false)->as_bool();

*/
    return true;
}

bool GridStrategy::handleGcode(Gcode *gcode)
{
        if(gcode->has_m) {

            // manual bed grid calbration: M370 - M375
            // M370: Clear current grid for calibration, and move to first position
            case 370: {
                this->home();
                for (int i=0; i<25; i++){
                    this->bed_level_data.pData[i] = 0.0F;        // Clear the grid
                }

                this->cal[X_AXIS] = 0;                                              // Clear calibration position
                this->cal[Y_AXIS] = 0;
                this->in_cal = true;                                         // In calbration mode

            }
            return;
            // M371: Move to next initial position
            case 371: {
                if (in_cal){
                    this->move(this->cal, slow_rate);
                    this->next_cal();
                }

            }
            return;
            // M372: save current position in grid, and move to next initial position
            case 372: {
                if (in_cal){
                    float cartesian[3];
                    int pindex = 0;

                    THEKERNEL->robot->get_axis_position(cartesian);         // get actual position from robot
                    pindex = (int) ((cartesian[X_AXIS]/10.0F)+(cartesian[Y_AXIS]/50.0F));

                    this->move(this->cal, slow_rate);                       // move to the next position
                    this->next_cal();                                       // to not cause damage to machine due to Z-offset

                    THEKERNEL->robot->bed_level_data.pData[pindex] = 0.0F + cartesian[Z_AXIS];  // save the offset

                }                   
            }
            return;
            // M373: finalize calibration  
            case 373: {
                 this->in_cal = false;

            }
            return;
            // M374: manually inject calibration - Alphabetical grid assignment
            case 374: {
                int i=0,
                    x=0;
                
                if(gcode->has_letter('X')) { // Column nr (X)
                    x = gcode->get_value('X');
                }
                if (x<5){                    // Only for valid rows
                    for (i=0; i<5; i++){
                        if(gcode->has_letter('A'+i)) {
                            THEKERNEL->robot->bed_level_data.pData[i+(x*5)] = gcode->get_value('A'+i);
                        }
                    }
                }    
            }
            return;
            case 500: // M500 saves some volatile settings to config override file
            case 503:  // M503 just prints the settings

                // Bed grid data as gcode: Still not working right...
                gcode->stream->printf(";Bed Level settings:\n");
                int x,y;
                for (x=0; x<5; x++){
                    gcode->stream->printf("M374 X%i",x);
                    for (y=0; y<5; y++){
                         gcode->stream->printf(" %c%1.2f", 'A'+y, THEKERNEL->robot->bed_level_data.pData[(x*5)+y]);
                    }
                    gcode->stream->printf("\n"); 
                }  
                gcode->mark_as_taken();
                break;
            
            return;


  
        }
        #if 0
         else if(gcode->m == 999) {
            // DEBUG run a test M999 A B C X Y set Z to A B C and test for point at X Y
            Vector3 v[3];
            float x, y, z, a= 0, b= 0, c= 0;
            if(gcode->has_letter('A')) a = gcode->get_value('A');
            if(gcode->has_letter('B')) b = gcode->get_value('B');
            if(gcode->has_letter('C')) c = gcode->get_value('C');
            std::tie(x, y) = probe_points[0]; v[0].set(x, y, a);
            std::tie(x, y) = probe_points[1]; v[1].set(x, y, b);
            std::tie(x, y) = probe_points[2]; v[2].set(x, y, c);
            delete this->plane;
            this->plane = new Plane3D(v[0], v[1], v[2]);
            gcode->stream->printf("plane normal= %f, %f, %f\n", plane->getNormal()[0], plane->getNormal()[1], plane->getNormal()[2]);
            x= 0; y=0;
            if(gcode->has_letter('X')) x = gcode->get_value('X');
            if(gcode->has_letter('Y')) y = gcode->get_value('Y');
            z= getZOffset(x, y);
            gcode->stream->printf("z= %f\n", z);
            // tell robot to adjust z on each move
            setAdjustFunction(true);
            return true;
        }
        #endif
    }

    return false;
}

void GridStrategy::home()
{
    Gcode gc("G28", &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);
}

/*bool GridStrategy::doProbing(StreamOutput *stream)
{
    float x, y;
    // check the probe points have been defined
    for (int i = 0; i < 3; ++i) {
        std::tie(x, y) = probe_points[i];
        if(isnan(x) || isnan(y)) {
            stream->printf("Probe point P%d has not been defined, use M557 P%d Xnnn Ynnn to define it\n", i, i);
            return false;
        }
    }

    // optionally home XY axis first, but allow for manual homing
    if(this->home)
        home();

    // move to the first probe point
    std::tie(x, y) = probe_points[0];
    // offset by the probe XY offset
    x -= std::get<X_AXIS>(this->probe_offsets);
    y -= std::get<Y_AXIS>(this->probe_offsets);
    zprobe->coordinated_move(x, y, NAN, zprobe->getFastFeedrate());

    // for now we use probe to find bed and not the Z min endstop
    // the first probe point becomes Z == 0 effectively so if we home Z or manually set z after this, it needs to be at the first probe point

    // TODO this needs to be configurable to use min z or probe

    // find bed via probe
    int s;
    if(!zprobe->run_probe(s)) return false;

    // TODO if using probe then we probably need to set Z to 0 at first probe point, but take into account probe offset from head
    THEKERNEL->robot->reset_axis_position(std::get<Z_AXIS>(this->probe_offsets), Z_AXIS);

    // move up to specified probe start position
    zprobe->coordinated_move(NAN, NAN, zprobe->getProbeHeight(), zprobe->getSlowFeedrate()); // move to probe start position

    // probe the three points
    Vector3 v[3];
    for (int i = 0; i < 3; ++i) {
        std::tie(x, y) = probe_points[i];
        // offset moves by the probe XY offset
        float z = zprobe->probeDistance(x-std::get<X_AXIS>(this->probe_offsets), y-std::get<Y_AXIS>(this->probe_offsets));
        if(isnan(z)) return false; // probe failed
        z= zprobe->getProbeHeight() - z; // relative distance between the probe points, lower is negative z
        stream->printf("DEBUG: P%d:%1.4f\n", i, z);
        v[i].set(x, y, z);
    }

    // if first point is not within tolerance report it, it should ideally be 0
    if(abs(v[0][2]) > this->tolerance) {
        stream->printf("WARNING: probe is not within tolerance: %f > %f\n", abs(v[0][2]), this->tolerance);
    }

    // define the plane
    delete this->plane;
    // check tolerance level here default 0.03mm
    auto mm = std::minmax({v[0][2], v[1][2], v[2][2]});
    if((mm.second - mm.first) <= this->tolerance) {
        this->plane= nullptr; // plane is flat no need to do anything
        stream->printf("DEBUG: flat plane\n");
        // clear the adjustZfnc in robot
        setAdjustFunction(false);

    }else{
        this->plane = new Plane3D(v[0], v[1], v[2]);
        stream->printf("DEBUG: plane normal= %f, %f, %f\n", plane->getNormal()[0], plane->getNormal()[1], plane->getNormal()[2]);
        setAdjustFunction(true);
    }

    return true;
}*/

void GridStrategy::setAdjustFunction(bool on)
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
float GridStrategy::getZOffset(float x, float y)
{
    if(this->plane == nullptr) return NAN;
    return this->arm_bilinear_interp(x, y);
}

float GridStrategy::arm_bilinear_interp(float X,
                                 float Y)
{
    int xIndex, yIndex, xIndex2, yIndex2;
    float xdiff, ydiff;

    float dCartX1, dCartX2;

    xIndex = (int) X;                  // Get the current sector (X)
    yIndex = (int) Y;                  // Get the current sector (Y)

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
  
    xdiff = X - xIndex;                    // Find floating point
    ydiff = Y - yIndex;                    // Find floating point

    dCartX1 = (1-xdiff) * this->bed_level_data.pData[(xIndex*this->bed_level_data.numCols)+yIndex] + (xdiff) * this->bed_level_data.pData[(xIndex2)*this->bed_level_data.numCols+yIndex];
    dCartX2 = (1-xdiff) * this->bed_level_data.pData[(xIndex*this->bed_level_data.numCols)+yIndex2] + (xdiff) * this->bed_level_data.pData[(xIndex2)*this->bed_level_data.numCols+yIndex2];

    return ydiff * dCartX2 + (1-ydiff) * dCartX1;    // Calculated Z-delta  

}

/*
// parse a "X,Y" string return x,y
std::tuple<float, float> GridStrategy::parseXY(const char *str)
{
    float x = NAN, y = NAN;
    char *p;
    x = strtof(str, &p);
    if(p + 1 < str + strlen(str)) {
        y = strtof(p + 1, nullptr);
    }
    return std::make_tuple(x, y);
}

// parse a "X,Y,Z" string return x,y,z tuple
std::tuple<float, float, float> GridStrategy::parseXYZ(const char *str)
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
*/
