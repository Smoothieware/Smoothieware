/*
	This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
	Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
	Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
	You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Kernel.h"
#include "Iremitter.h"
#include "checksumm.h"
#include "Config.h"
#include "ConfigValue.h"
#include "Gcode.h"
#include "Conveyor.h"
#include "StreamOutputPool.h"

// config names
#define iremitter_checksum CHECKSUM("iremitter")
#define enable_checksum    CHECKSUM("enable")
#define mcode_checksum     CHECKSUM("mcode")
#define pin_checksum       CHECKSUM("pin")
#define camera_checksum    CHECKSUM("camera")

Iremitter::Iremitter() {}

void Iremitter::on_module_loaded()
{
	// if the module is disabled -> do nothing
	if(! THEKERNEL->config->value(iremitter_checksum, enable_checksum)->by_default(false)->as_bool()) {
		// as this module is not needed free up the resource
		delete this;
		return;
	}

	// settings
	this->on_config_reload(this);

	// events
	this->register_for_event(ON_GCODE_RECEIVED);
	this->register_for_event(ON_GCODE_EXECUTE);
}

void Iremitter::on_config_reload(void *argument)
{
	this->mcode = THEKERNEL->config->value(iremitter_checksum, mcode_checksum)->by_default(0)->as_int();
	this->pin.from_string(THEKERNEL->config->value(iremitter_checksum, pin_checksum)->by_default("nc")->as_string())->as_output();
	string camera = THEKERNEL->config->value(iremitter_checksum, camera_checksum)->by_default("nikon")->as_string();
	
	if      (camera == "canon")        this->camera = CANON;
	else if (camera == "canonwldc100") this->camera = CANONWLDC100;
	else if (camera == "minolta")      this->camera = MINOLTA;
	else if (camera == "nikon")        this->camera = NIKON;
	else if (camera == "olympus")      this->camera = OLYMPUS;
	else if (camera == "pentax")       this->camera = PENTAX;
	else if (camera == "sony")         this->camera = SONY;
	else 
	{
		THEKERNEL->streams->printf("iremitter: camera not found, 'Nikon' set by default.\n");
		this->camera = NIKON;
	}
}

bool Iremitter::is_executable(Gcode *gcode)
{
	// return true if all condition for execution is satisfied...
	return this->pin.connected() and this->mcode > 0 and gcode->has_m and gcode->m == this->mcode;
}

void Iremitter::on_gcode_received(void *argument)
{
	// received gcode
	Gcode *gcode = static_cast<Gcode *>(argument);

	// if executable, append to the queue
	if (this->is_executable(gcode))
		THEKERNEL->conveyor->append_gcode(gcode);
}

void Iremitter::on_gcode_execute(void *argument)
{
	// executed gcode
	Gcode *gcode = static_cast<Gcode *>(argument);

	// exit if it is not executable
	if (! this->is_executable(gcode)) 
		return;

	// take a shot
	this->trigger();
}

/*
	original code from Sébastian Setz at :
	http://sebastian.setz.name/arduino/my-libraries/multi-camera-iremitter-control/
*/

// pwm at freqency for x microseconds
void Iremitter::high_us(unsigned int us, int freq)
{
	uint32_t start = us_ticker_read();
	int pause = (1000 / freq / 2) - 4;

	while((us_ticker_read() - start) < us)
	{
		this->pin.set(true);
		wait_us(pause);
		this->pin.set(false);
		wait_us(pause);
	}
}

// trigger the ir code
void Iremitter::trigger()
{
  	//THEKERNEL->streams->printf("iremitter: nikon trigger!\n");

	if      (this->camera == CANON)        this->canon_trigger();
	else if (this->camera == CANONWLDC100) this->canonwldc100_trigger();
	else if (this->camera == MINOLTA)      this->minolta_trigger();
	else if (this->camera == NIKON)        this->nikon_trigger();
	else if (this->camera == OLYMPUS)      this->olympus_trigger();
	else if (this->camera == PENTAX)       this->pentax_trigger();
	else if (this->camera == SONY)         this->sony_trigger();
}

void Iremitter::canon_trigger()
{
	// sequence
	for(int i = 0; i < 16; i++) 
	{ 
		this->pin.set(true);
		wait_us(11);
		this->pin.set(false);
		wait_us(11);
	} 

	wait_us(7330); 

	for(int i = 0; i < 16; i++) 
	{ 
		this->pin.set(true);
		wait_us(11);
		this->pin.set(false);
		wait_us(11);
	}
}

void Iremitter::canonwldc100_trigger()
{
	// frequency
	int f = 0;

	// sequence
	bool s[] = {0,1,0,1,0,0,1,1,1,0,0,0,1,1,0,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1};
	int l = 32;
	
	this->high_us(9042, f);
	wait_us(4379);

	for (int i = 0; i < l; i++)
	{
		if (s[i] == 0)
		{
			this->high_us(612, f);
			wait_us(512);
		}
		else
		 {
			this->high_us(612, f);
			wait_us(1621);
		}
	}

	this->high_us(599, f);
}

void Iremitter::minolta_trigger()
{
	// frequency
	int f = 38;

	// sequence
	bool s[] = {0,0,1,0,1,1,0,0,0,1,0,1,0,0,1,1,1,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,1};
	int l = 33;
	
	this->high_us(3750, f);
	wait_us(1890);
	
	for (int i = 0; i < l; i++)
	{
		if (s[i] == 0)
		{
			this->high_us(456, f);
			wait_us(487);
		}
		else
		{
			this->high_us(456, f);
			wait_us(1430);
		}
	}
}

void Iremitter::nikon_trigger()
{
	// frequency
	int f = 40;

	// sequence
	this->high_us(2000, f);
	wait_us(27830);
	this->high_us(390, f);
	wait_us(1580);
	this->high_us(410, f);
	wait_us(3580);
	this->high_us(400, f);
}

void Iremitter::olympus_trigger()
{
	// frequency
	int f = 40;

	// sequence
	bool s[] = {0,1,1,0,0,0,0,1,1,1,0,1,1,1,0,0,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1};
	int l = 32;
	
	this->high_us(8972, f);
	wait_us(4384);
	this->high_us(624, f);
	
	for (int i = 0; i < l; i++)
	{
		if (s[i] == 0)
		{
			wait_us(488);
			this->high_us(600, f);
		}
		else
		{
			wait_us(1600);
			this->high_us(600, f);
		}
	}
}

void Iremitter::pentax_trigger()
{
	// frequency
	int f = 38;

	// sequence
	this->high_us(13000, f);
	wait_us(3000);
	
	for (int i = 0; i < 7; i++)
	{
		this->high_us(1000, f);
		wait_us(1000);
	}
}

void Iremitter::sony_trigger()
{
	// frequency
	int f = 40;

	// sequence
	bool s[] = {1,0,1,1,0,1,0,0,1,0,1,1,1,0,0,0,1,1,1,1};
	int l = 20;

	for (int j = 0; j < 3; j++) 
	{
		this->high_us(2320, f);
		wait_us(650);

		for (int i = 0; i < l; i++)
		{
			if (s[i] == 0)
			{
				this->high_us(575, f);
				wait_us(650);
			}
			else
			{
				this->high_us(1175, f);
				wait_us(650);
			}
		}

		wait_us(10000);
	}
}

