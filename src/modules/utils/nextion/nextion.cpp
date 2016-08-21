#include "Nextion.h"
#include "gcode.h"
#include "mbed.h"
#include <string>
#include <stdarg.h>
#include <map>
using std::string;
#include "libs/StreamOutputPool.h"
Display::Display()
{
	this->serial = new mbed::Serial(P0_0,P0_1 );
	this->serial->baud(115200);
	tempExtrusor = new map<Material,int>();
	tempMesa = new map<Material,int>();
	tempCamara = new map<Material,int>();

	//Define Temperaturas do Extrusor
	tempExtrusor->insert(make_pair(Material::ABS_P,250));
	tempExtrusor->insert(make_pair(Material::ASA,250));
	tempExtrusor->insert(make_pair(Material::PLA,200));
	tempExtrusor->insert(make_pair(Material::PET_G,230));
	tempExtrusor->insert(make_pair(Material::PET_T,270));
	//Define Temperaturas da Mesa
	tempMesa->insert(make_pair(Material::ABS_P,108));
	tempMesa->insert(make_pair(Material::ASA,110));
	tempMesa->insert(make_pair(Material::PLA,60));
	tempMesa->insert(make_pair(Material::PET_G,80));
	tempMesa->insert(make_pair(Material::PET_T,105));
	//Define Temperaturas da Câmara
	tempCamara->insert(make_pair(Material::ABS_P,75));
	tempCamara->insert(make_pair(Material::ASA,75));
	tempCamara->insert(make_pair(Material::PLA,40));
	tempCamara->insert(make_pair(Material::PET_G,60));
	tempCamara->insert(make_pair(Material::PET_T,70));
	tempCamara[Material::ABS_P];
}
Display::~Display()
{
	delete tempExtrusor;
	delete tempMesa;
	delete tempCamara;
	delete this;
}
void Display::on_module_loaded()
{
	//this->serial->attach(this, &Display::on_serial_char_received, mbed::Serial::RxIrq);
	//this->register_for_event(ON_GCODE_EXECUTE);
	this->register_for_event(ON_MAIN_LOOP);
	this->register_for_event(ON_IDLE);
}
void Display::on_gcode_execute(void* argument)
{
	Gcode* gcode = static_cast<Gcode*>(argument);
	if(gcode->has_m)
	{
		if(gcode->m == 778)
		{
			//Chama funcao de calibracao do display
		}
	}

}
void Display::on_serial_char_received()
{
	while(this->serial->readable())
	{
	        char received = this->serial->getc();
	        this->buffer.push_back(received);
	}
}
void Display::on_idle(void* argument)
{
	//Atualiza Informações de Status
	if(this->buffer.size() > 0)
	{
		char c;
		this->buffer.pop_front(c);
		switch(c)
		{
			case 'E'://Ativa Extrusor
			break;
			case 'e'://Desativa Extrusor
			break;
			case 'A'://Pagina Arquivos
			break;
			case 'S'://Stop
			break;
			case 'P'://Play
			break;
			case 'p'://Pause
			break;
			case 'M'://Ativa Mesa
			break;
			case 'm'://Desativa Mesa
			break;
			case 'C'://Ativa Camara
			break;
			case 'c'://Desativa Câmara
			break;
			case 'U'://Ativa USB
			break;
			case 'u'://Desativa USB
			break;
			case 'j'://Ejeta Peça
			break;
			case 'F'://Proximo Filamento
			break;
			case 'f':
			break;
		}
	}


}
