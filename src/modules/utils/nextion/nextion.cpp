#include "Nextion.h"
#include "StreamOutputPool.h"
#include "PublicDataRequest.h"
#include "gcode.h"
#include "mbed.h"
#include "kernel.h"
#include "BaseSolution.h"
#include "Config.h"
#include "utils.h"
#include <iostream>
#include <sstream>  // Required for stringstreams
#include <string>
#include <stdarg.h>
#include <stdlib.h>
#include <cstdlib>
#include <stdio.h>
#include <map>
#include "StreamOutput.h"
#include "libs/StreamOutputPool.h"
#include "DirHandle.h"
#include "mri.h"
#include "PublicData.h"
#include "SwitchPublicAccess.h"
#include "checksumm.h"
#include "TemperatureControlPool.h"
#include "modules/tools/temperaturecontrol/TemperatureControlPublicAccess.h"
#include "SlowTicker.h"
#include <math.h>
#include <string.h>
#include <string>
#include <stdio.h>
#include <algorithm>

using std::string;
template <typename T>
string NumberToString ( T Number )
{
 std::ostringstream ss;
 ss << Number;
 return ss.str();
}



static struct pad_temperature getTemperatures(uint16_t heater_cs)
{
    struct pad_temperature temp;
    PublicData::get_value( temperature_control_checksum, current_temperature_checksum, heater_cs, &temp );
    return temp;
}

Display::Display()
{
	this->serial = new mbed::Serial(p9,p10 );
	this->serial->baud(115200);
	tempExtrusor = new map<int,int>();
	tempMesa = new map<int,int>();
	tempCamara = new map<int,int>();
	arquivos = new map<int, string>();

	//Define Temperaturas do Extrusor
	tempExtrusor->insert(make_pair(ABS_P,250));
	tempExtrusor->insert(make_pair(ASA,250));
	tempExtrusor->insert(make_pair(PLA,200));
	tempExtrusor->insert(make_pair(PET_G,230));
	tempExtrusor->insert(make_pair(PET_T,270));
	tempExtrusor->insert(make_pair(HIPS,230));
	//Define Temperaturas da Mesa
	tempMesa->insert(make_pair(ABS_P,108));
	tempMesa->insert(make_pair(ASA,110));
	tempMesa->insert(make_pair(PLA,60));
	tempMesa->insert(make_pair(PET_G,80));
	tempMesa->insert(make_pair(PET_T,105));
	tempMesa->insert(make_pair(HIPS,100));
	//Define Temperaturas da Câmara
	tempCamara->insert(make_pair(ABS_P,75));
	tempCamara->insert(make_pair(ASA,75));
	tempCamara->insert(make_pair(PLA,40));
	tempCamara->insert(make_pair(PET_G,60));
	tempCamara->insert(make_pair(PET_T,70));
	tempCamara->insert(make_pair(HIPS,60));
	paginas = Pagina::Init;
	materialAtivo = ABS_P;
	THEKERNEL->streams->printf("Modulo Display Instanciado\n");
	cnt= 0;
	refresh = false;
	laststatus = " ";
	status = "Impressora Pronta. Selecione um arquivo para impressão!";
}
Display::~Display()
{
	delete tempExtrusor;
	delete tempMesa;
	delete tempCamara;
	delete serial;
	THEKERNEL->streams->printf("Modulo Display Apagado\n");
	delete this;
}
void Display::on_module_loaded()
{
	this->serial->attach(this, &Display::on_serial_char_received, mbed::Serial::RxIrq);
	this->register_for_event(ON_GCODE_RECEIVED);
	this->register_for_event(ON_MAIN_LOOP);
	this->register_for_event(ON_IDLE);

	temp_controllers.clear();
	std::vector<struct pad_temperature> controllers;
	bool ok = PublicData::get_value(temperature_control_checksum, poll_controls_checksum, &controllers);
	if (ok) {
		for (auto &c : controllers) {
			temp_controllers.push_back(c.id);
		}
	}
	    THEKERNEL->slow_ticker->attach(12, this, &Display::timer);
	    THEKERNEL->streams->printf("Modulo Carregado\n");

}
void Display::on_gcode_received(void* argument)
{
	Gcode* gcode = static_cast<Gcode*>(argument);
	if(gcode->has_m)
	{
		if(gcode->m == 778)
			ExecutaComando("touch_j");
		if(gcode->m == 117)
			status = gcode->subcode;
	}

}
uint32_t Display::timer(uint32_t)
{
	if(++cnt >= 12)
	{
		refresh = true;
	    cnt= 0;
	}
	return 0;
}
void Display::on_serial_char_received()
{
	while(this->serial->readable())
	{
		char received = this->serial->getc();
		this->buffer.push_back(received);
	}
}
void Display::on_serial_char_writed()
{
}

void Display::on_idle(void* argument)
{
	Mapeador();
	Varredura();
}
void Display::Mapeador()
{
	unsigned int i;
	char c;
	string s ="";
	//Recebe um caractere
	if(this->buffer.size() > 0)
	{
		this->buffer.pop_front(c);
		if(c=='#') //Marcador de Página
		{
			this->buffer.pop_front(c);
			i = c - '0';
			paginas = (Pagina) i;
		}
		else if(c=='*') // Marcador de Arquivo
		{
			this->buffer.pop_front(c);
			i = c - '0';
			if(i < arquivos->size())
				arquivo = arquivos->at(i);
		}
		else switch(paginas)
		{
			case Init: break;
			case Estado:
				switch(c)
				{
					case'U': //  Ativa USB
						Mensagem("connect");
						break;
					case'u': // Desativa USB
						Mensagem("disconnect");
						break;
					case'S':
						Mensagem("stop");
						break;
					case 'P':
						s = "play " + arquivo;
						Mensagem(s);
						break;
					case 'p':
						Mensagem("pause");
						break;
					case 'a':
						if(AtualizaArquivos())
							ExecutaComando("page 7"); //Gerenciador de Arquivos
						else
							ExecutaComando("page 8"); // Pagina de Erro de Cartão
						break;
					case 'E':
							GC("M104 S" + NumberToString(&tempExtrusor[materialAtivo]));
						break;
					case 'e':
							GC("M104 S0");
						break;
					case 'M':
							GC("M140 S" + NumberToString(&tempMesa[materialAtivo]));
						break;
					case 'm':
							GC("M140 S0");
						break;
					case 'C':
							GC("M141 S" + NumberToString(&tempCamara[materialAtivo]));
						break;
					case 'c':
							GC("M141 S0");
						break;
				}
				break;
				case Menu:
					break;
				case Aquecimento:

					break;
				case Movimento:	break;
				case Nivelamento: break;
				case Configuracao: break;
				case Arquivos: break;
				case SemCartao: break;
		}
	}
}
bool Display::AtualizaArquivos()
{
	string cmd = "",fn = "";
	int c;
	bool isdir;
	THEKERNEL->current_path = "/";
	c = count_folder_content();
	if(c == 0)
			return false; // Cartão de memória não presente

	THEKERNEL->current_path = "/sd";
	arquivos->clear();
	c = count_folder_content();
	for(int f =0; f < c; f++)
	{
		fn= this->file_at(f, isdir).substr(0, 18);
		if(!isdir)
			arquivos->insert(make_pair(f,fn));
	}
	return true;
}
void Display::Varredura()
{
	string cmd = "";
	string temp = "";
	if(refresh)
	{
		refresh = false;
		switch(paginas)
		{
			case Init:
				// Ponto de Entrada
				ExecutaComando("Page 1");
			break;
			case Estado:
				//Atualiza mensagem de estado
				if(status != laststatus)
					EnviaParametro("status.txt",status);
				//Varre os Termometros e atualiza na tela

				//Atualiza temperaturas
				for(auto id : temp_controllers)
				{
					struct pad_temperature c= getTemperatures(id);
					if(c.designator.front() == 'B')
					{
						//Mesa setpoint
						temp = NumberToString(c.target_temperature) + "°C";
						EnviaParametro("tM.txt",temp);
						//Mesa atual
						temp = NumberToString(c.current_temperature) + "°C";
						EnviaParametro("tMAtual",temp);
					}
					if(c.designator.front() == 'T')
					{
						//Extrusor setpoint
						temp = NumberToString(c.target_temperature) + "°C";
						EnviaParametro("tEx.txt",temp);
						//Extrusor atual
						temp = NumberToString(c.current_temperature) + "°C";
						EnviaParametro("tExAtual.txt",temp);
					}
					if(c.designator.front() == 'C')
					{
						//CA setpoint
						temp = NumberToString(c.target_temperature) + "°C";
						EnviaParametro("tCA.txt",temp);
						//CA atual
						temp = NumberToString(c.current_temperature) + "°C";
						EnviaParametro("tCAAtual.txt",temp);
					}
				}
			break;
			case Menu: break;
			case Aquecimento:

			break;
			case Movimento:

			break;
			case Nivelamento: break;
			case Configuracao: break;
			case Arquivos:
				EnviaParametro("t1.txt",arquivos->size() >0 ? arquivos->at(0) : "");
				EnviaParametro("t2.txt",arquivos->size() >1 ? arquivos->at(1) : "");
				EnviaParametro("t3.txt",arquivos->size() >2 ? arquivos->at(2) : "");
				EnviaParametro("t4.txt",arquivos->size() >3 ? arquivos->at(3) : "");
				EnviaParametro("t5.txt",arquivos->size() >4 ? arquivos->at(4) : "");
				EnviaParametro("t6.txt",arquivos->size() >5 ? arquivos->at(5) : "");
				EnviaParametro("t7.txt",arquivos->size() >6 ? arquivos->at(6) : "");
				break;
			case SemCartao: break;
		}
	}
}
void Display::EnviaParametro(string obj,string val)
{
	string cmd = "";
	cmd += obj;
	cmd += "=\"";
	cmd += val;
	cmd +="\"";
	ExecutaComando(cmd);
}
void Display::ExecutaComando(string cmd)
{
	string c = "";
	c += 0xff;
	c += cmd;
	c += 0xff;
	c += 0xff;
	c += 0xff;
	for(unsigned int i =0 ;i< cmd.size(); i++)
	{
		serial->putc(c[i]);
	}
}
void Display::GC(string g)
{
	Gcode gc(g, &(StreamOutput::NullStream));
	THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);
}
void Display::Mensagem(string msg)
{
	struct SerialMessage message;
	message.message = msg;
	message.stream = &(StreamOutput::NullStream);
	THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
}
uint16_t Display::count_folder_content()
{
    DIR *d;
    struct dirent *p;
    uint16_t count = 0;
    d = opendir(THEKERNEL->current_path.c_str());
    if (d != NULL) {
        while ((p = readdir(d)) != NULL) {
            if((p->d_isdir && p->d_name[0] != '.') || filter_file(p->d_name)) count++;
        }
        closedir(d);
        return count;
    }
    return 0;
}
bool Display::filter_file(const char *f)
{
    string fn= lc(f);
    return (fn.at(0) != '.') &&
             ((fn.find(".g") != string::npos) ||
              (fn.find(".ngc") != string::npos) ||
			  (fn.find(".gcode") != string::npos) ||
              (fn.find(".nc") != string::npos));
}
string Display::file_at(uint16_t line, bool& isdir)
{
    DIR *d;
    struct dirent *p;
    uint16_t count = 0;
    d = opendir(THEKERNEL->current_path.c_str());
    if (d != NULL) {
        while ((p = readdir(d)) != NULL) {
            // only filter files that have a .g in them and directories not starting with a .
          if(((p->d_isdir && p->d_name[0] != '.') || filter_file(p->d_name)) && count++ == line ) {
                isdir= p->d_isdir;
                string fn= p->d_name;
                closedir(d);
                return fn;
            }
        }
    }

    if (d != NULL) closedir(d);
    isdir= false;
    return "";
}
