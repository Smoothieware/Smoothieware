/**
 * @file Nextion.h
 *
 * The header file including all other header files provided by this library. 
 *
 * Every example sketch should include this file. 
 *
 * @author  Wu Pengfei (email:<pengfei.wu@itead.cc>)
 * @date    2015/8/12
 * @copyright 
 * Copyright (C) 2014-2015 ITEAD Intelligent Systems Co., Ltd. \n
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */
#ifndef __NEXTION_H__
#define __NEXTION_H__

#include "module.h"
#include "mbed.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/RingBuffer.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include <map>
#include <tuple>
#define ABS_P 0
#define ASA   1
#define PET_G 2
#define PET_T 3
#define HIPS  4
#define PLA   5
class Display : public Module
{
public:
	enum Pagina
	{
		Init =0,
		Estado,
		Menu,
		Aquecimento,
		Movimento,
		Nivelamento,
		Configuracao,
		Arquivos,
		SemCartao
	};
	Display();
	~Display();
	void on_module_loaded();
	void on_gcode_received(void* argument);
	void on_idle(void* argument);
	void on_serial_char_received();
	void on_serial_char_writed();
	uint32_t timer(uint32_t);
	uint16_t count_folder_content();
private:
	bool filter_file(const char *f);
	string file_at(uint16_t line, bool& isdir);
	string status, laststatus;
	bool refresh;
	uint8_t cnt;
	Pagina paginas;
	mbed::Serial* serial;
	RingBuffer<char,256> buffer;
	std::vector<uint16_t> temp_controllers;
	int materialAtivo;
	map<int,string> *arquivos;
	string arquivo;
	map<int,int> *tempExtrusor;
	map<int,int> *tempMesa;
	map<int,int> *tempCamara;
	void Mapeador();
	void Varredura();
	void ExecutaComando(string cmd);
	void EnviaParametro(string obj,string val);
	void Mensagem(string msg);
	void GC(string g);
	bool AtualizaArquivos();
};

#endif /* #ifndef __NEXTION_H__ */
