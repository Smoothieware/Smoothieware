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
enum Pagina
{
	Init =0,
	Status,
	Menu,
	Aquecimento,
	Movimento,
	Nivelamento,
	Configuracao,
	Arquivos
};
enum Material
{
	ABS_P,
	PLA,
	ASA,
	PET_G,
	PET_T
};

class Display : public Module
{
public:
	Display();
	~Display();
	void on_module_loaded();
	void on_gcode_execute(void* argument);
	void on_idle(void* argument);
	void on_serial_char_received();
private:
	Pagina paginas;
	mbed::Serial* serial;
	RingBuffer<char,256> buffer;
	map<Material,int> *tempExtrusor;
	map<Material,int> *tempMesa;
	map<Material,int> *tempCamara;
};

#endif /* #ifndef __NEXTION_H__ */
