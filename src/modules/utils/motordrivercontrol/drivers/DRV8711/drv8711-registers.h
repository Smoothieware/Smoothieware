/* --COPYRIGHT--,BSD
 * Copyright (c) 2013, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

/*******************************************************************************
 *
 *  registers.h - DRV8711 Registers
 *  Nick Oborny
 *  BOOST-DRV8711_FIRMWARE
 *  3/12/2014
 *
 ******************************************************************************/

#pragma once

// CTRL Register
struct CTRL_Register
{
	unsigned int Address;	// bits 14-12
	unsigned int DTIME;		// bits 11-10
	unsigned int ISGAIN;	// bits 9-8
	unsigned int EXSTALL;	// bit 7
	unsigned int MODE;		// bits 6-3
	unsigned int RSTEP;		// bit 2
	unsigned int RDIR;		// bit 1
	unsigned int ENBL;		// bit 0
};

// TORQUE Register
struct TORQUE_Register
{
	unsigned int Address;	// bits 14-12
	/* Reserved */ 			// bit 11
	unsigned int SIMPLTH;  	// bits 10-8
	unsigned int TORQUE;	// bits 7-0
};

// OFF Register
struct OFF_Register
{
	unsigned int Address;	// bits 14-12
	/* Reserved */ 			// bits 11-9
	unsigned int PWMMODE;  	// bit 8
	unsigned int TOFF;		// bits 7-0
};

// BLANK Register
struct BLANK_Register
{
	unsigned int Address;	// bits 14-12
	/* Reserved */ 			// bits 11-9
	unsigned int ABT;  		// bit 8
	unsigned int TBLANK;	// bits 7-0
};

// DECAY Register
struct DECAY_Register
{
	unsigned int Address;	// bits 14-12
	/* Reserved */ 			// bit 11
	unsigned int DECMOD;  	// bits 10-8
	unsigned int TDECAY;	// bits 7-0
};

// STALL Register
struct STALL_Register
{
	unsigned int Address;	// bits 14-12
	unsigned int VDIV;  	// bits 11-10
	unsigned int SDCNT;		// bits 9-8
	unsigned int SDTHR;		// bits 7-0
};

// DRIVE Register
struct DRIVE_Register
{
	unsigned int Address;	// bits 14-12
	unsigned int IDRIVEP;  	// bits 11-10
	unsigned int IDRIVEN;	// bits 9-8
	unsigned int TDRIVEP;	// bits 7-6
	unsigned int TDRIVEN;	// bits 5-4
	unsigned int OCPDEG;	// bits 3-2
	unsigned int OCPTH;		// bits 1-0
};

// STATUS Register
struct STATUS_Register
{
	unsigned int Address;	// bits 14-12
	/* Reserved */			// bits 11-8
	unsigned int STDLAT;  	// bit 7
	unsigned int STD;		// bit 6
	unsigned int UVLO;		// bit 5
	unsigned int BPDF;		// bit 4
	unsigned int APDF;		// bit 3
	unsigned int BOCP;		// bit 2
	unsigned int AOCP;		// bit 1
	unsigned int OTS;		// bit 0
};

