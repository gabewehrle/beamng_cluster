/*
 * CanFrame.h
 *
 *  Created on: Mar 17, 2021
 *      Author: Gabe Wehrle
 */
#pragma once

#include "xxhash.h"

//Inspired by Due Can by Collin80 (https://github.com/collin80/due_can)
typedef union {
	uint64_t value;
	struct {
		uint32_t low;
		uint32_t high;
	};
	struct {
		uint16_t s0;
		uint16_t s1;
		uint16_t s2;
		uint16_t s3;
	};
	uint8_t bytes[8];
} BytesUnion;

typedef struct {
	CAN_TxHeaderTypeDef		TxHeader;
	/*
		TxHeader.StdId = 				0x316;
		TxHeader.ExtId = 				0x01;
		TxHeader.RTR = 					CAN_RTR_DATA;
		TxHeader.IDE = 					CAN_ID_STD;
		TxHeader.DLC = 					8;
		TxHeader.TransmitGlobalTime = 	DISABLE;
	 */
	BytesUnion 				data;
} CanFrame;

typedef struct ClusterData {
	uint8_t 		loc;
	XXH32_hash_t	hashCalc;
	float			lastFuel;
	uint32_t		lights;
	uint32_t		lightsOverride;

	union {
		struct {
			unsigned int		timeOrder;		// time in milliseconds (to check order)	//4		0
			char				car[4];        	// Car name (4 bytes)						//4		4
			unsigned short		flags;      	// Info (see OG_x below)					//2		8
			unsigned char 		gear;       	// Current gear								//2		10
			float 				speedMS;    	// M/S										//4		12
			float 				rpm;        	// RPM										//4		16
			float 				turbo;      	// BAR										//4		20
			float 				engtemp;    	// C										//4		24
			float 				fuel;       	// 0 to 1									//4		28
			float 				oilpressure;	// BAR										//4		32
			float 				oiltemp;    	// C										//4		36
			unsigned int 		dashlights; 	// Dash lights available (see DL_x below)	//4		40
			unsigned int 		showlights; 	// Dash lights currently switched on		//4		44
			//DL_SHIFT,            bit 0    - shift light
			//DL_FULLBEAM,         bit 1    - full beam
			//DL_HANDBRAKE,        bit 2    - handbrake
			//DL_PITSPEED,         bit 3    - pit speed limiter
			//DL_TC,               bit 4    - TC active or switched off
			//DL_SIGNAL_L,         bit 5    - left turn signal
			//DL_SIGNAL_R,         bit 6    - right turn signal
			//DL_SIGNAL_ANY,       bit 7    - shared turn signal
			//DL_OILWARN,          bit 8    - oil pressure warning
			//DL_BATTERY,          bit 9    - battery warning
			//DL_ABS,              bit 10   - ABS active or switched off
			//DL_ESC,              bit 11   - ESC
			float 				throttle;   	// 0 to 1									//4		48
			float 				brake;      	// 0 to 1									//4		52
			float 				clutch;     	// 0 to 1									//4		56
			char 				display1[16];   // Usually Fuel (16)						//16	60
			char				display2[16];   // Usually Settings (16)					//16	76

			XXH32_hash_t 		hashRecv;		//uint32_t*									//4		92
		};
		uint8_t bytes[96];
	} data;
} ClusterData;
