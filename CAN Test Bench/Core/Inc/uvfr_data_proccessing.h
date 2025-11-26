/*
 * uvfr_data_proccessing.h
 *
 *  Created on: Nov 25, 2025
 *      Author: byo10
 */

#ifndef INC_UVFR_DATA_PROCCESSING_H_
#define INC_UVFR_DATA_PROCCESSING_H_

#include "uvfr_utils.h"

typedef enum uv_status_t uv_status;

/* Quick list of features I would like to add:
 * - Lookup tables with interpolation
 * - Ring buffers and averaging stuff
 * - FFT and frequency domain analysis
 */

typedef enum lut_reversability{
	NON_REVERSABLE,
	REVERSABLE_ASCENDING,
	REVERSABLE_DESCENDING

}lut_reversability;

#define LUT_EXTRAPOLATION_MASK 0x0003
#define LUT_CAP_AT_MAX_MIN 0x0000
#define LUT_EXTRAPOLATE_LINEAR 0x0001

#define LUT_INTERP_MASK 0b11100
#define LUT_LINTERP 0b00000
#define LUT_SPLINE 0b00100


/** @brief Integer to integer lookup table
 *
 */
typedef struct LUT_ii_t{
	uint8_t n;
	uint8_t reversability;
	uint8_t idx_ymax;
	uint8_t idx_ymin;
	int32_t* x;
	int32_t* y;
	uint16_t flags;

}LUT_ii_t;

/** @brief Integer to float lookup table
 *
 */
typedef struct LUT_if_t{
	uint8_t n;
	uint8_t reversability;
	uint8_t idx_ymax;
	uint8_t idx_ymin;
	int32_t* x;
	float* y;
	uint16_t flags;

}LUT_if_t;

typedef struct LUT_ff_t{
	uint8_t n;
	uint8_t reversability;
	uint8_t idx_ymax;
	uint8_t idx_ymin;
	float* x;
	float* y;
	uint16_t flags;

}LUT_ff_t;

/** @brief Ring buffer
 *
 */
typedef struct ringbuf_ui{
	uint32_t latest;
	uint32_t avg;
	uint8_t n;
	uint8_t sz;
	uint8_t idx;
	uint8_t res;
}ringbuf_ui;

typedef struct ringbuf_i{
	int32_t latest;
	int32_t avg;
	uint8_t n;
	uint8_t sz;
	uint8_t idx;
	uint8_t res;
}ringbuf_i;



typedef struct ringbuf_f{
	float latest;
	float avg;
	uint8_t n;
	uint8_t sz;
	uint8_t idx;
	uint8_t res;
}ringbuf_f;

//These go from x to y
uint32_t xToY_ii(LUT_ii_t* lut,int32_t x);
float xToY_if(LUT_if_t* lut,int32_t x);
float xToY_ff(LUT_ff_t* lut,float x);

//These go from y to x, however only if the lookup table is reversable
uv_status yToX_ii(LUT_ii_t* lut,int32_t x,int32_t* retval);
uv_status yToX_if(LUT_if_t* lut,float x,int32_t* retval);
uv_status yToX_ff(LUT_ff_t* lut,float x,float* retval);

//Create a LUT given x, and y values
LUT_ii_t* createLUT_ii(int32_t* x,int32_t* y,uint8_t n, uint16_t flags);
LUT_if_t* createLUT_if(int32_t* x,float* y,uint8_t n, uint16_t flags);
LUT_ff_t* createLUT_ff(float* x,float* y,uint8_t n, uint16_t flags);

//Dynamically swap out X and Y values
uv_status LUTupdateXY_ii(int32_t* x,int32_t* y,uint8_t n, uint16_t flags);
uv_status LUTupdateXY_if(int32_t* x,float* y,uint8_t n, uint16_t flags);
uv_status LUTupdateXY_ff(float* x,float* y,uint8_t n, uint16_t flags);

uv_status addToRingBuf_ui(ringbuf_i* rb,uint32_t val);
uv_status addToRingBuf_i(ringbuf_i* rb,int32_t val);
uv_status addToRingBuf_f(ringbuf_i* rb,int32_t val);

uint32_t getRingBufLatest_ui(ringbuf_ui* rb);
int32_t getRingBufLatest_i(ringbuf_i* rb);
float getRingBufLatest_f(ringbuf_f* rb);

uint32_t getRingBufAvg_ui(ringbuf_ui* rb);
int32_t getRingBufAvg_i(ringbuf_i* rb);
float getRingBufAvg_f(ringbuf_f* rb);



#endif /* INC_UVFR_DATA_PROCCESSING_H_ */
