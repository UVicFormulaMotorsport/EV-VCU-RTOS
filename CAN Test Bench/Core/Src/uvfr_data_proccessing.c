/*
 * uvfr_data_proccessing.c
 *
 *  Created on: Nov 25, 2025
 *      Author: byo10
 */

#include "uvfr_utils.h"

uint8_t isLutReverseable_ii(LUT_ii_t* lut){
	uint8_t n = lut->n;
	int32_t* Y = lut->y;
	uint8_t idir = 0;
	if(Y[1] == Y[2]){
		return NON_REVERSABLE;
	}

	if(Y[2] > Y[1]){
		idir = 1;
	}else{
		idir = -1;
	}

	for(int i = 1;i<n;i++){
		uint8_t dir = 0;
		if(Y[i] == Y[i-1]){
			return NON_REVERSABLE;
		}

		if(Y[i] > Y[i-1]){
			dir = 1;
		}else{
			dir = -1;
		}

		if(idir != dir){
			return NON_REVERSABLE;
		}

	}

	if(idir == -1){
		return REVERSABLE_DESCENDING;
	}else if (idir == 1){
		return REVERSABLE_ASCENDING;
	}

	return NON_REVERSABLE;

}

uint8_t isLutReverseable_if(LUT_if_t* lut){
	uint8_t n = lut->n;
	float* Y = lut->y;
	uint8_t idir = 0;
	if(Y[1] == Y[2]){
		return NON_REVERSABLE;
	}

	if(Y[2] > Y[1]){
		idir = 1;
	}else{
		idir = -1;
	}

	for(int i = 1;i<n;i++){
		uint8_t dir = 0;
		if(Y[i] == Y[i-1]){
			return NON_REVERSABLE;
		}

		if(Y[i] > Y[i-1]){
			dir = 1;
		}else{
			dir = -1;
		}

		if(idir != dir){
			return NON_REVERSABLE;
		}

	}

	if(idir == -1){
		return REVERSABLE_DESCENDING;
	}else if (idir == 1){
		return REVERSABLE_ASCENDING;
	}

	return NON_REVERSABLE;

}

uint8_t isLutReverseable_ff(LUT_ff_t* lut){
	uint8_t n = lut->n;
	float* Y = lut->y;
	uint8_t idir = 0;
	if(Y[1] == Y[2]){
		return NON_REVERSABLE;
	}

	if(Y[2] > Y[1]){
		idir = 1;
	}else{
		idir = -1;
	}

	for(int i = 1;i<n;i++){
		uint8_t dir = 0;
		if(Y[i] == Y[i-1]){
			return NON_REVERSABLE;
		}

		if(Y[i] > Y[i-1]){
			dir = 1;
		}else{
			dir = -1;
		}

		if(idir != dir){
			return NON_REVERSABLE;
		}

	}

	if(idir == -1){
		return REVERSABLE_DESCENDING;
	}else if (idir == 1){
		return REVERSABLE_ASCENDING;
	}

	return NON_REVERSABLE;

}

uv_status validateLUT_ii(LUT_ii_t* lut){
	if(lut == NULL){
		return UV_ERROR;
	}

	if(lut->x == NULL || lut->y == NULL){
		return UV_ERROR;
	}

	uint8_t n = lut->n;

	if(n<2){
		return UV_ERROR;
	}

	for(int i = 1; i < n;i++){
		if(lut->x[i] <= lut->x[i-1]){
			return UV_ERROR;
		}
	}

	return UV_OK;

}

uv_status validateLUT_if(LUT_if_t* lut){
	if(lut == NULL){
			return UV_ERROR;
		}

		if(lut->x == NULL || lut->y == NULL){
			return UV_ERROR;
		}

		uint8_t n = lut->n;

		if(n<2){
			return UV_ERROR;
		}

		for(int i = 1; i < n;i++){
			if(lut->x[i] <= lut->x[i-1]){
				return UV_ERROR;
			}
		}

		return UV_OK;

}

uv_status validateLUT_ff(LUT_ff_t* lut){
	if(lut == NULL){
			return UV_ERROR;
		}

		if(lut->x == NULL || lut->y == NULL){
			return UV_ERROR;
		}

		uint8_t n = lut->n;

		if(n<2){
			return UV_ERROR;
		}

		for(int i = 1; i < n;i++){
			if(lut->x[i] <= lut->x[i-1]){
				return UV_ERROR;
			}
		}

		return UV_OK;

}

inline int32_t linterp_ii(int32_t* x, int32_t* y,int32_t x0,uint8_t n){
	uint8_t bot = 0;
	uint8_t top = n - 1;
	uint8_t mid = bot + (top - bot/2);

	//Binary search
	while(top - bot > 1){
		if(x0 > x[mid]){
			bot = mid;
			mid = bot + (top - bot/2);

		}else if(x0 < x[mid]){
			top = mid;
			mid = bot + (top - bot/2);

		}else{
			//x == x[mid]
			return x[mid];
		}
	}

	//since we know that top - bot should be one in this case
	int32_t dx = x[top] - x[bot];
	int32_t dy = y[top] - y[bot];
	float m = ((float)dx)/((float)dy);
	return (x[bot] + m*(x0 - x[bot]));

}

inline float linterp_if(int32_t* x, float* y,int32_t x0,uint8_t n){
	uint8_t bot = 0;
	uint8_t top = n - 1;
	uint8_t mid = bot + (top - bot/2);

	//Binary search
	while(top - bot > 1){
		if(x0 > x[mid]){
			bot = mid;
			mid = bot + (top - bot/2);

		}else if(x0 < x[mid]){
			top = mid;
			mid = bot + (top - bot/2);

		}else{
			//x == x[mid]
			return x[mid];
		}
	}

	//since we know that top - bot should be one in this case
	int32_t dx = x[top] - x[bot];
	float dy = y[top] - y[bot];
	float m = ((float)dx)/((float)dy);
	return (x[bot] + m*(x0 - x[bot]));

}

inline int32_t linterp_fi(float* x, int32_t* y,float x0,uint8_t n){
	uint8_t bot = 0;
	uint8_t top = n - 1;
	uint8_t mid = bot + (top - bot/2);

	//Binary search
	while(top - bot > 1){
		if(x0 > x[mid]){
			bot = mid;
			mid = bot + (top - bot/2);

		}else if(x0 < x[mid]){
			top = mid;
			mid = bot + (top - bot/2);

		}else{
			//x == x[mid]
			return x[mid];
		}
	}

	//since we know that top - bot should be one in this case
	float dx = x[top] - x[bot];
	int32_t dy = y[top] - y[bot];
	float m = ((float)dx)/((float)dy);
	return (x[bot] + m*(x0 - x[bot]));

}

inline float linterp_ff(float* x, float* y,float x0,uint8_t n){
	uint8_t bot = 0;
	uint8_t top = n - 1;
	uint8_t mid = bot + (top - bot/2);

	//Binary search
	while(top - bot > 1){
		if(x0 > x[mid]){
			bot = mid;
			mid = bot + (top - bot/2);

		}else if(x0 < x[mid]){
			top = mid;
			mid = bot + (top - bot/2);

		}else{
			//x == x[mid]
			return x[mid];
		}
	}

	//since we know that top - bot should be one in this case
	float dx = x[top] - x[bot];
	float dy = y[top] - y[bot];
	float m = ((float)dx)/((float)dy);
	return (x[bot] + m*(x0 - x[bot]));

}

int32_t splineInt_ii(int32_t* x, int32_t* y, x0, n){
S
}

float splineInt_if(int32_t* x, float* y, x0, n){

}

float splineInt_ff(float* x, float* y, x0, n){

}


//These go from x to y
int32_t xToY_ii(LUT_ii_t* lut,int32_t x){
	if(lut == NULL){
		return 0;
	}

	if(lut == NULL){ //No null lookup tables
			return 0;
	}

	if(lut->x == NULL || lut->y == NULL){ //Your X and Y need to exist
		return 0;
	}
	int32_t* X = lut->x;
	float* Y = lut->y;

	uint8_t n = lut->n;

	if(n<2){ //Needs at least two points, otherwise it isn't really a lookup table
		return 0;
	}

		//Handle edge case where x is off the bottom of the LUT
	if(x < X[0]){
		//Interpolate using method of chosing
		uint8_t extrap = lut->flags&LUT_EXTRAPOLATION_MASK;
		if(extrap == LUT_CAP_AT_MAX_MIN){
			return Y[0];
		}else if(extrap == LUT_EXTRAPOLATE_LINEAR){
			int32_t dx = X[1] - X[0];
			int32_t dy = Y[1] - Y[0];
			float m = ((float)dy)/((float)dx);//slope
			return (int32_t)(Y[0] + m*(x - X[0])); // Extrapolates the lookup table using the slope

		}else{
			return 0;
		}
	}

		//Handle edge case where x flies off the top end of the LUT
	if(x > X[n -1]){
		//Interpolate using method of chosing
		uint8_t extrap = lut->flags&LUT_EXTRAPOLATION_MASK;
		if(extrap == LUT_CAP_AT_MAX_MIN){
			return Y[n-1];
		}else if(extrap == LUT_EXTRAPOLATE_LINEAR){
			int32_t dx = X[n-1] - X[n-2];				float dy = Y[n-1] - Y[n-2];
			float m = dy/dx;
			return (Y[0] + m*(x - X[n-1]));
		}else{
			return 0;
		}
	}

	uint8_t interp = lut->flags&LUT_INTERP_MASK;

	switch(interp){
	case LUT_LINTERP:
		break;

	case LUT_SPLINE:
		break;
	default:
		break;
	}
}

float xToY_if(LUT_if_t* lut,int32_t x){
	if(lut == NULL){ //No null lookup tables
		return 0;
	}

	if(lut->x == NULL || lut->y == NULL){ //Your X and Y need to exist
		return 0;
	}
	int32_t* X = lut->x;
	float* Y = lut->y;

	uint8_t n = lut->n;

	if(n<2){ //Needs at least two points, otherwise it isn't really a lookup table
		return 0;
	}

	//Handle edge case where x is off the bottom of the LUT
	if(x < X[0]){
		//Interpolate using method of chosing
		uint8_t extrap = lut->flags&LUT_EXTRAPOLATION_MASK;
		if(extrap == LUT_CAP_AT_MAX_MIN){
			return Y[0];
		}else if(extrap == LUT_EXTRAPOLATE_LINEAR){
			int32_t dx = X[1] - X[0];
			float dy = Y[1] - Y[0];
			float m = dy/dx;//slope
			return (Y[0] + m*(x - X[0])); // Extrapolates the lookup table using the slope

		}else{
			return 0;
		}
	}

	//Handle edge case where x flies off the top end of the LUT
	if(x > X[n -1]){
		//Interpolate using method of chosing
		uint8_t extrap = lut->flags&LUT_EXTRAPOLATION_MASK;
		if(extrap == LUT_CAP_AT_MAX_MIN){
			return Y[n-1];
		}else if(extrap == LUT_EXTRAPOLATE_LINEAR){
			int32_t dx = X[n-1] - X[n-2];
			float dy = Y[n-1] - Y[n-2];
			float m = dy/dx;
			return (Y[0] + m*(x - X[n-1]));
		}else{
			return 0;
		}
	}

	uint8_t interp = lut->flags&LUT_INTERP_MASK;

	switch(interp){
	case LUT_LINTERP:
		break;

	case LUT_SPLINE:
		break;
	default:
		break;
	}
	return 0;
}

float xToY_ff(LUT_ff_t* lut,float x){


	if(lut == NULL){ //No null lookup tables
		return 0;
	}

	if(lut->x == NULL || lut->y == NULL){ //Your X and Y need to exist
		return 0;
	}
	int32_t* X = lut->x;
	float* Y = lut->y;

	uint8_t n = lut->n;

	if(n<2){ //Needs at least two points, otherwise it isn't really a lookup table
		return 0;
	}

	//Handle edge case where x is off the bottom of the LUT
	if(x < X[0]){
			//Interpolate using method of chosing
		uint8_t extrap = lut->flags&LUT_EXTRAPOLATION_MASK;
		if(extrap == LUT_CAP_AT_MAX_MIN){
			return Y[0];
		}else if(extrap == LUT_EXTRAPOLATE_LINEAR){
			int32_t dx = X[1] - X[0];
			float dy = Y[1] - Y[0];
			float m = dy/dx;//slope
			return (Y[0] + m*(x - X[0])); // Extrapolates the lookup table using the slope

		}else{
			return 0;
		}
	}

		//Handle edge case where x flies off the top end of the LUT
	if(x > X[n -1]){
		//Interpolate using method of chosing
		uint8_t extrap = lut->flags&LUT_EXTRAPOLATION_MASK;
		if(extrap == LUT_CAP_AT_MAX_MIN){
			return Y[n-1];
		}else if(extrap == LUT_EXTRAPOLATE_LINEAR){
			int32_t dx = X[n-1] - X[n-2];
			float dy = Y[n-1] - Y[n-2];
			float m = dy/dx;
			return (Y[0] + m*(x - X[n-1]));
		}else{
			return 0;
		}
	}

	uint8_t interp = lut->flags&LUT_INTERP_MASK;

	switch(interp){
	case LUT_LINTERP:
		break;

	case LUT_SPLINE:
		break;
	default:
		break;
	}

	return 0;
}

//These go from y to x, however only if the lookup table is reversable
uv_status yToX_ii(LUT_ii_t* lut,int32_t x,int32_t* retval);
uv_status yToX_if(LUT_if_t* lut,float x,int32_t* retval);
uv_status yToX_ff(LUT_ff_t* lut,float x,float* retval);

LUT_ii_t* createLUT_ii(int32_t* x,int32_t* y,uint8_t n, uint16_t flags){
	return NULL;
}

LUT_if_t* createLUT_if(int32_t* x,float* y,uint8_t n, uint16_t flags){
	return NULL;
}

LUT_ff_t* createLUT_ff(float* x,float* y,uint8_t n, uint16_t flags){
	return NULL;
}

uv_status LUTupdateXY_ii(int32_t* x,int32_t* y,uint8_t n, uint16_t flags);
uv_status LUTupdateXY_if(int32_t* x,float* y,uint8_t n, uint16_t flags);
uv_status LUTupdateXY_ff(float* x,float* y,uint8_t n, uint16_t flags);
