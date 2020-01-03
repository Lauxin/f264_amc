/*
 ***************************************************************************
 * \file quant.h
 *
 * \date
 *   jun 24 2011
 *
 * change log
 *
 **************************************************************************
 */

#ifndef _QUANT_H_
#define _QUANT_H_
#include "type_1.h"

#define SHIFT(x,s) ((s)<0 ? (x)<<-(s) : (s)==0 ? (x) : ((x)+(1<<((s)-1)))>>(s))
#define DIV(n,d) (((n) + ((d)>>1)) / (d))

//typedef int (*coeff_level_run_t)( int16_t *dct, f264_run_level_t *runlevel );
//void coeff_level_run_init( coeff_level_run_t pf[5] );

//NEW
static const uint8_t f264_ue_size_tab[256] =
{
	1, 1, 3, 3, 5, 5, 5, 5, 7, 7, 7, 7, 7, 7, 7, 7,
	9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
	11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
	11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
	13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,
	13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,
	13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,
	13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,
	15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
	15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
	15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
	15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
	15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
	15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
	15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
	15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
};
/****************************************************************************
*                   ME Cost Cal
****************************************************************************/
static inline int bs_size_ue(unsigned int val)
{
	return f264_ue_size_tab[val + 1];
}

static inline int bs_size_ue_big(unsigned int val)
{
	if (val < 255)
		return f264_ue_size_tab[val + 1];
	else
		return f264_ue_size_tab[(val + 1) >> 8] + 16;
}

static inline int bs_size_se(int val)
{
	return bs_size_ue_big(val <= 0 ? -val * 2 : val * 2 - 1);
}



//Multiplication factors for quantization
static const int MF[6][4][4] =
{
	{{13107,8066,13107,8066},{8066,5243,8066,5243},{13107,8066,13107,8066},{8066,5243,8066,5243}},
	{{11916,7490,11916,7490},{7490,4660,7490,4660},{11916,7490,11916,7490},{7490,4660,7490,4660}},
	{{10082,6554,10082,6554},{6554,4194,6554,4194},{10082,6554,10082,6554},{6554,4194,6554,4194}},
	{{9362,5825, 9352,5825},{5825,3647,5825,3647},{9362,5825, 9352,5825},{5825,3647,5825,3647}},
	{{8192,5243, 8192,5243},{5243,3355,5243,3355},{8192,5243, 8192,5243},{5243,3355,5243,3355}},
	{{7282,4559, 7282,4559},{4559,2893,4559,2893},{7282,4559, 7282,4559},{4559,2893,4559,2893}}
};

// Quantization factors based on level scale factor from decoder (round(reciprocal>>15))
static const int LevelQuantize[6][4][4] =
{
	{{205, 158, 205, 158}, {158, 128, 158, 128}, {205, 158, 205, 158}, {158, 128, 158, 128}},
	{{186, 146, 186, 146}, {146, 114, 146, 114}, {186, 146, 186, 146}, {146, 114, 146, 114}},
	{{158, 128, 158, 128}, {128, 102, 128, 102}, {158, 128, 158, 128}, {128, 102, 128, 102}},
	{{146, 114, 146, 114}, {114, 89, 114, 89}, {146, 114, 146, 114}, {114, 89, 114, 89}},
	{{128, 102, 128, 102}, {102, 82, 102, 82}, {128, 102, 128, 102}, {102, 82, 102, 82}},
	{{114, 89, 114, 89}, {89, 71, 89, 71}, {114, 89, 114, 89}, {89, 71, 89, 71}}
};

static const int LevelScale[6][4][4] =
{
	{{160,208,160,208},{208,256,208,256},{160,208,160,208},{208,256,208,256}},
	{{176,224,176,224},{224,288,224,288},{176,224,176,224},{224,288,224,288}},
	{{208,256,208,256},{256,320,256,320},{208,256,208,256},{256,320,256,320}},
	{{224,288,224,288},{288,368,288,368},{224,288,224,288},{288,368,288,368}},
	{{256,320,256,320},{320,400,320,400},{256,320,256,320},{320,400,320,400}},
	{{288,368,288,368},{368,464,368,464},{288,368,288,368},{368,464,368,464}}
};


static const int dequant4_scale[6][3] =
{
    { 10, 13, 16 },
    { 11, 14, 18 },
    { 13, 16, 20 },
    { 14, 18, 23 },
    { 16, 20, 25 },
    { 18, 23, 29 }
};
static const int quant4_scale[6][3] =
{//column 1 and 2 are swaped

    { 13107, 8066, 5243 },
    { 11916, 7490, 4660 },
    { 10082, 6554, 4194 },
    {  9362, 5825, 3647 },
    {  8192, 5243, 3355 },
    {  7282, 4559, 2893 },

};

static const int quant8_scan[16] =
{
    0,3,4,3, 3,1,5,1, 4,5,2,5, 3,1,5,1
};
static const int dequant8_scale[6][6] =
{
    { 20, 18, 32, 19, 25, 24 },
    { 22, 19, 35, 21, 28, 26 },
    { 26, 23, 42, 24, 33, 31 },
    { 28, 25, 45, 26, 35, 33 },
    { 32, 28, 51, 30, 40, 38 },
    { 36, 32, 58, 34, 46, 43 },
};
static const int quant8_scale[6][6] =
{
    { 13107, 11428, 20972, 12222, 16777, 15481 },
    { 11916, 10826, 19174, 11058, 14980, 14290 },
    { 10082,  8943, 15978,  9675, 12710, 11985 },
    {  9362,  8228, 14913,  8931, 11984, 11259 },
    {  8192,  7346, 13159,  7740, 10486,  9777 },
    {  7282,  6428, 11570,  6830,  9118,  8640 }
};

void quant_8x8( int16_t dct[8][8], uint16_t mf[64], uint32_t bias[64], int i_qp);
void quant_4x4( int16_t dct[4][4], uint16_t mf[16], uint32_t bias[16], int i_qp);
void quant_4x4_dc( int16_t dct[4][4], int mf, int bias , int i_qp);
void quant_2x2_dc( int16_t dct[2][2], int mf, int bias,  int i_qp);
void dequant_8x8( int16_t dct[8][8], int dequant_mf[6][8][8], int i_qp );
void dequant_4x4( int16_t dct[4][4], int dequant_mf[6][4][4], int i_qp );
void dequant_4x4_dc( int16_t dct[4][4], int dequant_mf[6][4][4], int i_qp );

static const uint8_t scaling_list[64] =
{
    16,16,16,16,16,16,16,16,
    16,16,16,16,16,16,16,16,
    16,16,16,16,16,16,16,16,
    16,16,16,16,16,16,16,16,
    16,16,16,16,16,16,16,16,
    16,16,16,16,16,16,16,16,
    16,16,16,16,16,16,16,16,
    16,16,16,16,16,16,16,16
};

static const uint8_t f264_cqm_jvt4i[16] =
{
      6,13,20,28,
     13,20,28,32,
     20,28,32,37,
     28,32,37,42
};


static const uint8_t f264_cqm_jvt4p[16] =
{
    10,14,20,24,
    14,20,24,27,
    20,24,27,30,
    24,27,30,34
};
static const uint8_t f264_cqm_jvt8i[64] =
{
     6,10,13,16,18,23,25,27,
    10,11,16,18,23,25,27,29,
    13,16,18,23,25,27,29,31,
    16,18,23,25,27,29,31,33,
    18,23,25,27,29,31,33,36,
    23,25,27,29,31,33,36,38,
    25,27,29,31,33,36,38,40,
    27,29,31,33,36,38,40,42
};
static const uint8_t f264_cqm_jvt8p[64] =
{
     9,13,15,17,19,21,22,24,
    13,13,17,19,21,22,24,25,
    15,17,19,21,22,24,25,27,
    17,19,21,22,24,25,27,28,
    19,21,22,24,25,27,28,30,
    21,22,24,25,27,28,30,32,
    22,24,25,27,28,30,32,33,
    24,25,27,28,30,32,33,35
};
static const uint8_t f264_cqm_flat16[64] =
{
    16,16,16,16,16,16,16,16,
    16,16,16,16,16,16,16,16,
    16,16,16,16,16,16,16,16,
    16,16,16,16,16,16,16,16,
    16,16,16,16,16,16,16,16,
    16,16,16,16,16,16,16,16,
    16,16,16,16,16,16,16,16,
    16,16,16,16,16,16,16,16
};
static const uint8_t * const f264_cqm_jvt[6] =
{
    f264_cqm_jvt4i, f264_cqm_jvt4p,
    f264_cqm_jvt4i, f264_cqm_jvt4p,
    f264_cqm_jvt8i, f264_cqm_jvt8p
};


//******************************************
//from jm,offset matrix  6.24
//******************************************
static const short Offset_intra_default[16] = {
  682, 682, 682, 682,
  682, 682, 682, 682,
  682, 682, 682, 682,
  682, 682, 682, 682
};

static const short Offset_inter_default[16] = {
  342, 342, 342, 342,
  342, 342, 342, 342,
  342, 342, 342, 342,
  342, 342, 342, 342,
};

static const short Offset8_intra_default[64] = {
  682, 682, 682, 682, 682, 682, 682, 682,
  682, 682, 682, 682, 682, 682, 682, 682,
  682, 682, 682, 682, 682, 682, 682, 682,
  682, 682, 682, 682, 682, 682, 682, 682,
  682, 682, 682, 682, 682, 682, 682, 682,
  682, 682, 682, 682, 682, 682, 682, 682,
  682, 682, 682, 682, 682, 682, 682, 682,
  682, 682, 682, 682, 682, 682, 682, 682
};

static const short Offset8_inter_default[64] = {
  342, 342, 342, 342, 342, 342, 342, 342,
  342, 342, 342, 342, 342, 342, 342, 342,
  342, 342, 342, 342, 342, 342, 342, 342,
  342, 342, 342, 342, 342, 342, 342, 342,
  342, 342, 342, 342, 342, 342, 342, 342,
  342, 342, 342, 342, 342, 342, 342, 342,
  342, 342, 342, 342, 342, 342, 342, 342,
  342, 342, 342, 342, 342, 342, 342, 342
};





static const short Offset_intra_default_chroma[16] = {
  682, 682, 682, 682,
  682, 682, 682, 682,
  682, 682, 682, 682,
  682, 682, 682, 682
};


static const short Offset8_intra_default_chroma[64] = {
  682, 682, 682, 682, 682, 682, 682, 682,
  682, 682, 682, 682, 682, 682, 682, 682,
  682, 682, 682, 682, 682, 682, 682, 682,
  682, 682, 682, 682, 682, 682, 682, 682,
  682, 682, 682, 682, 682, 682, 682, 682,
  682, 682, 682, 682, 682, 682, 682, 682,
  682, 682, 682, 682, 682, 682, 682, 682,
  682, 682, 682, 682, 682, 682, 682, 682
};

static const short Offset8_intra_default_inter[64] = {
  342, 342, 342, 342, 342, 342, 342, 342,
  342, 342, 342, 342, 342, 342, 342, 342,
  342, 342, 342, 342, 342, 342, 342, 342,
  342, 342, 342, 342, 342, 342, 342, 342,
  342, 342, 342, 342, 342, 342, 342, 342,
  342, 342, 342, 342, 342, 342, 342, 342,
  342, 342, 342, 342, 342, 342, 342, 342,
  342, 342, 342, 342, 342, 342, 342, 342
};

static const int QP_FOR_CHROMA[52]=
{
	//qpC=qpI while qpI<=29
     0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
	10,11,12,13,14,15,16,17,18,19,
	20,21,22,23,24,25,26,27,28,29,
	//qpC correspond with qpI while qpI>=30
    29,30,31,32,32,33,34,34,35,35,
	36,36,37,37,37,38,38,38,39,39,
	39,39
};

#endif
