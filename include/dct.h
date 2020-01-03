/*
 ***************************************************************************
 * \file dct.h
 *
 * \date
 *   Dec 15 2010
 *
 * change log
 *
 **************************************************************************
 */

#ifndef _DCT_H
#define _DCT_H
#include "type_1.h"

void dct4x4dc( int16_t d[4][4] );
void idct4x4dc( int16_t d[4][4] );
void pixel_sub_wxh( int16_t *diff, int i_size, uint8_t *pix1, int i_pix1, uint8_t *pix2, int i_pix2 );

int  sub4x4_dct( int16_t dct[4][4], uint8_t *pix1, int stride1, uint8_t *pix2, int stride2 );
int  sub8x8_dct( int16_t dct[4][4][4], uint8_t *pix1, int stride1, uint8_t *pix2, int stride2 );
int  sub16x16_dct( int16_t dct[16][4][4], uint8_t *pix1, int stride1, uint8_t *pix2, int stride2 );

void add4x4_idct( uint8_t *p_dst, int stride, int16_t dct[4][4] );
void add8x8_idct( uint8_t *p_dst, int stride, int16_t dct[4][4][4] );
void add16x16_idct( uint8_t *p_dst, int stride, int16_t dct[16][4][4] );

//added for 8x8
int  sub8x8_dct8( int16_t dct[8][8], uint8_t *pix1, uint8_t *pix2, int stride1, int stride2  );
int  sub16x16_dct8( int16_t dct[4][8][8], uint8_t *pix1, uint8_t *pix2, int stride1, int stride2  );

void add8x8_idct8( uint8_t *p_dst, int16_t dct[8][8], int stride );
void add16x16_idct8( uint8_t *dst, int16_t dct[4][8][8], int stride );

//2x2
void dct2x2dc( int16_t d[2][2], int16_t dct4x4[4][4][4] );
void idct_dequant_2x2_dc( int16_t dct[2][2], int16_t dct4x4[4][4][4], int dequant_mf[6][4][4], int i_qp );


#endif