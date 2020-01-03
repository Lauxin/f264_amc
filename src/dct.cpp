#include "dct.h"
#include "math.h"
#include "common.h"

/****************************************************************************
* 2x2 transform:
****************************************************************************/
void idct_dequant_2x2_dc( int16_t dct[2][2], int16_t dct4x4[4][4][4], int dequant_mf[6][4][4], int i_qp )
{
    int d0 = dct[0][0] + dct[0][1];
    int d1 = dct[1][0] + dct[1][1];
    int d2 = dct[0][0] - dct[0][1];
    int d3 = dct[1][0] - dct[1][1];
    int dmf = dequant_mf[i_qp%6][0][0];
    int qbits = i_qp/6 - 5;
    if( qbits > 0 )
    {
        dmf <<= qbits;
        qbits = 0;
    }
    dct4x4[0][0][0] = (d0 + d1) * dmf >> -qbits;
    dct4x4[1][0][0] = (d0 - d1) * dmf >> -qbits;
    dct4x4[2][0][0] = (d2 + d3) * dmf >> -qbits;
    dct4x4[3][0][0] = (d2 - d3) * dmf>> -qbits;

}

void dct2x2dc( int16_t d[2][2], int16_t dct4x4[4][4][4] )
{
    int d0 = dct4x4[0][0][0] + dct4x4[1][0][0];
    int d1 = dct4x4[2][0][0] + dct4x4[3][0][0];
    int d2 = dct4x4[0][0][0] - dct4x4[1][0][0];
    int d3 = dct4x4[2][0][0] - dct4x4[3][0][0];

    d[0][0] = (d0 + d1);
    d[1][0] = (d2 + d3);
    d[0][1] = (d0 - d1);
    d[1][1] = (d2 - d3);

    dct4x4[0][0][0] = 0;
    dct4x4[1][0][0] = 0;
    dct4x4[2][0][0] = 0;
    dct4x4[3][0][0] = 0;

}

/****************************************************************************
* 4x4 transform:
****************************************************************************/
void dct4x4dc( int16_t d[4][4] )
{
    int16_t tmp[4][4];
    int s01, s23;
    int d01, d23;
    int i;

    for( i = 0; i < 4; i++ )
    {
        s01 = d[i][0] + d[i][1];
        d01 = d[i][0] - d[i][1];
        s23 = d[i][2] + d[i][3];
        d23 = d[i][2] - d[i][3];

        tmp[0][i] = s01 + s23;
        tmp[1][i] = s01 - s23;
        tmp[2][i] = d01 - d23;
        tmp[3][i] = d01 + d23;
    }

    for( i = 0; i < 4; i++ )
    {
        s01 = tmp[i][0] + tmp[i][1];
        d01 = tmp[i][0] - tmp[i][1];
        s23 = tmp[i][2] + tmp[i][3];
        d23 = tmp[i][2] - tmp[i][3];

        d[i][0] = ( s01 + s23 + 1 ) >> 1;
        d[i][1] = ( s01 - s23 + 1 ) >> 1;
        d[i][2] = ( d01 - d23 + 1 ) >> 1;
        d[i][3] = ( d01 + d23 + 1 ) >> 1;
    }
}

void idct4x4dc( int16_t d[4][4] )
{
    int16_t tmp[4][4];
    int s01, s23;
    int d01, d23;
    int i;

    for( i = 0; i < 4; i++ )
    {
        s01 = d[i][0] + d[i][1];
        d01 = d[i][0] - d[i][1];
        s23 = d[i][2] + d[i][3];
        d23 = d[i][2] - d[i][3];

        tmp[0][i] = s01 + s23;
        tmp[1][i] = s01 - s23;
        tmp[2][i] = d01 - d23;
        tmp[3][i] = d01 + d23;
    }

    for( i = 0; i < 4; i++ )
    {
        s01 = tmp[i][0] + tmp[i][1];
        d01 = tmp[i][0] - tmp[i][1];
        s23 = tmp[i][2] + tmp[i][3];
        d23 = tmp[i][2] - tmp[i][3];

        d[i][0] = s01 + s23;
        d[i][1] = s01 - s23;
        d[i][2] = d01 - d23;
        d[i][3] = d01 + d23;
    }
}

inline void pixel_sub_wxh( int16_t *diff, int i_size,
                                  uint8_t *pix1, int i_pix1, uint8_t *pix2, int i_pix2 )
{
    int y, x;
    for( y = 0; y < i_size; y++ )
    {
        for( x = 0; x < i_size; x++ )
        {
            diff[x + y*i_size] = pix1[x] - pix2[x];
        }
        pix1 += i_pix1;
        pix2 += i_pix2;
    }
}

//4*4²Ð²îDCT±ä»»
int sub4x4_dct( int16_t dct[4][4], uint8_t *pix1, int stride1, uint8_t *pix2, int stride2 )
{
    int16_t d[4][4];
    int16_t tmp[4][4];
    int i, j;
    int sad = 0;

    pixel_sub_wxh( (int16_t*)d, 4, pix1, stride1, pix2, stride2 );
    for(i=0; i<4; i++)
    {
        for(j=0; j<4; j++)
        {
            sad += abs(d[i][j]);
        }

    }

    for( i = 0; i < 4; i++ )
    {
        const int s03 = d[i][0] + d[i][3];
        const int s12 = d[i][1] + d[i][2];
        const int d03 = d[i][0] - d[i][3];
        const int d12 = d[i][1] - d[i][2];

        tmp[0][i] =   s03 +   s12;
        tmp[1][i] = 2*d03 +   d12;
        tmp[2][i] =   s03 -   s12;
        tmp[3][i] =   d03 - 2*d12;
    }

    for( i = 0; i < 4; i++ )
    {
        const int s03 = tmp[i][0] + tmp[i][3];
        const int s12 = tmp[i][1] + tmp[i][2];
        const int d03 = tmp[i][0] - tmp[i][3];
        const int d12 = tmp[i][1] - tmp[i][2];

        dct[i][0] =   s03 +   s12;
        dct[i][1] = 2*d03 +   d12;
        dct[i][2] =   s03 -   s12;
        dct[i][3] =   d03 - 2*d12;
    }
    return sad;
}

void add4x4_idct( uint8_t *p_dst, int stride, int16_t dct[4][4] )
{
    int16_t d[4][4];
    int16_t tmp[4][4];
    int x, y;
    int i;

    for( i = 0; i < 4; i++ )
    {
        const int s02 =  dct[0][i]     +  dct[2][i];
        const int d02 =  dct[0][i]     -  dct[2][i];
        const int s13 =  dct[1][i]     + (dct[3][i]>>1);
        const int d13 = (dct[1][i]>>1) -  dct[3][i];

        tmp[i][0] = s02 + s13;
        tmp[i][1] = d02 + d13;
        tmp[i][2] = d02 - d13;
        tmp[i][3] = s02 - s13;
    }

    for( i = 0; i < 4; i++ )
    {
        const int s02 =  tmp[0][i]     +  tmp[2][i];
        const int d02 =  tmp[0][i]     -  tmp[2][i];
        const int s13 =  tmp[1][i]     + (tmp[3][i]>>1);
        const int d13 = (tmp[1][i]>>1) -  tmp[3][i];

        d[0][i] = ( s02 + s13 + 32 ) >> 6;
        d[1][i] = ( d02 + d13 + 32 ) >> 6;
        d[2][i] = ( d02 - d13 + 32 ) >> 6;
        d[3][i] = ( s02 - s13 + 32 ) >> 6;
    }


    for( y = 0; y < 4; y++ )
    {
        for( x = 0; x < 4; x++ )
        {
            p_dst[x] = f264_clip_uint8( p_dst[x] + d[y][x] );
        }
        p_dst += stride;
    }
}

void add8x8_idct( uint8_t *p_dst, int stride, int16_t dct[4][4][4] )
{
    add4x4_idct( &p_dst[0], stride, dct[0] );
    add4x4_idct( &p_dst[4], stride, dct[1] );
    add4x4_idct( &p_dst[4*stride+0], stride, dct[2] );
    add4x4_idct( &p_dst[4*stride+4], stride, dct[3] );
}

void add16x16_idct( uint8_t *p_dst, int stride, int16_t dct[16][4][4] )
{
    add8x8_idct( &p_dst[0], stride, &dct[0] );
    add8x8_idct( &p_dst[8], stride, &dct[4] );
    add8x8_idct( &p_dst[8*stride+0], stride, &dct[8] );
    add8x8_idct( &p_dst[8*stride+8], stride, &dct[12] );
}

int sub8x8_dct( int16_t dct[4][4][4], uint8_t *pix1, int stride1, uint8_t *pix2, int stride2 )
{
    int sad=0;
    sad += sub4x4_dct( dct[0], &pix1[0], stride1, &pix2[0], stride2 );
    sad += sub4x4_dct( dct[1], &pix1[4], stride1, &pix2[4], stride2 );
    sad += sub4x4_dct( dct[2], &pix1[4*stride1+0], stride1, &pix2[4*stride2+0], stride2 );
    sad += sub4x4_dct( dct[3], &pix1[4*stride1+4], stride1, &pix2[4*stride2+4], stride2 );
    return sad;
}

int sub16x16_dct( int16_t dct[16][4][4], uint8_t *pix1, int stride1, uint8_t *pix2, int stride2 )
{
    int sad=0;
    sad += sub8x8_dct( &dct[ 0], &pix1[0], stride1, &pix2[0], stride2 );
    sad += sub8x8_dct( &dct[ 4], &pix1[8], stride1, &pix2[8], stride2 );
    sad += sub8x8_dct( &dct[ 8], &pix1[8*stride1+0], stride1, &pix2[8*stride2+0], stride2 );
    sad += sub8x8_dct( &dct[12], &pix1[8*stride1+8], stride1, &pix2[8*stride2+8], stride2 );
    return sad;

}

/****************************************************************************
* 8x8 transform:
****************************************************************************/

#define DCT8_1D {\
    const int s07 = SRC(0) + SRC(7);\
    const int s16 = SRC(1) + SRC(6);\
    const int s25 = SRC(2) + SRC(5);\
    const int s34 = SRC(3) + SRC(4);\
    const int a0 = s07 + s34;\
    const int a1 = s16 + s25;\
    const int a2 = s07 - s34;\
    const int a3 = s16 - s25;\
    const int d07 = SRC(0) - SRC(7);\
    const int d16 = SRC(1) - SRC(6);\
    const int d25 = SRC(2) - SRC(5);\
    const int d34 = SRC(3) - SRC(4);\
    const int a4 = d16 + d25 + (d07 + (d07>>1));\
    const int a5 = d07 - d34 - (d25 + (d25>>1));\
    const int a6 = d07 + d34 - (d16 + (d16>>1));\
    const int a7 = d16 - d25 + (d34 + (d34>>1));\
    DST(0) =  a0 + a1     ;\
    DST(1) =  a4 + (a7>>2);\
    DST(2) =  a2 + (a3>>1);\
    DST(3) =  a5 + (a6>>2);\
    DST(4) =  a0 - a1     ;\
    DST(5) =  a6 - (a5>>2);\
    DST(6) = (a2>>1) - a3 ;\
    DST(7) = (a4>>2) - a7 ;\
}

int sub8x8_dct8( int16_t dct[8][8], uint8_t *pix1, uint8_t *pix2, int stride1, int stride2 )
{
    int i, j;
    int16_t tmp[8][8];
    int sad = 0;

    pixel_sub_wxh( (int16_t*)tmp, 8, pix1, stride1, pix2, stride2 );
    for(i=0; i<8; i++)
    {
        for(j=0; j<8; j++)
        {
            sad += abs(tmp[i][j]);
        }

    }

#define SRC(x) tmp[x][i]
#define DST(x) tmp[x][i]
    for( i = 0; i < 8; i++ )
        DCT8_1D
#undef SRC
#undef DST

#define SRC(x) tmp[i][x]
#define DST(x) dct[x][i]
        for( i = 0; i < 8; i++ )
            DCT8_1D
#undef SRC
#undef DST
    return sad;
}

int sub16x16_dct8( int16_t dct[4][8][8], uint8_t *pix1, uint8_t *pix2, int stride1, int stride2  )
{
    int sad=0;
    sad += sub8x8_dct8( dct[0], &pix1[0],               &pix2[0], stride1, stride2 );
    sad += sub8x8_dct8( dct[1], &pix1[8],               &pix2[8], stride1, stride2 );
    sad += sub8x8_dct8( dct[2], &pix1[8*stride1+0], &pix2[8*stride2+0], stride1, stride2 );
    sad += sub8x8_dct8( dct[3], &pix1[8*stride1+8], &pix2[8*stride2+8], stride1, stride2 );
    return sad;
}

#define IDCT8_1D {\
    const int a0 =  SRC(0) + SRC(4);\
    const int a2 =  SRC(0) - SRC(4);\
    const int a4 = (SRC(2)>>1) - SRC(6);\
    const int a6 = (SRC(6)>>1) + SRC(2);\
    const int b0 = a0 + a6;\
    const int b2 = a2 + a4;\
    const int b4 = a2 - a4;\
    const int b6 = a0 - a6;\
    const int a1 = -SRC(3) + SRC(5) - SRC(7) - (SRC(7)>>1);\
    const int a3 =  SRC(1) + SRC(7) - SRC(3) - (SRC(3)>>1);\
    const int a5 = -SRC(1) + SRC(7) + SRC(5) + (SRC(5)>>1);\
    const int a7 =  SRC(3) + SRC(5) + SRC(1) + (SRC(1)>>1);\
    const int b1 = (a7>>2) + a1;\
    const int b3 =  a3 + (a5>>2);\
    const int b5 = (a3>>2) - a5;\
    const int b7 =  a7 - (a1>>2);\
    DST(0, b0 + b7,stride);\
    DST(1, b2 + b5,stride);\
    DST(2, b4 + b3,stride);\
    DST(3, b6 + b1,stride);\
    DST(4, b6 - b1,stride);\
    DST(5, b4 - b3,stride);\
    DST(6, b2 - b5,stride);\
    DST(7, b0 - b7,stride);\
}

void add8x8_idct8( uint8_t *dst, int16_t dct[8][8], int stride )
{
    int i;

    dct[0][0] += 32; // rounding for the >>6 at the end

#define SRC(x)     dct[x][i]
#define DST(x,rhs,stride) dct[x][i] = (rhs)
    for( i = 0; i < 8; i++ )
        IDCT8_1D
#undef SRC
#undef DST

#define SRC(x)     dct[i][x]
#define DST(x,rhs,stride) dst[i + x*stride] = f264_clip_uint8( dst[i + x*stride] + ((rhs) >> 6) );
        for( i = 0; i < 8; i++ )
            IDCT8_1D
#undef SRC
#undef DST
}

void add16x16_idct8( uint8_t *dst, int16_t dct[4][8][8], int stride )
{
    add8x8_idct8( &dst[0],          dct[0], stride );
    add8x8_idct8( &dst[8],          dct[1], stride );
    add8x8_idct8( &dst[8*stride+0], dct[2], stride );
    add8x8_idct8( &dst[8*stride+8], dct[3], stride );
}

