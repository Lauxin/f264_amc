#include "common.h"

#define HADAMARD4(d0,d1,d2,d3,s0,s1,s2,s3) {\
    int t0 = s0 + s1;\
    int t1 = s0 - s1;\
    int t2 = s2 + s3;\
    int t3 = s2 - s3;\
    d0 = t0 + t2;\
    d2 = t0 - t2;\
    d1 = t1 + t3;\
    d3 = t1 - t3;\
}

/****************************************************************************
 * pixel_satd_WxH: sum of 4x4 Hadamard transformed differences
 ****************************************************************************/
int pixel_satd_wxh( uint8_t *pix1, int i_pix1, uint8_t *pix2, int i_pix2, int i_width, int i_height )
{
    int16_t tmp[4][4];
    int x, y;
    int i_satd = 0;

    for( y = 0; y < i_height; y += 4 )
    {
        for( x = 0; x < i_width; x += 4 )
        {
            int i;
            uint8_t *p1 = pix1+x, *p2 = pix2+x;

            for( i=0; i<4; i++, p1+=i_pix1, p2+=i_pix2 )
            {
                int a0 = p1[0] - p2[0];
                int a1 = p1[1] - p2[1];
                int a2 = p1[2] - p2[2];
                int a3 = p1[3] - p2[3];
                HADAMARD4( tmp[i][0], tmp[i][1], tmp[i][2], tmp[i][3], a0,a1,a2,a3 );
            }
            for( i=0; i<4; i++ )
            {
                int a0,a1,a2,a3;
                HADAMARD4( a0,a1,a2,a3, tmp[0][i], tmp[1][i], tmp[2][i], tmp[3][i] );
                i_satd += abs(a0) + abs(a1) + abs(a2) + abs(a3);
            }

        }
        pix1 += 4 * i_pix1;
        pix2 += 4 * i_pix2;
    }

    return i_satd / 2;
}


void pixel_copy_wxh( int y_size, int x_size, uint8_t *pix1, int i_pix1, uint8_t *pix2, int i_pix2 )
{
    int y, x;
    for( y = 0; y < y_size; y++ )
    {
        for( x = 0; x < x_size; x++ )
        {
            pix1[x] = pix2[x];
        }
        pix1 += i_pix1;
        pix2 += i_pix2;
    }
}

/****************************************************************************
 *                 scan
 ****************************************************************************/
#define ZIG(i,y,x) level[i] = dct[0][x*8+y];
#define ZIGZAG8_FRAME\
    ZIG( 0,0,0) ZIG( 1,0,1) ZIG( 2,1,0) ZIG( 3,2,0)\
    ZIG( 4,1,1) ZIG( 5,0,2) ZIG( 6,0,3) ZIG( 7,1,2)\
    ZIG( 8,2,1) ZIG( 9,3,0) ZIG(10,4,0) ZIG(11,3,1)\
    ZIG(12,2,2) ZIG(13,1,3) ZIG(14,0,4) ZIG(15,0,5)\
    ZIG(16,1,4) ZIG(17,2,3) ZIG(18,3,2) ZIG(19,4,1)\
    ZIG(20,5,0) ZIG(21,6,0) ZIG(22,5,1) ZIG(23,4,2)\
    ZIG(24,3,3) ZIG(25,2,4) ZIG(26,1,5) ZIG(27,0,6)\
    ZIG(28,0,7) ZIG(29,1,6) ZIG(30,2,5) ZIG(31,3,4)\
    ZIG(32,4,3) ZIG(33,5,2) ZIG(34,6,1) ZIG(35,7,0)\
    ZIG(36,7,1) ZIG(37,6,2) ZIG(38,5,3) ZIG(39,4,4)\
    ZIG(40,3,5) ZIG(41,2,6) ZIG(42,1,7) ZIG(43,2,7)\
    ZIG(44,3,6) ZIG(45,4,5) ZIG(46,5,4) ZIG(47,6,3)\
    ZIG(48,7,2) ZIG(49,7,3) ZIG(50,6,4) ZIG(51,5,5)\
    ZIG(52,4,6) ZIG(53,3,7) ZIG(54,4,7) ZIG(55,5,6)\
    ZIG(56,6,5) ZIG(57,7,4) ZIG(58,7,5) ZIG(59,6,6)\
    ZIG(60,5,7) ZIG(61,6,7) ZIG(62,7,6) ZIG(63,7,7)
void zigzag_scan_8x8_frame( int16_t level[64], int16_t dct[8][8] )
{
    ZIGZAG8_FRAME
}
#undef ZIG

#define ZIG(i,y,x) level[i] = dct[0][x*4+y];
#define ZIGZAG4_FRAME\
    ZIG( 0,0,0) ZIG( 1,0,1) ZIG( 2,1,0) ZIG( 3,2,0)\
    ZIG( 4,1,1) ZIG( 5,0,2) ZIG( 6,0,3) ZIG( 7,1,2)\
    ZIG( 8,2,1) ZIG( 9,3,0) ZIG(10,3,1) ZIG(11,2,2)\
    ZIG(12,1,3) ZIG(13,2,3) ZIG(14,3,2) ZIG(15,3,3)
void zigzag_scan_4x4_frame( int16_t level[16], int16_t dct[4][4] )
{
    ZIGZAG4_FRAME
}
#undef ZIG

#define ZIG(i,y,x) {\
    int oe = x+y*FENC_STRIDE;\
    int od = x+y*FDEC_STRIDE;\
    level[i] = p_src[oe] - p_dst[od];\
}
#define COPY4x4\
    *(uint32_t*)(p_dst+0*FDEC_STRIDE) = *(uint32_t*)(p_src+0*FENC_STRIDE);\
    *(uint32_t*)(p_dst+1*FDEC_STRIDE) = *(uint32_t*)(p_src+1*FENC_STRIDE);\
    *(uint32_t*)(p_dst+2*FDEC_STRIDE) = *(uint32_t*)(p_src+2*FENC_STRIDE);\
    *(uint32_t*)(p_dst+3*FDEC_STRIDE) = *(uint32_t*)(p_src+3*FENC_STRIDE);
#define COPY8x8\
    *(uint64_t*)(p_dst+0*FDEC_STRIDE) = *(uint64_t*)(p_src+0*FENC_STRIDE);\
    *(uint64_t*)(p_dst+1*FDEC_STRIDE) = *(uint64_t*)(p_src+1*FENC_STRIDE);\
    *(uint64_t*)(p_dst+2*FDEC_STRIDE) = *(uint64_t*)(p_src+2*FENC_STRIDE);\
    *(uint64_t*)(p_dst+3*FDEC_STRIDE) = *(uint64_t*)(p_src+3*FENC_STRIDE);\
    *(uint64_t*)(p_dst+4*FDEC_STRIDE) = *(uint64_t*)(p_src+4*FENC_STRIDE);\
    *(uint64_t*)(p_dst+5*FDEC_STRIDE) = *(uint64_t*)(p_src+5*FENC_STRIDE);\
    *(uint64_t*)(p_dst+6*FDEC_STRIDE) = *(uint64_t*)(p_src+6*FENC_STRIDE);\
    *(uint64_t*)(p_dst+7*FDEC_STRIDE) = *(uint64_t*)(p_src+7*FENC_STRIDE);

void zigzag_sub_4x4_frame( int16_t level[16], const uint8_t *p_src, uint8_t *p_dst )
{
    ZIGZAG4_FRAME
    COPY4x4
}

void zigzag_sub_8x8_frame( int16_t level[64], const uint8_t *p_src, uint8_t *p_dst )
{
    ZIGZAG8_FRAME
    COPY8x8
}

#undef ZIG
#undef COPY4x4

#define ZIG(i,y,x) level[i] = dct[x][y];
void zigzag_scan_2x2_dc( int16_t level[4], int16_t dct[2][2] )
{
    ZIG(0,0,0)
    ZIG(1,0,1)
    ZIG(2,1,0)
    ZIG(3,1,1)
}
#undef ZIG