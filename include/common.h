/* 
 * File:   common.h
 * Author: ybfan
 *
 * Created on June 20, 2012
 */

#ifndef _COMMON_H
#define _COMMON_H
#include "type_1.h"

#define FDEC_STRIDE 32
#define FENC_STRIDE 16

#define COST_MAX (1<<28)
#define BIT_DEPTH 8

enum macroblock_position_e
{
    MB_LEFT     = 0x01,
    MB_TOP      = 0x02,
    MB_TOPRIGHT = 0x04,
    MB_TOPLEFT  = 0x08,
    ALL_NEIGHBORS = 0x0f,
};

enum mb_class_e
{
    I_4x4           = 0,
    I_8x8           = 1,
    I_16x16         = 2,
    I_PCM           = 3,

    P_L0            = 4,
    P_8x8           = 5,
    P_SKIP          = 6,

    B_DIRECT        = 7,
    B_L0_L0         = 8,
    B_L0_L1         = 9,
    B_L0_BI         = 10,
    B_L1_L0         = 11,
    B_L1_L1         = 12,
    B_L1_BI         = 13,
    B_BI_L0         = 14,
    B_BI_L1         = 15,
    B_BI_BI         = 16,
    B_8x8           = 17,
    B_SKIP          = 18,

    f264_MBTYPE_MAX = 19
};

enum mb_partition_e
{
    /* sub partition type for P_8x8 and B_8x8 */
    D_L0_4x4          = 0,
    D_L0_8x4          = 1,
    D_L0_4x8          = 2,
    D_L0_8x8          = 3,

    /* sub partition type for B_8x8 only */
    D_L1_4x4          = 4,
    D_L1_8x4          = 5,
    D_L1_4x8          = 6,
    D_L1_8x8          = 7,

    D_BI_4x4          = 8,
    D_BI_8x4          = 9,
    D_BI_4x8          = 10,
    D_BI_8x8          = 11,
    D_DIRECT_8x8      = 12,

    /* partition */
    D_8x8             = 13,
    D_16x8            = 14,
    D_8x16            = 15,
    D_16x16           = 16,
    f264_PARTTYPE_MAX = 17,
};

enum slice_type_e
{
    SLICE_TYPE_P  = 0,
    SLICE_TYPE_B  = 1,
    SLICE_TYPE_I  = 2,
    SLICE_TYPE_SP = 3,
    SLICE_TYPE_SI = 4
};

/* nal */
enum nal_unit_type_e
{
    NAL_UNKNOWN		= 0,
    NAL_SLICE		= 1,
    NAL_SLICE_DPA   = 2,
    NAL_SLICE_DPB   = 3,
    NAL_SLICE_DPC   = 4,
    NAL_SLICE_IDR   = 5,    /* ref_idc != 0 */
    NAL_SEI         = 6,    /* ref_idc == 0 */
    NAL_SPS         = 7,
    NAL_PPS         = 8,
    NAL_AUD         = 9,
    /* ref_idc == 0 for 6,9,10,11,12 */
};

enum nal_priority_e
{
    NAL_PRIORITY_DISPOSABLE = 0,
    NAL_PRIORITY_LOW        = 1,
    NAL_PRIORITY_HIGH       = 2,
    NAL_PRIORITY_HIGHEST    = 3,
};

enum profile_e
{
    PROFILE_BASELINE = 66,
    PROFILE_MAIN     = 77,
    PROFILE_EXTENDED = 88,
    PROFILE_HIGH    = 100,
    PROFILE_HIGH10  = 110,
    PROFILE_HIGH422 = 122,
    PROFILE_HIGH444 = 144,
    PROFILE_HIGH444_PREDICTIVE = 244,
};

/* lambda = pow(2,qp/6-2) */
const int lambda_tab[52] = {
   1, 1, 1, 1, 1, 1, 1, 1,  /*  0-7 */
   1, 1, 1, 1,              /*  8-11 */
   1, 1, 1, 1, 2, 2, 2, 2,  /* 12-19 */
   3, 3, 3, 4, 4, 4, 5, 6,  /* 20-27 */
   6, 7, 8, 9,10,11,13,14,  /* 28-35 */
  16,18,20,23,25,29,32,36,  /* 36-43 */
  40,45,51,57,64,72,81,91   /* 44-51 */
};

#define QP_MAX_MAX (51+2*6)
//table for lagrangian multiplier
/* lambda = pow(2,qp/6-2) */
const uint16_t f264_lambda_tab[QP_MAX_MAX+1] = {
	1,   1,   1,   1,   1,   1,   1,   1, /*  0- 7 */
	1,   1,   1,   1,   1,   1,   1,   1, /*  8-15 */
	2,   2,   2,   2,   3,   3,   3,   4, /* 16-23 */
	4,   4,   5,   6,   6,   7,   8,   9, /* 24-31 */
	10,  11,  13,  14,  16,  18,  20,  23, /* 32-39 */
	25,  29,  32,  36,  40,  45,  51,  57, /* 40-47 */
	64,  72,  81,  91, 102, 114, 128, 144, /* 48-55 */
	161, 181, 203, 228, 256, 287, 323, 362, /* 56-63 */
};

static const uint8_t mb_partition_listX_table[2][17] =
{{
    1, 1, 1, 1, /* D_L0_* */
    0, 0, 0, 0, /* D_L1_* */
    1, 1, 1, 1, /* D_BI_* */
    0,          /* D_DIRECT_8x8 */
    0, 0, 0, 0  /* 8x8 .. 16x16 */
},
{
    0, 0, 0, 0, /* D_L0_* */
    1, 1, 1, 1, /* D_L1_* */
    1, 1, 1, 1, /* D_BI_* */
    0,          /* D_DIRECT_8x8 */
    0, 0, 0, 0  /* 8x8 .. 16x16 */
}};

/*
   0 1 2 3 4 5 6 7
 0
 1   B B   L L L L
 2   B B   L L L L
 3         L L L L
 4   R R   L L L L
 5   R R   DyDuDv
*/
static const int f264_scan8[16+2*4+3] =
{
    /* Luma */
    4+1*8, 5+1*8, 4+2*8, 5+2*8,
    6+1*8, 7+1*8, 6+2*8, 7+2*8,
    4+3*8, 5+3*8, 4+4*8, 5+4*8,
    6+3*8, 7+3*8, 6+4*8, 7+4*8,

    /* Cb */
    1+1*8, 2+1*8,
    1+2*8, 2+2*8,

    /* Cr */
    1+4*8, 2+4*8,
    1+5*8, 2+5*8,

    /* Luma DC */
    4+5*8,

    /* Chroma DC */
    5+5*8, 6+5*8
};

static const uint8_t block_idx_xy_1d[16] =
{
    0, 1, 4, 5, 2, 3, 6, 7, 8, 9, 12, 13, 10, 11, 14, 15
};

static const uint8_t block_idx_xy_fenc[16] =
{
    0*4 + 0*4*FENC_STRIDE, 1*4 + 0*4*FENC_STRIDE,
    0*4 + 1*4*FENC_STRIDE, 1*4 + 1*4*FENC_STRIDE,
    2*4 + 0*4*FENC_STRIDE, 3*4 + 0*4*FENC_STRIDE,
    2*4 + 1*4*FENC_STRIDE, 3*4 + 1*4*FENC_STRIDE,
    0*4 + 2*4*FENC_STRIDE, 1*4 + 2*4*FENC_STRIDE,
    0*4 + 3*4*FENC_STRIDE, 1*4 + 3*4*FENC_STRIDE,
    2*4 + 2*4*FENC_STRIDE, 3*4 + 2*4*FENC_STRIDE,
    2*4 + 3*4*FENC_STRIDE, 3*4 + 3*4*FENC_STRIDE
};

static const uint16_t block_idx_xy_fdec[16] =
{
    0*4 + 0*4*FDEC_STRIDE, 1*4 + 0*4*FDEC_STRIDE,
    0*4 + 1*4*FDEC_STRIDE, 1*4 + 1*4*FDEC_STRIDE,
    2*4 + 0*4*FDEC_STRIDE, 3*4 + 0*4*FDEC_STRIDE,
    2*4 + 1*4*FDEC_STRIDE, 3*4 + 1*4*FDEC_STRIDE,
    0*4 + 2*4*FDEC_STRIDE, 1*4 + 2*4*FDEC_STRIDE,
    0*4 + 3*4*FDEC_STRIDE, 1*4 + 3*4*FDEC_STRIDE,
    2*4 + 2*4*FDEC_STRIDE, 3*4 + 2*4*FDEC_STRIDE,
    2*4 + 3*4*FDEC_STRIDE, 3*4 + 3*4*FDEC_STRIDE
};

static inline uint8_t f264_clip_uint8( int x )
{
    return x&(~255) ? (-x)>>31 : x;
}

int pixel_satd_wxh( uint8_t *pix1, int i_pix1, uint8_t *pix2, int i_pix2, int i_width, int i_height );
void pixel_copy_wxh( int y_size, int x_size, uint8_t *pix1, int i_pix1, uint8_t *pix2, int i_pix2 );

#define COPY2_IF_LT(x,y,a,b)\
if((y)<(x))\
{\
    (x)=(y);\
    (a)=(b);\
}

#define f264_MIN(a,b) ( (a)<(b) ? (a) : (b) )

#define Clip3(x,y,z) (((z)<(x))?(x):(((z)>(y))?(y):(z)))
#define Cliply(x) Clip3(0,((1<<BIT_DEPTH)-1),(x))

#define Abs(x) (((x)>=0)?(x):(-x))

/****************************************************
*                    scan 
*****************************************************/
#define ZIG(i,y,x) level[i] = dct[x][y];

static inline void scan_zigzag_8x8full( int level[64], short dct[8][8] )
{
    ZIG( 0,0,0) ZIG( 1,0,1) ZIG( 2,1,0) ZIG( 3,2,0)
    ZIG( 4,1,1) ZIG( 5,0,2) ZIG( 6,0,3) ZIG( 7,1,2)
    ZIG( 8,2,1) ZIG( 9,3,0) ZIG(10,4,0) ZIG(11,3,1)
    ZIG(12,2,2) ZIG(13,1,3) ZIG(14,0,4) ZIG(15,0,5)
    ZIG(16,1,4) ZIG(17,2,3) ZIG(18,3,2) ZIG(19,4,1)
    ZIG(20,5,0) ZIG(21,6,0) ZIG(22,5,1) ZIG(23,4,2)
    ZIG(24,3,3) ZIG(25,2,4) ZIG(26,1,5) ZIG(27,0,6)
    ZIG(28,0,7) ZIG(29,1,6) ZIG(30,2,5) ZIG(31,3,4)
    ZIG(32,4,3) ZIG(33,5,2) ZIG(34,6,1) ZIG(35,7,0)
    ZIG(36,7,1) ZIG(37,6,2) ZIG(38,5,3) ZIG(39,4,4)
    ZIG(40,3,5) ZIG(41,2,6) ZIG(42,1,7) ZIG(43,2,7)
    ZIG(44,3,6) ZIG(45,4,5) ZIG(46,5,4) ZIG(47,6,3)
    ZIG(48,7,2) ZIG(49,7,3) ZIG(50,6,4) ZIG(51,5,5)
    ZIG(52,4,6) ZIG(53,3,7) ZIG(54,4,7) ZIG(55,5,6)
    ZIG(56,6,5) ZIG(57,7,4) ZIG(58,7,5) ZIG(59,6,6)
    ZIG(60,5,7) ZIG(61,6,7) ZIG(62,7,6) ZIG(63,7,7)
}
static inline void scan_zigzag_4x4full( int level[16], short dct[4][4] )
{
    ZIG( 0,0,0) ZIG( 1,0,1) ZIG( 2,1,0) ZIG( 3,2,0)
    ZIG( 4,1,1) ZIG( 5,0,2) ZIG( 6,0,3) ZIG( 7,1,2)
    ZIG( 8,2,1) ZIG( 9,3,0) ZIG(10,3,1) ZIG(11,2,2)
    ZIG(12,1,3) ZIG(13,2,3) ZIG(14,3,2) ZIG(15,3,3)
}
static inline void scan_zigzag_4x4( int level[15], short dct[4][4] )//ac
{
                ZIG( 0,0,1) ZIG( 1,1,0) ZIG( 2,2,0)
    ZIG( 3,1,1) ZIG( 4,0,2) ZIG( 5,0,3) ZIG( 6,1,2)
    ZIG( 7,2,1) ZIG( 8,3,0) ZIG( 9,3,1) ZIG(10,2,2)
    ZIG(11,1,3) ZIG(12,2,3) ZIG(13,3,2) ZIG(14,3,3)
}

static inline void scan_zigzag_2x2_dc( int level[4], short dct[2][2] )
{
    ZIG(0,0,0)
    ZIG(1,0,1)
    ZIG(2,1,0)
    ZIG(3,1,1)
}
#undef ZIG

void zigzag_sub_8x8_frame( int16_t level[64], const uint8_t *p_src, uint8_t *p_dst );
void zigzag_sub_4x4_frame( int16_t level[16], const uint8_t *p_src, uint8_t *p_dst );
void zigzag_scan_4x4_frame( int16_t level[16], int16_t dct[4][4] );
void zigzag_scan_8x8_frame( int16_t level[64], int16_t dct[8][8] );
void zigzag_scan_2x2_dc( int16_t level[4], int16_t dct[2][2] );


const int zscan[16][16] =
{
	{  0,  1,  4,  5, 16, 17, 20, 21, 64, 65, 68, 69, 80, 81, 84, 85},
	{  2,  3,  6,  7, 18, 19, 22, 23, 66, 67, 70, 71, 82, 83, 86, 87},
	{  8,  9, 12, 13, 24, 25, 28, 29, 72, 73, 76, 77, 88, 89, 92, 93},
	{ 10, 11, 14, 15, 26, 27, 30, 31, 74, 75, 78, 79, 90, 91, 94, 95},
	{ 32, 33, 36, 37, 48, 49, 52, 53, 96, 97,100,101,112,113,116,117},
	{ 34, 35, 38, 39, 50, 51, 54, 55, 98, 99,102,103,114,115,118,119},
	{ 40, 41, 44, 45, 56, 57, 60, 61,104,105,108,109,120,121,124,125},
	{ 42, 43, 46, 47, 58, 59, 62, 63,106,107,110,111,122,123,126,127},
	{128,129,132,133,144,145,148,149,192,193,196,197,208,209,212,213},
	{130,131,134,135,146,147,150,151,194,195,198,199,210,211,214,215},
	{136,137,140,141,152,153,156,157,200,201,204,205,216,217,220,221},
	{138,139,142,143,154,155,158,159,202,203,206,207,218,219,222,223},
	{160,161,164,165,176,177,180,181,224,225,228,229,240,241,244,245},
	{162,163,166,167,178,179,182,183,226,227,230,231,242,243,246,247},
	{168,169,172,173,184,185,188,189,232,233,236,237,248,249,252,253},
	{170,171,174,175,186,187,190,191,234,235,238,239,250,251,254,255}
};

const int zscan_to_raster_4x4[16] = 
{
	0,  1,  4,  5, 
	2,  3,  6,  7,
	8,  9, 12, 13, 
	10, 11, 14, 15
};

const int raster_to_zscan_4x4[16] = 
{
	0,  1,  4,  5, 
	2,  3,  6,  7,
	8,  9, 12, 13, 
	10, 11, 14, 15
};

const int zscan_to_raster_2x2[4] =
{
	0, 1,
	2, 3
};


#define array_non_zero(a) array_non_zero_int(a, sizeof(a))
#define array_non_zero_int array_non_zero_int_c
static inline int array_non_zero_int_c( void *v, int i_count )
{
	uint64_t *x = (uint64_t *)v;
	//uint64_t *x = v;
    if(i_count == 8)
        return !!x[0];
    else if(i_count == 16)
        return !!(x[0]|x[1]);
    else if(i_count == 32)
        return !!(x[0]|x[1]|x[2]|x[3]);
    else
    {
        int i;
        i_count /= sizeof(uint64_t);
        for( i = 0; i < i_count; i++ )
            if( x[i] ) return 1;
        return 0;
    }
}

#ifdef _NODEBUG

#define LOG_START(t, e, x, y, q) ((void)0)
#define LOG_END(t,e) ((void)0)
#define LOG_BS(t,p,x,y)  ((void)0)
#define LOG(x) ((void)0)

#elif  _DEBUG

#define LOG_START(t, e, x, y, q) \
	cout <<t<< "# "<<e<<" start, mb_x:"<<x<<" mb_y:"<<y<<" qp:"<<q<<endl;

#define LOG_END(t,e) \
	cout << t << "# "<<e<<" done" <<endl;

#define LOG_BS(t,p,x,y) \
	if(p == "h") {\
	cout<<t<< "# bs header start,frame No.:"<<x<<" frame qp:"<<y<<endl;\
	}\
	else{\
	cout<<t<< "# bs packer start,mb_x:"<<x<<" mb_y:"<<y<<endl;\
}

#define LOG(x) cout<<x<<endl;

#endif 


#endif
