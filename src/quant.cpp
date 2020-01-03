#include "quant.h"
#include "math.h"

static inline int iabs(int x)
{
  static const int INT_BITS = (sizeof(int) * CHAR_BIT) - 1;
  int y = x >> INT_BITS;
  return (x ^ y) - y;
}

static inline int isignab(int a, int b)
{
  return ((b) < 0) ? -iabs(a) : iabs(a);
}


void quant_4x4( int16_t dct[4][4], uint16_t mf[16], uint32_t bias[16],int i_qp)
{
    int i;
    int m7;
	 int scaled_coeff;
     int   level;
    int qbits = 15 + i_qp/6;
    for( i = 0; i < 16; i++ )
    {  
    	 m7 = dct[0][i];     
    	 scaled_coeff = iabs (dct[0][i]) * mf[i];
    	 level = (scaled_coeff + bias[i]) >> qbits;
    	 dct[0][i]  = isignab(level, m7);
    	}
}

void quant_8x8( int16_t dct[8][8], uint16_t mf[64], uint32_t bias[64],int i_qp)
{
    int i;
    int m7;
	int scaled_coeff;
    int   level;
    int qbits = 16 + i_qp/6;
    for( i = 0; i < 64; i++ )
    {  
    	 m7 = dct[0][i];     
    	 scaled_coeff = iabs (dct[0][i]) * mf[i];
    	 level = (scaled_coeff + bias[i]) >> qbits;
    	 dct[0][i]  = isignab(level, m7);
    	}
}


void quant_4x4_dc( int16_t dct[4][4], int mf, int bias,int i_qp )
{
    int i;
    int m7;
	int scaled_coeff;
    int   level;
    int qbits = 15 + i_qp/6 + 1;//
    for( i = 0; i < 16; i++ )
    {  
    	 m7 = dct[0][i];     
    	 scaled_coeff = iabs (dct[0][i]) * mf;
    	 level = (scaled_coeff + bias) >> qbits;
    	 dct[0][i]  = isignab(level, m7);
    	}
}

void quant_2x2_dc( int16_t dct[2][2], int mf, int bias,int i_qp  )
{
    int i;
    int m7;
	int scaled_coeff;
    int   level;
    int qbits = 15 + i_qp/6 + 1;
    for( i = 0; i < 4; i++ )
    {  
    	 m7 = dct[0][i];     
    	 scaled_coeff = iabs (dct[0][i]) * mf;
    	 level = (scaled_coeff + bias) >> qbits;
    	 dct[0][i]  = isignab(level, m7);
    	}
    
}



//*************jm method of dequant
//static inline int rshift_rnd_sf(int x, int a)
//{
//  return ((x + (1 << (a-1) )) >> a);
//}
//
//*m7    = rshift_rnd_sf(((level * q_params->InvScaleComp) << qp_per), 4);
//// inverse scale can be alternative performed as follows to ensure 16bitarithmetic is satisfied.
//// *m7 = (qp_per<4) ? rshift_rnd_sf((level*q_params->InvScaleComp),4-qp_per) : (level*q_params->InvScaleComp)<<(qp_per-4);


//***************************still f264        
#define DEQUANT_SHL( x ) \
    dct[y][x] = ( dct[y][x] * dequant_mf[i_mf][y][x] ) << i_qbits

#define DEQUANT_SHR( x ) \
    dct[y][x] = ( dct[y][x] * dequant_mf[i_mf][y][x] + f ) >> (-i_qbits)
        
void dequant_4x4( int16_t dct[4][4], int dequant_mf[6][4][4], int i_qp )
{
    const int i_mf = i_qp%6;
    const int i_qbits = i_qp/6 - 4;
    int y;

    if( i_qbits >= 0 )
    {
        for( y = 0; y < 4; y++ )
        {
            DEQUANT_SHL( 0 );
            DEQUANT_SHL( 1 );
            DEQUANT_SHL( 2 );
            DEQUANT_SHL( 3 );
        }
    }
    else
    {
        const int f = 1 << (-i_qbits-1);
        for( y = 0; y < 4; y++ )
        {
            DEQUANT_SHR( 0 );
            DEQUANT_SHR( 1 );
            DEQUANT_SHR( 2 );
            DEQUANT_SHR( 3 );
        }
    }
}

void dequant_8x8( int16_t dct[8][8], int dequant_mf[6][8][8], int i_qp )
{
    const int i_mf = i_qp%6;
    const int i_qbits = i_qp/6 - 6;
    int y;

    if( i_qbits >= 0 )
    {
        for( y = 0; y < 8; y++ )
        {
            DEQUANT_SHL( 0 );
            DEQUANT_SHL( 1 );
            DEQUANT_SHL( 2 );
            DEQUANT_SHL( 3 );
            DEQUANT_SHL( 4 );
            DEQUANT_SHL( 5 );
            DEQUANT_SHL( 6 );
            DEQUANT_SHL( 7 );
        }
    }
    else
    {
        const int f = 1 << (-i_qbits-1);
        for( y = 0; y < 8; y++ )
        {
            DEQUANT_SHR( 0 );
            DEQUANT_SHR( 1 );
            DEQUANT_SHR( 2 );
            DEQUANT_SHR( 3 );
            DEQUANT_SHR( 4 );
            DEQUANT_SHR( 5 );
            DEQUANT_SHR( 6 );
            DEQUANT_SHR( 7 );
        }
    }
}


void dequant_4x4_dc( int16_t dct[4][4], int dequant_mf[6][4][4], int i_qp )
{
    const int i_qbits = i_qp/6 - 6;
    int y;

    if( i_qbits >= 0 )
    {
        const int i_dmf = dequant_mf[i_qp%6][0][0] << i_qbits;

        for( y = 0; y < 4; y++ )
        {
            dct[y][0] *= i_dmf;
            dct[y][1] *= i_dmf;
            dct[y][2] *= i_dmf;
            dct[y][3] *= i_dmf;
        }
    }
    else
    {
        const int i_dmf = dequant_mf[i_qp%6][0][0];
        const int f = 1 << (-i_qbits-1);

        for( y = 0; y < 4; y++ )
        {
            dct[y][0] = ( dct[y][0] * i_dmf + f ) >> (-i_qbits);
            dct[y][1] = ( dct[y][1] * i_dmf + f ) >> (-i_qbits);
            dct[y][2] = ( dct[y][2] * i_dmf + f ) >> (-i_qbits);
            dct[y][3] = ( dct[y][3] * i_dmf + f ) >> (-i_qbits);
        }
    }
}
