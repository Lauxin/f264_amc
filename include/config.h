/* 
 * File:   config.h
 * Author: ybfan
 *
 * Created on May 3, 2012
 */


#ifndef _CONFIG_H
#define	_CONFIG_H

#define YUV_FILE "E:\\ms\\sequence\\foreman_qcif.yuv"
//#define YUV_FILE "/Users/jacob/foreman_qcif.yuv"


//#define    QP_RANDOM
//#define  DUMP_CUR_MB_SIM


/**************************************************************************************
*                                Encoder Setting                                      *
***************************************************************************************/
#define QCIF 1

//FRAMEWIDTH & FRAMEHEIGHT
#ifdef QCIF
#define FRAMEWIDTH 176
#define FRAMEHEIGHT 144
#elif CIF
#define FRAMEWIDTH 352
#define FRAMEHEIGHT 288
#elif D1
#define FRAMEWIDTH 720
#define FRAMEHEIGHT 576
#elif p720
#define FRAMEWIDTH 1280
#define FRAMEHEIGHT 720
#elif p1080
#define FRAMEWIDTH 1920
#define FRAMEHEIGHT 1080
#endif


#define f_LCU_SIZE              16

#define MB_X_TOTAL				((FRAMEWIDTH+15)/f_LCU_SIZE)
#define MB_Y_TOTAL				((FRAMEHEIGHT+15)/f_LCU_SIZE)

#define FRAME_PER_SECOND		25
#define GOP_LENGTH				2
#define FRAME_TOTAL				2

#define FRAME_START				0

#define SW_W                    48
#define SW_H                    48

#define BS_BUF_SIZE				800

#define INIT_QP					22

#define f264_CABAC              0

#define f264_SLICE_MAX			4
#define f264_NAL_MAX			(4 + f264_SLICE_MAX)
#define f264_CQM_FLAT			0
/**************************************************************************************
*                                  Analyse flags                                      *
***************************************************************************************/
#define f264_ANALYSE_I16x16		1
#define f264_ANALYSE_I4x4		1
#define f264_ANALYSE_PSUB8x8    1
#define f264_PE_NUM			    4

#endif	/* _CONFIG_H */