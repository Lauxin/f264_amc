/*
 * File:   intra.h
 * Author: Yixuan Zeng
 *
 * Created on October 31, 2018, 2:18 PM
 */

#ifndef _INTRA_H
#define	_INTRA_H

#include "type_1.h"
#include "config.h"

#define  DUMP_INTRA_CHECK
//#define  GET_CHECK

 /****************************************************************************
  *						intra prediction function pointers
  ****************************************************************************/
typedef void(*predict_16x16_t)(unsigned char *src);
typedef void(*predict_4x4_t)(unsigned char *src);
typedef void(*predict_8x8c_t)(unsigned char *src);


//亮度色度的预测模式
enum intra16x16_pred_e
{
	I_PRED_16x16_V = 0,
	I_PRED_16x16_H = 1,
	I_PRED_16x16_DC = 2,
	I_PRED_16x16_P = 3,

	I_PRED_16x16_DC_LEFT = 4,
	I_PRED_16x16_DC_TOP = 5,
	I_PRED_16x16_DC_128 = 6,
};
enum intra_chroma_pred_e
{
	I_PRED_CHROMA_DC = 0,
	I_PRED_CHROMA_H = 1,
	I_PRED_CHROMA_V = 2,
	I_PRED_CHROMA_P = 3,

	I_PRED_CHROMA_DC_LEFT = 4,
	I_PRED_CHROMA_DC_TOP = 5,
	I_PRED_CHROMA_DC_128 = 6
};
enum intra4x4_pred_e
{
	I_PRED_4x4_V = 0,
	I_PRED_4x4_H = 1,
	I_PRED_4x4_DC = 2,
	I_PRED_4x4_DDL = 3,
	I_PRED_4x4_DDR = 4,
	I_PRED_4x4_VR = 5,
	I_PRED_4x4_HD = 6,
	I_PRED_4x4_VL = 7,
	I_PRED_4x4_HU = 8,

	I_PRED_4x4_DC_LEFT = 9,
	I_PRED_4x4_DC_TOP = 10,
	I_PRED_4x4_DC_128 = 11,
};

//predict mode for luma4x4
static const int8_t mb_pred_mode4x4_fix[13] =
{
	-1,
	I_PRED_4x4_V,   I_PRED_4x4_H,   I_PRED_4x4_DC,
	I_PRED_4x4_DDL, I_PRED_4x4_DDR, I_PRED_4x4_VR,
	I_PRED_4x4_HD,  I_PRED_4x4_VL,  I_PRED_4x4_HU,
	I_PRED_4x4_DC,  I_PRED_4x4_DC,  I_PRED_4x4_DC
};
#define mb_pred_mode4x4_fix(t) mb_pred_mode4x4_fix[(t)+1]

static const uint8_t mb_pred_mode8x8c_fix[7] =
{
	I_PRED_CHROMA_DC, I_PRED_CHROMA_H, I_PRED_CHROMA_V, I_PRED_CHROMA_P,
	I_PRED_CHROMA_DC, I_PRED_CHROMA_DC,I_PRED_CHROMA_DC
};

static const uint8_t mb_pred_mode16x16_fix[7] =
{
	I_PRED_16x16_V, I_PRED_16x16_H, I_PRED_16x16_DC, I_PRED_16x16_P,
	I_PRED_16x16_DC,I_PRED_16x16_DC,I_PRED_16x16_DC
};

//8x8 chroma
	void predict_8x8c_dc_128(uint8_t *src);
	void predict_8x8c_dc_left(uint8_t *src);
	void predict_8x8c_dc_top(uint8_t *src);
	void predict_8x8c_dc(uint8_t *src);
	void predict_8x8c_h(uint8_t *src);
	void predict_8x8c_v(uint8_t *src);
	void predict_8x8c_p(uint8_t *src);
	//16x16 luma
	void predict_16x16_dc(uint8_t *src);
	void predict_16x16_dc_left(uint8_t *src);
	void predict_16x16_dc_top(uint8_t *src);
	void predict_16x16_dc_128(uint8_t *src);
	void predict_16x16_h(uint8_t *src);
	void predict_16x16_v(uint8_t *src);
	void predict_16x16_p(uint8_t *src);
	//4x4 luma
	void predict_4x4_dc_128(uint8_t *src);
	void predict_4x4_dc_left(uint8_t *src);
	void predict_4x4_dc_top(uint8_t *src);
	void predict_4x4_dc(uint8_t *src);
	void predict_4x4_h(uint8_t *src);
	void predict_4x4_v(uint8_t *src);
	void predict_4x4_ddl(uint8_t *src);
	void predict_4x4_ddr(uint8_t *src);
	void predict_4x4_vr(uint8_t *src);
	void predict_4x4_hd(uint8_t *src);
	void predict_4x4_vl(uint8_t *src);
	void predict_4x4_hu(uint8_t *src);


/****************************************************************************
 *					 intra class
 ****************************************************************************/
class intra {

private:


	/*****************************************
	*              private var               *
	*****************************************/
	// input var
	mb_t cur_mb;
	cqm_t cqm;
	param_t param;
	// output var
	intra_t intra_output;
	// pixel var
	uint8_t fenc[24][16];     // cur_mb 
	uint8_t fdec[27][32];     // predict_mb (for 16x16 luma & chroma)
	uint8_t fdec2[18][32];    // predict_mb (for 4x4 luma)
	uint8_t* pixel_line;  // Y-U-V(32p)上方参考像素
	uint8_t pixel_topleft[3];           // Y-U-V(3p)左上角参考像素
	uint8_t i_neighbour;                // 16x16 neighbour
	uint8_t i_neighbour4[16];           // 4x4 neighbour
	// mode var
	int8_t  mb_type;//宏块类型
	int8_t  intra16x16_mode;            // 16x16 mode
	int8_t  intra4x4_mode[16];          // 4x4 mode
	int8_t  chroma_mode;                // 8x8 chroma mode
	int8_t* mode_line; //上方参考的预测模式

	int8_t  intra4x4_pred_mode[16];     // 4x4 predicted mode
	int8_t  intra4x4_pred_mode_cache[40]; // 8x5 pred_mode array
	//标志是否使用了参考的预测模式
	int8_t    prev_intra4x4_pred_mode_flag[16];

	// cost var
	int        i_lambda;
	//用于比较，进行模式判决
	int        i_satd_i16x16;
	int        i_satd_i4x4_total;

	int        sad4;
	int        sad16;

	// transform var
	int16_t luma_dct4x4_z4[16][16];         // zig-zag scan of coeff for I_4x4.
	int16_t luma_dct4x4_z16[16][16];        // zig-zag scan of coeff for I_16x16.
	int16_t luma_dct4x4_dc_z4[16];
	int16_t chroma_dct4x4_z4[2][4][16];   //zig-zag scan of coeff for chroma
	int16_t chroma_dct4x4_dc_z2[2][4];

	//cbp & non_zero_count 
	uint8_t non_zero_count[27];
	int16_t cbp;
	int cbp_dc;
	int cbp_luma;
	int cbp_chroma;

	//function pointer
	predict_16x16_t predict_16x16[7];
	predict_4x4_t   predict_4x4[12];
	predict_8x8c_t  predict_8x8c[7];

	//dump
	FILE *fp;
	FILE *fp_lc;
	FILE *fp_cong;


	/*****************************************
	*              private func              *
	*****************************************/
	
	// function init(函数指针）
	void predict_8x8c_init(predict_8x8c_t predf_chroma[7]);
	void predict_16x16_init(predict_16x16_t predf_16x16[7]);
	void predict_4x4_init(predict_4x4_t predf_4x4[12]);
	// mode available
	void predict_16x16_mode_available(uint8_t i_neighbour, int8_t *mode, int8_t *mode_cnt);
	void predict_8x8c_mode_available(uint8_t i_neighbour, int8_t *mode, int8_t *mode_cnt);
	void predict_luma_mode_available(int8_t i, uint8_t i_neighbour, int8_t *mode, int8_t *mode_cnt);//(for 4x4 & 8x8)

	//prediction function
	void predict_luma16x16();
	void predict_luma4x4();
	void predict_chroma8x8();
	int predict_intra_luma_mode(int idx);//预测第idx个亮度子块的预测模式（for 8x8 & 4x4 子块)

	void mode_decision_luma();//亮度块模式判决
	void cbp_encode();//cbp编码
	void set_neighbour();//设置邻块状态参数
    //void intra_proc();
	void load();//加载数据
	void run();//帧内预测
	void update();//内部更新
	void dump();//输出测试文件
	//void get_data(FILE *intra_input);


	/*****************************************
	*              public func              *
	*****************************************/

public:
	//intra();
	//~intra();
	void init();
	void read(mb_t& cur_mb, param_t& param, cqm_t& cqm);//读入参数
	void proc();
	intra_t & write();//更新输出值
};

#endif	/* _INTRA_H */#pragma once
