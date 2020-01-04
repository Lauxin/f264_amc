/*
* File:   ime.h
* Author: ybfan
*
* Created on November 11, 2011, 12:31 AM
*/

#ifndef _IME_H
#define	_IME_H

#include "quant.h"
#include "type_1.h"
#include "config.h"
#include "common.h"
#include "dct.h"
#include <iostream>
#include <string>

using namespace std;

#define H_6TAPFIR(halfpel,pel0,pel1,pel2,pel3,pel4,pel5)\
	halfpel = ((pel0 + pel5) - 5 * (pel1 + pel4) + 20 * (pel2 + pel3))
#define V_6TAPFIR(halfpel,pel0,pel1,pel2,pel3,pel4,pel5)\
	halfpel = Cliply((((pel0 + pel5) - 5 * (pel1 + pel4) + 20 * (pel2 + pel3) + 16)) >> 5)
#define D_6TAPFIR(halfpel,pel0,pel1,pel2,pel3,pel4,pel5)\
	halfpel = Cliply((((pel0 + pel5) - 5 * (pel1 + pel4) + 20 * (pel2 + pel3) + 512)) >> 10)
#define UV_BILINEAR(fracpel,fracpel_ext,n)\
	(fracpel) = Cliply((((fracpel_ext)+(1 << ((n)-1))) >> (n)))


class inter{
private:
	//Input Var
	mb_t			cur_mb;
	sw_t			sw;
	cqm_t			cqm;
	param_t         param;
	//Output Var
	inter_t			inter_output;

	//search windows
	int				sw_center[2]; // 0 for x, 1 for y
	uint8_t			ref_mb[9][16][16];// fme search cache
	uint8_t			min_mb[16][16];

	//ime cost
	int				cost16x16, cost8x16[2], cost16x8[2], cost8x4[8], cost4x8[8], cost8x8[4], cost4x4[16];
	int				cost8x4_s[4], cost4x8_s[4], cost8x8_s[4], cost4x4_s[4]; //total cost for one 8x8 block
	int				cost16x16_s, cost8x16_s, cost16x8_s, cost8x8_s_min; //total cost for one MB
	int				mb_cost_min;

	//fme cost
	int32_t			cost;

	//mv
	int16_t			mv16x16[2], mv16x8[2][2], mv8x16[2][2], mv8x8[4][2], mv8x4[4][2][2], mv4x8[4][2][2], mv4x4[4][4][2];//imv
	int16_t			fmv16x16[2], fmv16x8[2][2], fmv8x16[2][2], fmv8x8[4][2], fmv8x4[4][2][2], fmv4x8[4][2][2], fmv4x4[4][4][2];//fmv = imv + dmv
	int16_t			fmv[4][4][2];

	//tpye and partition
	int8_t			mb_type;
	int8_t			mb_partition;  //16x16 partition
	int8_t			mb_subpartition[4];//8x8 subpartition



	// interpolate cache
	uint8_t			h_half_pel[17][16];//horizental interpolate x+1 y
	uint8_t			v_half_pel[16][17];//vertical interpolate   x   y+1
	uint8_t			d_half_pel[17][17];//diagonal interpolate   x+1 y+1
	uint8_t			int_pel[16][16];   //integer pel			x   y

	// transform
	int16_t			luma_dct4x4[16][16];
	uint8_t			luma_rec[16][16];
	int16_t			chroma_dct4x4[2][4][16];
	int16_t			chroma_dct4x4_dc[2][4];
	uint8_t			chroma_rec[2][8][8];

	// mv cache
	int16_t			mvp[4][4][2];
	int16_t			mv_cache[5][6][2];//mv[4][4][2]+topleft+topright,therefore, mv_cache[i][j][k],wherei*j =0;
	int8_t			ref_cache[5][6];
	int16_t*		mv_line;
	int16_t			mv_topleft[2];

	//cbp & non_zero_count 
	uint8_t			non_zero_count[27];
	int16_t			cbp;
	int				cbp_dc;
	int				cbp_luma;
	int				cbp_chroma;

	//chroma

	uint8_t			min_cb[8][8];
	uint8_t			min_cr[8][8];

	// dump
	FILE			*fp;

	//ME buffer
	struct inter_bf *me_buffer;
	struct inter_bf mb_info;

	//ame related variable
	int16_t         ame_mvp[2][2][2];//mvp buffer
	int16_t         amv[2][2];//control point mv
	int32_t         ame_min_cost;
	int8_t          ame_step;
	uint8_t         pred_luma[f_LCU_SIZE][f_LCU_SIZE];

    // dump org&rec yuv
    uint8_t org_yuv[9 * 16][11 * 16]; //!!! h&w are fixed
    uint8_t rec_yuv[9 * 16][11 * 16];
    FILE* fp_org;
    FILE* fp_rec;

	// function

	void load();
	void run();
	void ime();
	//Subfunction--ime
	int ime_bs_size_se(int val);
	int pixel_sad_wxh(uint8_t *pix1, int i_stride_pix1, uint8_t *pix2, int i_stride_pix2, int w, int h);
	void fme();
	//Subfunction--fme
	void interpolate_h(int pos_x, int pos_y, int len_x, int len_y);
	int32_t subpel_me(int pos_x, int pos_y, int len_x, int len_y, int16_t mv[2], int16_t dmv[2], bool b_half);
	void hadamard_1d(int16_t &o_data0, int16_t &o_data1, int16_t &o_data2, int16_t &o_data3, int16_t i_data0, int16_t i_data1, int16_t i_data2, int16_t i_data3);
	int32_t sub_hadmard_satd(uint8_t *cur_4x4blk, uint8_t *ref_4x4blk);
	void loadmc();
	void mc();
	//Subfunction--mc
	void get_chroma();
	void mvp_compute();
	void transform();
	void cbp_encode();
	void mc_chroma(int pos_x, int pos_y, int ref_x, int ref_y, int len_x, int len_y, int16_t imv[2], int16_t fmv[2]);
	void mvp_median(int idx, int i_width, int16_t mvp[2]);
	int  f264_median(int a, int b, int c);
	void update();
	void dump();
    void dump_yuv();
	
	//subfunction--ame
	void ame();
	void get_mvp();
	int  ame_count(int16_t mv[2][2]);
	void ame_mcp(int16_t mv[2][2]);
	int32_t subpel_me(int pos_x, int pos_y, int16_t mv[2],int index);
	void solveEqual(double** dEqualCoeff,double* dAffinePara);
	bool Gauss(double **A,double B[][4]);
	void mode_decision_sort();


public:
	//inter(); //construtor, with vars init
	//~inter(); // destructor, free dynamic space
	void init(); // frame init: search range....    
	void read(mb_t & i_cur_mb, sw_t & sw, cqm_t & i_cqm, param_t & i_param); // mb_t, sw_t, cqm_t, param_t(parameter)
	void proc();  // mb run
	inter_t& write();// inter_t
	void del();

};



#endif	/* _IME_H */

