/*
 * File:   defines.h
 * Author: ybfan
 *
 * Created on May 3, 2012
 */

#ifndef _TYPE_H
#define	_TYPE_H
//#include "systemc.h"
#include <string>
#include "config.h"
using std::string;

typedef signed   char    int8_t;
typedef unsigned char    uint8_t;
typedef short            int16_t;
typedef unsigned short   uint16_t;
typedef int              int32_t;
typedef unsigned         uint32_t;
typedef long long        int64_t;
typedef unsigned long long uint64_t;

/********************************************************************************
*                            Normal Type Def.
*********************************************************************************/
/* bitstream*/
struct bs_t {
	uint8_t *p_start;   //buffer start address
	uint8_t *p;         //current address
	uint8_t *p_end;     //buffer end address
	intptr_t cur_bits;  //current stored bits (32-bit length buf)
	int     i_left;     //available bits in cur_bits
	int     i_bits_encoded; /* RD only */
};

/********************************************************************************
*                            SystemC Type Def.
*********************************************************************************/
/*************************************
				 mb_t
*************************************/
struct mb_t {
	string name;
	int x;
	int y;
	uint8_t  luma[f_LCU_SIZE][f_LCU_SIZE];
	uint8_t  cr[f_LCU_SIZE / 2][f_LCU_SIZE / 2];
	uint8_t  cb[f_LCU_SIZE / 2][f_LCU_SIZE / 2];

	mb_t& operator= (const mb_t& arg) {
		name = arg.name;
		x = arg.x;
		y = arg.y;
		memcpy(luma, arg.luma, sizeof(luma));
		memcpy(cr, arg.cr, sizeof(cr));
		memcpy(cb, arg.cb, sizeof(cb));
		return *this;
	}
	bool operator== (const mb_t& arg) const {
		return ((name == arg.name) && (x == arg.x) && (y == arg.y));
	}
};

//ostream& operator<< (ostream& os, const mb_t& arg);
//void sc_trace(sc_trace_file* tf, const mb_t& arg, const string& name);

/*************************************
				 cqm_t
*************************************/
struct cqm_t {
	string name;
	int qp;
	uint16_t quant4_mf[52][16];     /* [4][52][16] */
	uint32_t intra_quant4_bias[52][16];   /* [4][52][16] */
	uint32_t inter_quant4_bias[52][16];   /* [4][52][16] */
	int      dequant4_mf[6][4][4];  /* [4][6][4][4] */
	uint16_t quant8_mf[52][64];     /* [2][52][64] */
	uint32_t intra_quant8_bias[52][64];   /* [2][52][64] */
	uint32_t inter_quant8_bias[52][64];   /* [2][52][64] */
	int      dequant8_mf[6][8][8]; /* [2][6][8][8] */

	cqm_t& operator= (const cqm_t& arg) {
		name = arg.name;
		qp = arg.qp;
		memcpy(quant4_mf, arg.quant4_mf, sizeof(quant4_mf));
		memcpy(intra_quant4_bias, arg.intra_quant4_bias, sizeof(intra_quant4_bias));
		memcpy(inter_quant4_bias, arg.inter_quant4_bias, sizeof(inter_quant4_bias));
		memcpy(dequant4_mf, arg.dequant4_mf, sizeof(dequant4_mf));
		memcpy(quant8_mf, arg.quant8_mf, sizeof(quant8_mf));
		memcpy(intra_quant8_bias, arg.intra_quant8_bias, sizeof(intra_quant8_bias));
		memcpy(inter_quant8_bias, arg.inter_quant8_bias, sizeof(inter_quant8_bias));
		memcpy(dequant8_mf, arg.dequant8_mf, sizeof(dequant8_mf));
		return *this;
	}
	bool operator== (const cqm_t& arg) const {
		return ((name == arg.name) && (qp == arg.qp));
	}
};

//ostream& operator<< (ostream& os, const cqm_t& arg);
//void sc_trace(sc_trace_file* tf, const cqm_t& arg, const string& name);

//ostream& operator<< (ostream& os, const ctrl_t& arg);
//void sc_trace(sc_trace_file* tf, const ctrl_t& arg, const string& name);

/*************************************
				 intra_t
*************************************/
struct intra_t {
	string name;
	int x;
	int y;
	int qp;
	int8_t  mb_type;
	// intra mode
	int8_t  intra4x4_mode[16];
	int8_t	intra4x4_pred_mode[16];
	int8_t  intra16x16_mode;
	int8_t	chroma_mode;
	// residual coeff.
	int16_t luma[16][16];
	int16_t luma_dc[16];
	int16_t chroma[2][4][16];
	int16_t chroma_dc[2][4];
	// reconstructed pixel 
	uint8_t	luma_rec[16][16];
	uint8_t	chroma_rec[2][8][8];
	// cbp & non_zero_count
	int16_t cbp;
	uint8_t non_zero_count[27];
	// cost
	int		intra_cost;

	intra_t& operator= (const intra_t& arg) {
		name = arg.name;
		x = arg.x;
		y = arg.y;
		qp = arg.qp;
		mb_type = arg.mb_type;
		intra16x16_mode = arg.intra16x16_mode;
		chroma_mode = arg.chroma_mode;
		intra_cost = arg.intra_cost;
		cbp = arg.cbp;
		memcpy(intra4x4_mode, arg.intra4x4_mode, sizeof(intra4x4_mode));
		memcpy(intra4x4_pred_mode, arg.intra4x4_pred_mode, sizeof(intra4x4_pred_mode));
		memcpy(luma, arg.luma, sizeof(luma));
		memcpy(luma_dc, arg.luma_dc, sizeof(luma_dc));
		memcpy(chroma, arg.chroma, sizeof(chroma));
		memcpy(chroma_dc, arg.chroma_dc, sizeof(chroma_dc));
		memcpy(luma_rec, arg.luma_rec, sizeof(luma_rec));
		memcpy(chroma_rec, arg.chroma_rec, sizeof(chroma_rec));
		memcpy(non_zero_count, arg.non_zero_count, sizeof(non_zero_count));
		return *this;
	}
	bool operator== (const intra_t& arg) const {
		return ((name == arg.name) && (x == arg.x) && (y == arg.y));
	}
};

//ostream& operator<< (ostream& os, const intra_t& arg);
//void sc_trace(sc_trace_file* tf, const intra_t& arg, const string& name);





/*************************************
				 ec_t
*************************************/
struct ec_t {
	string name;
	int x;
	int y;

	bs_t	bs_buf_pt;
	uint8_t bs_buf[BS_BUF_SIZE];

	ec_t& operator= (const ec_t& arg) {
		name = arg.name;
		x = arg.x;
		y = arg.y;
		memcpy(bs_buf, arg.bs_buf, sizeof(bs_buf));
		bs_buf_pt.p_start = bs_buf;
		bs_buf_pt.p_end = bs_buf + BS_BUF_SIZE;
		bs_buf_pt.p = bs_buf + (arg.bs_buf_pt.p - arg.bs_buf_pt.p_start);
		bs_buf_pt.cur_bits = arg.bs_buf_pt.cur_bits;
		bs_buf_pt.i_left = arg.bs_buf_pt.i_left;
		bs_buf_pt.i_bits_encoded = arg.bs_buf_pt.i_bits_encoded;
		return *this;
	}
	bool operator== (const ec_t& arg) const {
		return ((name == arg.name) && (x == arg.x) && (y == arg.y));
	}
};

//ostream& operator<< (ostream& os, const ec_t& arg);
//void sc_trace(sc_trace_file* tf, const ec_t& arg, const string& name);


/*************************************
				 param_t
*************************************/
struct param_t {
	string name;
	string file_name;
	int type;
	int frame_num;
	int qp;
	int gop_length;
	int frame_width;
	int frame_height;
	int frame_total;
	int frame_mb_x_total;
	int frame_mb_y_total;


	param_t& operator= (const param_t& arg) {
		name = arg.name;
		file_name = arg.file_name;
		type = arg.type;
		frame_num = arg.frame_num;
		qp = arg.qp;
		gop_length = arg.gop_length;
		frame_width = arg.frame_width;
		frame_height = arg.frame_height;
		frame_total = arg.frame_total;
		frame_mb_x_total = arg.frame_mb_x_total;
		frame_mb_y_total = arg.frame_mb_y_total;
		return *this;
	}

	bool operator== (const param_t& arg) const {
		return ((name == arg.name) && (type == arg.type) && (frame_num == arg.frame_num));
	}
};

//ostream& operator<< (ostream& os, const param_t& arg);
//void sc_trace(sc_trace_file* tf, const param_t& arg, const string& name);


//ostream& operator<< (ostream& os, const fme_t& arg);
//void sc_trace(sc_trace_file* tf, const fme_t& arg, const string& name);

/*************************************
				 mc_t
*************************************/
struct inter_t {
	string name;
	int x;
	int y;
	// inter info
	int    qp;
	int8_t mb_type;
	int8_t mb_partition;
	int8_t mb_subpartition[4];
	int16_t mv[4][4][2];
	int16_t mvp[4][4][2];
	// residual coeff.
	int16_t luma[16][16];
	int16_t chroma[2][4][16];
	int16_t chroma_dc[2][4];
	// reconstructed pixel 
	uint8_t	luma_rec[16][16];
	uint8_t	chroma_rec[2][8][8];
	// cbp & non_zero_count
	int16_t cbp;
	uint8_t non_zero_count[27];

	inter_t& operator= (const inter_t& arg) {
		name = arg.name;
		x = arg.x;
		y = arg.y;
		qp = arg.qp;
		mb_type = arg.mb_type;
		mb_partition = arg.mb_partition;
		cbp = arg.cbp;
		memcpy(mb_subpartition, arg.mb_subpartition, sizeof(mb_subpartition));
		memcpy(mv, arg.mv, sizeof(mv));
		memcpy(mvp, arg.mvp, sizeof(mvp));
		memcpy(luma, arg.luma, sizeof(luma));
		memcpy(chroma, arg.chroma, sizeof(chroma));
		memcpy(chroma_dc, arg.chroma_dc, sizeof(chroma_dc));
		memcpy(luma_rec, arg.luma_rec, sizeof(luma_rec));
		memcpy(chroma_rec, arg.chroma_rec, sizeof(chroma_rec));
		memcpy(non_zero_count, arg.non_zero_count, sizeof(non_zero_count));
		return *this;
	}

	bool operator== (const inter_t& arg) const {
		return ((name == arg.name) && (x == arg.x) && (y == arg.y));
	}
};

//ostream& operator<< (ostream& os, const inter_t& arg);
//void sc_trace(sc_trace_file* tf, const inter_t& arg, const string& name);

/*************************************
				 db_t
*************************************/
struct db_t {
	string name;
	int x;
	int y;
	uint8_t  cache[32][32];

	db_t& operator= (const db_t& arg) {
		name = arg.name;
		x = arg.x;
		y = arg.y;
		memcpy(cache, arg.cache, sizeof(cache));
		return *this;
	}
	bool operator== (const db_t& arg) const {
		return ((name == arg.name) && (x == arg.x) && (y == arg.y));
	}
};

//ostream& operator<< (ostream& os, const db_t& arg);
//void sc_trace(sc_trace_file* tf, const db_t& arg, const string& name);
struct sw_t {
	int sr_w;
	int sr_h;
	uint8_t		sw_luma[SW_H][SW_W];
	uint8_t		ref_mb_cb[SW_H / 2][SW_H / 2];
	uint8_t		ref_mb_cr[SW_H / 2][SW_H / 2];
	sw_t& operator= (const sw_t& arg){
		sr_w = arg.sr_w;
		sr_h = arg.sr_h;
		memcpy(sw_luma, arg.sw_luma, sizeof(sw_luma));
		memcpy(ref_mb_cb, arg.ref_mb_cb, sizeof(ref_mb_cb));
		memcpy(ref_mb_cr, arg.ref_mb_cr, sizeof(ref_mb_cr));
		return *this;
	}
};


#endif	/* _TYPE_H */

#pragma once
