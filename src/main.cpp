/*
 * File:   main.cpp
 * Author: ybfan
 *
 * Created on November 10, 2011, 9:38 PM
 */
//#include "systemc.h"
#include "type_1.h"
#include "quant.h"
#include "inter_1.h"
#include "common.h"
#include <iostream>
#include <chrono>

using namespace std;
using namespace std::chrono;
 /*
  *
  */
  //argv[]是存相关参数的数组
  //相关参数（视频文件相关信息）
static void get_input_param(param_t* param, int argc, char* argv[]) {
	param->name = "param";
	param->file_name = YUV_FILE;
	param->type = SLICE_TYPE_I;
	param->frame_num = 0;
	param->qp = INIT_QP;
	param->gop_length = GOP_LENGTH;
	param->frame_width = FRAMEWIDTH;
	param->frame_height = FRAMEHEIGHT;
	param->frame_total = FRAME_TOTAL;
	param->frame_mb_x_total = MB_X_TOTAL;
	param->frame_mb_y_total = MB_Y_TOTAL;

	bool help = 0;
	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "-i")) {
			param->file_name = argv[++i];
		}
		else if (!strcmp(argv[i], "-w")) {
			param->frame_width = atoi(argv[++i]);
		}
		else if (!strcmp(argv[i], "-h")) {
			param->frame_height = atoi(argv[++i]);
		}
		else if (!strcmp(argv[i], "-g")) {
			param->gop_length = atoi(argv[++i]);
		}
		else if (!strcmp(argv[i], "-f")) {
			param->frame_total = atoi(argv[++i]);
		}
		else if (!strcmp(argv[i], "-q")) {
			param->qp = atoi(argv[++i]);
		}
		else if (!strcmp(argv[i], "-help")) {
			help = 1;
		}
		else {
			fprintf(stderr, "Unknown option [%s]\n", argv[i]);
		}
	}

	if (help) {
		fprintf(stderr, "Usage:\n\t%s [-i|-help] inYuvFile [options]\n", argv[0]);
		fprintf(stderr, "\nOptions:\n");
		fprintf(stderr, "    -help       :Printf usage               \n");
		fprintf(stderr, "    -i          :Input file                 \n");
		fprintf(stderr, "    -w          :Width of Video             \n");
		fprintf(stderr, "    -h          :Height of Video            \n");
		fprintf(stderr, "    -g          :GOP Length                 \n");
		fprintf(stderr, "    -f          :Encode frames              \n");
		fprintf(stderr, "    -q          :Encode qp                  \n");
		fprintf(stderr, "\nExamples:\n");
		fprintf(stderr, "win32.exe -i BlowingBubbles_416x240_50.yuv -w 416 -h 240 -g 5 -f 10 -q 27\n");
		exit(0);
	}

	param->frame_mb_x_total = ((param->frame_width + (f_LCU_SIZE - 1)) / f_LCU_SIZE);
	param->frame_mb_y_total = ((param->frame_height + (f_LCU_SIZE - 1)) / f_LCU_SIZE);

	fprintf(stderr, "    -i          :Input file                 [%s]\n", (param->file_name).c_str());
	fprintf(stderr, "    -w          :Width of Video             [%4d]\n", param->frame_width);
	fprintf(stderr, "    -h          :Height of Video            [%4d]\n", param->frame_height);
	fprintf(stderr, "    -g          :GOP Length                 [%4d]\n", param->gop_length);
	fprintf(stderr, "    -f          :Encode frames              [%4d]\n", param->frame_total);
	fprintf(stderr, "    -q          :Encode qp                  [%4d]\n", param->qp);
}

static void get_input_cqm(cqm_t& cqm) {

	cqm.name = "cqm";
	//------- cqm init ------------//
	int def_quant4in[6][16];
	int def_dequant4in[6][16];
	int quant4_mfin[6][4][4];

	int def_quant8in[6][64];
	int def_dequant8in[6][64];
	int quant8_mfin[6][8][8];

	int q, i;
	for (q = 0; q < 6; q++)
	{
		for (i = 0; i < 16; i++)
		{
			int j = (i & 1) + ((i >> 2) & 1);//0101121201011212
			def_dequant4in[q][i] = dequant4_scale[q][j];
			def_quant4in[q][i] = quant4_scale[q][j];
		}
		//added by hlren
		for (i = 0; i < 64; i++)
		{
			int j = quant8_scan[((i >> 1) & 12) | (i & 3)];
			def_dequant8in[q][i] = dequant8_scale[q][j];
			def_quant8in[q][i] = quant8_scale[q][j];
		}
		//end
	}

	for (q = 0; q < 6; q++)
	{
		for (i = 0; i < 16; i++)
		{
			cqm.dequant4_mf[q][0][i] = def_dequant4in[q][i] * scaling_list[i];
			quant4_mfin[q][0][i] = DIV(def_quant4in[q][i] * 16, scaling_list[i]);
		}
		//added by hlren
		for (i = 0; i < 64; i++)
		{
			cqm.dequant8_mf[q][0][i] = def_dequant8in[q][i] * scaling_list[i];
			quant8_mfin[q][0][i] = DIV(def_quant8in[q][i] * 16, scaling_list[i]);
		}
		//end

	}

	//according to jm 
	for (q = 0; q < 52; q++) {
		for (i = 0; i < 16; i++)
		{
			cqm.quant4_mf[q][i] = quant4_mfin[q % 6][0][i];
			cqm.intra_quant4_bias[q][i] = Offset_intra_default[i] << (q / 6 + 4);//682 = 2^((15 +qp/6) -(qp/6 +4))/3
			cqm.inter_quant4_bias[q][i] = Offset_inter_default[i] << (q / 6 + 4);//342 = 2^((15 +qp/6) -(qp/6 +4))/6
		}
		for (i = 0; i < 64; i++)
		{
			cqm.quant8_mf[q][i] = quant8_mfin[q % 6][0][i];
			cqm.intra_quant8_bias[q][i] = Offset8_intra_default[i] << (q / 6 + 5);
			cqm.inter_quant8_bias[q][i] = Offset8_inter_default[i] << (q / 6 + 5);
		}
	}
	cqm.qp = INIT_QP;
}

void get_data(FILE *finfo, mb_t& cur_mb, sw_t& sw) {
	int i, j;
	fscanf(finfo, "(%d,%d)\n", &i, &j);
	cur_mb.x = i;
	cur_mb.y = j;
	fscanf(finfo, "# ORG_Y\n");
	int temp;
	for (i = 0; i<16; i++){
		for (j = 0; j<16; j++){
			fscanf(finfo, "%2x ", &temp);
			cur_mb.luma[i][j] = temp;
		}
		fscanf(finfo, "\n");

	}
	fscanf(finfo, "# ORG_U|V\n");
	for (i = 0; i<8; i++){
		for (j = 0; j<8; j++){
			fscanf(finfo, "%2x ", &temp);
			cur_mb.cr[i][j] = temp;
		}
		for (j = 0; j<8; j++){
			fscanf(finfo, "%2x ", &temp);
			cur_mb.cb[i][j] = temp;
		}
		fscanf(finfo, "\n");

	}
	fscanf(finfo, "# REF_Y\n");
	for (i = 0; i<48; i++){
		for (j = 0; j<48; j++){
			fscanf(finfo, "%2x ", &temp);
			sw.sw_luma[i][j] = temp;
		}
		fscanf(finfo, "\n");

	}
	fscanf(finfo, "# REF_U|V\n");
	for (i = 0; i<24; i++){
		for (j = 0; j < 24; j++){
			fscanf(finfo, "%2x ", &temp);
			sw.ref_mb_cr[i][j] = temp;
		}
		for (j = 0; j<24; j++){
			fscanf(finfo, "%2x ", &temp);
			sw.ref_mb_cb[i][j] = temp;
		}
		fscanf(finfo, "\n");
	}

}

void main(int ac, char* av[])
{
	mb_t cur_mb;
	cqm_t cqm;
	param_t param;
	sw_t sw;
    inter u_ime;
	FILE *inter_input;
	inter_input = fopen("D:\\MyFile\\cources\\H264\\pj264\\cmodel\\g01\\inter.cpp\\inter_input.txt", "r");
	//获取相关视频信息至结构体sc_param
	get_input_param(&param, ac, av);
	get_input_cqm(cqm);
	//u_intra.proc(param,cqm);
/****************************************************************************
 *								main_proc()
 ****************************************************************************/
	u_ime.init();
	steady_clock::time_point t1 = steady_clock::now();
	while (true) {
		get_data(inter_input, cur_mb, sw);
		u_ime.read(cur_mb, sw , cqm, param);
		u_ime.proc();
		u_ime.write();
	    if ((cur_mb.x == param.frame_mb_x_total - 1) && (cur_mb.y == param.frame_mb_y_total - 1)){
			param.frame_num=param.frame_num+1;
			u_ime.del();
		}
		if (param.frame_num==param.frame_total)
	    	break;
		}
	steady_clock::time_point t2 = steady_clock::now();
	cout << "run time: " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
	getchar();

	fclose(inter_input);
	
}