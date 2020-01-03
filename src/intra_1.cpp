#include "intra_1.h"
#include "common.h"
#include "quant.h"
#include "dct.h"

/****************************************************************************
 *								proc()
 ****************************************************************************/
void intra::proc() {
			load();
			run();
			dump();
			update();
		}
		

void intra::init() {

	//函数指针，将每个函数指针定义成相应预测函数的入口地址
	predict_16x16_init(predict_16x16);
	predict_4x4_init(predict_4x4);
	predict_8x8c_init(predict_8x8c);

	//初始化，sizeof以字节为单位
	memset(fenc, 0, sizeof(fenc));
	memset(fdec, 0, sizeof(fdec));
	memset(fdec2, 0, sizeof(fdec2));


#ifdef DUMP_INTRA_CHECK
	fp = fopen("intra.dat", "w");
	fp_lc = fopen("intra_input.dat", "w");
	fp_cong = fopen("intra_output.dat", "w");
	fclose(fp_cong);
	fclose(fp_lc);
	fclose(fp);
#endif
}

void intra::read(mb_t& i_cur_mb, param_t& i_param, cqm_t& i_cqm){
	cur_mb = i_cur_mb;
	cqm = i_cqm;
	param = i_param;
}

void intra::load() {
	int i, j;

	//--------------load reference pixel 加载参考像素（三个块的左/上/左上角/右上）------------------//
	// load left
	if (cur_mb.x > 0) {
		//Y
		for (i = 0; i < 16; i++) {
			fdec[2 + i][-1] = (mb_type == I_16x16) ? fdec[2 + i][15] : fdec2[2 + i][15];
			fdec2[2 + i][-1] = (mb_type == I_16x16) ? fdec[2 + i][15] : fdec2[2 + i][15];
		}
		for (i = 0; i < 8; i++) {
			fdec[19 + i][-1] = fdec[19 + i][7];//U
			fdec[19 + i][15] = fdec[19 + i][23];//V
		}
	}
	else {
		for (i = 0; i < 16; i++) {
			fdec[2 + i][-1] = 0;
			fdec2[2 + i][-1] = 0;
		}
		for (i = 0; i < 8; i++) {
			fdec[19 + i][-1] = 0;
			fdec[19 + i][15] = 0;
		}
	}
	// load top
	if (cur_mb.y > 0) {
		for (i = 0; i < 16; i++) {
			fdec[2][i - FDEC_STRIDE] = pixel_line[cur_mb.x * 32 + i];
			fdec2[2][i - FDEC_STRIDE] = pixel_line[cur_mb.x * 32 + i];
		}
		for (i = 0; i < 8; i++) {
			fdec[19][i - FDEC_STRIDE] = pixel_line[cur_mb.x * 32 + 16 + i];
			fdec[19][i - FDEC_STRIDE + 16] = pixel_line[cur_mb.x * 32 + 24 + i];
		}
	}
	else {
		for (i = 0; i < 16; i++) {
			fdec[2][i - FDEC_STRIDE] = 0;
			fdec2[2][i - FDEC_STRIDE] = 0;
		}
		for (i = 0; i < 8; i++) {
			fdec[19][i - FDEC_STRIDE] = 0;
			fdec[19][i - FDEC_STRIDE + 16] = 0;
		}
	}
	// load top_left，左上角
	if (cur_mb.x > 0 && cur_mb.y > 0) {
		fdec[2][-1 - FDEC_STRIDE] = pixel_topleft[0];
		fdec2[2][-1 - FDEC_STRIDE] = pixel_topleft[0];
		fdec[19][-1 - FDEC_STRIDE] = pixel_topleft[1];
		fdec[19][-1 - FDEC_STRIDE + 16] = pixel_topleft[2];
	}
	else {
		fdec[2][-1 - FDEC_STRIDE] = 0;
		fdec2[2][-1 - FDEC_STRIDE] = 0;
		fdec[19][-1 - FDEC_STRIDE] = 0;
		fdec[19][-1 - FDEC_STRIDE + 16] = 0;
	}
	// load top_right
	if (cur_mb.x < param.frame_mb_x_total - 1 && cur_mb.y>0) {
		for (i = 0; i < 4; i++)
			fdec2[1][16 + i] = pixel_line[(cur_mb.x + 1) * 32 + i];
	}
	else {
		for (i = 0; i < 4; i++)
			fdec2[1][16 + i] = 0;
	}

	//--------------load reference predict mode（加载参考的预测模式）？？？？------------------//
	if (cur_mb.x == 0 && cur_mb.y == 0) {
		memset(prev_intra4x4_pred_mode_flag, 0, sizeof(prev_intra4x4_pred_mode_flag));
		memset(intra4x4_mode, 0, sizeof(intra4x4_mode));
		memset(intra4x4_pred_mode_cache, 0, sizeof(intra4x4_pred_mode_cache));
	}
	// left
	if (cur_mb.x > 0) {
		intra4x4_pred_mode_cache[f264_scan8[0] - 1] = intra4x4_pred_mode_cache[f264_scan8[5]];
		intra4x4_pred_mode_cache[f264_scan8[2] - 1] = intra4x4_pred_mode_cache[f264_scan8[7]];
		intra4x4_pred_mode_cache[f264_scan8[8] - 1] = intra4x4_pred_mode_cache[f264_scan8[13]];
		intra4x4_pred_mode_cache[f264_scan8[10] - 1] = intra4x4_pred_mode_cache[f264_scan8[15]];
	}
	else {
		intra4x4_pred_mode_cache[f264_scan8[0] - 1] = -1;
		intra4x4_pred_mode_cache[f264_scan8[2] - 1] = -1;
		intra4x4_pred_mode_cache[f264_scan8[8] - 1] = -1;
		intra4x4_pred_mode_cache[f264_scan8[10] - 1] = -1;
	}
	// top
	if (cur_mb.y > 0) {
		intra4x4_pred_mode_cache[f264_scan8[0] - 8] = mode_line[cur_mb.x * 4 + 0];
		intra4x4_pred_mode_cache[f264_scan8[0] - 7] = mode_line[cur_mb.x * 4 + 1];
		intra4x4_pred_mode_cache[f264_scan8[0] - 6] = mode_line[cur_mb.x * 4 + 2];
		intra4x4_pred_mode_cache[f264_scan8[0] - 5] = mode_line[cur_mb.x * 4 + 3];
	}
	else {
		intra4x4_pred_mode_cache[f264_scan8[0] - 8] = -1;
		intra4x4_pred_mode_cache[f264_scan8[0] - 7] = -1;
		intra4x4_pred_mode_cache[f264_scan8[0] - 6] = -1;
		intra4x4_pred_mode_cache[f264_scan8[0] - 5] = -1;
	}

	//------------------- load cur_mb 加载当前编码块-------------------//
	//亮度
	for (j = 0; j < 16; j++)
		for (i = 0; i < 16; i++)
			fenc[j][i] = cur_mb.luma[j][i];
	//色度
	for (j = 0; j < 8; j++)
		for (i = 0; i < 8; i++) {
			fenc[j + 16][i] = cur_mb.cb[j][i];
			fenc[j + 16][i + 8] = cur_mb.cr[j][i];
		}

	if (param.frame_num == 0 && cur_mb.x == 0 && cur_mb.y == 0) {
		pixel_line = new uint8_t[param.frame_mb_x_total * 32];  // 分配内存 Y-U-V(32p)
		mode_line = new int8_t[param.frame_mb_x_total * 4];  //mode_line???
	}

}
intra_t & intra::write(){
	//--------------- write output 写输出值---------//
	intra_output.name = "intra";
	intra_output.x = cur_mb.x;
	intra_output.y = cur_mb.y;
	intra_output.qp = cqm.qp;
	intra_output.mb_type = mb_type;

	intra_output.intra16x16_mode = intra16x16_mode;
	intra_output.chroma_mode = chroma_mode;

	if (mb_type == I_16x16) {
		intra_output.intra_cost = sad16;
		memset(intra_output.intra4x4_mode, 2, sizeof(intra4x4_mode));
		memset(intra_output.intra4x4_pred_mode, 0, sizeof(intra4x4_pred_mode));
		memcpy(intra_output.luma, luma_dct4x4_z16, sizeof(intra_output.luma));
		memcpy(intra_output.luma_dc, luma_dct4x4_dc_z4, sizeof(intra_output.luma_dc));
		for (int i = 0; i < 16; i++)
			memcpy(intra_output.luma_rec[i], fdec[2 + i], 16 * sizeof(uint8_t));
	}
	
	else if (mb_type == I_4x4) {
		intra_output.intra_cost = sad4;
		memcpy(intra_output.intra4x4_mode, intra4x4_mode, sizeof(intra4x4_mode));
		memcpy(intra_output.intra4x4_pred_mode, intra4x4_pred_mode, sizeof(intra4x4_pred_mode));
		memcpy(intra_output.luma, luma_dct4x4_z4, sizeof(intra_output.luma));
		memset(intra_output.luma_dc, 0, sizeof(intra_output.luma_dc));
		for (int i = 0; i < 16; i++)
			memcpy(intra_output.luma_rec[i], fdec2[2 + i], 16 * sizeof(uint8_t));
	}

	memcpy(intra_output.chroma, chroma_dct4x4_z4, sizeof(intra_output.chroma));
	memcpy(intra_output.chroma_dc, chroma_dct4x4_dc_z2, sizeof(intra_output.chroma_dc));

	for (int i = 0; i < 8; i++) {
		memcpy(intra_output.chroma_rec[0][i], fdec[19 + i], 8 * sizeof(uint8_t));
		memcpy(intra_output.chroma_rec[1][i], fdec[19 + i] + 16, 8 * sizeof(uint8_t));
	}

	intra_output.cbp = cbp;
	memcpy(intra_output.non_zero_count, non_zero_count, sizeof(non_zero_count));
	return intra_output;
}

void intra::update() {
	
	//--------------- store ref pixel & mode ---------//
	//pixel_topleft初始化
	pixel_topleft[0] = pixel_line[cur_mb.x * 32 + 15]; // Y
	pixel_topleft[1] = pixel_line[cur_mb.x * 32 + 23]; // U
	pixel_topleft[2] = pixel_line[cur_mb.x * 32 + 31]; // V

	if (mb_type == I_16x16) {
		// pixel_line数组是存放Y/U/V三个块上方的相邻像素
		for (int i = 0; i < 16; i++)
			pixel_line[cur_mb.x * 32 + i] = fdec[17][i];//Y
		// intra4x4_pred_mode_cache
		for (int i = 0; i < 16; i++)
			intra4x4_pred_mode_cache[f264_scan8[i]] = 2;
	}
	else if (mb_type == I_4x4) {
		for (int i = 0; i < 16; i++)
			pixel_line[cur_mb.x * 32 + i] = fdec2[17][i];
	}

	for (int i = 0; i < 8; i++) {
		pixel_line[cur_mb.x * 32 + 16 + i] = fdec[26][i];//U
		pixel_line[cur_mb.x * 32 + 24 + i] = fdec[26][i + 16];//V
	}

	mode_line[cur_mb.x * 4 + 0] = intra4x4_pred_mode_cache[f264_scan8[10]];
	mode_line[cur_mb.x * 4 + 1] = intra4x4_pred_mode_cache[f264_scan8[11]];
	mode_line[cur_mb.x * 4 + 2] = intra4x4_pred_mode_cache[f264_scan8[14]];
	mode_line[cur_mb.x * 4 + 3] = intra4x4_pred_mode_cache[f264_scan8[15]];

}

void intra::dump() {

#ifdef DUMP_INTRA_CHECK
	fp = fopen("intra.dat", "a");//以附加的方式打开只写文件。若文件不存在，则会建立该文件，如果文件存在，写入的数据会被加到文件尾，即文件原先的内容会被保留（EOF 符保留）。
	fp_lc = fopen("intra_input.dat", "a");
	fp_cong = fopen("intra_output.dat", "a");
	//******************************************************
	//input*************************************************
	//******************************************************
		//fprintf( )会根据参数format 字符串来转换并格式化数据, 然后将结果输出到参数stream 指定的文件中
		//写参数
	fprintf(fp_lc, "%02x\n", cur_mb.x);
	fprintf(fp_lc, "%02x\n", cur_mb.y);
	fprintf(fp_lc, "%02x\n", param.frame_mb_x_total - 1);
	fprintf(fp_lc, "%d\n", 1);//mb_type==0?1:0);
	fprintf(fp_lc, "%x\n", cqm.qp);
	//original pixel 像文件中写原始像素值，编码块************************************
	for (int i = 15; i >= 0; i--)
		for (int j = 15; j >= 0; j--)
			fprintf(fp_lc, "%02x", fenc[i][j]);
	fprintf(fp_lc, "\n");//luma

	for (int i = 7; i >= 0; i--)
		for (int j = 7; j >= 0; j--)
			fprintf(fp_lc, "%02x", fenc[i + 16][j]);
	fprintf(fp_lc, "\n");//cb

	for (int i = 7; i >= 0; i--)
		for (int j = 7; j >= 0; j--)
			fprintf(fp_lc, "%02x", fenc[i + 16][j + 8]);
	fprintf(fp_lc, "\n");//cr
//**********************************************************

//reference pixel for luma 向文件中写入亮度参考像素***********************************
	if (cur_mb.x == 0 || cur_mb.y == 0)
	{
		fprintf(fp_lc, "xx\n", fdec[0][31]);
	}
	else
	{
		fprintf(fp_lc, "%02x\n", fdec[0][31]);//luma_top-left
	}

	//？？？8*8
	if (cur_mb.x == param.frame_mb_x_total - 1 || cur_mb.y == 0)
	{
		for (int i = 0; i < 4; i++)
			fprintf(fp_lc, "xx");
		fprintf(fp_lc, "\n");//luma_top-right
	}
	else
	{
		for (int i = 0; i < 4; i++)
			fprintf(fp_lc, "%02x", fdec2[1][16 + i]);
		fprintf(fp_lc, "\n");//luma_top-right
	}


	if (cur_mb.y == 0)
	{
		for (int i = 0; i < 16; i++)
			fprintf(fp_lc, "xx");
		fprintf(fp_lc, "\n");//luma_top
	}
	else
	{
		for (int i = 0; i < 16; i++)
			fprintf(fp_lc, "%02x", fdec[1][i]);
		fprintf(fp_lc, "\n");//luma_top
	}

	if (cur_mb.x == 0)
	{
		for (int i = 0; i < 16; i++)
			fprintf(fp_lc, "xx");
		fprintf(fp_lc, "\n");//luma_left
	}
	else
	{
		for (int i = 0; i < 16; i++)
			fprintf(fp_lc, "%02x", fdec[1 + i][31]);
		fprintf(fp_lc, "\n");//luma_left
	}
	//***********************************************************

	//reference pixel for chroma  向文件中写入色度参考像素*********************************
		//U分量
	if (cur_mb.x == 0 || cur_mb.y == 0)
	{
		fprintf(fp_lc, "xx\n");//cb_top-left
	}
	else
	{
		fprintf(fp_lc, "%02x\n", fdec[17][31]);//cb_top-left
	}

	if (cur_mb.y == 0)
	{
		for (int i = 0; i < 8; i++)
			fprintf(fp_lc, "xx");
		fprintf(fp_lc, "\n");//cb_top
	}
	else
	{
		for (int i = 0; i < 8; i++)
			fprintf(fp_lc, "%02x", fdec[18][i]);
		fprintf(fp_lc, "\n");//cb_top
	}

	if (cur_mb.x == 0)
	{
		for (int i = 0; i < 8; i++)
			fprintf(fp_lc, "xx");
		fprintf(fp_lc, "\n");//cb_left
	}
	else
	{
		for (int i = 0; i < 8; i++)
			fprintf(fp_lc, "%02x", fdec[18 + i][31]);
		fprintf(fp_lc, "\n");//cb_left
	}



	//V分量
	if (cur_mb.x == 0 || cur_mb.y == 0)
	{
		fprintf(fp_lc, "xx\n");//cr_top-left
	}
	else
	{
		fprintf(fp_lc, "%02x\n", fdec[18][15]);//cr_top-left
	}

	if (cur_mb.y == 0)
	{
		for (int i = 0; i < 8; i++)
			fprintf(fp_lc, "xx");
		fprintf(fp_lc, "\n");//cr_top
	}
	else
	{
		for (int i = 0; i < 8; i++)
			fprintf(fp_lc, "%02x", fdec[18][16 + i]);
		fprintf(fp_lc, "\n");//cr_top
	}

	if (cur_mb.x == 0)
	{
		for (int i = 0; i < 8; i++)
			fprintf(fp_lc, "xx");
		fprintf(fp_lc, "\n");//cr_left
	}
	else
	{
		for (int i = 0; i < 8; i++)
			fprintf(fp_lc, "%02x", fdec[19 + i][15]);
		fprintf(fp_lc, "\n");//cr_left
	}
	//********************************************************

	//mode****************************************************
	for (int i = 0; i < 4; i++)
		intra4x4_pred_mode_cache[4 + i] == -1 ? fprintf(fp_lc, "x") : fprintf(fp_lc, "%d", mb_pred_mode4x4_fix(intra4x4_pred_mode_cache[4 + i]));
	fprintf(fp_lc, "\n");//top

	for (int i = 11; i < 36; i = i + 8)
		intra4x4_pred_mode_cache[i] == -1 ? fprintf(fp_lc, "x") : fprintf(fp_lc, "%d", mb_pred_mode4x4_fix(intra4x4_pred_mode_cache[i]));
	fprintf(fp_lc, "\n");//left
//*********************************************************
//input finish*********************************************
//*********************************************************


//*********************************************************
//output **************************************************
//*********************************************************
	fprintf(fp_cong, "===== Frame:   0  MB x:   %d  y:   %d =====\n", cur_mb.x, cur_mb.y);
	fprintf(fp_cong, "$intra_mode     : %d\n", mb_type == 0 ? 0 : 1);

	fprintf(fp_cong, "$intra4x4_mode    : ");
	for (int i = 0; i < 16; i++)
		fprintf(fp_cong, "%d", intra4x4_mode[i]);
	fprintf(fp_cong, "\n");

	fprintf(fp_cong, "$intra4x4_pred_mode    : ");
	for (int i = 0; i < 16; i++)
		fprintf(fp_cong, "%d", intra4x4_pred_mode[i]);
	fprintf(fp_cong, "\n");

	fprintf(fp_cong, "$intra16x16_mode: %d\n", mb_pred_mode16x16_fix[intra16x16_mode]);
	fprintf(fp_cong, "$chroma_mode    : %d\n", mb_pred_mode8x8c_fix[chroma_mode]);

	//*********************************************************
	//output finish********************************************
	//*********************************************************




		//写参数
	fprintf(fp, "======== Frame:%3d MB x:%3d y:%3d ========\n", param.frame_num, cur_mb.x, cur_mb.y);
	fprintf(fp, "# mb_type			: %d\n", mb_type);
	fprintf(fp, "# i_satd_i16x16		: %d\n", i_satd_i16x16);
	fprintf(fp, "# i_satd_i4x4_total	: %d\n", i_satd_i4x4_total);
	fprintf(fp, "# intra16x16_mode	: %d\n", intra16x16_mode);
	fprintf(fp, "# chroma_mode		: %d\n", chroma_mode);
	fprintf(fp, "# intra4x4_mode		: ");
	for (int i = 0; i < 16; i++) fprintf(fp, "%2d ", intra4x4_mode[i]);
	fprintf(fp, "\n");
	fprintf(fp, "# intra4x4_pred_mode: ");
	for (int i = 0; i < 16; i++) fprintf(fp, "%2d ", intra4x4_pred_mode[i]);
	fprintf(fp, "\n");

	//写原始像素
	fprintf(fp, "# fenc: \n");
	for (int i = 0; i < 24; i++) {
		for (int j = 0; j < 16; j++)
			fprintf(fp, "%2x ", int(fenc[i][j]));
		fprintf(fp, "\n");
	}

	//写重建像素块
	fprintf(fp, "# fdec: \n");
	for (int i = 0; i < 27; i++) {
		for (int j = 0; j < 32; j++)
			fprintf(fp, "%02x ", int(fdec[i][j]));
		fprintf(fp, "\n");
	}

	fprintf(fp, "# fdec2: \n");
	for (int i = 0; i < 18; i++) {
		for (int j = 0; j < 32; j++)
			fprintf(fp, "%2x ", int(fdec2[i][j]));
		fprintf(fp, "\n");
	}


	fclose(fp);
	fclose(fp_lc);
	fclose(fp_cong);
#endif

}

void intra::run() {
	set_neighbour();
	predict_luma16x16();
	predict_luma4x4();
	predict_chroma8x8();
	mode_decision_luma();//亮度块模式判决，是采用16*16还是4*4
	cbp_encode();
}

/****************************************************************************
 *                          intra predict
 ****************************************************************************/

 //16*16亮度块预测
void intra::predict_luma16x16() {
	int8_t mode_16x16[4], mode_16x16_cnt;
	int8_t i, i_mode;
	int i_satd_16x16;

	i_satd_i16x16 = COST_MAX;//记录最小残差，先赋予最大值
	mode_16x16_cnt = 0;
	predict_16x16_mode_available(i_neighbour, mode_16x16, &mode_16x16_cnt);//获取可选择的预测模式及预测模式数量

	//选择预测模式，计算stad，选择使得stad最小的预测模式作为16*16块的预测模式，并进行预测

	for (i = 0; i < mode_16x16_cnt; i++)
	{
		i_mode = mode_16x16[i];
		predict_16x16[i_mode](fdec[2]);
		i_satd_16x16 = pixel_satd_wxh(fdec[2], FDEC_STRIDE, fenc[0], FENC_STRIDE, 16, 16);//哈达玛变换计算satd
		COPY2_IF_LT(i_satd_i16x16, i_satd_16x16, intra16x16_mode, i_mode);//intra16X16_mode保存最终的预测模式
	}
	predict_16x16[intra16x16_mode](fdec[2]);
}

//8*8色度块预测

void intra::predict_chroma8x8() {
	int8_t mode_chroma[4], mode_chroma_cnt;
	int8_t i, i_mode;
	uint8_t *p_src[2], *p_dst[2];
	int i_satd_chroma;
	int16_t dct4x4c[2][4][4][4];
	int16_t dct2x2_dc[2][2][2];

	//两个色度分量cr,cb
	p_src[0] = fenc[16];
	p_src[1] = fenc[16] + 8;
	p_dst[0] = fdec[19];
	p_dst[1] = fdec[19] + 16;
	i_satd_chroma = COST_MAX;
	mode_chroma_cnt = 0;
	predict_8x8c_mode_available(i_neighbour, mode_chroma, &mode_chroma_cnt);
	for (i = 0; i < mode_chroma_cnt; i++)
	{
		int i_satd;
		i_mode = mode_chroma[i];
		predict_8x8c[i_mode](p_dst[0]);
		predict_8x8c[i_mode](p_dst[1]);
		i_satd = pixel_satd_wxh(p_dst[0], FDEC_STRIDE, p_src[0], FENC_STRIDE, 8, 8);
		i_satd += pixel_satd_wxh(p_dst[1], FDEC_STRIDE, p_src[1], FENC_STRIDE, 8, 8);
		COPY2_IF_LT(i_satd_chroma, i_satd, chroma_mode, i_mode);
	}

	//两种色度分量采用同一种预测模式
	predict_8x8c[chroma_mode](p_dst[0]);
	predict_8x8c[chroma_mode](p_dst[1]);

	//两种色度分量的残差的DCT变化
	sub8x8_dct(dct4x4c[0], p_src[0], FENC_STRIDE, p_dst[0], FDEC_STRIDE);
	sub8x8_dct(dct4x4c[1], p_src[1], FENC_STRIDE, p_dst[1], FDEC_STRIDE);

	dct2x2dc(dct2x2_dc[0], dct4x4c[0]);
	dct2x2dc(dct2x2_dc[1], dct4x4c[1]);

	for (i = 0; i < 4; i++) {
		quant_4x4(dct4x4c[0][i], cqm.quant4_mf[QP_FOR_CHROMA[cqm.qp]], cqm.intra_quant4_bias[QP_FOR_CHROMA[cqm.qp]], QP_FOR_CHROMA[cqm.qp]);
		quant_4x4(dct4x4c[1][i], cqm.quant4_mf[QP_FOR_CHROMA[cqm.qp]], cqm.intra_quant4_bias[QP_FOR_CHROMA[cqm.qp]], QP_FOR_CHROMA[cqm.qp]);
		zigzag_scan_4x4_frame(chroma_dct4x4_z4[0][i], dct4x4c[0][i]);
		zigzag_scan_4x4_frame(chroma_dct4x4_z4[1][i], dct4x4c[1][i]);
		dequant_4x4(dct4x4c[0][i], cqm.dequant4_mf, QP_FOR_CHROMA[cqm.qp]);
		dequant_4x4(dct4x4c[1][i], cqm.dequant4_mf, QP_FOR_CHROMA[cqm.qp]);
	}
	quant_2x2_dc(dct2x2_dc[0], cqm.quant4_mf[QP_FOR_CHROMA[cqm.qp]][0], cqm.intra_quant4_bias[QP_FOR_CHROMA[cqm.qp]][0] << 1, QP_FOR_CHROMA[cqm.qp]);
	quant_2x2_dc(dct2x2_dc[1], cqm.quant4_mf[QP_FOR_CHROMA[cqm.qp]][0], cqm.intra_quant4_bias[QP_FOR_CHROMA[cqm.qp]][0] << 1, QP_FOR_CHROMA[cqm.qp]);
	zigzag_scan_2x2_dc(chroma_dct4x4_dc_z2[0], dct2x2_dc[0]);
	zigzag_scan_2x2_dc(chroma_dct4x4_dc_z2[1], dct2x2_dc[1]);
	idct_dequant_2x2_dc(dct2x2_dc[0], dct4x4c[0], cqm.dequant4_mf, QP_FOR_CHROMA[cqm.qp]);
	idct_dequant_2x2_dc(dct2x2_dc[1], dct4x4c[1], cqm.dequant4_mf, QP_FOR_CHROMA[cqm.qp]);
	add8x8_idct(p_dst[0], FDEC_STRIDE, dct4x4c[0]);
	add8x8_idct(p_dst[1], FDEC_STRIDE, dct4x4c[1]);
}

//4x4亮度块预测

void intra::predict_luma4x4() {
	int16_t dct4x4[16][4][4];
	int8_t mode_4x4[9], mode_4x4_cnt;
	int i_satd_4x4[16];  // temp satd for each mode 
	int i_satd_i4x4[16]; // min satd for one 4x4

	// init satd
	i_satd_i4x4_total = 0;
	for (int k = 0; k < 16; k++) {
		i_satd_i4x4[k] = COST_MAX;
	}
	i_lambda = lambda_tab[cqm.qp];

	// i-> 0 1 4 5 
	//     2 3 6 7
	//     8 9 c d  
	//     a b e f
	for (int i = 0; i < 16; i++) {
		int xy_fdec = block_idx_xy_fdec[i];
		int xy_fenc = block_idx_xy_fenc[i];
		int i_mode;

		i_satd_i4x4[i] = COST_MAX;
		intra4x4_pred_mode[i] = predict_intra_luma_mode(i);
		predict_luma_mode_available(i, i_neighbour4[i], mode_4x4, &mode_4x4_cnt);
		for (int j = 0; j < mode_4x4_cnt; j++) {
			i_mode = mode_4x4[j];
			predict_4x4[i_mode](fdec2[2] + xy_fdec);
			i_satd_4x4[i] = pixel_satd_wxh(fdec2[2] + xy_fdec, FDEC_STRIDE, fenc[0] + xy_fenc, FENC_STRIDE, 4, 4)
				+ i_lambda * ((mb_pred_mode4x4_fix(i_mode) == intra4x4_pred_mode[i] ? 0 : 4));
			COPY2_IF_LT(i_satd_i4x4[i], i_satd_4x4[i], intra4x4_mode[i], i_mode);
		}
		predict_4x4[intra4x4_mode[i]](fdec2[2] + xy_fdec);

		if (mb_pred_mode4x4_fix(intra4x4_mode[i]) == intra4x4_pred_mode[i])
			prev_intra4x4_pred_mode_flag[i] = 1;
		intra4x4_pred_mode_cache[f264_scan8[i]] = intra4x4_mode[i];

		i_satd_i4x4_total += i_satd_i4x4[i];
		// dct-quant-zigzag-dequant-idctadd
		sad4 += sub4x4_dct(dct4x4[i], fenc[0] + xy_fenc, FENC_STRIDE, fdec2[2] + xy_fdec, FDEC_STRIDE);//sub4*4_dct函数返回了一个残差值
		quant_4x4(dct4x4[i], cqm.quant4_mf[cqm.qp], cqm.intra_quant4_bias[cqm.qp], cqm.qp);
		zigzag_scan_4x4_frame(luma_dct4x4_z4[i], dct4x4[i]);
		dequant_4x4(dct4x4[i], cqm.dequant4_mf, cqm.qp);
		add4x4_idct(fdec2[2] + xy_fdec, FDEC_STRIDE, dct4x4[i]);
	}
}


//决定16*16的亮度块还需不需要细化分为16个4*4的亮度块，倘若不需要，按照16*16亮度块预测并继续后续变换量化操作

void intra::mode_decision_luma() {
	if (!f264_ANALYSE_I16x16)
		i_satd_i16x16 = COST_MAX;
	if (!f264_ANALYSE_I4x4)
		i_satd_i4x4_total = COST_MAX;

	if (i_satd_i16x16 < i_satd_i4x4_total)
		mb_type = I_16x16;
	else
		mb_type = I_4x4;

	if (mb_type == I_16x16) {
		int16_t dct4x4[16][4][4];
		int16_t	dct4x4_dc[4][4];

		// dct 
		sad16 = sub16x16_dct(dct4x4, fenc[0], FENC_STRIDE, fdec[2], FDEC_STRIDE);
		// quant-zigzag-dequant
		for (int i = 0; i < 16; i++) {
			dct4x4_dc[block_idx_xy_1d[i] / 4][block_idx_xy_1d[i] % 4] = dct4x4[i][0][0]; // change scan order
			dct4x4[i][0][0] = 0;
			quant_4x4(dct4x4[i], cqm.quant4_mf[cqm.qp], cqm.intra_quant4_bias[cqm.qp], cqm.qp);
			zigzag_scan_4x4_frame(luma_dct4x4_z16[i], dct4x4[i]);
			dequant_4x4(dct4x4[i], cqm.dequant4_mf, cqm.qp);
		}
		dct4x4dc(dct4x4_dc);
		quant_4x4_dc(dct4x4_dc, cqm.quant4_mf[cqm.qp][0], cqm.intra_quant4_bias[cqm.qp][0] << 1, cqm.qp);
		zigzag_scan_4x4_frame(luma_dct4x4_dc_z4, dct4x4_dc);
		idct4x4dc(dct4x4_dc);
		dequant_4x4_dc(dct4x4_dc, cqm.dequant4_mf, cqm.qp);
		for (int i = 0; i < 16; i++) {
			dct4x4[i][0][0] = dct4x4_dc[block_idx_xy_1d[i] / 4][block_idx_xy_1d[i] % 4];
		}
		add16x16_idct(fdec[2], FDEC_STRIDE, dct4x4);
	}
	else if (mb_type == I_4x4) {
		for (int i = 0; i < 16; i++)
			intra4x4_mode[i] = mb_pred_mode4x4_fix(intra4x4_mode[i]);
	}
	
}

//非0块的标志???8x8块

void intra::cbp_encode() {
	cbp_luma = 0;
	if (mb_type == I_16x16) {
		for (int i = 0; i < 16; i++) {
			int nz = array_non_zero(luma_dct4x4_z16[i]);
			non_zero_count[i] = nz;
			cbp_luma |= nz;
		}
		cbp_luma *= 0xf;
		non_zero_count[24] = array_non_zero(luma_dct4x4_dc_z4);
	}
	else if (mb_type == I_4x4) {
		for (int i = 0; i < 4; i++)
		{
			int nz, cbp = 0;
			for (int j = 0; j < 4; j++)
			{
				nz = array_non_zero(luma_dct4x4_z4[j + 4 * i]);
				non_zero_count[j + 4 * i] = nz;
				cbp |= nz;
			}
			cbp_luma |= cbp << i;
		}
		non_zero_count[24] = 0;
	}

	cbp_chroma = 0;
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 4; j++) {
			int nz = array_non_zero(chroma_dct4x4_z4[i][j]);
			non_zero_count[16 + i * 4 + j] = nz;
			cbp_chroma |= nz;
		}
	non_zero_count[25] = array_non_zero(chroma_dct4x4_dc_z2[0]);
	non_zero_count[26] = array_non_zero(chroma_dct4x4_dc_z2[1]);
	if (cbp_chroma)
		cbp_chroma = 2;  /*   dc+ac (we can't do only ac) */
	else if (non_zero_count[25] |
		non_zero_count[26])
		cbp_chroma = 1;  /* dc only */

	cbp_dc = non_zero_count[24]
		| non_zero_count[25] << 1
		| non_zero_count[26] << 2;

	/* final output cbp */
	cbp = (cbp_dc << 8) | (cbp_chroma << 4) | cbp_luma;
}

void intra::set_neighbour() {
	i_neighbour = 0; //4bit: TL-TR-T-L
	if (cur_mb.x > 0) i_neighbour |= MB_LEFT;//按位或，当前块的x大于0，左边有
	if (cur_mb.y > 0) i_neighbour |= MB_TOP;//当前块的y大于0，上边有
	if (cur_mb.x > 0 && cur_mb.y > 0) i_neighbour |= MB_TOPLEFT;
	if (cur_mb.x < param.frame_mb_x_total - 1 && cur_mb.y>0) i_neighbour |= MB_TOPRIGHT;
	i_neighbour4[0] = (i_neighbour & (MB_TOP | MB_LEFT | MB_TOPLEFT)) | ((i_neighbour & MB_TOP) ? MB_TOPRIGHT : 0);
	i_neighbour4[4] =
		i_neighbour4[1] = MB_LEFT | ((i_neighbour & MB_TOP) ? (MB_TOP | MB_TOPLEFT | MB_TOPRIGHT) : 0);
	i_neighbour4[2] =
		i_neighbour4[8] =
		i_neighbour4[10] = MB_TOP | MB_TOPRIGHT | ((i_neighbour & MB_LEFT) ? (MB_LEFT | MB_TOPLEFT) : 0);
	i_neighbour4[3] =
		i_neighbour4[7] =
		i_neighbour4[11] =
		i_neighbour4[13] =
		i_neighbour4[15] = MB_LEFT | MB_TOP | MB_TOPLEFT;
	i_neighbour4[5] = MB_LEFT | (i_neighbour & MB_TOPRIGHT) | ((i_neighbour&MB_TOP) ? MB_TOP | MB_TOPLEFT : 0);
	i_neighbour4[6] =
		i_neighbour4[9] =
		i_neighbour4[12] =
		i_neighbour4[14] = MB_LEFT | MB_TOP | MB_TOPLEFT | MB_TOPRIGHT;
}

int intra::predict_intra_luma_mode(int idx)//获取第idx个4x4子块参考的预测模式
{

	const int ma = intra4x4_pred_mode_cache[f264_scan8[idx] - 1];//change intra4x4_used_mode[]update by other function
	const int mb = intra4x4_pred_mode_cache[f264_scan8[idx] - 8];//change
	const int m = f264_MIN(mb_pred_mode4x4_fix(ma),
		mb_pred_mode4x4_fix(mb));//add 1

	if (m < 0)
		return I_PRED_4x4_DC;

	return m;
}

/****************************************************************************
 *                 intra predict available mode
 ****************************************************************************/
void intra::predict_16x16_mode_available(uint8_t i_neighbour, int8_t *mode, int8_t *mode_cnt)
{
	if (i_neighbour & MB_TOPLEFT)//按位与
	{
		/* top and left available */
		*mode++ = I_PRED_16x16_V;
		*mode++ = I_PRED_16x16_H;
		*mode++ = I_PRED_16x16_DC;
		*mode++ = I_PRED_16x16_P;
		*mode_cnt = 4;
	}
	else if (i_neighbour & MB_LEFT)
	{
		/* left available*/
		*mode++ = I_PRED_16x16_H;
		*mode++ = I_PRED_16x16_DC_LEFT;
		*mode_cnt = 2;
	}
	else if (i_neighbour & MB_TOP)
	{
		/* top available*/
		*mode++ = I_PRED_16x16_V;
		*mode++ = I_PRED_16x16_DC_TOP;
		*mode_cnt = 2;
	}
	else
	{
		/* none available */
		*mode = I_PRED_16x16_DC_128;
		*mode_cnt = 1;
	}
}

void intra::predict_8x8c_mode_available(uint8_t i_neighbour, int8_t *mode, int8_t *mode_cnt)
{
	if (i_neighbour & MB_TOPLEFT)
	{
		// top and left available
		*mode++ = I_PRED_CHROMA_V;
		*mode++ = I_PRED_CHROMA_H;
		*mode++ = I_PRED_CHROMA_DC;
		*mode++ = I_PRED_CHROMA_P;
		*mode_cnt = 4;
	}
	else if (i_neighbour & MB_LEFT)
	{
		/* left available*/
		*mode++ = I_PRED_CHROMA_H;
		*mode++ = I_PRED_CHROMA_DC_LEFT;
		*mode_cnt = 2;
	}
	else if (i_neighbour & MB_TOP)
	{
		/* top available*/
		*mode++ = I_PRED_CHROMA_V;
		*mode++ = I_PRED_CHROMA_DC_TOP;
		*mode_cnt = 2;
	}
	else
	{
		/* none available */
		*mode = I_PRED_CHROMA_DC_128;
		*mode_cnt = 1;
	}
}

void intra::predict_luma_mode_available(int8_t i, uint8_t i_neighbour, int8_t *mode, int8_t *mode_cnt)
{
	int b_l = i_neighbour & MB_LEFT;
	int b_t = i_neighbour & MB_TOP;

	if (b_l && b_t)
	{
		*mode_cnt = 7;
		*mode++ = I_PRED_4x4_V;
		if (i == 2 || i == 6 || i == 10 || i == 14) {
			*mode++ = I_PRED_4x4_DC;
			*mode++ = I_PRED_4x4_H;
			*mode++ = I_PRED_4x4_HU;
			*mode++ = I_PRED_4x4_DDR;
			*mode++ = I_PRED_4x4_VR;
			*mode++ = I_PRED_4x4_HD;
			if (i_neighbour & MB_TOPRIGHT)
			{
				*mode++ = I_PRED_4x4_DDL;
				*mode++ = I_PRED_4x4_VL;
				*mode_cnt += 2;
			}
		}
		else
		{
			if (i_neighbour & MB_TOPRIGHT)
			{
				*mode++ = I_PRED_4x4_DDL;
				*mode++ = I_PRED_4x4_VL;
				*mode_cnt += 2;
			}
			*mode++ = I_PRED_4x4_DC;
			*mode++ = I_PRED_4x4_H;
			*mode++ = I_PRED_4x4_HU;
			*mode++ = I_PRED_4x4_DDR;
			*mode++ = I_PRED_4x4_VR;
			*mode++ = I_PRED_4x4_HD;
		}
	}
	else if (b_l)
	{
		*mode++ = I_PRED_4x4_DC_LEFT;
		*mode++ = I_PRED_4x4_H;
		*mode++ = I_PRED_4x4_HU;
		*mode_cnt = 3;
	}
	else if (b_t)
	{
		*mode++ = I_PRED_4x4_V;
		*mode++ = I_PRED_4x4_DDL;
		*mode++ = I_PRED_4x4_VL;
		*mode++ = I_PRED_4x4_DC_TOP;
		*mode_cnt = 4;
	}
	else
	{
		*mode++ = I_PRED_4x4_DC_128;
		*mode_cnt = 1;
	}
}

/****************************************************************************
 * 16x16 prediction for intra luma block  16*16帧内亮度预测
 ****************************************************************************/
#define PREDICT_16x16_DC(v) \
    for( i = 0; i < 16; i++ )\
    {\
        uint32_t *p = (uint32_t*)src;\
        *p++ = v;\
        *p++ = v;\
        *p++ = v;\
        *p++ = v;\
        src += FDEC_STRIDE;\
    }

void predict_16x16_dc(uint8_t *src)//模式2，DC
{
	uint32_t dc = 0;
	int i;

	for (i = 0; i < 16; i++)
	{
		dc += src[-1 + i * FDEC_STRIDE];    //left
		dc += src[i - FDEC_STRIDE];         //top
	}
	dc = ((dc + 16) >> 5) * 0x01010101;   //copy 4

	PREDICT_16x16_DC(dc);
}
void predict_16x16_dc_left(uint8_t *src)//左边界DC
{
	uint32_t dc = 0;
	int i;

	for (i = 0; i < 16; i++)
	{
		dc += src[-1 + i * FDEC_STRIDE];
	}
	dc = ((dc + 8) >> 4) * 0x01010101;

	PREDICT_16x16_DC(dc);
}
void predict_16x16_dc_top(uint8_t *src)//上边界DC
{
	uint32_t dc = 0;
	int i;

	for (i = 0; i < 16; i++)
	{
		dc += src[i - FDEC_STRIDE];
	}
	dc = ((dc + 8) >> 4) * 0x01010101;

	PREDICT_16x16_DC(dc);
}
void predict_16x16_dc_128(uint8_t *src)//全部预测为128
{
	int i;
	PREDICT_16x16_DC(0x80808080);
}
void predict_16x16_h(uint8_t *src)//模式1,水平
{
	int i;

	for (i = 0; i < 16; i++)
	{
		const uint32_t v = 0x01010101 * src[-1];
		uint32_t *p = (uint32_t*)src;

		*p++ = v;
		*p++ = v;
		*p++ = v;
		*p++ = v;

		src += FDEC_STRIDE;
	}
}
void predict_16x16_v(uint8_t *src)//模式0，垂直
{
	uint32_t v0 = *(uint32_t*)&src[0 - FDEC_STRIDE];
	uint32_t v1 = *(uint32_t*)&src[4 - FDEC_STRIDE];
	uint32_t v2 = *(uint32_t*)&src[8 - FDEC_STRIDE];
	uint32_t v3 = *(uint32_t*)&src[12 - FDEC_STRIDE];
	int i;

	for (i = 0; i < 16; i++)
	{
		uint32_t *p = (uint32_t*)src;
		*p++ = v0;
		*p++ = v1;
		*p++ = v2;
		*p++ = v3;
		src += FDEC_STRIDE;
	}
}
void predict_16x16_p(uint8_t *src)//模式3,plane
{
	int x, y, i;
	int a, b, c;
	int H = 0;
	int V = 0;
	int i00;

	/* calculate H and V 算水平梯度和垂直梯度 */
	for (i = 0; i <= 7; i++)
	{
		H += (i + 1) * (src[8 + i - FDEC_STRIDE] - src[6 - i - FDEC_STRIDE]);
		V += (i + 1) * (src[-1 + (8 + i)*FDEC_STRIDE] - src[-1 + (6 - i)*FDEC_STRIDE]);
	}

	a = 16 * (src[-1 + 15 * FDEC_STRIDE] + src[15 - FDEC_STRIDE]);
	b = (5 * H + 32) >> 6;
	c = (5 * V + 32) >> 6;
	i00 = a - b * 7 - c * 7 + 16;

	for (y = 0; y < 16; y++)//对每行进行预测
	{
		int pix = i00;
		for (x = 0; x < 16; x++)//对每一行的每一个像素进行预测
		{
			src[x] = f264_clip_uint8(pix >> 5);//????
			pix += b;
		}
		src += FDEC_STRIDE;
		i00 += c;
	}
}

/****************************************************************************
 * 8x8 prediction for intra chroma block 8*8帧内色度预测，7种模式
 ****************************************************************************/
void predict_8x8c_dc_128(uint8_t *src)
{
	int y;

	for (y = 0; y < 8; y++)//利用循环一行一行处理
	{
		uint32_t *p = (uint32_t*)src;//p指向8*8块的一行的4个字节
		//处理一行8字节
		*p++ = 0x80808080;
		*p++ = 0x80808080;
		//指向下一行
		src += FDEC_STRIDE;
	}
}
void predict_8x8c_dc_left(uint8_t *src)
{
	int y;
	uint32_t dc0 = 0, dc1 = 0;

	for (y = 0; y < 4; y++)
	{
		dc0 += src[y * FDEC_STRIDE - 1];
		dc1 += src[(y + 4) * FDEC_STRIDE - 1];
	}
	dc0 = ((dc0 + 2) >> 2) * 0x01010101;
	dc1 = ((dc1 + 2) >> 2) * 0x01010101;

	for (y = 0; y < 4; y++)
	{
		uint32_t *p = (uint32_t*)src;
		*p++ = dc0;
		*p++ = dc0;
		src += FDEC_STRIDE;
	}
	for (y = 0; y < 4; y++)
	{
		uint32_t *p = (uint32_t*)src;
		*p++ = dc1;
		*p++ = dc1;
		src += FDEC_STRIDE;
	}

}

void predict_8x8c_dc_top(uint8_t *src)
{
	int y, x;
	uint32_t dc0 = 0, dc1 = 0;

	for (x = 0; x < 4; x++)
	{
		dc0 += src[x - FDEC_STRIDE];
		dc1 += src[x + 4 - FDEC_STRIDE];
	}
	dc0 = ((dc0 + 2) >> 2) * 0x01010101;
	dc1 = ((dc1 + 2) >> 2) * 0x01010101;

	for (y = 0; y < 8; y++)
	{
		uint32_t *p = (uint32_t*)src;
		*p++ = dc0;
		*p++ = dc1;
		src += FDEC_STRIDE;
	}
}

void predict_8x8c_dc(uint8_t *src)//模式0，DC
{
	int y;
	int s0 = 0, s1 = 0, s2 = 0, s3 = 0;
	uint32_t dc0, dc1, dc2, dc3;
	int i;

	/*
		  s0 s1
	   s2
	   s3
	*/
	for (i = 0; i < 4; i++)
	{
		s0 += src[i - FDEC_STRIDE];
		s1 += src[i + 4 - FDEC_STRIDE];
		s2 += src[-1 + i * FDEC_STRIDE];
		s3 += src[-1 + (i + 4)*FDEC_STRIDE];
	}
	/*
	   dc0 dc1
	   dc2 dc3
	 */
	dc0 = ((s0 + s2 + 4) >> 3) * 0x01010101;
	dc1 = ((s1 + 2) >> 2) * 0x01010101;
	dc2 = ((s3 + 2) >> 2) * 0x01010101;
	dc3 = ((s1 + s3 + 4) >> 3) * 0x01010101;

	for (y = 0; y < 4; y++)
	{
		uint32_t *p = (uint32_t*)src;
		*p++ = dc0;
		*p++ = dc1;
		src += FDEC_STRIDE;
	}

	for (y = 0; y < 4; y++)
	{
		uint32_t *p = (uint32_t*)src;
		*p++ = dc2;
		*p++ = dc3;
		src += FDEC_STRIDE;
	}
}

void predict_8x8c_h(uint8_t *src)//模式1，水平
{
	int i;

	for (i = 0; i < 8; i++)
	{
		uint32_t v = 0x01010101 * src[-1];
		uint32_t *p = (uint32_t*)src;
		*p++ = v;
		*p++ = v;
		src += FDEC_STRIDE;
	}
}

void predict_8x8c_v(uint8_t *src)//模式2，垂直
{
	uint32_t v0 = *(uint32_t*)&src[0 - FDEC_STRIDE];
	uint32_t v1 = *(uint32_t*)&src[4 - FDEC_STRIDE];
	int i;

	for (i = 0; i < 8; i++)
	{
		uint32_t *p = (uint32_t*)src;
		*p++ = v0;
		*p++ = v1;
		src += FDEC_STRIDE;
	}
}

void predict_8x8c_p(uint8_t *src)//模式3，平面
{
	int i;
	int x, y;
	int a, b, c;
	int H = 0;
	int V = 0;
	int i00;

	for (i = 0; i < 4; i++)
	{
		H += (i + 1) * (src[4 + i - FDEC_STRIDE] - src[2 - i - FDEC_STRIDE]);
		V += (i + 1) * (src[-1 + (i + 4)*FDEC_STRIDE] - src[-1 + (2 - i)*FDEC_STRIDE]);
	}

	a = 16 * (src[-1 + 7 * FDEC_STRIDE] + src[7 - FDEC_STRIDE]);
	b = (17 * H + 16) >> 5;
	c = (17 * V + 16) >> 5;
	i00 = a - 3 * b - 3 * c + 16;

	for (y = 0; y < 8; y++)
	{
		int pix = i00;
		for (x = 0; x < 8; x++)
		{
			src[x] = f264_clip_uint8(pix >> 5);
			pix += b;
		}
		src += FDEC_STRIDE;
		i00 += c;
	}
}

/****************************************************************************
 * 4x4 prediction for intra luma block 4*4帧内亮度预测 12种模式
 ****************************************************************************/
#define SRC(x,y) src[(x)+(y)*FDEC_STRIDE]
#define SRC32(x,y) *(uint32_t*)&SRC(x,y)

 //v:四字节数，32bit，用于给4*4块的每一行赋值

#define PREDICT_4x4_DC(v)\
    SRC32(0,0) = SRC32(0,1) = SRC32(0,2) = SRC32(0,3) = v;

void predict_4x4_dc_128(uint8_t *src)
{
	PREDICT_4x4_DC(0x80808080);
}
void predict_4x4_dc_left(uint8_t *src)
{
	uint32_t dc = ((SRC(-1, 0) + SRC(-1, 1) + SRC(-1, 2) + SRC(-1, 3) + 2) >> 2) * 0x01010101;
	PREDICT_4x4_DC(dc);
}
void predict_4x4_dc_top(uint8_t *src)
{
	uint32_t dc = ((SRC(0, -1) + SRC(1, -1) + SRC(2, -1) + SRC(3, -1) + 2) >> 2) * 0x01010101;
	PREDICT_4x4_DC(dc);
}

void predict_4x4_dc(uint8_t *src)//模式2，DC模式
{
	uint32_t dc = ((SRC(-1, 0) + SRC(-1, 1) + SRC(-1, 2) + SRC(-1, 3) +
		SRC(0, -1) + SRC(1, -1) + SRC(2, -1) + SRC(3, -1) + 4) >> 3) * 0x01010101;
	PREDICT_4x4_DC(dc);
}
void predict_4x4_h(uint8_t *src)//模式1，水平预测
{
	SRC32(0, 0) = SRC(-1, 0) * 0x01010101;
	SRC32(0, 1) = SRC(-1, 1) * 0x01010101;
	SRC32(0, 2) = SRC(-1, 2) * 0x01010101;
	SRC32(0, 3) = SRC(-1, 3) * 0x01010101;
}
void predict_4x4_v(uint8_t *src)//模式0，垂直预测
{
	PREDICT_4x4_DC(SRC32(0, -1));
}

#define F1(a,b)   (((a)+(b)+1)>>1)
#define F2(a,b,c) (((a)+2*(b)+(c)+2)>>2)

void predict_4x4_ddl(uint8_t *src)//模式3，左下对角线
{
	uint8_t t0 = SRC(0, -1);
	uint8_t t1 = SRC(1, -1);
	uint8_t t2 = SRC(2, -1);
	uint8_t t3 = SRC(3, -1);
	uint8_t t4 = SRC(4, -1);
	uint8_t t5 = SRC(5, -1);
	uint8_t t6 = SRC(6, -1);
	uint8_t t7 = SRC(7, -1);

	SRC(0, 0) = F2(t0, t1, t2);
	SRC(1, 0) = SRC(0, 1) = F2(t1, t2, t3);
	SRC(2, 0) = SRC(1, 1) = SRC(0, 2) = F2(t2, t3, t4);
	SRC(3, 0) = SRC(2, 1) = SRC(1, 2) = SRC(0, 3) = F2(t3, t4, t5);
	SRC(3, 1) = SRC(2, 2) = SRC(1, 3) = F2(t4, t5, t6);
	SRC(3, 2) = SRC(2, 3) = F2(t5, t6, t7);
	SRC(3, 3) = F2(t6, t7, t7);
}
void predict_4x4_ddr(uint8_t *src)//模式4 右下对角线
{
	const int lt = SRC(-1, -1);
	int l0 = SRC(-1, 0);
	int l1 = SRC(-1, 1);
	int l2 = SRC(-1, 2);
	int l3 = SRC(-1, 3);
	int t0 = SRC(0, -1);
	int t1 = SRC(1, -1);
	int t2 = SRC(2, -1);
	int t3 = SRC(3, -1);

	SRC(3, 0) = F2(t3, t2, t1);
	SRC(2, 0) = SRC(3, 1) = F2(t2, t1, t0);
	SRC(1, 0) = SRC(2, 1) = SRC(3, 2) = F2(t1, t0, lt);
	SRC(0, 0) = SRC(1, 1) = SRC(2, 2) = SRC(3, 3) = F2(t0, lt, l0);
	SRC(0, 1) = SRC(1, 2) = SRC(2, 3) = F2(lt, l0, l1);
	SRC(0, 2) = SRC(1, 3) = F2(l0, l1, l2);
	SRC(0, 3) = F2(l1, l2, l3);
}

void predict_4x4_vr(uint8_t *src)//模式5，垂直向右
{
	const int lt = SRC(-1, -1);
	int l0 = SRC(-1, 0);
	int l1 = SRC(-1, 1);
	int l2 = SRC(-1, 2);
	int l3 = SRC(-1, 3);
	int t0 = SRC(0, -1);
	int t1 = SRC(1, -1);
	int t2 = SRC(2, -1);
	int t3 = SRC(3, -1);

	SRC(0, 3) = F2(l2, l1, l0);
	SRC(0, 2) = F2(l1, l0, lt);
	SRC(0, 1) = SRC(1, 3) = F2(l0, lt, t0);
	SRC(0, 0) = SRC(1, 2) = F1(lt, t0);
	SRC(1, 1) = SRC(2, 3) = F2(lt, t0, t1);
	SRC(1, 0) = SRC(2, 2) = F1(t0, t1);
	SRC(2, 1) = SRC(3, 3) = F2(t0, t1, t2);
	SRC(2, 0) = SRC(3, 2) = F1(t1, t2);
	SRC(3, 1) = F2(t1, t2, t3);
	SRC(3, 0) = F1(t2, t3);
}

void predict_4x4_hd(uint8_t *src)//模式6，水平向下
{
	const int lt = SRC(-1, -1);
	int l0 = SRC(-1, 0);
	int l1 = SRC(-1, 1);
	int l2 = SRC(-1, 2);
	int l3 = SRC(-1, 3);
	int t0 = SRC(0, -1);
	int t1 = SRC(1, -1);
	int t2 = SRC(2, -1);
	int t3 = SRC(3, -1);

	SRC(0, 3) = F1(l2, l3);
	SRC(1, 3) = F2(l1, l2, l3);
	SRC(0, 2) = SRC(2, 3) = F1(l1, l2);
	SRC(1, 2) = SRC(3, 3) = F2(l0, l1, l2);
	SRC(0, 1) = SRC(2, 2) = F1(l0, l1);
	SRC(1, 1) = SRC(3, 2) = F2(lt, l0, l1);
	SRC(0, 0) = SRC(2, 1) = F1(lt, l0);
	SRC(1, 0) = SRC(3, 1) = F2(t0, lt, l0);
	SRC(2, 0) = F2(t1, t0, lt);
	SRC(3, 0) = F2(t2, t1, t0);
}

void predict_4x4_vl(uint8_t *src)//模式7，垂直向左
{
	int t0 = SRC(0, -1);
	int t1 = SRC(1, -1);
	int t2 = SRC(2, -1);
	int t3 = SRC(3, -1);
	int t4 = SRC(4, -1);
	int t5 = SRC(5, -1);
	int t6 = SRC(6, -1);
	int t7 = SRC(7, -1);

	SRC(0, 0) = F1(t0, t1);
	SRC(0, 1) = F2(t0, t1, t2);
	SRC(1, 0) = SRC(0, 2) = F1(t1, t2);
	SRC(1, 1) = SRC(0, 3) = F2(t1, t2, t3);
	SRC(2, 0) = SRC(1, 2) = F1(t2, t3);
	SRC(2, 1) = SRC(1, 3) = F2(t2, t3, t4);
	SRC(3, 0) = SRC(2, 2) = F1(t3, t4);
	SRC(3, 1) = SRC(2, 3) = F2(t3, t4, t5);
	SRC(3, 2) = F1(t4, t5);
	SRC(3, 3) = F2(t4, t5, t6);
}

void predict_4x4_hu(uint8_t *src)//模式8，水平向上
{
	int l0 = SRC(-1, 0);
	int l1 = SRC(-1, 1);
	int l2 = SRC(-1, 2);
	int l3 = SRC(-1, 3);

	SRC(0, 0) = F1(l0, l1);
	SRC(1, 0) = F2(l0, l1, l2);
	SRC(2, 0) = SRC(0, 1) = F1(l1, l2);
	SRC(3, 0) = SRC(1, 1) = F2(l1, l2, l3);
	SRC(2, 1) = SRC(0, 2) = F1(l2, l3);
	SRC(3, 1) = SRC(1, 2) = F2(l2, l3, l3);
	SRC(3, 2) = SRC(1, 3) = SRC(0, 3) =
		SRC(2, 2) = SRC(2, 3) = SRC(3, 3) = l3;
}


/****************************************************************************
 * prediction function point init
 ****************************************************************************/
 //函数指针的方式进行定义，参数是函数指针数组，将每个函数指针定义成相应函数的入口地址
 //16*16亮度预测

void intra::predict_16x16_init(predict_16x16_t predf_16x16[7])
{
	predf_16x16[I_PRED_16x16_V] = predict_16x16_v;
	predf_16x16[I_PRED_16x16_H] = predict_16x16_h;
	predf_16x16[I_PRED_16x16_DC] = predict_16x16_dc;
	predf_16x16[I_PRED_16x16_P] = predict_16x16_p;
	predf_16x16[I_PRED_16x16_DC_LEFT] = predict_16x16_dc_left;
	predf_16x16[I_PRED_16x16_DC_TOP] = predict_16x16_dc_top;
	predf_16x16[I_PRED_16x16_DC_128] = predict_16x16_dc_128;

}

//4*4亮度预测

void intra::predict_4x4_init(predict_4x4_t predf_4x4[12])
{
	predf_4x4[I_PRED_4x4_V] = predict_4x4_v;
	predf_4x4[I_PRED_4x4_H] = predict_4x4_h;
	predf_4x4[I_PRED_4x4_DC] = predict_4x4_dc;
	predf_4x4[I_PRED_4x4_DDL] = predict_4x4_ddl;
	predf_4x4[I_PRED_4x4_DDR] = predict_4x4_ddr;
	predf_4x4[I_PRED_4x4_VR] = predict_4x4_vr;
	predf_4x4[I_PRED_4x4_HD] = predict_4x4_hd;
	predf_4x4[I_PRED_4x4_VL] = predict_4x4_vl;
	predf_4x4[I_PRED_4x4_HU] = predict_4x4_hu;
	predf_4x4[I_PRED_4x4_DC_LEFT] = predict_4x4_dc_left;
	predf_4x4[I_PRED_4x4_DC_TOP] = predict_4x4_dc_top;
	predf_4x4[I_PRED_4x4_DC_128] = predict_4x4_dc_128;

}


//8*8色度预测

void intra::predict_8x8c_init(predict_8x8c_t predf_chroma[7])
{
	predf_chroma[I_PRED_CHROMA_V] = predict_8x8c_v;
	predf_chroma[I_PRED_CHROMA_H] = predict_8x8c_h;
	predf_chroma[I_PRED_CHROMA_DC] = predict_8x8c_dc;
	predf_chroma[I_PRED_CHROMA_P] = predict_8x8c_p;
	predf_chroma[I_PRED_CHROMA_DC_LEFT] = predict_8x8c_dc_left;
	predf_chroma[I_PRED_CHROMA_DC_TOP] = predict_8x8c_dc_top;
	predf_chroma[I_PRED_CHROMA_DC_128] = predict_8x8c_dc_128;
}