#include "inter_1.h"



void inter::init(){
	sw_center[0] = (SW_W - 16)>>1;
	sw_center[1] = (SW_H - 16)>>1;
}

void inter::load(){

}

void inter::read(mb_t& i_cur_mb, sw_t& i_sw, cqm_t& i_cqm, param_t& i_param){
	cur_mb = i_cur_mb;
	sw = i_sw;
	cqm = i_cqm;
	param = i_param;
	sw.sr_w = 12;
	sw.sr_h = 12;
}


void inter::proc()
{
	//init mb info buffer for a frame
    if ((cur_mb.x == 0) && (cur_mb.y == 0)) {
        me_buffer = new struct inter_bf[param.frame_mb_x_total * 2];
        memset(ame_flg_mat, 0, sizeof(ame_flg_mat));
    }
    printf("\n\tstart proc %d at mb(%d, %d)...\n", cur_mb.x + cur_mb.y * param.frame_mb_x_total, cur_mb.x, cur_mb.y);
	run();
	dump();
}

void inter::run()
{
	ime();
	fme();
	ame();
	mode_decision_sort();
	mc();
}

void inter::del(){
	delete []me_buffer;
	delete []mv_line;
}

void inter::ime(){
	int lambda;
	int16_t imv[2];
	int16_t mvd[2];
	int mv_cost;
	// motion estimation
	uint8_t *p_block, *p_sw;
	int part_x, part_y;
	int sad4x4_tmp[4][4];
	int sad8x4_tmp[4][2];
	int sad4x8_tmp[4][2];
	int sad8x8_tmp[4];
	int sad8x16_tmp[2];
	int sad16x8_tmp[2];
	int sad16x16_tmp;
	int cost4x4_tmp[4][4];
	int cost8x4_tmp[4][2];
	int cost4x8_tmp[4][2];
	int cost8x8_tmp[4];
	int cost8x16_tmp[2];
	int cost16x8_tmp[2];
	int cost16x16_tmp;


	lambda = f264_lambda_tab[param.qp];

	// init cost
	cost16x16 = COST_MAX;
	for (int blk = 0; blk<2; blk++){
		cost16x8[blk] = COST_MAX;
		cost8x16[blk] = COST_MAX;
	}
	for (int blk8x8 = 0; blk8x8<4; blk8x8++){
		cost8x8[blk8x8] = COST_MAX;
		for (int blk4x8 = 0; blk4x8<2; blk4x8++){
			cost8x4[blk8x8 * 2 + blk4x8] = COST_MAX;
			cost4x8[blk8x8 * 2 + blk4x8] = COST_MAX;
		}
		for (int blk4x4 = 0; blk4x4<4; blk4x4++){
			cost4x4[blk8x8 * 4 + blk4x4] = COST_MAX;
		}
	}

	/**********************************************************
	**                Motion Estimation                      **
	***********************************************************/
	int dx, dy;
	int y_dir; // scan direction of y, 0: down, 1: up
	y_dir = 1;

	/********************Caculate all of mv and block cost************************/
	for (int dir_x = -sw.sr_w; dir_x<(sw.sr_w - f264_PE_NUM + 1); dir_x += f264_PE_NUM){
		dx = dir_x;
		y_dir = !y_dir;
		for (int dir_y = -sw.sr_h; dir_y<sw.sr_h + 1; dir_y++){
			dy = (y_dir == 1) ? -dir_y : dir_y;
			for (int pe = 0; pe<f264_PE_NUM; pe++){
				imv[0] = (dx + pe) << 2;//here we use <<2 means the int mv is 0 4 8 12 16.... and the float mv 0 1 2 3
				imv[1] = dy << 2;
				mvd[0] = abs(imv[0]);
				mvd[1] = abs(imv[1]);
				mv_cost = lambda*(ime_bs_size_se(mvd[0]) + ime_bs_size_se(mvd[1]));
				// cost cal
				for (int blk8x8 = 0; blk8x8 < 4; blk8x8++) {
					for (int blk4x4 = 0; blk4x4 < 4; blk4x4++){
						part_x = 8 * (blk8x8 % 2) + (blk4x4 % 2) * 4;
						part_y = 8 * (blk8x8 / 2) + (blk4x4 / 2) * 4;
						//   SW_WxSW_W(48x48)           
						// -----------------           
						// |    |     |    |     
						// -----------------
						// |    |16x16|    |     
						// -----------------    
						// |    |     |    |
						// -----------------
						//  search_windows
						p_block = &cur_mb.luma[part_y][part_x];
						p_sw = &sw.sw_luma[sw_center[1] + part_y][sw_center[0] + part_x + pe];
						//4x4
						sad4x4_tmp[blk8x8][blk4x4] = pixel_sad_wxh(p_block, 16, (p_sw + dy*SW_W + dx), SW_W, 4, 4);
						cost4x4_tmp[blk8x8][blk4x4] = sad4x4_tmp[blk8x8][blk4x4] + mv_cost;
					}
				}

				for (int blk8x8 = 0; blk8x8<4; blk8x8++) {
					//8x4
					for (int blk = 0; blk<2; blk++){
						sad8x4_tmp[blk8x8][blk] = sad4x4_tmp[blk8x8][blk * 2] + sad4x4_tmp[blk8x8][blk * 2 + 1];
						cost8x4_tmp[blk8x8][blk] = sad8x4_tmp[blk8x8][blk] + mv_cost;
					}
					//4x8
					for (int blk = 0; blk<2; blk++){
						sad4x8_tmp[blk8x8][blk] = sad4x4_tmp[blk8x8][blk] + sad4x4_tmp[blk8x8][blk + 2];
						cost4x8_tmp[blk8x8][blk] = sad4x8_tmp[blk8x8][blk] + mv_cost;
					}
					//8x8
					sad8x8_tmp[blk8x8] = sad8x4_tmp[blk8x8][0] + sad8x4_tmp[blk8x8][1];
					cost8x8_tmp[blk8x8] = sad8x8_tmp[blk8x8] + mv_cost;
				}

				//16x16
				sad16x16_tmp = sad8x8_tmp[0] + sad8x8_tmp[1] + sad8x8_tmp[2] + sad8x8_tmp[3];
				cost16x16_tmp = sad16x16_tmp + mv_cost;

				//16x8
				for (int blk = 0; blk<2; blk++){
					sad16x8_tmp[blk] = (sad8x8_tmp[blk * 2] + sad8x8_tmp[blk * 2 + 1]);
					cost16x8_tmp[blk] = sad16x8_tmp[blk] + mv_cost;
				}

				//8x16
				for (int blk = 0; blk<2; blk++){
					sad8x16_tmp[blk] = (sad8x8_tmp[blk] + sad8x8_tmp[blk + 2]);
					cost8x16_tmp[blk] = sad8x16_tmp[blk] + mv_cost;
				}

				/********************Cost Compare************************/
				for (int blk8x8 = 0; blk8x8<4; blk8x8++){
					//4x4
					for (int blk4x4 = 0; blk4x4<4; blk4x4++){
						if (cost4x4_tmp[blk8x8][blk4x4] < cost4x4[blk8x8 * 4 + blk4x4]){
							cost4x4[blk8x8 * 4 + blk4x4] = cost4x4_tmp[blk8x8][blk4x4];
							mv4x4[blk8x8][blk4x4][0] = imv[0];
							mv4x4[blk8x8][blk4x4][1] = imv[1];
						}
					}
					//4x8
					for (int blk4x4 = 0; blk4x4<2; blk4x4++){
						if (cost4x8_tmp[blk8x8][blk4x4] < cost4x8[blk8x8 * 2 + blk4x4]){
							cost4x8[blk8x8 * 2 + blk4x4] = cost4x8_tmp[blk8x8][blk4x4];
							mv4x8[blk8x8][blk4x4][0] = imv[0];
							mv4x8[blk8x8][blk4x4][1] = imv[1];
						}
					}
					//8x4
					for (int blk4x4 = 0; blk4x4<2; blk4x4++){
						if (cost8x4_tmp[blk8x8][blk4x4] < cost8x4[blk8x8 * 2 + blk4x4]){
							cost8x4[blk8x8 * 2 + blk4x4] = cost8x4_tmp[blk8x8][blk4x4];
							mv8x4[blk8x8][blk4x4][0] = imv[0];
							mv8x4[blk8x8][blk4x4][1] = imv[1];
						}
					}
					//8x8
					if (cost8x8_tmp[blk8x8]  < cost8x8[blk8x8]){
						cost8x8[blk8x8] = cost8x8_tmp[blk8x8];
						mv8x8[blk8x8][0] = imv[0];
						mv8x8[blk8x8][1] = imv[1];
					}
				}

				for (int blk = 0; blk<2; blk++){
					//8x16
					if (cost8x16_tmp[blk] < cost8x16[blk]){
						cost8x16[blk] = cost8x16_tmp[blk];
						mv8x16[blk][0] = imv[0];
						mv8x16[blk][1] = imv[1];
					}
					//16x8
					if (cost16x8_tmp[blk] < cost16x8[blk]){
						cost16x8[blk] = cost16x8_tmp[blk];
						mv16x8[blk][0] = imv[0];
						mv16x8[blk][1] = imv[1];
					}
				}
				//16X16
				if (cost16x16_tmp < cost16x16){
					cost16x16 = cost16x16_tmp;
					mv16x16[0] = imv[0];
					mv16x16[1] = imv[1];
				}
			}
		}
	}

	/**********************************************************
	**     Cost Calculation for each partition mode          **
	***********************************************************/
	//16x16
	cost16x16_s = cost16x16;

	//16x8
	cost16x8_s = 0;
	for (int i = 0; i <= 1; i++)
		cost16x8_s += cost16x8[i];

	//8x16
	cost8x16_s = 0;
	for (int i = 0; i <= 1; i++)
		cost8x16_s += cost8x16[i];

	//8x8
	for (int i = 0; i <= 3; i++)
		cost8x8_s[i] = cost8x8[i];

	//8x8 block cost if current partition is 8x4
	for (int i = 0; i <= 3; i++) {
		cost8x4_s[i] = 0;
		for (int j = 0; j <= 1; j++){
			cost8x4_s[i] += cost8x4[i * 2 + j];
		}
	}

	//8x8 block cost if current partition is 4x8
	for (int i = 0; i <= 3; i++){
		cost4x8_s[i] = 0;
		for (int j = 0; j <= 1; j++){
			cost4x8_s[i] += cost4x8[i * 2 + j];
		}
	}

	//8x8 block cost if current partition is 4x4
	for (int i = 0; i <= 3; i++){
		cost4x4_s[i] = 0;
		for (int j = 0; j <= 3; j++){
			cost4x4_s[i] += cost4x4[i * 4 + j];
		}
	}


	/**********************************************************
	**                Mode Decision                          **
	***********************************************************/
	//sub partition mode decision
	cost8x8_s_min = 0;
	for (int i = 0; i <= 3; i++){
		int j = COST_MAX;
		if (cost8x8_s[i]<j) { j = cost8x8_s[i]; mb_subpartition[i] = D_L0_8x8; }//decide sub partiton as 8x8 first
		if (cost8x4_s[i]<j) { j = cost8x4_s[i]; mb_subpartition[i] = D_L0_8x4; }//8x4 cost is much lower than 8x8? yes:choose 8x4, no:choose 8x8
		if (cost4x8_s[i]<j) { j = cost4x8_s[i]; mb_subpartition[i] = D_L0_4x8; }//same as the up one 
		if (cost4x4_s[i]<j) { j = cost4x4_s[i]; mb_subpartition[i] = D_L0_4x4; }
		cost8x8_s_min = cost8x8_s_min + j;// calculate the sum of four 8x8 best cost 
	}

	//MB partition mode decision
	int j = COST_MAX;
	if (cost16x16_s <j)  { j = cost16x16_s;   mb_partition = D_16x16; mb_type = P_L0; }
	if (cost16x8_s  <j)  { j = cost16x8_s;    mb_partition = D_16x8;  mb_type = P_L0; }
	if (cost8x16_s  <j)  { j = cost8x16_s;    mb_partition = D_8x16;  mb_type = P_L0; }
	if (cost8x8_s_min<j) { j = cost8x8_s_min; mb_partition = D_8x8;   mb_type = P_8x8; }

	mb_cost_min = j;

	//dump ime information into current mb inter buffer
	mb_info.row = cur_mb.y;
	mb_info.col = cur_mb.x;
	mb_info.mb_partition = mb_partition;
	for(int i=0;i<4;i++)
		mb_info.mb_subpartition[i] = mb_subpartition[i];

}
//do affine motion estimation only on 16x16 block
//algorithem step:1. get mvp 2. do iteration based on the gratitude of SATD 3. Add cost mv and flag
void inter::ame(){
	//init iteration amv of 2 cp
	int16_t amv_temp[2][2];
	amv_temp[0][0] = 0;
	amv_temp[0][1] = 0;
	amv_temp[1][0] = 0;
	amv_temp[1][1] = 0;
	ame_min_cost = 0x7fffffff;
	ame_step = 6;//param get from VVCSoftware_BMS-Revision 1583 
	//InterSearch.cpp/interSearch::xAffineMotionEstimation function
	//init mvp list
	for(int i=0;i<2;i++){
		for(int j=0;j<2;j++){
			ame_mvp[i][j][0] = 0;
			ame_mvp[i][j][1] = 0;
		}
	}

	int ame_cost = 0;
	//error related variable,get from VVCSoftware_BMS-Revision 1583 
	//InterSearch.cpp/interSearch::xAffineMotionEstimation function
	uint8_t pError[16][16];
	double pdDerivate[2][256];
	static const int iParaNum = 5;
	double **pdEqualCoeff;
	pdEqualCoeff = new double*[iParaNum];
	for(int i=0;i<iParaNum;i++){
		pdEqualCoeff[i] = new double[iParaNum];
	}
	//get ame mvp
	get_mvp();
	//iteration initialization
	ame_mvp[1][0][0] = mv16x16[0];
	ame_mvp[1][0][1] = mv16x16[1];
	ame_mvp[1][1][0] = mv16x16[0];
	ame_mvp[1][1][1] = mv16x16[1];
	/*
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {
			cout << "ame mvp[" << i << "][" << j << "]=(" << ame_mvp[i][j][0] << "," << ame_mvp[i][j][1] << ")";
		}
	}
	printf("\n");
	*/
	for(int i=0;i<2;i++){
		ame_cost = ame_count(ame_mvp[i]);
		if(ame_cost < ame_min_cost){
			ame_min_cost = ame_cost;
			amv_temp[0][0] = ame_mvp[i][0][0];
			amv_temp[0][1] = ame_mvp[i][0][1];
			amv_temp[1][0] = ame_mvp[i][1][0];
			amv_temp[1][1] = ame_mvp[i][1][1];
		}
		else
		{
			continue;
		}	
	}
	
	//do iteration
	//iteration part reffers to VVCSoftware_BMS-Revision 1583 
	//InterSearch.cpp/interSearch::xAffineMotionEstimation function
	for(int iter=0;iter<ame_step;iter++){
        ame_mcp(amv_temp);
		//get error matrix
		for(int i=0;i<16;i++){
			for(int j=0;j<16;j++){
				pError[i][j] = pred_luma[i][j]-cur_mb.luma[i][j];
			}
		}
		//use sobel count gradient
		//sx = 1/8*[(-1,0,1),(-2,0,2),(-1,0,1)]
		//sy = 1/8*[(-1,-2,-1),(0,0,0),(1,2,1)]
		//sobel x direction
		for(int i=1;i<16-1;i++){
			for(int j=1;j<16-1;j++){
				pdDerivate[0][i*16+j]   = ((double)(pred_luma[i][j+1]*2)-(double)(pred_luma[i][j-1]*2)
										 +(double)(pred_luma[i-1][j+1])-(double)(pred_luma[i-1][j-1])
										 +(double)(pred_luma[i+1][j+1])-(double)(pred_luma[i+1][j-1]))/8;
			}
			pdDerivate[0][i*16]         = pdDerivate[0][i*16+1];
			pdDerivate[0][i*16+16-1]    = pdDerivate[0][i*16+16-2];
		} 
		pdDerivate[0][0]                = pdDerivate[0][16+1];
		pdDerivate[0][16-1]             = pdDerivate[0][16+16-2];
		pdDerivate[0][(16-1)*16]        = pdDerivate[0][(16-2)*16+1];
		pdDerivate[0][(16-1)*16+16-1]   = pdDerivate[0][(16-2)*16+16-2];
		for(int j=1;j<16-1;j++){ 
			pdDerivate[0][j]            = pdDerivate[0][j+16];
			pdDerivate[0][(16-1)*16+j]  = pdDerivate[0][(16-2)*16+j];
		}
		//sobel y direction
		for(int i=1;i<16-1;i++){
			for(int j=1;j<16-1;j++){
				pdDerivate[1][i*16+j]   = ((double)(pred_luma[i+1][j]*2)-(double)(pred_luma[i-1][j]*2)
										 + (double)(pred_luma[i+1][j-1])- (double)(pred_luma[i-1][j-1])
										 + (double)(pred_luma[i+1][j+1])- (double)(pred_luma[i-1][j+1]))/8;
			}
			pdDerivate[1][i*16]         = pdDerivate[1][i*16+1];
			pdDerivate[1][i*16+16-1]    = pdDerivate[1][i*16+16-2];
		} 
		pdDerivate[1][0]                = pdDerivate[1][16+1];
		pdDerivate[1][16-1]             = pdDerivate[1][16+16-2];
		pdDerivate[1][(16-1)*16]        = pdDerivate[1][(16-2)*16+1];
		pdDerivate[1][(16-1)*16+16-1]   = pdDerivate[1][(16-2)*16+16-2];
		for(int j=1;j<16-1;j++){ 
			pdDerivate[1][j]            = pdDerivate[1][j+16];
			pdDerivate[1][(16-1)*16+j]  = pdDerivate[1][(16-2)*16+j];
		}
		//solve delta x and y
		for(int m=0;m!=iParaNum;m++){
			for(int n=0;n!=iParaNum;n++){
				pdEqualCoeff[m][n] = 0.0;
			}
		}
		double dc[4];
		for(int j=0;j<16;j++){
			for(int k=0;k<16;k++){
				int iIdx = j*16+k;
				dc[0] = (1-k/16.0)*pdDerivate[0][iIdx]-j/16.0*pdDerivate[1][iIdx];
				dc[1] = k/16.0*pdDerivate[0][iIdx]+j/16.0*pdDerivate[1][iIdx];
				dc[2] = j/16.0*pdDerivate[0][iIdx]+(1-k/16.0)*pdDerivate[1][iIdx];
				dc[3] = -j/16.0*pdDerivate[0][iIdx]+k/16.0*pdDerivate[1][iIdx];

				for(int col=0;col<4;col++){
					for(int row=0;row<4;row++){
						pdEqualCoeff[col][row] += dc[col]*dc[row];
					}
					pdEqualCoeff[col][4] += -dc[col]*pError[j][k];
				}
			}
		}
		double dAffinePara[4] = {0};
		//求逆矩阵
		solveEqual(pdEqualCoeff,dAffinePara);
		//convert to delta mv
		double dDeltaMV[4];
		dDeltaMV[0] = dAffinePara[0];
		dDeltaMV[1] = dAffinePara[1];
		dDeltaMV[2] = dAffinePara[2];
		dDeltaMV[3] = dAffinePara[3];
		
		int16_t acDeltaMv[2][2];
		//<< 1 to fit with the precision of fmv
		acDeltaMv[0][0] = (int16_t)(dDeltaMV[0]*4);
		acDeltaMv[0][1] = (int16_t)(dDeltaMV[2]*4);
		acDeltaMv[1][0] = (int16_t)(dDeltaMV[1]*4);
		acDeltaMv[1][1] = (int16_t)(dDeltaMV[3]*4); 
	
		bool bAllZero = false;
		for(int i=0;i<2;i++){
			if(acDeltaMv[i][0]!=0 || acDeltaMv[i][1]!=0){
				bAllZero=false;
				break;
			}
			bAllZero=true;
		}
		if(bAllZero)
			break;
		//do mc with updated mv and count cost
		for(int i=0;i<2;i++){
			amv_temp[i][0] += acDeltaMv[i][0];
			amv_temp[i][1] += acDeltaMv[i][1];
			
			if(amv_temp[i][0] < -4*sw.sr_w){
				amv_temp[i][0] = -4*sw.sr_w;  
			}
            else if (amv_temp[i][0] > 4 * sw.sr_w) {
                amv_temp[i][0] = 4*sw.sr_w;
            }

            if (amv_temp[i][1] < -4 * sw.sr_h) {
                amv_temp[i][1] = -4 * sw.sr_h;
            }
            else if (amv_temp[i][1] > 4 * sw.sr_h) {
                amv_temp[i][1] = 4 * sw.sr_h;
            }
		}
		int16_t cost_temp = 0;
		cost_temp = ame_count(amv_temp);
		//store best cost and mv
		if(cost_temp < ame_min_cost){
			ame_min_cost = cost_temp;
			memcpy(amv,amv_temp,sizeof(amv_temp));
		}
		else
		{
			continue;
		}
		
	}
	//free buffer
	for(int i=0;i<iParaNum;i++)
		delete []pdEqualCoeff[i];
	delete []pdEqualCoeff;
}
//------------------------------------------------------------------------//
//---------------------------ame_related_functions------------------------//
//------------------------------------------------------------------------// 
void inter::get_mvp(){
	int16_t s1[3][2],s2[2][2],s3[2];
	int16_t dmv[2];
	int32_t D = 0x7fffffff;
	int x,y,total_w;
	bool buffer_row = 0;
	x = cur_mb.x;
	y = cur_mb.y;
	total_w = param.frame_mb_x_total;
	buffer_row = (y%2);
	//get mvp list s1,s2,s3
	//s1 = {A,B,C}; s2 = {D,E}, s3 = {F}
	if(cur_mb.x==0){
		//top left
		if(cur_mb.y==0){
			memset(s1,0,sizeof(s1));
			memset(s2,0,sizeof(s2));
			memset(s3,0,sizeof(s3));
		}
		//left but not top
		else{
			 s1[0][0] = 0; s1[0][1] = 0;
			 s1[2][0] = 0; s1[2][1] = 0;
			 s3[0] = 0; s3[1] = 0;
			 s1[1][0] = me_buffer[(!buffer_row)*total_w+x].mv[3][0][0];
			 s1[1][1] = me_buffer[(!buffer_row)*total_w+x].mv[3][0][1];
			 s2[0][0] = me_buffer[(!buffer_row)*total_w+x].mv[3][3][0];
			 s2[0][1] = me_buffer[(!buffer_row)*total_w+x].mv[3][3][1];
			 s2[1][0] = me_buffer[(!buffer_row)*total_w+x+1].mv[3][0][0];
			 s2[1][1] = me_buffer[(!buffer_row)*total_w+x+1].mv[3][0][1];
		}
	}
	else if(cur_mb.x==total_w-1){
		//top right
		if(cur_mb.y==0){
			memset(s2,0,sizeof(s2));
			memset(s1,0,sizeof(s1));
			s1[2][0] = me_buffer[buffer_row*total_w+x-1].mv[0][3][0];
			s1[2][1] = me_buffer[buffer_row*total_w+x-1].mv[0][3][1];
			s3[0]    = me_buffer[buffer_row*total_w+x-1].mv[3][3][0];
			s3[1]    = me_buffer[buffer_row*total_w+x-1].mv[3][3][1];
		}
		//right but not top
		else{
			s1[0][0] = me_buffer[(!buffer_row)*total_w+x-1].mv[3][3][0];
			s1[0][1] = me_buffer[(!buffer_row)*total_w+x-1].mv[3][3][1];
			s1[1][0] = me_buffer[(!buffer_row)*total_w+x].mv[3][0][0];
			s1[1][1] = me_buffer[(!buffer_row)*total_w+x].mv[3][0][1];
			s1[2][0] = me_buffer[buffer_row*total_w+x-1].mv[0][3][0];
			s1[2][1] = me_buffer[buffer_row*total_w+x-1].mv[0][3][1];
			s2[0][0] = me_buffer[(!buffer_row)*total_w+x].mv[3][3][0];
			s2[0][1] = me_buffer[(!buffer_row)*total_w+x].mv[3][3][1];
			s2[1][0] = 0;
			s2[1][1] = 0;
			s3[0]    = me_buffer[buffer_row*total_w+x-1].mv[3][3][0];
			s3[1]    = me_buffer[buffer_row*total_w+x-1].mv[3][3][1];
		}
	}
	else{
		//top middle 
		if(cur_mb.y==0){
			memset(s1,0,sizeof(s1));
			memset(s2,0,sizeof(s2));
			s1[2][0] = me_buffer[buffer_row*total_w+x-1].mv[0][3][0];
			s1[2][1] = me_buffer[buffer_row*total_w+x-1].mv[0][3][1];
			s3[0]    = me_buffer[buffer_row*total_w+x-1].mv[3][3][0];
			s3[1]    = me_buffer[buffer_row*total_w+x-1].mv[3][3][1];
		}
		else{
			s1[0][0] = me_buffer[(!buffer_row)*total_w+x-1].mv[3][3][0];
			s1[0][1] = me_buffer[(!buffer_row)*total_w+x-1].mv[3][3][1];
			s1[1][0] = me_buffer[(!buffer_row)*total_w+x].mv[3][0][0];
			s1[1][1] = me_buffer[(!buffer_row)*total_w+x].mv[3][0][1];
			s1[2][0] = me_buffer[buffer_row*total_w+x-1].mv[0][3][0];
			s1[2][1] = me_buffer[buffer_row*total_w+x-1].mv[0][3][1];
			s2[0][0] = me_buffer[(!buffer_row)*total_w+x].mv[3][3][0];
			s2[0][1] = me_buffer[(!buffer_row)*total_w+x].mv[3][3][1];
			s2[1][0] = me_buffer[(!buffer_row)*total_w+x+1].mv[3][0][0];
			s2[1][1] = me_buffer[(!buffer_row)*total_w+x+1].mv[3][0][1];
			s3[0]    = me_buffer[buffer_row*total_w+x-1].mv[3][3][0];
			s3[1]    = me_buffer[buffer_row*total_w+x-1].mv[3][3][1];
		}
	}
	//find the best result for all {s1,s2,s3}
	for(int i=0;i<3;i++){
		for(int j=0;j<2;j++){
			dmv[0] = -(s2[j][1]-s1[i][1])/16*16+s1[i][0];
			dmv[1] = (s2[j][0]-s1[i][0])/16*16+s1[i][1];
			if((abs(dmv[0])+abs(dmv[1])) < D){
				D = abs(dmv[0]+abs(dmv[1]));
				ame_mvp[0][0][0] = s1[i][0];
				ame_mvp[0][0][1] = s1[i][1];
				ame_mvp[0][1][0] = s2[j][0];
				ame_mvp[0][1][1] = s2[j][1]; 
			}
			else
			{
				continue;
			}
		}
	}
}

int inter::ame_count(int16_t mv[2][2]){
	int16_t ame_mv[2];
	int16_t ame_imv[2];
	int16_t ref_pos[2];//position pointed to sw
	uint8_t *p_block, *p_sw;
	int interplot_index = 0;
	int32_t ame_cost = 0;
	for(int i=0;i<16;i++){
		for(int j=0;j<16;j++){
			ame_mv[0] = (int16_t)((mv[1][0]-mv[0][0])/64.0*j-(mv[1][1]-mv[0][1])/64.0*i+mv[0][0]/4.0)<<2;
			ame_mv[1] = (int16_t)((mv[1][1]-mv[0][1])/64.0*j-(mv[1][0]-mv[0][0])/64.0*i+mv[0][1]/4.0)<<2;
			//interplot_index
			// 0 1 2
			// 3 4 5
			// 6 7 8 where 4 represents the org integer pixel
			if(ame_mv[0]%4==0 && ame_mv[1]%4==0){
				ame_imv[0] = ame_mv[0];
				ame_imv[1] = ame_mv[1];
				interplot_index = 4;
			}
			else if(ame_mv[0]%4==0 && ame_mv[1]%4!=0){
				ame_imv[0] = ame_mv[0];
				ame_imv[1] = ame_mv[1] + 2;
				interplot_index = 7;
			}
			else if(ame_mv[0]%4!=0 && ame_mv[1]%4==0){
				ame_imv[0] = ame_mv[0] + 2;
				ame_imv[1] = ame_mv[1];
				interplot_index = 3;
			}
			else{
				ame_imv[0] = ame_mv[0] + 2;
				ame_imv[1] = ame_mv[1] + 2;
				interplot_index = 6;
			}
			ref_pos[0] = j+(ame_imv[0]>>2)+sw_center[1];
			ref_pos[1] = i+(ame_imv[1]>>2)+sw_center[0];
			interpolate_h(ref_pos[0],ref_pos[1],1,1);
			for (int i = 0; i < 2; i++) {
				if (mv[i][0] > 10000 || mv[i][0] < -10000 || mv[i][1]>10000 || mv[i][1] < -10000)
					printf("%x\n", &ref_mb[j + (ame_imv[0] >> 2) + sw_center[1]][i + (ame_imv[1] >> 2) + sw_center[0]]);
			}
			ame_cost += subpel_me(j,i,ame_mv,interplot_index);
		}
	}
	return ame_cost;
}

//amv精度与fmv相同，重载subpel_me
int32_t inter::subpel_me(int pos_x,int pos_y,int16_t mv[2],int index){
	int32_t sad = 0;
    int32_t cost = 0;
	//use sad instead of satd because 1 pixel precision
	sad = abs(cur_mb.luma[pos_y][pos_x] - ref_mb[index][0][0]);
	cost = sad + lambda_tab[param.qp] * (bs_size_se(abs(mv[0]))+bs_size_se(abs(mv[1])));
	return cost;
} 

void inter::ame_mcp(int16_t mv[2][2]){
	int16_t ame_mv[2];
	int16_t ame_imv[2];
	int16_t ref_pos[2];
	int interplot_index = 0;
	for(int i=0;i<16;i++){
		for(int j=0;j<16;j++){
			ame_mv[0] = (mv[1][0]-mv[0][0])/16*j-(mv[1][1]-mv[0][1])/16*i+mv[0][0];
			ame_mv[1] = (mv[1][1]-mv[0][1])/16*j-(mv[1][0]-mv[0][0])/16*i+mv[0][1];
			//从amv判断最邻近的imv以进行插值计算satd
			if(ame_mv[0]%4==0 && ame_mv[1]%4==0){
				ame_imv[0] = ame_mv[0];
				ame_imv[1] = ame_mv[1];
				interplot_index = 4;
			}
			else if(ame_mv[0]%4==0 && ame_mv[1]%4!=0){
				ame_imv[0] = ame_mv[0];
				ame_imv[1] = ame_mv[1] + 2;
				interplot_index = 7;
			}
			else if(ame_mv[0]%4!=0 && ame_mv[1]%4==0){
				ame_imv[0] = ame_mv[0] + 2;
				ame_imv[1] = ame_mv[1];
				interplot_index = 3;
			}
			else{
				ame_imv[0] = ame_mv[0] + 2;
				ame_imv[1] = ame_mv[1] + 2;
				interplot_index = 6;
			}
			ref_pos[0] = j+(ame_imv[0]>>2)+sw_center[1];
			ref_pos[1] = i+(ame_imv[1]>>2)+sw_center[0];
			interpolate_h(ref_pos[0],ref_pos[1],1,1);
			pixel_copy_wxh(1,1,&pred_luma[i][j],1,&ref_mb[interplot_index][0][0],1);
		}
	}
}

//solve linear equation Ax=b
void inter::solveEqual(double** dEqualCoeff, double* dAffinePara){
	double b[4][4];
	if(Gauss(dEqualCoeff,b)){
		for(int i=0;i<4;i++){
			for(int j=0;j<4;j++){
				dAffinePara[i] += b[i][j]*dEqualCoeff[j][4];
			}
		}
	}
}
//use Gauss to solve linear equation Ax=b
bool inter::Gauss(double **A, double B[][4]){
	int i,j,k;
	float max,temp;
	float t[4][4];
	for(i=0;i<4;i++){
		for(j=0;j<4;j++){
			t[i][j] = A[i][j];
		}
	}
	for(i=0;i<4;i++){
		for(j=0;j<4;j++){
			B[i][j] = (i==j)?(double)1:0;
		}
	}
	for(i=0;i<4;i++){
		max = t[i][i];
		k=i;
		for(j=i+1;j<4;j++){
			if(fabs(t[j][i])>fabs(max)){
				max = t[j][i];
				k = j;
			}
		}
		if(k!=i){
			for(j=0;j<4;j++){
				temp = t[i][j];
				t[i][j] = t[k][j];
				t[k][j] = temp;
				temp = B[i][j];
				B[i][j] = B[k][j];
				B[k][j] = temp;
			}
		}
		if(t[i][i] == 0)
		{
			printf("There is no inverse matrix!\n");
			return false;
		}
		temp = t[i][i];
		for(j=0;j<4;j++){
			t[i][j] = t[i][j]/temp;
			B[i][j] = B[i][j]/temp;
		}
		for(j=0;j<4;j++){
			if(j!=i){
				temp = t[j][i];
				for(k=0;k<4;k++){
					t[j][k] = t[j][k] - t[i][k]*temp;
					B[j][k] = B[j][k] - B[i][k]*temp;
				}
			}
		}
	}
	return true;
}

//模式判决函数重载，判断是否采纳AME
void inter::mode_decision_sort(){
    printf("amv = (%d, %d) & (%d, %d) \n", amv[0][0], amv[0][1], amv[1][0], amv[1][1]);
	if(ame_min_cost < mb_info.me_cost){
		mb_info.ame_flag = 1;
		mb_info.me_cost = ame_min_cost;
		mb_info.mb_partition = D_16x16;
		mb_info.mb_subpartition[0] = D_L0_8x8; 
		mb_info.mb_subpartition[1] = D_L0_8x8;
		mb_info.mb_subpartition[2] = D_L0_8x8; 
		mb_info.mb_subpartition[3] = D_L0_8x8;
		memcpy(mb_info.cp_amv,amv,sizeof(amv));
		me_buffer[(cur_mb.y%2)*param.frame_mb_x_total+cur_mb.x] = mb_info;

        printf("ame_flag = 1\n");
        ame_mcp(mb_info.cp_amv);
        pixel_copy_wxh(16, 16, &min_mb[0][0], 16, &pred_luma[0][0], 16);
        ame_flg_mat[cur_mb.y][cur_mb.x] = 1;
	}
	else
		return;
}


void inter::fme(){
	int16_t dmv[2];
	int		min_index;
	int16_t ref_pos[2]; // position pointed to sw

	cost = 0;
	memset(ref_mb, 0, sizeof(ref_mb));
	memset(h_half_pel, 0, sizeof(h_half_pel));
	memset(v_half_pel, 0, sizeof(v_half_pel));
	memset(d_half_pel, 0, sizeof(d_half_pel));
	memset(int_pel, 0, sizeof(int_pel));

	switch (mb_partition){
	case D_16x16:
		// half me
		ref_pos[0] = (mv16x16[0] >> 2) + sw_center[1]; // position pointed to sw
		ref_pos[1] = (mv16x16[1] >> 2) + sw_center[0];
		interpolate_h(ref_pos[0], ref_pos[1], 16, 16);
		cost = subpel_me(0, 0, 16, 16, mv16x16, dmv, 1);
		fmv16x16[0] = mv16x16[0] + dmv[0];
		fmv16x16[1] = mv16x16[1] + dmv[1];
		// copy pixel(dmv0,dmv1)
		//				dmv1
		//		(-1,-1) (-1,0) (-1,1)
		//dmv0	( 0,-1) ( 0,0) ( 0,1)
		//		( 1,-1) ( 1,0) ( 1,1)
		//ref_mb	
		//		0		1		2
		//		3		4		5
		//		6		7		8
		min_index = (dmv[1] + 1) * 3 + dmv[0] + 1;
		pixel_copy_wxh(16, 16, &min_mb[0][0], 16, &ref_mb[min_index][0][0], 16);

		break;
	case D_16x8:
		for (int blk = 0; blk<2; blk++){
			// half me	
			ref_pos[0] = (mv16x8[blk][0] >> 2) + sw_center[1]; // position pointed to sw
			ref_pos[1] = 8 * blk + (mv16x8[blk][1] >> 2) + sw_center[0];
			interpolate_h(ref_pos[0], ref_pos[1], 16, 8);
			cost = subpel_me(0, 8 * blk, 16, 8, mv16x8[blk], dmv, 1);
			fmv16x8[blk][0] = mv16x8[blk][0] + dmv[0];
			fmv16x8[blk][1] = mv16x8[blk][1] + dmv[1];

			// copy pixel
			min_index = (dmv[1] + 1) * 3 + dmv[0] + 1;
			pixel_copy_wxh(8, 16, &min_mb[8 * blk][0], 16, &ref_mb[min_index][0][0], 16);
		}
		break;
	case D_8x16:
		for (int blk = 0; blk<2; blk++){
			// half me
			ref_pos[0] = 8 * blk + (mv8x16[blk][0] >> 2) + sw_center[1]; // position pointed to sw
			ref_pos[1] = (mv8x16[blk][1] >> 2) + sw_center[0];
			interpolate_h(ref_pos[0], ref_pos[1], 8, 16);
			cost = subpel_me(8 * blk, 0, 8, 16, mv8x16[blk], dmv, 1);
			fmv8x16[blk][0] = mv8x16[blk][0] + dmv[0];
			fmv8x16[blk][1] = mv8x16[blk][1] + dmv[1];

			// copy pixel
			min_index = (dmv[1] + 1) * 3 + dmv[0] + 1;
			pixel_copy_wxh(16, 8, &min_mb[0][8 * blk], 16, &ref_mb[min_index][0][0], 16);
		}
		break;
	case D_8x8:
		for (int blk = 0; blk<4; blk++){
			switch (mb_subpartition[blk]){
			case D_L0_8x8:
				// half me
				ref_pos[0] = 8 * (blk % 2) + (mv8x8[blk][0] >> 2) + sw_center[1]; // position pointed to sw
				ref_pos[1] = 8 * (blk >> 1) + (mv8x8[blk][0] >> 2) + sw_center[0];
				interpolate_h(ref_pos[0], ref_pos[1], 8, 8);
				cost = subpel_me(8 * (blk % 2), 8 * (blk >> 1), 8, 8, mv8x8[blk], dmv, 1);
				fmv8x8[blk][0] = mv8x8[blk][0] + dmv[0];
				fmv8x8[blk][1] = mv8x8[blk][1] + dmv[1];

				// copy pixel
				min_index = (dmv[1] + 1) * 3 + dmv[0] + 1;
				pixel_copy_wxh(8, 8, &min_mb[8 * (blk >> 1)][8 * (blk % 2)], 16, &ref_mb[min_index][0][0], 16);
				break;
			case D_L0_8x4:
				for (int sblk = 0; sblk<2; sblk++){
					// half me
					ref_pos[0] = 8 * (blk % 2) + (mv8x4[blk][sblk][0] >> 2) + sw_center[1]; // position pointed to sw
					ref_pos[1] = 8 * (blk >> 1) + 4 * sblk + (mv8x4[blk][sblk][1] >> 2) + sw_center[0];
					interpolate_h(ref_pos[0], ref_pos[1], 8, 4);
					cost = subpel_me(8 * (blk % 2), 8 * (blk >> 1) + 4 * sblk, 8, 4, mv8x4[blk][sblk], dmv, 1);
					fmv8x4[blk][sblk][0] = mv8x4[blk][sblk][0] + dmv[0];
					fmv8x4[blk][sblk][1] = mv8x4[blk][sblk][1] + dmv[1];

					// copy pixel
					min_index = (dmv[1] + 1) * 3 + dmv[0] + 1;
					pixel_copy_wxh(4, 8, &min_mb[8 * (blk >> 1) + 4 * sblk][8 * (blk % 2)], 16, &ref_mb[min_index][0][0], 16);
				}
				break;
			case D_L0_4x8:
				for (int sblk = 0; sblk<2; sblk++){
					// half me	
					ref_pos[0] = 8 * (blk % 2) + 4 * sblk + (mv4x8[blk][sblk][0] >> 2) + sw_center[1]; // position pointed to sw
					ref_pos[1] = 8 * (blk >> 1) + (mv4x8[blk][sblk][1] >> 2) + sw_center[0];
					interpolate_h(ref_pos[0], ref_pos[1], 4, 8);
					cost = subpel_me(8 * (blk % 2) + 4 * sblk, 8 * (blk >> 1), 4, 8, mv4x8[blk][sblk], dmv, 1);
					fmv4x8[blk][sblk][0] = mv4x8[blk][sblk][0] + dmv[0];
					fmv4x8[blk][sblk][1] = mv4x8[blk][sblk][1] + dmv[1];

					// copy pixel
					min_index = (dmv[1] + 1) * 3 + dmv[0] + 1;
					pixel_copy_wxh(8, 4, &min_mb[8 * (blk >> 1)][8 * (blk % 2) + 4 * sblk], 16, &ref_mb[min_index][0][0], 16);
				}
				break;
			case D_L0_4x4:
				for (int sblk = 0; sblk<4; sblk++){
					// half me	
					ref_pos[0] = 8 * (blk % 2) + 4 * (sblk % 2) + (mv4x4[blk][sblk][0] >> 2) + sw_center[1]; // position pointed to sw
					ref_pos[1] = 8 * (blk >> 1) + 4 * (sblk >> 1) + (mv4x4[blk][sblk][1] >> 2) + sw_center[0];
					interpolate_h(ref_pos[0], ref_pos[1], 4, 4);
					cost = subpel_me(8 * (blk % 2) + 4 * (sblk % 2), 8 * (blk >> 1) + 4 * (sblk >> 1), 4, 4, mv4x4[blk][sblk], dmv, 1);
					fmv4x4[blk][sblk][0] = mv4x4[blk][sblk][0] + dmv[0];
					fmv4x4[blk][sblk][1] = mv4x4[blk][sblk][1] + dmv[1];

					// copy pixel
					min_index = (dmv[1] + 1) * 3 + dmv[0] + 1;
					pixel_copy_wxh(4, 4, &min_mb[8 * (blk >> 1) + 4 * (sblk >> 1)][8 * (blk % 2) + 4 * (sblk % 2)], 16, &ref_mb[min_index][0][0], 16);
				}
				break;
			}
		}
		break;
	}
	switch (mb_partition){
	case D_16x16:
		for (int i = 0; i<4; i++)
		for (int j = 0; j<4; j++)
		for (int k = 0; k<2; k++)
			fmv[i][j][k] = mv16x16[k];
		break;
	case D_16x8:
		for (int i = 0; i<4; i++)
		for (int j = 0; j<4; j++)
		for (int k = 0; k<2; k++)
			fmv[i][j][k] = mv16x8[i / 2][k];
		break;
	case D_8x16:
		for (int i = 0; i<4; i++)
		for (int j = 0; j<4; j++)
		for (int k = 0; k<2; k++)
			fmv[i][j][k] = mv8x16[i % 2][k];
		break;
	case D_8x8:
		for (int i = 0; i<4; i++){
			switch (mb_subpartition[i]){
			case D_L0_8x8:
				for (int j = 0; j<4; j++)
				for (int k = 0; k<2; k++)
					fmv[i][j][k] = mv8x8[i][k];
				break;
			case D_L0_8x4:
				for (int j = 0; j<4; j++)
				for (int k = 0; k<2; k++)
					fmv[i][j][k] = mv8x4[i][j / 2][k];
				break;
			case D_L0_4x8:
				for (int j = 0; j<4; j++)
				for (int k = 0; k<2; k++)
					fmv[i][j][k] = mv4x8[i][j % 2][k];
				break;
			case D_L0_4x4:
				for (int j = 0; j<4; j++)
				for (int k = 0; k<2; k++)
					fmv[i][j][k] = mv4x4[i][j][k];
				break;
			default:
				printf("ERROR: MB(x:%d, y:%d) sub(%d) partition is wrong!\n", cur_mb.x, cur_mb.y, i);
			}
		}
		break;
	default:
		printf("ERROR: MB(x:%d, y:%d) partition is wrong!\n", cur_mb.x, cur_mb.y);
	}
	//dump fme information into current mb inter buffer
	//add current mb inter buffer into me inter buffer
	
	memcpy(mb_info.mv,fmv,sizeof(fmv));
	mb_info.me_cost = cost;
	me_buffer[(cur_mb.y%2)*param.frame_mb_x_total+cur_mb.x] = mb_info;
}

void inter::mc(){
	loadmc();
	get_chroma();
	mvp_compute();
	transform();
	cbp_encode();
	update();
}


inter_t & inter::write(){
	inter_output.name = "mc";
	inter_output.x = cur_mb.x;
	inter_output.y = cur_mb.y;
	inter_output.qp = cqm.qp;
	inter_output.mb_type = mb_type;
	inter_output.mb_partition = mb_partition;
	memcpy(inter_output.mb_subpartition, mb_subpartition, sizeof(inter_output.mb_subpartition));
	memcpy(inter_output.mv, fmv, sizeof(inter_output.mv));
	memcpy(inter_output.mvp, mvp, sizeof(inter_output.mv));
	memcpy(inter_output.luma, luma_dct4x4, sizeof(inter_output.luma));
	memcpy(inter_output.chroma, chroma_dct4x4, sizeof(inter_output.chroma));
	memcpy(inter_output.chroma_dc, chroma_dct4x4_dc, sizeof(inter_output.chroma_dc));
	memcpy(inter_output.luma_rec, luma_rec, sizeof(inter_output.luma_rec));
	memcpy(inter_output.chroma_rec, chroma_rec, sizeof(inter_output.chroma_rec));
	memcpy(inter_output.non_zero_count, non_zero_count, sizeof(inter_output.non_zero_count));
	inter_output.cbp = cbp;


	return inter_output;
}

void inter::dump(){
    dump_yuv();

	fp = fopen("inter_out.txt", "a");
	fprintf(fp, "======== MB x:%3d y:%3d ========\n", cur_mb.x, cur_mb.y);
	fprintf(fp, "# mb_type			: %d\n", mb_type);
	fprintf(fp, "# mb_partition		: %d\n", mb_partition);
	fprintf(fp, "# mb_subpartition	: %d %d %d %d\n", mb_subpartition[0],
		mb_subpartition[1],
		mb_subpartition[2],
		mb_subpartition[3]);
	fprintf(fp, "# cur_mb:\n");
	for (int i = 0; i<16; i++){
		for (int j = 0; j < 16; j++){
			fprintf(fp, "%2x ", int(cur_mb.luma[i][j]));
		}
		
		fprintf(fp, "\n");
	}

	fprintf(fp, "# search window:\n");
	for (int i = 0; i<SW_H; i++){
		for (int j = 0; j<SW_W; j++) {
			fprintf(fp, "%2x ", int(sw.sw_luma[i][j]));
			if ((j + 1) % 16 == 0 && j != (SW_W - 1)) fprintf(fp, "| ");
		}
		fprintf(fp, "\n");
		if ((i + 1) % 16 == 0 && i != (SW_H - 1)) {
			for (int n = 0; n<(SW_W * 3 + SW_W / 16); n++) fprintf(fp, "-");
			fprintf(fp, "\n");
		}
	}
	fprintf(fp, "# min_mb:\n");
	for (int i = 0; i<16; i++){
		for (int j = 0; j < 16; j++){
			fprintf(fp, "%2x ", int(min_mb[i][j]));
		}

		fprintf(fp, "\n");
	}

	fclose(fp);
	fp = fopen("mc_out.txt", "a");

	fprintf(fp, "======== MB x:%3d y:%3d ========\n",  cur_mb.x, cur_mb.y);


	fprintf(fp, "# residual(luma):\n");
	for (int i = 0; i<16; i++){
		for (int j = 0; j<16; j++)
			fprintf(fp, "%3d ", int(luma_dct4x4[i][j]));
		fprintf(fp, "\n");
	}

	fprintf(fp, "# residual(cb):\n");
	for (int i = 0; i<4; i++){
		for (int j = 0; j<16; j++)
			fprintf(fp, "%3d ", int(chroma_dct4x4[0][i][j]));
		fprintf(fp, "\n");
	}

	fprintf(fp, "# residual(cr):\n");
	for (int i = 0; i<4; i++){
		for (int j = 0; j<16; j++)
			fprintf(fp, "%3d ", int(chroma_dct4x4[1][i][j]));
		fprintf(fp, "\n");
	}

	fprintf(fp, "# reconstruct(luma):\n");
	for (int i = 0; i<16; i++){
		for (int j = 0; j<16; j++)
			fprintf(fp, "%2x ", int(luma_rec[i][j]));
		fprintf(fp, "\n");
	}

	fprintf(fp, "# reconstruct(cb):\n");
	for (int i = 0; i<8; i++){
		for (int j = 0; j<8; j++)
			fprintf(fp, "%2x ", int(chroma_rec[0][i][j]));
		fprintf(fp, "\n");
	}

	fprintf(fp, "# reconstruct(cr):\n");
	for (int i = 0; i<8; i++){
		for (int j = 0; j<8; j++)
			fprintf(fp, "%2x ", int(chroma_rec[1][i][j]));
		fprintf(fp, "\n");
	}
	fclose(fp);

	//dump tq.dat and tq_input.dat
	fp = fopen("tq_out.txt", "a");
	fprintf(fp, "===== MB x: %3d  y: %3d =====\n", cur_mb.x, cur_mb.y);

	//y
	fprintf(fp, "$y_pre\n");
	for (int i = 0; i<16; i++) {
		fprintf(fp, "%2d:", i);
		for (int j = 0; j<16; j++) {
			fprintf(fp, " %02x", min_mb[((zscan_to_raster_4x4[i] / 4) * 4) + j / 4][((zscan_to_raster_4x4[i] % 4) * 4) + j % 4]);
		}
		fprintf(fp, "\n");
	}

	fprintf(fp, "$y_res\n");
	for (int i = 0; i<16; i++) {
		fprintf(fp, "%2d:", i);
		for (int j = 0; j<16; j++) {
			fprintf(fp, " %03x", ((cur_mb.luma[((zscan_to_raster_4x4[i] / 4) * 4) + j / 4][((zscan_to_raster_4x4[i] % 4) * 4) + j % 4] - min_mb[((zscan_to_raster_4x4[i] / 4) * 4) + j / 4][((zscan_to_raster_4x4[i] % 4) * 4) + j % 4]) & 0x1ff));
		}
		fprintf(fp, "\n");
	}

	fprintf(fp, "$y_rec\n");
	for (int i = 0; i<16; i++) {
		fprintf(fp, "%2d:", i);
		for (int j = 0; j<16; j++) {
			fprintf(fp, " %02x", (luma_rec[((zscan_to_raster_4x4[i] / 4) * 4) + j / 4][((zscan_to_raster_4x4[i] % 4) * 4) + j % 4]));
		}
		fprintf(fp, "\n");
	}

	//u
	fprintf(fp, "$u_pre\n");
	for (int i = 0; i<4; i++) {
		fprintf(fp, "%2d:", i);
		for (int j = 0; j<16; j++) {
			fprintf(fp, " %02x", min_cb[((zscan_to_raster_2x2[i] / 2) * 4) + j / 4][((zscan_to_raster_2x2[i] % 2) * 4) + j % 4]);
		}
		fprintf(fp, "\n");
	}

	fprintf(fp, "$u_res\n");
	for (int i = 0; i<4; i++) {
		fprintf(fp, "%2d:", i);
		for (int j = 0; j<16; j++) {
			fprintf(fp, " %03x", ((cur_mb.cb[((zscan_to_raster_2x2[i] / 2) * 4) + j / 4][((zscan_to_raster_2x2[i] % 2) * 4) + j % 4] - min_cb[((zscan_to_raster_2x2[i] / 2) * 4) + j / 4][((zscan_to_raster_2x2[i] % 2) * 4) + j % 4]) & 0x1ff));
		}
		fprintf(fp, "\n");
	}

	fprintf(fp, "$u_rec\n");
	for (int i = 0; i<4; i++) {
		fprintf(fp, "%2d:", i);
		for (int j = 0; j<16; j++) {
			fprintf(fp, " %02x", (chroma_rec[0][((zscan_to_raster_2x2[i] / 2) * 4) + j / 4][((zscan_to_raster_2x2[i] % 2) * 4) + j % 4]));
		}
		fprintf(fp, "\n");
	}

	//v
	fprintf(fp, "$v_pre\n");
	for (int i = 0; i<4; i++) {
		fprintf(fp, "%2d:", i);
		for (int j = 0; j<16; j++) {
			fprintf(fp, " %02x", min_cr[((zscan_to_raster_2x2[i] / 2) * 4) + j / 4][((zscan_to_raster_2x2[i] % 2) * 4) + j % 4]);
		}
		fprintf(fp, "\n");
	}

	fprintf(fp, "$v_res\n");
	for (int i = 0; i<4; i++) {
		fprintf(fp, "%2d:", i);
		for (int j = 0; j<16; j++) {
			fprintf(fp, " %03x", ((cur_mb.cr[((zscan_to_raster_2x2[i] / 2) * 4) + j / 4][((zscan_to_raster_2x2[i] % 2) * 4) + j % 4] - min_cr[((zscan_to_raster_2x2[i] / 2) * 4) + j / 4][((zscan_to_raster_2x2[i] % 2) * 4) + j % 4]) & 0x1ff));
		}
		fprintf(fp, "\n");
	}

	fprintf(fp, "$v_rec\n");
	for (int i = 0; i<4; i++) {
		fprintf(fp, "%2d:", i);
		for (int j = 0; j<16; j++) {
			fprintf(fp, " %02x", (chroma_rec[1][((zscan_to_raster_2x2[i] / 2) * 4) + j / 4][((zscan_to_raster_2x2[i] % 2) * 4) + j % 4]));
		}
		fprintf(fp, "\n");
	}
	fclose(fp);

	fp = fopen("tq_input.txt", "a");
	fprintf(fp, "%02x\n", cur_mb.x);
	fprintf(fp, "%02x\n", cur_mb.y);
	fprintf(fp, "%02x\n", cqm.qp);

	//y
	for (int i = 0; i<16; i++)
	for (int j = 0; j<16; j++)
		fprintf(fp, "%02x\n", min_mb[((zscan_to_raster_4x4[i] / 4) * 4) + j / 4][((zscan_to_raster_4x4[i] % 4) * 4) + j % 4]);
	for (int i = 0; i<16; i++)
	for (int j = 0; j<16; j++)
		fprintf(fp, "%03x\n", ((cur_mb.luma[((zscan_to_raster_4x4[i] / 4) * 4) + j / 4][((zscan_to_raster_4x4[i] % 4) * 4) + j % 4] - min_mb[((zscan_to_raster_4x4[i] / 4) * 4) + j / 4][((zscan_to_raster_4x4[i] % 4) * 4) + j % 4]) & 0x1ff));
	//u
	for (int i = 0; i<4; i++)
	for (int j = 0; j<16; j++)
		fprintf(fp, "%02x\n", min_cb[((zscan_to_raster_2x2[i] / 2) * 4) + j / 4][((zscan_to_raster_2x2[i] % 2) * 4) + j % 4]);
	for (int i = 0; i<4; i++)
	for (int j = 0; j<16; j++)
		fprintf(fp, "%03x\n", ((cur_mb.cb[((zscan_to_raster_2x2[i] / 2) * 4) + j / 4][((zscan_to_raster_2x2[i] % 2) * 4) + j % 4] - min_cb[((zscan_to_raster_2x2[i] / 2) * 4) + j / 4][((zscan_to_raster_2x2[i] % 2) * 4) + j % 4]) & 0x1ff));

	//v
	for (int i = 0; i<4; i++)
	for (int j = 0; j<16; j++)
		fprintf(fp, "%02x\n", min_cr[((zscan_to_raster_2x2[i] / 2) * 4) + j / 4][((zscan_to_raster_2x2[i] % 2) * 4) + j % 4]);
	for (int i = 0; i<4; i++)
	for (int j = 0; j<16; j++)
		fprintf(fp, "%03x\n", ((cur_mb.cr[((zscan_to_raster_2x2[i] / 2) * 4) + j / 4][((zscan_to_raster_2x2[i] % 2) * 4) + j % 4] - min_cr[((zscan_to_raster_2x2[i] / 2) * 4) + j / 4][((zscan_to_raster_2x2[i] % 2) * 4) + j % 4]) & 0x1ff));

	fclose(fp);

}







/****************************************************************************
* ime function
****************************************************************************/
int inter::ime_bs_size_se(int val)
{
	int tmp;
	tmp = (val < 0) ? (-val * 2) : (2 * val - 1);
	if (tmp < 255)
		return f264_ue_size_tab[(tmp + 1)];
	else
		return f264_ue_size_tab[(tmp + 1) >> 8] + 16;
}

//calculate the SAD of  block , whose size is width x height
//pix1 is current mb, pix2 is  search window in reference pic
int inter::pixel_sad_wxh(uint8_t *pix1, int i_stride_pix1,
	uint8_t *pix2, int i_stride_pix2,
	int w, int h)
{
	int i_sum = 0;
	int x, y;
	for (y = 0; y < h; y++)
	{
		for (x = 0; x < w; x++)
		{
			i_sum += abs(*(pix1 + y*i_stride_pix1 + x) - *(pix2 + y*i_stride_pix2 + x));
		}
	}
	return i_sum;
}





/****************************************************************************
*                             fme function
****************************************************************************/
void inter::interpolate_h(int pos_x, int pos_y, int len_x, int len_y)
{
	uint8_t	 int_tmp[f_LCU_SIZE + 6][f_LCU_SIZE + 1];
	int16_t  h_buf[f_LCU_SIZE + 6][f_LCU_SIZE + 1];
	// integer pel 
	for (int row = 0; row < (len_y + 6); row++) {
		for (int col = 0; col < (len_x + 1); col++) {
			int_tmp[row][col] = sw.sw_luma[pos_y + row - 3][pos_x + col];
		}
	}
	// clip integer 
	for (int row = 0; row < len_y; row++) {
		for (int col = 0; col < len_x; col++) {
			int offset = 0;
			int_pel[row][col] = Cliply(int_tmp[row + 3][col]);
		}
	}
	// horizental interpolate
	for (int row = 0; row<(len_y + 6); row++)
	for (int col = 0; col<(len_x + 1); col++){
		H_6TAPFIR(h_buf[row][col],
			sw.sw_luma[pos_y - 3 + row][pos_x - 3 + col], sw.sw_luma[pos_y - 3 + row][pos_x - 2 + col], sw.sw_luma[pos_y - 3 + row][pos_x - 1 + col],
			sw.sw_luma[pos_y - 3 + row][pos_x + 0 + col], sw.sw_luma[pos_y - 3 + row][pos_x + 1 + col], sw.sw_luma[pos_y - 3 + row][pos_x + 2 + col]);
	}
	// clip horizental 
	for (int row = 0; row< len_y; row++)
	for (int col = 0; col<(len_x + 1); col++){
		h_half_pel[row][col] = Cliply((h_buf[row + 3][col] + 16) >> 5);
	}

	// vertical interpolate
	for (int row = 0; row<len_y + 1; row++)
	for (int col = 0; col<len_x; col++){
		V_6TAPFIR(v_half_pel[row][col],
			int_tmp[pos_y + 0 + row][pos_x + col - 1], int_tmp[pos_y + 1 + row][pos_x + col - 1], int_tmp[pos_y + 2 + row][pos_x + col - 1],
			int_tmp[pos_y + 3 + row][pos_x + col - 1], int_tmp[pos_y + 4 + row][pos_x + col - 1], int_tmp[pos_y + 5 + row][pos_x + col - 1]);
	}
	// diagonal interpolate
	for (int row = 0; row<(len_y + 1); row++)
	for (int col = 0; col<(len_x + 1); col++){
		D_6TAPFIR(d_half_pel[row][col],
			h_buf[row + 0][col], h_buf[row + 1][col], h_buf[row + 2][col],
			h_buf[row + 3][col], h_buf[row + 4][col], h_buf[row + 5][col]);
	}
	// mem copy
	//  -------------   ----------------------------
	//  | 0 | 1 | 2 |   | d(0,0) | v(0,0) | d(0,1) |
	//  -------------   ----------------------------
	//  | 3 | 4 | 5 |   | h(0,0) | p(0,0) | h(0,1) |
	//  -------------   ----------------------------
	//  | 6 | 7 | 8 |   | d(1,0) | v(1,0) | d(1,,) |
	//  -------------   ----------------------------

	for (int row = 0; row < len_y; row++) {
		memcpy(&ref_mb[0][row][0], &d_half_pel[row + 0][0], len_x * sizeof(uint8_t));
		memcpy(&ref_mb[1][row][0], &v_half_pel[row + 0][0], len_x * sizeof(uint8_t));
		memcpy(&ref_mb[2][row][0], &d_half_pel[row + 0][1], len_x * sizeof(uint8_t));
		memcpy(&ref_mb[3][row][0], &h_half_pel[row + 0][0], len_x * sizeof(uint8_t));
		memcpy(&ref_mb[4][row][0], &int_pel[row][0], len_x * sizeof(uint8_t));
		memcpy(&ref_mb[5][row][0], &h_half_pel[row + 0][1], len_x * sizeof(uint8_t));
		memcpy(&ref_mb[6][row][0], &d_half_pel[row + 1][0], len_x * sizeof(uint8_t));
		memcpy(&ref_mb[7][row][0], &v_half_pel[row + 1][0], len_x * sizeof(uint8_t));
		memcpy(&ref_mb[8][row][0], &d_half_pel[row + 1][1], len_x * sizeof(uint8_t));
	}

	//	dump_cache();
}

int32_t inter::subpel_me(int pos_x, int pos_y, int len_x, int len_y, int16_t mv[2], int16_t dmv[2], bool b_half)
{
	int32_t satd[9];
	// int32_t satd[9] = { 0,0,0,0,0,0,0,0,0 };
	int32_t cost[9];
	int32_t min_cost;
	int16_t mv_x, mv_y;
	short	min_dir;
	// satd cal
    memset(satd, 0, sizeof(satd));
	min_dir = 0;
	min_cost = COST_MAX;
	for (int dir_y = -1; dir_y <= 1; dir_y++){
		for (int dir_x = -1; dir_x <= 1; dir_x++){
			int dir = (dir_y + 1) * 3 + (dir_x + 1);

			for (int blk4x4_y = 0; blk4x4_y < len_y; blk4x4_y += 4)
			{
				for (int blk4x4_x = 0; blk4x4_x<len_x; blk4x4_x += 4)
				{
					satd[dir] += sub_hadmard_satd(&cur_mb.luma[pos_y + blk4x4_y][pos_x + blk4x4_x], &ref_mb[dir][blk4x4_y][blk4x4_x]);
				}
			}


			mv_x = dir_x << (b_half ? 1 : 0);
			mv_y = dir_y << (b_half ? 1 : 0);
			cost[dir] = satd[dir] + lambda_tab[param.qp] * (bs_size_se(abs(mv[0] + mv_x)) + bs_size_se(abs(mv[1] + mv_y)));
			satd[dir] = 0;

			if (min_cost > cost[dir])
			{
				min_cost = cost[dir];
				min_dir = dir;

				dmv[0] = (b_half) ? 2 * dir_x : dir_x;
				dmv[1] = (b_half) ? 2 * dir_y : dir_y;
			}

		}
	}


	return min_cost;
}

void inter::hadamard_1d(int16_t &o_data0, int16_t &o_data1, int16_t &o_data2, int16_t &o_data3, int16_t i_data0, int16_t i_data1, int16_t i_data2, int16_t i_data3)
{
	int16_t wire0 = i_data0 + i_data1;
	int16_t wire1 = i_data0 - i_data1;
	int16_t wire2 = i_data2 + i_data3;
	int16_t wire3 = i_data2 - i_data3;

	o_data0 = wire0 + wire2;
	o_data1 = wire1 + wire3;
	o_data2 = wire0 - wire2;
	o_data3 = wire1 - wire3;
}

int32_t inter::sub_hadmard_satd(uint8_t *cur_4x4blk, uint8_t *ref_4x4blk)
{
	int i;
	int32_t o_satd;

	int16_t tranpose_reg[4][4];
	int16_t wire[4];

	o_satd = 0;

	for (i = 0; i<4; i++)
	{
		int16_t residual[4];
		residual[0] = (int16_t)(*(cur_4x4blk + i * 16 + 0) - *(ref_4x4blk + i * 16 + 0));
		residual[1] = (int16_t)(*(cur_4x4blk + i * 16 + 1) - *(ref_4x4blk + i * 16 + 1));
		residual[2] = (int16_t)(*(cur_4x4blk + i * 16 + 2) - *(ref_4x4blk + i * 16 + 2));
		residual[3] = (int16_t)(*(cur_4x4blk + i * 16 + 3) - *(ref_4x4blk + i * 16 + 3));

		hadamard_1d(tranpose_reg[i][0], tranpose_reg[i][1], tranpose_reg[i][2], tranpose_reg[i][3],
			residual[0], residual[1], residual[2], residual[3]);
	}

	for (i = 0; i<4; i++)
	{
		hadamard_1d(wire[0], wire[1], wire[2], wire[3],
			tranpose_reg[0][i], tranpose_reg[1][i], tranpose_reg[2][i], tranpose_reg[3][i]);
		o_satd += abs(wire[0]) + abs(wire[1]) + abs(wire[2]) + abs(wire[3]);
	}

	o_satd = o_satd / 2;

	return o_satd;
}







/****************************************************************************
*                             mc function
****************************************************************************/
void inter::loadmc()
{
	//--------------------------------------------------------------
	//				load mv_cache
	//--------------------------------------------------------------
	if (cur_mb.x == 0 && cur_mb.y == 0){
		memset(mv_cache, 0, sizeof(mv_cache));
		memset(ref_cache, 0, sizeof(ref_cache));
	}
	// load left
	if (cur_mb.x>0){
		for (int i = 0; i<4; i++){
			mv_cache[i + 1][0][0] = mv_cache[i + 1][4][0];
			mv_cache[i + 1][0][1] = mv_cache[i + 1][4][1];
			ref_cache[i + 1][0] = 0;
		}
	}
	else{
		for (int i = 0; i<4; i++){
			mv_cache[i + 1][0][0] = 0;
			mv_cache[i + 1][0][1] = 0;
			ref_cache[i + 1][0] = -2;
		}
	}
	// load top
	if (cur_mb.y>0){
		for (int i = 0; i<4; i++){
			mv_cache[0][i + 1][0] = *(mv_line + (cur_mb.x * 4 + i) * 2 + 0);
			mv_cache[0][i + 1][1] = *(mv_line + (cur_mb.x * 4 + i) * 2 + 1);
			ref_cache[0][i + 1] = 0;
		}
	}
	else {
		for (int i = 0; i<4; i++){
			mv_cache[0][i + 1][0] = 0;
			mv_cache[0][i + 1][1] = 0;
			ref_cache[0][i + 1] = -2;
		}
	}
	// load top left
	if (cur_mb.x>0 && cur_mb.y>0){
		mv_cache[0][0][0] = mv_topleft[0];
		mv_cache[0][0][1] = mv_topleft[1];
		ref_cache[0][0] = 0;
	}
	else{
		mv_cache[0][0][0] = 0;
		mv_cache[0][0][1] = 0;
		ref_cache[0][0] = -2;
	}
	// load top right
	if (cur_mb.x<(param.frame_mb_x_total - 1) && cur_mb.y>0){
		mv_cache[0][5][0] = *(mv_line + (cur_mb.x + 1) * 4 * 2 + 0);
		mv_cache[0][5][1] = *(mv_line + (cur_mb.x + 1) * 4 * 2 + 1);
		ref_cache[0][5] = 0;
	}
	else {
		mv_cache[0][5][0] = 0;
		mv_cache[0][5][1] = 0;
		ref_cache[0][5] = -2;
	}
	// load 4x4
	// fme.fmv: block->subblock level Z scan
	// mv_cache: x-y 2D scan

	for (int i = 0; i<4; i++)
	for (int j = 0; j<4; j++){
		mv_cache[i + 1][j + 1][0] = fmv[(j / 2) + (i / 2) * 2][(i % 2) * 2 + (j % 2)][0];
		mv_cache[i + 1][j + 1][1] = fmv[(j / 2) + (i / 2) * 2][(i % 2) * 2 + (j % 2)][1];
		ref_cache[i + 1][j + 1] = 0;
		ref_cache[i + 1][5] = -2;
	}

	if ( cur_mb.x == 0 && cur_mb.y == 0){
		mv_line = new int16_t[param.frame_mb_x_total * 4 * 2];
	}
}

void inter::get_chroma()
{
	int imv[2];
	memset(min_cb, 0, sizeof(min_cb));
	memset(min_cr, 0, sizeof(min_cr));
	switch (mb_partition){
	case D_16x16:
		imv[0] = mv16x16[0] >> 3;
		imv[1] = mv16x16[1] >> 3;
		pixel_copy_wxh(8, 8, &min_cb[0][0], 8,
			&sw.ref_mb_cb[sw_center[1]>>1 + imv[1]][sw_center[0]>>1 + imv[0]], SW_W / 2);
		pixel_copy_wxh(8, 8, &min_cr[0][0], 8,
			&sw.ref_mb_cr[sw_center[1]>>1 + imv[1]][sw_center[0]>>1 + imv[0]], SW_W / 2);
		break;
	case D_16x8:
		imv[0] = mv16x8[0][0] >> 3;
		imv[1] = mv16x8[0][1] >> 3;
		pixel_copy_wxh(4, 8, &min_cb[0][0], 8,
			&sw.ref_mb_cb[sw_center[1]>>1 + imv[1]][sw_center[0]>>1 + imv[0]], SW_W / 2);
		pixel_copy_wxh(4, 8, &min_cr[0][0], 8,
			&sw.ref_mb_cr[sw_center[1]>>1 + imv[1]][sw_center[0]>>1 + imv[0]], SW_W / 2);
		imv[0] = mv16x8[1][0] >> 3;
		imv[1] = mv16x8[1][1] >> 3;
		pixel_copy_wxh(4, 8, &min_cb[4][0], 8,
			&sw.ref_mb_cb[sw_center[1]>>1 + 4 + imv[1]][sw_center[0]>>1 + imv[0]], SW_W / 2);
		pixel_copy_wxh(4, 8, &min_cr[4][0], 8,
			&sw.ref_mb_cr[sw_center[1]>>1 + 4 + imv[1]][sw_center[0]>>1 + imv[0]], SW_W / 2);
		break;
	case D_8x16:
		imv[0] = mv8x16[0][0] >> 3;
		imv[1] = mv8x16[0][1] >> 3;
		pixel_copy_wxh(8, 4, &min_cb[0][0], 8,
			&sw.ref_mb_cb[sw_center[1]>>1 + imv[1]][sw_center[0]>>1 + imv[0]], SW_W / 2);
		pixel_copy_wxh(8, 4, &min_cr[0][0], 8,
			&sw.ref_mb_cr[sw_center[1]>>1 + imv[1] - 1][sw_center[0]>>1 + imv[0]], SW_W / 2);
		imv[0] = mv8x16[1][0] >> 3;
		imv[1] = mv8x16[1][1] >> 3;
		pixel_copy_wxh(8, 4, &min_cb[0][4], 8,
			&sw.ref_mb_cb[sw_center[1]>>1 + imv[1]][sw_center[0]>>1 + 4 + imv[0]], SW_W / 2);
		pixel_copy_wxh(8, 4, &min_cr[0][4], 8,
			&sw.ref_mb_cr[sw_center[1]>>1 + imv[1]][sw_center[0]>>1 + 4 + imv[0]], SW_W / 2);
		break;
	case D_8x8:
		for (int i = 0; i<4; i++)
		{
			switch (mb_subpartition[i])
			{
			case D_L0_8x8:
				imv[0] = mv8x8[i][0] >> 3;
				imv[1] = mv8x8[i][1] >> 3;
				pixel_copy_wxh(4, 4, &min_cb[(i / 2) * 4][(i % 2) * 4], 8,
					&sw.ref_mb_cb[sw_center[1]>>1 + int(i / 2) * 4 + imv[1] - 1][sw_center[0]>>1 + (i % 2) * 4 + imv[0]], SW_W / 2);
				pixel_copy_wxh(4, 4, &min_cr[(i / 2) * 4][(i % 2) * 4], 8,
					&sw.ref_mb_cr[sw_center[1]>>1 + int(i / 2) * 4 + imv[1] - 1][sw_center[0]>>1 + (i % 2) * 4 + imv[0]], SW_W / 2);
				break;
			case D_L0_8x4:
				imv[0] = mv8x4[i][0][0] >> 3;
				imv[1] = mv8x4[i][0][1] >> 3;
				pixel_copy_wxh(2, 4, &min_cb[(i / 2) * 4][(i % 2) * 4], 8,
					&sw.ref_mb_cb[sw_center[1]>>1 + int(i / 2) * 4 + imv[1] - 1][sw_center[0]>>1 + (i % 2) * 4 + imv[0]], SW_W / 2);
				pixel_copy_wxh(2, 4, &min_cr[(i / 2) * 4][(i % 2) * 4], 8,
					&sw.ref_mb_cr[sw_center[1]>>1 + int(i / 2) * 4 + imv[1] - 1][sw_center[0]>>1 + (i % 2) * 4 + imv[0]], SW_W / 2);

				imv[0] = mv8x4[i][1][0] >> 3;
				imv[1] = mv8x4[i][1][1] >> 3;
				pixel_copy_wxh(2, 4, &min_cb[(i / 2) * 4 + 2][(i % 2) * 8], 8,
					&sw.ref_mb_cb[sw_center[1]>>1 + int(i / 2) * 4 + 2 + imv[1]][sw_center[0]>>1 + (i % 2) * 4 + imv[0]], SW_W / 2);
				pixel_copy_wxh(2, 4, &min_cr[(i / 2) * 4 + 2][(i % 2) * 8], 8,
					&sw.ref_mb_cr[sw_center[1]>>1 + int(i / 2) * 4 + 2 + imv[1]][sw_center[0]>>1 + (i % 2) * 4 + imv[0]], SW_W / 2);
				break;
			case D_L0_4x8:
				imv[0] = mv4x8[i][0][0] >> 3;
				imv[1] = mv4x8[i][0][1] >> 3;
				pixel_copy_wxh(4, 2, &min_cb[(i / 2) * 4][(i % 2) * 4], 8,
					&sw.ref_mb_cb[sw_center[1]>>1 + int(i / 2) * 4 + imv[1]][sw_center[0]>>1 + (i % 2) * 4 + imv[0]], SW_W / 2);
				pixel_copy_wxh(4, 2, &min_cr[(i / 2) * 4][(i % 2) * 4], 8,
					&sw.ref_mb_cr[sw_center[1]>>1 + int(i / 2) * 4 + imv[1]][sw_center[0]>>1 + (i % 2) * 4 + imv[0]], SW_W / 2);

				imv[0] = mv4x8[i][1][0] >> 3;
				imv[1] = mv4x8[i][1][1] >> 3;
				pixel_copy_wxh(4, 2, &min_cb[(i / 2) * 4][(i % 2) * 4 + 2], 8,
					&sw.ref_mb_cb[sw_center[1]>>1 + int(i / 2) * 4 + imv[1]][sw_center[0]>>1 + (i % 2) * 4 + 2 + imv[0]], SW_W / 2);
				pixel_copy_wxh(4, 2, &min_cr[(i / 2) * 4][(i % 2) * 4 + 2], 8,
					&sw.ref_mb_cr[sw_center[1]>>1 + int(i / 2) * 4 + imv[1]][sw_center[0]>>1 + (i % 2) * 4 + 2 + imv[0]], SW_W / 2);
				break;
			case D_L0_4x4:
				imv[0] = mv4x4[i][0][0] >> 3;
				imv[1] = mv4x4[i][0][1] >> 3;
				pixel_copy_wxh(2, 2, &min_cb[(i / 2) * 4][(i % 2) * 4], 8,
					&sw.ref_mb_cb[sw_center[1]>>1 + int(i / 2) * 4 + imv[1]][sw_center[0]>>1 + (i % 2) * 4 + imv[0]], SW_W / 2);
				pixel_copy_wxh(2, 2, &min_cr[(i / 2) * 4][(i % 2) * 4], 8,
					&sw.ref_mb_cr[sw_center[1]>>1 + int(i / 2) * 4 + imv[1]][sw_center[0]>>1 + (i % 2) * 4 + imv[0]], SW_W / 2);

				imv[0] = mv4x4[i][1][0] >> 3;
				imv[1] = mv4x4[i][1][1] >> 3;
				pixel_copy_wxh(4, 4, &min_cb[(i / 2) * 4][(i % 2) * 4 + 2], 8,
					&sw.ref_mb_cb[sw_center[1]>>1 + int(i / 2) * 4 + imv[1]][sw_center[0]>>1 + (i % 2) * 4 + 2 + imv[0]], SW_W / 2);
				pixel_copy_wxh(4, 4, &min_cr[(i / 2) * 4][(i % 2) * 4 + 2], 8,
					&sw.ref_mb_cr[sw_center[1]>>1 + int(i / 2) * 4 + imv[1]][sw_center[0]>>1 + (i % 2) * 4 + 2 + imv[0]], SW_W / 2);

				imv[0] = mv4x4[i][2][0] >> 3;
				imv[1] = mv4x4[i][2][1] >> 3;
				pixel_copy_wxh(4, 4, &min_cb[(i / 2) * 8 + 2][(i % 2) * 8], 8,
					&sw.ref_mb_cb[sw_center[1]>>1 + int(i / 2) * 4 + 2 + imv[1]][sw_center[0]>>1 + (i % 2) * 4 + imv[0]], SW_W / 2);
				pixel_copy_wxh(4, 4, &min_cr[(i / 2) * 8 + 2][(i % 2) * 8], 8,
					&sw.ref_mb_cr[sw_center[1]>>1 + int(i / 2) * 4 + 2 + imv[1]][sw_center[0]>>1 + (i % 2) * 4 + imv[0]], SW_W / 2);

				imv[0] = mv4x4[i][3][0] >> 3;
				imv[1] = mv4x4[i][3][1] >> 3;
				pixel_copy_wxh(4, 4, &min_cb[(i / 2) * 8 + 2][(i % 2) * 8 + 2], 8,
					&sw.ref_mb_cb[sw_center[1]>>1 + int(i / 2) * 4 + 2 + imv[1]][sw_center[0]>>1 + (i % 2) * 4 + 2 + imv[0]], SW_W / 2);
				pixel_copy_wxh(4, 4, &min_cr[(i / 2) * 8 + 2][(i % 2) * 8 + 2], 8,
					&sw.ref_mb_cr[sw_center[1]>>1 + int(i / 2) * 4 + 2 + imv[1]][sw_center[0]>>1 + (i % 2) * 4 + 2 + imv[0]], SW_W / 2);
				break;
			default: printf("FME Fetching ERROR: subpartition is wrong!\n");
			}
		}
		break;
	default: printf("FME Fetching ERROR: partition is wrong!\n");
	}
}

void inter::mvp_compute()
{
	switch (mb_partition){
	case D_16x16:
		mvp_median(0 + 7, 4, mvp[0][0]);
		for (int i = 0; i<4; i++)
		for (int j = 0; j<4; j++){
			mvp[i][j][0] = mvp[0][0][0];
			mvp[i][j][1] = mvp[0][0][1];
		}
		break;
	case D_16x8:
		for (int blk = 0; blk<2; blk++){
			mvp_median(0 + 7 + 6 * 2 * blk, 4, mvp[0 + blk * 2][0]);
		}
		for (int j = 0; j<4; j++)
		{
			mvp[0][j][0] = mvp[0][0][0];
			mvp[0][j][1] = mvp[0][0][1];

			mvp[1][j][0] = mvp[0][0][0];
			mvp[1][j][1] = mvp[0][0][1];

			mvp[2][j][0] = mvp[2][0][0];
			mvp[2][j][1] = mvp[2][0][1];

			mvp[3][j][0] = mvp[2][0][0];
			mvp[3][j][1] = mvp[2][0][1];
		}
		break;
	case D_8x16:
		for (int blk = 0; blk<2; blk++){
			mvp_median(0 + 7 + 2 * blk, 2, mvp[0 + blk * 1][0]);
		}
		for (int j = 0; j<4; j++)
		{
			mvp[0][j][0] = mvp[0][0][0];
			mvp[0][j][1] = mvp[0][0][1];

			mvp[2][j][0] = mvp[0][0][0];
			mvp[2][j][1] = mvp[0][0][1];

			mvp[1][j][0] = mvp[1][0][0];
			mvp[1][j][1] = mvp[1][0][1];

			mvp[3][j][0] = mvp[1][0][0];
			mvp[3][j][1] = mvp[1][0][1];
		}
		break;
	case D_8x8:
		for (int blk = 0; blk<4; blk++){
			switch (mb_subpartition[blk]){
			case D_L0_8x8:
				mvp_median(0 + 7 + 2 * (blk % 2) + 6 * 2 * (blk / 2), 2, mvp[0 + blk][0]);
				for (int j = 0; j<4; j++){
					mvp[blk][j][0] = mvp[blk][0][0];
					mvp[blk][j][1] = mvp[blk][0][1];
				}
				break;
			case D_L0_8x4:
				for (int j = 0; j<2; j++){
					mvp_median(0 + 7 + 2 * (blk % 2) + 6 * 2 * (blk / 2) + j * 6, 2, mvp[0 + blk][0 + 2 * j]);
				}
				for (int j = 0; j<2; j++){
					mvp[blk][j][0] = mvp[blk][0][0];
					mvp[blk][j][1] = mvp[blk][0][1];
					mvp[blk][j + 2][0] = mvp[blk][2][0];
					mvp[blk][j + 2][1] = mvp[blk][2][1];
				}
				break;
			case D_L0_4x8:
				for (int j = 0; j<2; j++)	{
					mvp_median(0 + 7 + 2 * (blk % 2) + 6 * 2 * (blk / 2) + j, 1, mvp[0 + blk][j]);
				}

				for (int j = 0; j<2; j++)	{
					mvp[blk][j * 2][0] = mvp[blk][0][0];
					mvp[blk][j * 2][1] = mvp[blk][0][1];
					mvp[blk][1 + j * 2][0] = mvp[blk][1][0];
					mvp[blk][1 + j * 2][1] = mvp[blk][1][1];
				}
				break;
			case D_L0_4x4:
				for (int j = 0; j<4; j++) {
					mvp_median(0 + 7 + 6 * 2 * (blk / 2) + (j / 2) * 6 + j % 2 + 2 * (blk % 2), 1, mvp[0 + blk][j]);
				}
				break;
			}
		}
		break;
	}
}

void inter::transform()
{
	int16_t dct4x4[16][4][4];
	int16_t dct4x4c[2][4][4][4];
	int16_t dct2x2_dc[2][2][2];
	int sad;

	// luma transform
	memcpy(luma_rec, min_mb, sizeof(luma_rec));
	sad = sub16x16_dct(dct4x4, &cur_mb.luma[0][0], 16, &luma_rec[0][0], 16);// Dctout,*pix1,size1,*pix2,size2
	for (int i = 0; i < 16; i++){
		quant_4x4(dct4x4[i], cqm.quant4_mf[cqm.qp], cqm.inter_quant4_bias[cqm.qp], cqm.qp);
		zigzag_scan_4x4_frame(luma_dct4x4[i], dct4x4[i]);
		dequant_4x4(dct4x4[i], cqm.dequant4_mf, cqm.qp);
	}
	add16x16_idct(&luma_rec[0][0], 16, dct4x4);

	// chroma transform
	memcpy(chroma_rec[0], min_cb, sizeof(min_cb));
	memcpy(chroma_rec[1], min_cr, sizeof(min_cr));

	sub8x8_dct(dct4x4c[0], cur_mb.cb[0], 8, chroma_rec[0][0], 8);
	sub8x8_dct(dct4x4c[1], cur_mb.cr[0], 8, chroma_rec[1][0], 8);
	dct2x2dc(dct2x2_dc[0], dct4x4c[0]);
	dct2x2dc(dct2x2_dc[1], dct4x4c[1]);

	for (int i = 0; i<4; i++){
		quant_4x4(dct4x4c[0][i], cqm.quant4_mf[QP_FOR_CHROMA[cqm.qp]], cqm.intra_quant4_bias[QP_FOR_CHROMA[cqm.qp]], QP_FOR_CHROMA[cqm.qp]);
		quant_4x4(dct4x4c[1][i], cqm.quant4_mf[QP_FOR_CHROMA[cqm.qp]], cqm.intra_quant4_bias[QP_FOR_CHROMA[cqm.qp]], QP_FOR_CHROMA[cqm.qp]);
		zigzag_scan_4x4_frame(chroma_dct4x4[0][i], dct4x4c[0][i]);
		zigzag_scan_4x4_frame(chroma_dct4x4[1][i], dct4x4c[1][i]);
		dequant_4x4(dct4x4c[0][i], cqm.dequant4_mf, QP_FOR_CHROMA[cqm.qp]);
		dequant_4x4(dct4x4c[1][i], cqm.dequant4_mf, QP_FOR_CHROMA[cqm.qp]);
	}
	quant_2x2_dc(dct2x2_dc[0], cqm.quant4_mf[QP_FOR_CHROMA[cqm.qp]][0], cqm.intra_quant4_bias[QP_FOR_CHROMA[cqm.qp]][0] << 1, QP_FOR_CHROMA[cqm.qp]);
	quant_2x2_dc(dct2x2_dc[1], cqm.quant4_mf[QP_FOR_CHROMA[cqm.qp]][0], cqm.intra_quant4_bias[QP_FOR_CHROMA[cqm.qp]][0] << 1, QP_FOR_CHROMA[cqm.qp]);
	zigzag_scan_2x2_dc(chroma_dct4x4_dc[0], dct2x2_dc[0]);
	zigzag_scan_2x2_dc(chroma_dct4x4_dc[1], dct2x2_dc[1]);
	idct_dequant_2x2_dc(dct2x2_dc[0], dct4x4c[0], cqm.dequant4_mf, QP_FOR_CHROMA[cqm.qp]);
	idct_dequant_2x2_dc(dct2x2_dc[1], dct4x4c[1], cqm.dequant4_mf, QP_FOR_CHROMA[cqm.qp]);
	add8x8_idct(chroma_rec[0][0], 8, dct4x4c[0]);
	add8x8_idct(chroma_rec[1][0], 8, dct4x4c[1]);
}

void inter::cbp_encode()
{
	cbp_luma = 0;
	for (int i = 0; i < 4; i++){
		int nz, cbp = 0;
		for (int j = 0; j < 4; j++){
			nz = array_non_zero(luma_dct4x4[j + 4 * i]);
			non_zero_count[j + 4 * i] = nz;
			cbp |= nz;
		}
		cbp_luma |= cbp << i;
	}
	non_zero_count[24] = 0;

	cbp_chroma = 0;
	for (int i = 0; i < 2; i++)
	for (int j = 0; j < 4; j++) {
		int nz = array_non_zero(chroma_dct4x4[i][j]);
		non_zero_count[16 + i * 4 + j] = nz;
		cbp_chroma |= nz;
	}
	non_zero_count[25] = array_non_zero(chroma_dct4x4_dc[0]);
	non_zero_count[26] = array_non_zero(chroma_dct4x4_dc[1]);
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


void inter::mvp_median(int idx, int i_width, int16_t mvp[2])
{
	//int *mvp;
	const int i_ref = ref_cache[idx / 6][idx % 6];      //cur
	int     i_refa = ref_cache[(idx - 1) / 6][(idx - 1) % 6];  //left
	int16_t *mv_a = mv_cache[(idx - 1) / 6][(idx - 1) % 6];
	int     i_refb = ref_cache[(idx - 6) / 6][(idx - 6) % 6];  //top.
	int16_t *mv_b = mv_cache[(idx - 6) / 6][(idx - 6) % 6];
	int     i_refc = ref_cache[(idx - 6 + i_width) / 6][(idx - 6 + i_width) % 6];//topright.
	int16_t *mv_c = mv_cache[(idx - 6 + i_width) / 6][(idx - 6 + i_width) % 6];
	//i_list reference list.
	int i_count;
	int pos = idx - 7;
	//011.
	// 3 when 4x4;           //3,6,7, when 8x4.              // or not available.
	// if  c is not available, replace it by d.
	if ((((pos & 0x03) == 3) && pos>6)
		|| ((i_width == 2 && (pos & 0x3) == 2) && (pos != 2))
		|| i_refc == -2)
	{
		i_refc = ref_cache[(idx - 6 - 1) / 6][(idx - 6 - 1) % 6];
		mv_c = mv_cache[(idx - 6 - 1) / 6][(idx - 6 - 1) % 6];
	}

	if (mb_partition == D_16x8)
	{
		if (pos == 0 && i_refb == i_ref)
		{
			// 0
			//---
			// 1
			*(uint32_t*)mvp = *(uint32_t*)mv_b;
			return;
		}
		else if (pos != 0 && i_refa == i_ref)
		{
			*(uint32_t*)mvp = *(uint32_t*)mv_a;
			return;
		}
	}
	else if (mb_partition == D_8x16)
	{
		if (pos == 0 && i_refa == i_ref)
		{
			*(uint32_t*)mvp = *(uint32_t*)mv_a;
			return;
		}
		else if (pos != 0 && i_refc == i_ref)
		{
			*(uint32_t*)mvp = *(uint32_t*)mv_c;
			return;
		}
	}



	i_count = 0;
	if (i_refa == i_ref) i_count++;
	if (i_refb == i_ref) i_count++;
	if (i_refc == i_ref) i_count++;

	if (i_count > 1)
	{
		mvp[0] = f264_median(mv_a[0], mv_b[0], mv_c[0]);    //horizontal mv
		mvp[1] = f264_median(mv_a[1], mv_b[1], mv_c[1]);    //vertical mv
	}
	else if (i_count == 1)
	{
		if (i_refa == i_ref)
			*(uint32_t*)mvp = *(uint32_t*)mv_a;
		else if (i_refb == i_ref)
			*(uint32_t*)mvp = *(uint32_t*)mv_b;
		else
			*(uint32_t*)mvp = *(uint32_t*)mv_c;
	}
	else if (i_refb == -2 && i_refc == -2 && i_refa != -2)
		*(uint32_t*)mvp = *(uint32_t*)mv_a;
	else
	{
		mvp[0] = f264_median(mv_a[0], mv_b[0], mv_c[0]);    //horizontal mv
		mvp[1] = f264_median(mv_a[1], mv_b[1], mv_c[1]);    //vertical mv
	}
}

int  inter::f264_median(int a, int b, int c)
{
	int t = (a - b)&((a - b) >> 31);
	a -= t;
	b += t;
	b -= (b - c)&((b - c) >> 31);
	b += (a - b)&((a - b) >> 31);
	return b;
}

void inter::update(){
	//store line cache
	if (cur_mb.y>0){
		mv_topleft[0] = *(mv_line + (cur_mb.x * 4 + 3) * 2 + 0);
		mv_topleft[1] = *(mv_line + (cur_mb.x * 4 + 3) * 2 + 1);
	}

	for (int i = 0; i<4; i++){
		*(mv_line + (cur_mb.x * 4 + i) * 2 + 0) = mv_cache[4][i + 1][0];
		*(mv_line + (cur_mb.x * 4 + i) * 2 + 1) = mv_cache[4][i + 1][1];
	}
}

void inter::dump_yuv() {
    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 16; j++) {
            //org_yuv[cur_mb.x * 16 + i][cur_mb.y * 16 + j] = cur_mb.luma[((zscan_to_raster_4x4[i] / 4) * 4) + j / 4][((zscan_to_raster_4x4[i] % 4) * 4) + j % 4];
            //rec_yuv[cur_mb.x * 16 + i][cur_mb.y * 16 + j] = luma_rec[((zscan_to_raster_4x4[i] / 4) * 4) + j / 4][((zscan_to_raster_4x4[i] % 4) * 4) + j % 4];
            org_yuv[cur_mb.y * 16 + i][cur_mb.x * 16 + j] = cur_mb.luma[i][j];
            rec_yuv[cur_mb.y * 16 + i][cur_mb.x * 16 + j] = luma_rec[i][j];
        }
    }
    if ((cur_mb.x == param.frame_mb_x_total - 1) && (cur_mb.y == param.frame_mb_y_total - 1)) {
        char*  file_org = new char[100];
        sprintf(file_org, "D:\\MyFile\\MATLAB\\h264_mc\\org_y_%d.csv", param.frame_num);
        fp_org = fopen(file_org,"w");
        for (int i = 0; i < 9 * 16; i++) {
            for (int j = 0; j < 11 * 16; j++) {
                fprintf(fp_org, "%03d,", org_yuv[i][j]);
            }
            fprintf(fp_org, "\n");
        }

        char* file_rec = new char[100];
        sprintf(file_rec, "D:\\MyFile\\MATLAB\\h264_mc\\rec_y_%d.csv", param.frame_num);
        fp_rec = fopen(file_rec, "w");
        for (int i = 0; i < 9 * 16; i++) {
            for (int j = 0; j < 11 * 16; j++) {
                fprintf(fp_rec, "%03d,", rec_yuv[i][j]);
            }
            fprintf(fp_rec, "\n");
        }

        char* file_ameflg = new char[100];
        sprintf(file_ameflg, "D:\\MyFile\\MATLAB\\h264_mc\\ame_flg_%d.csv", param.frame_num);
        fp_ameflg = fopen(file_ameflg, "w");
        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 11; j++) {
                fprintf(fp_ameflg, "%1d, ", ame_flg_mat[i][j]);
            }
            fprintf(fp_ameflg, "\n");
        }

        fclose(fp_org);
        fclose(fp_rec);
        fclose(fp_ameflg);

    }
}
