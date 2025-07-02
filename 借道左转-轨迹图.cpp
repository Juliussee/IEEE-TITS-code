/*
改变参数变化情况 ：
	车长变短：左右车道流量提升、右车道增加更多
	增大LC：左车道流量降低，右车道流量提升
	增加掉头率：左右车道流量降低
	改变最大速度：变化不明显
	增大LB：左车道流量提升，右车道流量降低，差值变小

	换道车辆数：15000 左车道流量：0.095 右车道流量：0.108
	符合实际数据的参数：LC=40，LB=30，Vmax=20, p_change=0.2,car_right[i].isAuto==0&&(car_right[i].gap - car_right[i].gap_other) <= 1 && car_right[i].gap_back > car_right[i].vob+ getDoubleMin2(car_right[i].vob+1,Vmax) -Vmin_right
*/


//#include <bits/stdc++.h>
#define _CRT_SECURE_NO_WARNINGS
#include<iostream>
#include<fstream>
#include<cmath>
#include<ctime>
#include<string>
#include<cstdlib>
#include<windows.h>

#define road_long_right 1500		//路长（格）、、改为2000 
#define Vmax 20		//最大速度
#define veh_length 7.5		//车长（格）
#define L_cell 1.5			//一个元胞实际长度(m) 
#define total_timestep 11150
#define count_timestep 2000


double LC = 100;
double LB = 30;


double LA = road_long_right - LB-LC;//记住，LA计算必须要在LB赋值之后！

int i, j, k, veh_sum_save, randnumber, x, e;
double occupancy;
double density;
int sample;
int time_step;
double veh_init_gap;  //道路初始化间距
//改进数据
double p_add;
double p_slow=0.3;
double Varr;
double m2=1.3;
double m3=100;
double m1;
double pf=0.55;

//主干道模型
int veh_sum_left;				//左车道车辆总数
double vel_sum_left;				//左车道速度和
int veh_sum_right;				//右车道车辆总数
double vel_sum_right;				//右车道速度和
int test_cell=road_long_right-40;//表示监测点
double Qin;
double dsafe = 20;
double left_num_save[road_long_right] = { -1 };
double right_num_save[road_long_right] = { -1 };
int outNum_left = 0;//记录出车数
int outNum_right = 0;//记录出车数
int coil_time_gap = 60; //线圈输出间隔
int LB_change_num = 0;//换道数目
int time_red=0;//红灯时间


double vleft_save[road_long_right]= {-1}; //保存速度
double vright_save[road_long_right]= {-1};
double h_con=9;//精细化的值
double gapsafe=10.5;//精细化的值
double vanti;
double pb=0.94;//此为慢化概率
double p0=0.5;
double pd=0.1;
double tc=10;
double th_left;
double ts_left;
double th_right;
double ts_right;
int LCsum_left;//左车道LC区域车辆数
int LCsum_right;
double temp;//左车道排队长度与lc比值
double pd_change;
int light_time;//当前时间与215相除的余数

double velocity_sume_left = 0;//统计速度
double velocity_sume_right = 0;
int coil_veh_sum_right = 0;
double coil_velosity_sum_right = 0;
int coil_veh_sum_left = 0;
double coil_velosity_sum_left = 0;

//CACC
double r_pro;//自动驾驶车辆占比

double amax=2;//加速度
double b_ease=2;//舒适减速度
double bmax=3;//最大减速度(单位为m/s)

double Ti;//gipps跟驰模型中反应时间
double Ta;//ACC期望车间距
double Tc;//CACC期望车间距
double k1=0.23;
double k2=0.07;
double j11=1.0;
double j2=0.2;
double j3=0.3;

double s0=1;
double s1=0;
double s2=0;

double front_v_left[2000];//保存前车速度
double front_a_left[2000];//保存前车加速度
double var_v_left[2000];//速度变化量
double var_x_left[2000];//位移变化量
double front_v_right[2000];//保存前车速度
double front_a_right[2000];//保存前车加速度
double var_v_right[2000];//速度变化量
double var_x_right[2000];//位移变化量

double a_left[2000];
double a_right[2000];

double Dsafe;
double Vsafe;
double ACC_a;
double CACC_a;
double frontnextv;
double safev_left;
double safev_right;
int changeNum;
int fortemp;
double p_change;//换道概率

int orbit_sum=20;
int orbit[20]= {0}; //每一辆车的位置
int isleft[20]= {0}; // =0表示车辆在右车道，=1表示车辆在左车道
int lo=0;//左右车道准备统计的编号
int ro=0;
int time_start = 11000;//开始时间
int position_start=1100;//统计位置
int osave_left[2000]= {0};//左右车道保存速度
int osave_right[2000]= {0};


typedef struct car_left {
	double x;
	double v;
	double vob;
	double gap;
	double gap_other;
	double gap_back;
	int position;
	int isinsert;//代表车辆是否可插入
	int isAuto;
	int type;//0表示gipps更新，1表示ACC更新，2表示CACC更新 3表示首辆是手动 4表示首辆是自动
} CARL;

typedef struct car_right {
	double x;
	double v;
	double vob;
	double gap;
	double gap_other;
	double gap_back;
	double position;
	int isinsert;
	int isAuto;
	int type;//0表示gipps更新，1表示ACC更新，2表示CACC更新 3表示首辆是手动 4表示首辆是自动
} CARR;

//两个数产生最小值
int getMin2(int m, int n) {
	if (m < n) {
		return m;
	} else {
		return n;
	}
}

//两个数产生最大值
int getMax2(int m, int n) {
	if (m < n) {
		return n;
	} else {
		return m;
	}
}

//三个数求最小值
int getMin3(int u, int m, int z) {
	int  b;
	if (u > z) {
		if (z > m) {
			b = m;
		} else {
			b = z;
		}
	} else if (u < m) {
		b = u;
	} else {
		b = m;
	}
	return b;
}

//两个数产生最大值
double getDoubleMax2(double m, double n) {
	if (m<n) {
		return n;
	} else {
		return m;
	}
}

double getDoubleMin2(double m, double n) {
	if (m<n) {
		return m;
	} else {
		return n;
	}
}

double getDoubleMin3(double m,double n,double q) {
	return getDoubleMin2(getDoubleMin2(m,n),q);
}

double getDoubleMin4(double m,double n,double q,double w) {
	return getDoubleMin2(getDoubleMin2(m,n),getDoubleMin2(q,w));
}

//右车道进车
void enterMainRoad(CARL* car_left, CARR* car_right) {
	double randomNum_qin2 = ((double)rand()) / RAND_MAX;
	double auto_veh2=((double)rand()) / RAND_MAX;
	if (veh_sum_right == 0) {

		veh_sum_right++;
		car_right[veh_sum_right - 1].v = 0;
		car_right[veh_sum_right - 1].x = rand() % (int)Vmax;//以车尾坐标来确定车辆位置,车辆首次进入
		car_right[veh_sum_right - 1].isinsert = 0;
		car_right[veh_sum_right - 1].isAuto=0;//表示手动


	} else if (randomNum_qin2 < Qin) {

		if(car_right[veh_sum_right - 1].x>Vmax - 1.5) {

			veh_sum_right++;
			car_right[veh_sum_right - 1].v = Vmax;
			car_right[veh_sum_right - 1].x = getDoubleMin2(car_right[veh_sum_right - 2].x - Vmax, Vmax);
			car_right[veh_sum_right - 1].isinsert = 0;

			if(auto_veh2<r_pro) {
				car_right[veh_sum_right - 1].isAuto=1;//表示自动

			} else {
				car_right[veh_sum_right - 1].isAuto=0;//表示手动
			}

			//printf("当前进车time:%d 红灯时间：%d 左车数：%d 右车数 ：%d 左出车：%d 右出车：%d 出车差值：%d 左0车：%.3f 右0车：%.3f\n",time_step,time_red,veh_sum_left,veh_sum_right,outNum_left,outNum_right,outNum_right-outNum_left,car_left[0].x,car_right[0].x);
		}
	}

}


/*
tep1:计算gap_other,gap_back,gap,vob分别表示相邻车道前后距离，和当前车与前车间距,另一车道跟随车距离
*step2:找出所有符合换道规则的车辆
*step3:车辆换道
*step4：左右车道进行NS规则变换，左车道可出车，右车道不可。
*/
int velocitySum(CARL* car_left, CARR* car_right) {

	//进行初始化操作
	for (i = 0; i < veh_sum_left; i++) {
		car_left[i].gap_back = -1;
		car_left[i].gap_other = -1;
		car_left[i].vob = 999;
	}
	for (i = 0; i < veh_sum_right; i++) {
		car_right[i].gap_back = -1;
		car_right[i].gap_other = -1;
		car_right[i].vob = 999;
	}

	/*step1*/
	/*计算左车道gap_other, gap_back, gap, vob*/
	int LABlast = -1;//、、右车道LB区域最后一辆车
	LCsum_left=0;
	LCsum_right=0;

	//计算间距
	for (i = 0; i < veh_sum_left; i++) {
		if (i == 0) {//最前面一辆车子
			car_left[i].gap = Vmax;
		} else {
			car_left[i].gap = car_left[i - 1].x - car_left[i].x - veh_length;
		}
		if((int)car_left[i].gap<0) {
			printf("左车道换道之前小于0 当前时步：%d 车辆总数：%d 间距：%f \n",time_step,veh_sum_left,car_left[i].gap);
		}
		if(car_left[i].x>LA+LB-veh_length) {
			LCsum_left++;
		}
	}
	//printf("车辆总数：%d LC区域车辆：%d LB区域第一辆车的位置：%d 速度：%d\n",veh_sum_left,LCsum_left,car_left[LCsum_left].x,car_left[LCsum_left].v);

	for (i = 0; i < veh_sum_right; i++) {
		if (i == 0) {//最前面一辆车子
			car_right[i].gap = Vmax;//右车道前面有边界
		} else {
			car_right[i].gap = car_right[i - 1].x - car_right[i].x - veh_length;
		}
		if(car_right[i].x>=LA) {
			LABlast++;//LB区域最左端车辆
		}
		if(car_right[i].x>LA+LB-veh_length) {
			LCsum_right++;//LB区域最右端车辆，即LC区域车辆总数
		}
		if((int)car_right[i].gap<0) {
			printf("右车道换道之前小于0，当前时步：%d 序号为：%d 位置为：%f 间距为：%f 速度为：%f 前车位置为：%f\n",time_step,i,car_right[i].x,car_right[i].gap,car_right[i].v,car_right[i-1].x);
		}
	}

	/*计算右车道gap_other, gap_back, gap, vob*/
	for (i = 0; i <= LABlast; i++) {
		if (car_right[i].x > car_left[0].x) {//特殊情形1：换道区域右车道车辆在左车道第一辆车前面
			car_right[i].gap_other = Vmax;
			car_right[i].gap_back = car_right[i].x - car_left[0].x - veh_length;
			car_right[i].vob = car_left[0].v;
			//printf("abfirst:%d other:%d back:%d vob:%d\n", LABfirst, car_right[i].gap_other, car_right[i].gap_back, car_right[i].vob);
		} else if (car_right[i].x < car_left[veh_sum_left - 1].x) { //特殊情形2：换道区域右车道车辆在左车道最后一辆车后面
			car_right[i].gap_other = car_left[veh_sum_left - 1].x - car_right[i].x - veh_length;
			car_right[i].gap_back = car_right[i].x-LA;
			car_right[i].vob = -1;
		} else {
			for (j = 0; j < veh_sum_left; j++) {
				if (car_right[i].x == car_left[j].x) {//特殊情形3：若相邻车道有车，则直接返回-1
					car_right[i].gap_back = -1;
					car_right[i].gap_other = -1;
					car_right[i].vob = 999;
				} else if (car_right[i].x<car_left[j].x && car_right[i].x>car_left[j + 1].x) {
					car_right[i].gap_other = car_left[j].x - car_right[i].x - veh_length;
					car_right[i].gap_back = car_right[i].x - car_left[j + 1].x - veh_length;
					car_right[i].vob = car_left[j + 1].v;
				}
			}
		}
	}

	/*step2*/
	/*判断车道需要换道的车辆*/
	light_time=time_step%110;
	for (i = 0; i <= veh_sum_right; i++) {
		if (LA <= car_right[i].x && car_right[i].x < LA+LB) { //找出LB区域右车道满足换道车辆，使用参数isinsert==1来表示

			double Vmin_right=getDoubleMin2(car_right[i].v+1, Vmax);

			double Dsafe_right=getDoubleMin2(Vmax,car_right[i].vob+amax);

			if (car_right[i].isAuto==1&&(car_right[i].gap - car_right[i].gap_other) <= 1 && car_right[i].gap_back > car_right[i].vob+ getDoubleMin2(car_right[i].vob+1,Vmax) -Vmin_right) {

				car_right[i].isinsert = 1;//自动驾驶车辆换道

				printf("test");

			} else if(car_right[i].isAuto==0&&(car_right[i].gap - car_right[i].gap_other) <= 1 && car_right[i].gap_back > car_right[i].vob+ getDoubleMin2(car_right[i].vob+1,Vmax) -Vmin_right) {

				car_right[i].isinsert = 1;//人工驾驶车辆换道

			} else {

				car_right[i].isinsert = 0;
			}

			if(light_time<50||light_time>=110) {//预信号灯控制

				car_right[i].isinsert = 0;
			}

		} else {
			car_right[i].isinsert = 0;
		}
	}



	for (i = 0; i < veh_sum_right; i++) {
		if(car_right[i].gap_back<0||car_right[i].gap_other<0) {
			car_right[i].isinsert = 0;
		}
	}


	/*step3*/
	/*右车道换左车道*/
	for (i = 0; i <= LABlast;) {//这里sum_right的值应该改变
		if (car_right[i].isinsert == 1) {

			if(time_step>time_start) { //统计时间开始过后
				for(j=0; j<20; j++) {
					if(orbit[j]==(int)car_right[i].x&&isleft[j]==0) { //车辆位于右车道，且位置相同
						isleft[j]=1;//表示车辆换道
					}
				}
			}

			LB_change_num++;//换道车辆数

			int rightNum = -2;
			if (car_right[i].x > car_left[0].x) {
				rightNum = 0 ;//插入位置在左车道第一辆车前面
			} else if (car_right[i].x < car_left[veh_sum_left - 1].x) {
				rightNum = veh_sum_left;//插入位置在左车道最后一辆车后面
			} else {
				for (j = 0; j < veh_sum_left; j++) {
					if (car_right[i].x<car_left[j].x && car_right[i].x>car_left[j + 1].x) {//
						rightNum = j + 1;
					}
				}
			}
			/*左车道情况*/
			for (k = veh_sum_left - 1; k >= rightNum; k--) {
				car_left[k + 1].v = car_left[k].v;
				car_left[k + 1].x = car_left[k].x;
				car_left[k + 1].isinsert = car_left[k].isinsert;
				car_left[k + 1].isAuto = car_left[k].isAuto;
			}
			car_left[rightNum].x = car_right[i].x;
			car_left[rightNum].v = car_right[i].v;
			car_left[rightNum].isinsert = 0;
			car_left[rightNum].isAuto = car_right[i].isAuto;
			//printf("右车道插入到左车道位置：%d  插入车后一辆位置：%d 插入车前一辆位置：%d\n", car_left[rightNum].x, car_left[rightNum+1].x,car_left[rightNum-1].x);
			veh_sum_left++;
			/*右车道情况*/
			for (j = i + 1; j <= veh_sum_right - 1; j++) {
				car_right[j - 1].x = car_right[j].x;
				car_right[j - 1].v = car_right[j].v;
				car_right[j - 1].isinsert = car_right[j].isinsert;

				car_right[j - 1].isAuto = car_right[j].isAuto;
			}
			veh_sum_right--;
			LABlast--;
		} else {
			i++;
		}
	}


	//换道之后进行掉头操作
	int index_turn=LCsum_left;//左车道车辆
	while(index_turn<veh_sum_left) { //表示车辆在LA和LB区域
		if(((double)rand()) / RAND_MAX<p_change) { //满足换道概率
			if(time_step>time_start){
				printf("///////////////////////////////////////当前时步：%d 位置：%d 发生掉头",time_step,(int)car_left[index_turn].x);
			}
			for (j = index_turn + 1; j <= veh_sum_left - 1; j++) {//后面每一辆车往前移动
				car_left[j - 1].x = car_left[j].x;
				car_left[j - 1].v = car_left[j].v;
				car_left[j - 1].isinsert = car_left[j].isinsert;
				car_left[j - 1].isAuto = car_left[j].isAuto;
			}
			if(veh_sum_left==1) { //添加此处代码，防止车道只有一辆车的时候，离开之后首辆车位置不变导致车辆不能换道进来
				car_left[0].x=0;
				car_left[0].v=0;
			}
			veh_sum_left--;
		} else {
			index_turn++;//这里表示，若没有掉头，则直接查找下一个，若发生掉头，这里index值不变，因为此时index目标车的后车
		}
	}



	//计算车辆的类型
	for (i = 0; i < veh_sum_left; i++) {
		if (i == 0) {
			car_left[i].gap = Vmax;

			car_left[i].type=3;//首辆车

		} else {
			car_left[i].gap = car_left[i - 1].x - car_left[i].x - veh_length;
			var_x_left[i]=car_left[i-1].x-car_left[i].x;//保存与前车位置差
			front_v_left[i]=car_left[i-1].v;//前车速度

			if(car_left[i].isAuto==0) {

				car_left[i].type=0;//Gipps

			} else if(car_left[i].isAuto==1) {
				if(car_left[i-1].isAuto==0) {
					car_left[i].type=1;//ACC
				} else {
					car_left[i].type=2;//CACC
				}
			} else {
				printf("/////////////////////////////");
			}
		}
		if ((int)car_left[i].gap < 0) {
			printf("左车道间距错误2: 右车道车辆总数：%d 左车道车辆总数：%d 当前车编号：%d  车位置：%d 后车位置：%d 前车位置：%d \n", veh_sum_right, veh_sum_left, i, car_left[i].x, car_left[i + 1].x, car_left[i - 1].x);
		}
	}

	for (i = 0; i < veh_sum_right; i++) {
		if (i == 0) {//最前面一辆车子
			car_right[i].gap = Vmax;

			car_right[i].type=3;

		} else {
			car_right[i].gap = car_right[i - 1].x - car_right[i].x - veh_length;
			var_x_right[i]=car_right[i-1].x-car_right[i].x;
			front_v_right[i]=car_right[i-1].v;//前车速度

			if(car_right[i].isAuto==0) {
				car_right[i].type=0;

			} else if(car_right[i].isAuto==1) {
				if(car_right[i-1].isAuto==0) {
					car_right[i].type=1;
				} else {
					car_right[i].type=2;
				}
			} else {
				printf("////////////////////////////////");
			}
		}

		if ((int)car_right[i].gap < 0) {
			printf("当前右车道车辆间距发生错误2！！！！！！\n");
		}
	}


	/*step4*/
	/*左车道NS模型*/
	for (i = 0; i < veh_sum_left; i++) {
		if(light_time<65&&i==0) {
			car_left[0].gap=road_long_right-car_left[0].x-veh_length;//左车道LC右端不放行
		}

		if(time_step>time_start) {
			osave_left[i]=(int)car_left[i].x;
		}


		if(car_left[i].type==0) {

			safev_left=car_left[i].v;//保存上一时步车辆速度

			Ti=0.8;

			double genhao=bmax*bmax*Ti*Ti+bmax*(2*(var_x_left[i]-veh_length-s0)-car_left[i].v*Ti+front_v_left[i]*front_v_left[i]/(bmax));//////这里不需要乘以2
			Dsafe=car_left[i].v*Ti+car_left[i].v*car_left[i].v/(bmax*2)-front_v_left[i]*front_v_left[i]/(bmax*2);//人工驾驶车辆安全距离


			if(genhao<0) {
				Vsafe=0;
			} else {
				Vsafe=getDoubleMax2(-bmax* Ti+sqrt(genhao),0);//人工驾驶车辆安全速度
			}


			if(car_left[i].gap>Dsafe) {
				car_left[i].v= getDoubleMin4(car_left[i].v+amax,Vmax,Vsafe,car_left[i].gap);

			} else if(car_left[i].gap==Dsafe) {
				//(2)匀速
				car_left[i].v=getDoubleMin3(car_left[i].v,Vsafe,car_left[i].gap);
			}


			//(3)随机减速
			if(((double)rand())/RAND_MAX<p_slow) {
				car_left[i].v=getDoubleMax2(car_left[i].v-b_ease,0);
			}

			//(4)确定性减速
			if(car_left[i].gap<Dsafe) {
				car_left[i].v=getDoubleMax2(getDoubleMin2(Vsafe,car_left[i].gap),0);
			}

		} else if(car_left[i].type==1) { //ACC

			var_v_left[i]=safev_left-car_left[i].v;//前车与当前车速度差（前车为人工驾驶，速度应该是上一时步速度）

			double ramta= ((double)rand())/RAND_MAX;

			Ta=1.5346;


			a_left[i]=k1*(var_x_left[i]-veh_length-s1-Ta*car_left[i].v)+k2*var_v_left[i];
			if(a_left[i]>0) {
				car_left[i].v=getDoubleMin3(car_left[i].v+a_left[i],car_left[i].v+amax,Vmax);
			} else if(a_left[i]<0) {
				car_left[i].v=getDoubleMax2(getDoubleMax2(car_left[i].v+a_left[i],car_left[i].v-bmax),0);
			}

			if(car_left[i].v-car_left[i-1].v>car_left[i].gap-car_left[i].v*Ta) {
				car_left[i].v=(car_left[i].gap+car_left[i-1].v)/(1+Ta);
			}

			if(car_left[i].v<-0.0) {
				car_left[i].v=0;
			}

		} else if(car_left[i].type==2) {

			var_v_left[i]=car_left[i-1].v-car_left[i].v;//前车与当前车速度差

			double ramtc=((double)rand())/RAND_MAX;

			Tc=0.705;

			a_left[i]=j11*a_left[i-1]+j2*(var_x_left[i]-veh_length-s2 -Tc*car_left[i].v)+j3*var_v_left[i];

			if(a_left[i]>0) {
				car_left[i].v=getDoubleMin3(car_left[i].v+a_left[i],car_left[i].v+amax,Vmax);
			} else if(a_left[i]<0) {
				car_left[i].v=getDoubleMax2(getDoubleMax2(car_left[i].v+a_left[i],car_left[i].v-bmax),0);
			}


			if(car_left[i].v-car_left[i-1].v>car_left[i].gap-car_left[i].v*Tc) {
				car_left[i].v=(car_left[i].gap+car_left[i-1].v)/(1+Tc);
			}
			if(car_left[i].v<0) {
				car_left[i].v=0;
			}


		} else if(car_left[i].type==3) {
			safev_left=car_left[i].v;
			car_left[i].v=getDoubleMin3(car_left[i].v+amax,Vmax,car_left[i].gap);
			a_left[i]= car_left[i].v-safev_left;
			
		} else {
			printf("还有别的类型:%d\n",car_left[i].type);
		}

	}

	/*右车道NS模型*/
	for (i = 0; i < veh_sum_right; i++) {
		//信号灯
		if(light_time<65&&i==0) {
			time_red++;
			car_right[0].gap=road_long_right-car_right[0].x-veh_length;//右车道LC右端不放行
		}

		if(time_step>time_start) {
			osave_right[i]=(int)car_right[i].x;
		}

		if(car_right[i].type==0) {

			safev_right= car_right[i].v;

			Ti=0.8;

			double genhao=bmax*bmax*Ti*Ti+bmax*(2*(var_x_right[i]-veh_length-s0)-car_right[i].v*Ti+front_v_right[i]*front_v_right[i]/(bmax));//////这里不需要乘以2
			Dsafe=car_right[i].v*Ti+car_right[i].v*car_right[i].v/(bmax*2)-front_v_right[i]*front_v_right[i]/(bmax*2);//人工驾驶车辆安全距离


			if(genhao<0) {
				Vsafe=0;
			} else {
				Vsafe=getDoubleMax2(-bmax* Ti+sqrt(genhao),0);//人工驾驶车辆安全速度
				//Vsafe=-bmax* Ti+sqrt(genhao);//人工驾驶车辆安全速度
			}
			if(car_right[i].gap>Dsafe) {
				car_right[i].v= getDoubleMin4(car_right[i].v+amax,Vmax,Vsafe,car_right[i].gap);

			} else if(car_right[i].gap==Dsafe) {
				//(2)匀速
				car_right[i].v=getDoubleMin3(car_right[i].v,Vsafe,car_right[i].gap);
			}


			//(3)随机减速
			if(((double)rand())/RAND_MAX<p_slow) {
				car_right[i].v=getDoubleMax2(car_right[i].v-b_ease,0);
			}

			//(4)确定性减速
			if(car_right[i].gap<Dsafe) {
				car_right[i].v=getDoubleMax2(getDoubleMin2(Vsafe,car_right[i].gap),0);
			}


		}	else if(car_right[i].type==1) {
			var_v_right[i]=safev_right-car_right[i].v;//前车与当前车速度差

			double ramta= ((double)rand())/RAND_MAX;

			Ta=1.5346;


			a_right[i]=k1*(var_x_right[i]-veh_length-s1-Ta*car_right[i].v)+k2*var_v_right[i];
			if(a_right[i]>0) {
				car_right[i].v=getDoubleMin3(car_right[i].v+a_right[i],car_right[i].v+amax,Vmax);
			} else if(a_right[i]<0) {
				car_right[i].v=getDoubleMax2(getDoubleMax2(car_right[i].v+a_right[i],car_right[i].v-bmax),0);
			}

			if(car_right[i].v-car_right[i-1].v>car_right[i].gap-car_right[i].v*Ta) {
				car_right[i].v=(car_right[i].gap+car_right[i-1].v)/(1+Ta);
			}

			if(car_right[i].v<-0.0) {
				car_right[i].v=0;
			}


		} else if(car_right[i].type==2) {
			var_v_right[i]=car_right[i-1].v-car_right[i].v;//前车与当前车速度差

			double ramtc=((double)rand())/RAND_MAX;

			Tc=0.705;

			a_right[i]=j11*a_right[i-1]+j2*(var_x_right[i]-veh_length-s2-Tc*car_right[i].v)+j3*var_v_right[i];

			if(a_right[i]>0) {
				car_right[i].v=getDoubleMin3(car_right[i].v+a_right[i],car_right[i].v+amax,Vmax);
			} else if(a_right[i]<0) {
				car_right[i].v=getDoubleMax2(getDoubleMax2(car_right[i].v+a_right[i],car_right[i].v-bmax),0);
			}

			if(car_right[i].v-car_right[i-1].v>car_right[i].gap-car_right[i].v*Tc) {
				car_right[i].v=(car_right[i].gap+car_right[i-1].v)/(1+Tc);
			}
			if(car_right[i].v<0) {
				car_right[i].v=0;
			}


		} else if(car_right[i].type==3) {
			safev_right=car_right[i].v;
			car_right[i].v=getDoubleMin3(car_right[i].v+amax,Vmax,car_right[i].gap);
			a_right[i]= car_right[i].v-safev_right;


		} else {
			printf("还有别的类型！") ;
		}
	}


	for (i = 0; i < veh_sum_left; i++) {
		if (i == 0) {
			car_left[i].gap = Vmax;
		} else {
			car_left[i].gap = car_left[i - 1].x - car_left[i].x - veh_length;
		}
		if ((int)car_left[i].gap < 0) {
			printf("左车道间距错误3: 右车道车辆总数：%d 左车道车辆总数：%d 当前车编号：%d  车位置：%d 后车位置：%d 前车位置：%d \n", veh_sum_right, veh_sum_left, i, car_left[i].x, car_left[i + 1].x, car_left[i - 1].x);
		}
	}
	for (i = 0; i < veh_sum_right; i++) {
		if (i == 0) {//最前面一辆车子
			car_right[i].gap = Vmax;//右车道前面有边界
			//printf("%d\n", car_right[0].gap);//这里间距基本为0
		} else {
			car_right[i].gap = car_right[i - 1].x - car_right[i].x - veh_length;
		}
		if ((int)car_right[i].gap < 0) {
			printf("当前右车道车辆间距发生错误3！！！！！！\n");
		}
	}


	if (time_step >= total_timestep - count_timestep) {
		for (j = 0; j < veh_sum_left; j++) {
			if ((car_left[j].x < test_cell) && ((car_left[j].x + car_left[j].v) >= test_cell)) {//表示车辆满足上下时步经过某个点。
				coil_veh_sum_left++;//线圈车辆数加一
				coil_velosity_sum_left += car_left[j].v;//速度累加

			}
		}
		for (j = 0; j < veh_sum_right; j++) {
			if ((car_right[j].x < test_cell) && ((car_right[j].x + car_right[j].v) >= test_cell)) {//表示车辆满足上下时步经过某个点。
				coil_veh_sum_right++;//线圈车辆数加一
				coil_velosity_sum_right += car_right[j].v;//速度累加

			}
		}
	}


//位置更新
	for (i = 0; i < veh_sum_left; i++) {
		/*左车道*/
		car_left[i].x = car_left[i].x + car_left[i].v;

	}
	for (i = 0; i < veh_sum_right; i++) {
		/*右车道*/
		car_right[i].x = car_right[i].x + car_right[i].v;
	}

	for (i = 0; i < veh_sum_left; i++) {
		if (i == 0) {
			car_left[i].gap = Vmax;
		} else {
			car_left[i].gap = car_left[i - 1].x - car_left[i].x - veh_length;
		}
		if ((int)car_left[i].gap < 0) {
			printf("左车道间距错误4: 右车道车辆总数：%d 左车道车辆总数：%d 当前车编号：%d  车位置：%f 后车位置：%f 前车位置：%f \n", veh_sum_right, veh_sum_left, i, car_left[i].x, car_left[i + 1].x, car_left[i - 1].x);
		}
	}
	for (i = 0; i < veh_sum_right; i++) {
		if (i == 0) {//最前面一辆车子
			car_right[i].gap = Vmax;//右车道前面有边界
			//printf("%d\n", car_right[0].gap);//这里间距基本为0
		} else {
			car_right[i].gap = car_right[i - 1].x - car_right[i].x - veh_length;
		}
		if ((int)car_right[i].gap < 0) {
			printf("右车道间距错误4: 右车道车辆总数：%d 左车道车辆总数：%d 当前车编号：%d  车位置：%f 后车位置：%f 前车位置：%f \n", veh_sum_right, veh_sum_left, i, car_right[i].x, car_right[i + 1].x, car_right[i - 1].x);

		}
	}

	return 0;
}


//左道出车
void car_out(CARL* car_left,CARR* car_right) {
	while (car_left[0].x > road_long_right-veh_length){
	

		//printf("当前车辆总数：%d,头车位置：%d,头车速度：%d\n", veh_sum_left,car_left[0].x,car_left[0].v);
		for (i = 1; i < veh_sum_left; i++) {
			car_left[i - 1].v = car_left[i].v;
			car_left[i - 1].x = car_left[i].x;
			car_left[i - 1].isAuto = car_left[i].isAuto;//此处需添加
		}
		if(veh_sum_left>0) {
			if(veh_sum_left==1) {
				car_left[0].x=0;
			}
			veh_sum_left--;
		}
	}


	while (car_right[0].x > road_long_right-veh_length) {
		outNum_right++;
		//printf("当前车辆总数：%d,头车位置：%d,头车速度：%d\n", veh_sum_left,car_left[0].x,car_left[0].v);
		for (i = 1; i < veh_sum_right; i++) {
			car_right[i - 1].v = car_right[i].v;
			car_right[i - 1].x = car_right[i].x;
			car_right[i - 1].isAuto = car_right[i].isAuto;//此处需添加
		}
		if(veh_sum_right>0) {
			if(veh_sum_right==1) {
				car_right[0].x=0;
			}
			veh_sum_right--;
		}

	}

}


//主函数
int main() {
	CARL* car_left;
	car_left = (CARL*)malloc(sizeof(CARL) * 11000);
	CARR* car_right;
	car_right = (CARR*)malloc(sizeof(CARR) * 11000);

	r_pro=0;//先不加入自动驾驶车辆

	sample=1;

	p_change=0;

	Qin=0.4;

	FILE* fp1;
	if ((fp1 = fopen("5Qin=0.40 左车道 统计位置：1300.txt", "a+")) == NULL) {
		printf("cannot open this file.\n");
		exit(0);
	}
	FILE* fp2;
	if ((fp2 = fopen("5Qin=0.40 右车道 统计位置：1300.txt", "a+")) == NULL) {
		printf("cannot open this file.\n");
		exit(0);
	}

	srand((unsigned)time(NULL));		//随机数种子


	printf("当前入车概率：%f 正在计算。。。\n",Qin);

	coil_veh_sum_right = 0;
	coil_velosity_sum_right = 0;
	coil_veh_sum_left = 0;
	coil_velosity_sum_left = 0;
	LB_change_num=0;
	time_red=0;
	outNum_left=0;
	outNum_right=0;



	//printf("当前样本：%d 正在计算...\n",sam);


	//道路初始化
	veh_sum_left = 0;
	vel_sum_left = 0;
	veh_sum_right = 0;
	vel_sum_right = 0;

	for (i = 0; i < 11000; i++) {
		car_left[i].x = 0;
		car_left[i].v = 0;
		car_left[i].gap = 0;
		car_left[i].position = 0;
		car_left[i].gap_back = -1;
		car_left[i].gap_other = -1;
		car_left[i].vob = 0;
		car_left[i].isAuto=-1;
		car_left[i].type=-1;

		car_right[i].x = 0;
		car_right[i].v = 0;
		car_right[i].gap = 0;
		car_right[i].position = 0;
		car_right[i].gap_back = -1;
		car_right[i].gap_other = -1;
		car_right[i].vob = 0;
		car_right[i].isAuto=-1;
		car_right[i].type=-1;
	}



	for (time_step = 0; time_step < total_timestep; time_step++) {

		//主干道进车
		enterMainRoad(car_left, car_right);

		//车速更新
		velocitySum(car_left, car_right);

		if(time_step==time_start) { //到达统计时间

			while(car_right[ro].x>position_start) {
				ro++;//即将进入7000的第一辆车
			}
			for(i=0; i<20; i++) {
				orbit[i]=(int)car_right[ro+i].x;
				printf("0时步：%d  轨迹统计第%d辆车位置：%d ro:%d 是否左车道:%d\n",time_step,i,orbit[i],ro,isleft[i]);
			}

		} else if(time_step>time_start) {

			for(i=0; i<20; i++) {
				if(isleft[i]==1) {
					for(j=0; j<veh_sum_left; j++) {
						if(orbit[i]==osave_left[j]) { //j为当前序号
							//printf("车辆找到 时步：%d 左车道轨迹车队第%d辆位置：%d 道路序号：%d 道路中位置：%d 速度：%d\n",time_step,i,orbit[i],j,osave_left[j],(int)car_left[j].v);
							orbit[i]= (int)car_left[j].x;
							
							if(car_left[j].x<road_long_right-veh_length){
								fprintf(fp1,"%d, %d\n",time_step,(int)car_left[j].x);
							}
						}
					}
				} else if(isleft[i]==0) {
					for(j=0; j<veh_sum_right; j++) {
						if(orbit[i]==osave_right[j]) { //j为当前序号
							//printf("车辆找到 时步：%d 右车道轨迹车队第%d辆位置：%d 道路序号：%d 速度：%d\n",time_step,i,orbit[i],j,(int)car_right[j].v);
							orbit[i]= (int)car_right[j].x;
							if(car_right[j].x<road_long_right-veh_length){
								fprintf(fp2,"%d, %d\n",time_step,(int)car_right[j].x);
							}
						}
					}
				} else {
					printf("还有别的类型");
				}
			}

			//第三步：更新之后
			if(time_step>time_start) { //统计时间开始过后
				for(i=0; i<20; i++) { //左车道标记车辆遍历

					if(orbit[i]<1492) {
						//printf("3时步：%d  轨迹统计第%d辆车位置：%d  是否左车道:%d 左车道车辆数：%d\n",time_step%110,i,orbit[i],isleft[i],veh_sum_left);

						if(isleft[i]==1) {
							//fprintf(fp1,"%d, %d\n",time_step,orbit[i]);
						} else {
							//fprintf(fp2,"%d, %d\n",time_step,orbit[i]);
						}
					}

				}

				//printf("\n");
			}
		}

		//出车系统
		car_out(car_left,car_right);


	}


	fclose(fp1);
	fclose(fp2);


	return 0;

}
