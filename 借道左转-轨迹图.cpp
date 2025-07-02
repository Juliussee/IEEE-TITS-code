/*
�ı�����仯��� ��
	������̣����ҳ��������������ҳ������Ӹ���
	����LC���󳵵��������ͣ��ҳ�����������
	���ӵ�ͷ�ʣ����ҳ�����������
	�ı�����ٶȣ��仯������
	����LB���󳵵������������ҳ����������ͣ���ֵ��С

	������������15000 �󳵵�������0.095 �ҳ���������0.108
	����ʵ�����ݵĲ�����LC=40��LB=30��Vmax=20, p_change=0.2,car_right[i].isAuto==0&&(car_right[i].gap - car_right[i].gap_other) <= 1 && car_right[i].gap_back > car_right[i].vob+ getDoubleMin2(car_right[i].vob+1,Vmax) -Vmin_right
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

#define road_long_right 1500		//·�����񣩡�����Ϊ2000 
#define Vmax 20		//����ٶ�
#define veh_length 7.5		//��������
#define L_cell 1.5			//һ��Ԫ��ʵ�ʳ���(m) 
#define total_timestep 11150
#define count_timestep 2000


double LC = 100;
double LB = 30;


double LA = road_long_right - LB-LC;//��ס��LA�������Ҫ��LB��ֵ֮��

int i, j, k, veh_sum_save, randnumber, x, e;
double occupancy;
double density;
int sample;
int time_step;
double veh_init_gap;  //��·��ʼ�����
//�Ľ�����
double p_add;
double p_slow=0.3;
double Varr;
double m2=1.3;
double m3=100;
double m1;
double pf=0.55;

//���ɵ�ģ��
int veh_sum_left;				//�󳵵���������
double vel_sum_left;				//�󳵵��ٶȺ�
int veh_sum_right;				//�ҳ�����������
double vel_sum_right;				//�ҳ����ٶȺ�
int test_cell=road_long_right-40;//��ʾ����
double Qin;
double dsafe = 20;
double left_num_save[road_long_right] = { -1 };
double right_num_save[road_long_right] = { -1 };
int outNum_left = 0;//��¼������
int outNum_right = 0;//��¼������
int coil_time_gap = 60; //��Ȧ������
int LB_change_num = 0;//������Ŀ
int time_red=0;//���ʱ��


double vleft_save[road_long_right]= {-1}; //�����ٶ�
double vright_save[road_long_right]= {-1};
double h_con=9;//��ϸ����ֵ
double gapsafe=10.5;//��ϸ����ֵ
double vanti;
double pb=0.94;//��Ϊ��������
double p0=0.5;
double pd=0.1;
double tc=10;
double th_left;
double ts_left;
double th_right;
double ts_right;
int LCsum_left;//�󳵵�LC��������
int LCsum_right;
double temp;//�󳵵��Ŷӳ�����lc��ֵ
double pd_change;
int light_time;//��ǰʱ����215���������

double velocity_sume_left = 0;//ͳ���ٶ�
double velocity_sume_right = 0;
int coil_veh_sum_right = 0;
double coil_velosity_sum_right = 0;
int coil_veh_sum_left = 0;
double coil_velosity_sum_left = 0;

//CACC
double r_pro;//�Զ���ʻ����ռ��

double amax=2;//���ٶ�
double b_ease=2;//���ʼ��ٶ�
double bmax=3;//�����ٶ�(��λΪm/s)

double Ti;//gipps����ģ���з�Ӧʱ��
double Ta;//ACC���������
double Tc;//CACC���������
double k1=0.23;
double k2=0.07;
double j11=1.0;
double j2=0.2;
double j3=0.3;

double s0=1;
double s1=0;
double s2=0;

double front_v_left[2000];//����ǰ���ٶ�
double front_a_left[2000];//����ǰ�����ٶ�
double var_v_left[2000];//�ٶȱ仯��
double var_x_left[2000];//λ�Ʊ仯��
double front_v_right[2000];//����ǰ���ٶ�
double front_a_right[2000];//����ǰ�����ٶ�
double var_v_right[2000];//�ٶȱ仯��
double var_x_right[2000];//λ�Ʊ仯��

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
double p_change;//��������

int orbit_sum=20;
int orbit[20]= {0}; //ÿһ������λ��
int isleft[20]= {0}; // =0��ʾ�������ҳ�����=1��ʾ�������󳵵�
int lo=0;//���ҳ���׼��ͳ�Ƶı��
int ro=0;
int time_start = 11000;//��ʼʱ��
int position_start=1100;//ͳ��λ��
int osave_left[2000]= {0};//���ҳ��������ٶ�
int osave_right[2000]= {0};


typedef struct car_left {
	double x;
	double v;
	double vob;
	double gap;
	double gap_other;
	double gap_back;
	int position;
	int isinsert;//�������Ƿ�ɲ���
	int isAuto;
	int type;//0��ʾgipps���£�1��ʾACC���£�2��ʾCACC���� 3��ʾ�������ֶ� 4��ʾ�������Զ�
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
	int type;//0��ʾgipps���£�1��ʾACC���£�2��ʾCACC���� 3��ʾ�������ֶ� 4��ʾ�������Զ�
} CARR;

//������������Сֵ
int getMin2(int m, int n) {
	if (m < n) {
		return m;
	} else {
		return n;
	}
}

//�������������ֵ
int getMax2(int m, int n) {
	if (m < n) {
		return n;
	} else {
		return m;
	}
}

//����������Сֵ
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

//�������������ֵ
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

//�ҳ�������
void enterMainRoad(CARL* car_left, CARR* car_right) {
	double randomNum_qin2 = ((double)rand()) / RAND_MAX;
	double auto_veh2=((double)rand()) / RAND_MAX;
	if (veh_sum_right == 0) {

		veh_sum_right++;
		car_right[veh_sum_right - 1].v = 0;
		car_right[veh_sum_right - 1].x = rand() % (int)Vmax;//�Գ�β������ȷ������λ��,�����״ν���
		car_right[veh_sum_right - 1].isinsert = 0;
		car_right[veh_sum_right - 1].isAuto=0;//��ʾ�ֶ�


	} else if (randomNum_qin2 < Qin) {

		if(car_right[veh_sum_right - 1].x>Vmax - 1.5) {

			veh_sum_right++;
			car_right[veh_sum_right - 1].v = Vmax;
			car_right[veh_sum_right - 1].x = getDoubleMin2(car_right[veh_sum_right - 2].x - Vmax, Vmax);
			car_right[veh_sum_right - 1].isinsert = 0;

			if(auto_veh2<r_pro) {
				car_right[veh_sum_right - 1].isAuto=1;//��ʾ�Զ�

			} else {
				car_right[veh_sum_right - 1].isAuto=0;//��ʾ�ֶ�
			}

			//printf("��ǰ����time:%d ���ʱ�䣺%d ������%d �ҳ��� ��%d �������%d �ҳ�����%d ������ֵ��%d ��0����%.3f ��0����%.3f\n",time_step,time_red,veh_sum_left,veh_sum_right,outNum_left,outNum_right,outNum_right-outNum_left,car_left[0].x,car_right[0].x);
		}
	}

}


/*
tep1:����gap_other,gap_back,gap,vob�ֱ��ʾ���ڳ���ǰ����룬�͵�ǰ����ǰ�����,��һ�������泵����
*step2:�ҳ����з��ϻ�������ĳ���
*step3:��������
*step4�����ҳ�������NS����任���󳵵��ɳ������ҳ������ɡ�
*/
int velocitySum(CARL* car_left, CARR* car_right) {

	//���г�ʼ������
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
	/*�����󳵵�gap_other, gap_back, gap, vob*/
	int LABlast = -1;//�����ҳ���LB�������һ����
	LCsum_left=0;
	LCsum_right=0;

	//������
	for (i = 0; i < veh_sum_left; i++) {
		if (i == 0) {//��ǰ��һ������
			car_left[i].gap = Vmax;
		} else {
			car_left[i].gap = car_left[i - 1].x - car_left[i].x - veh_length;
		}
		if((int)car_left[i].gap<0) {
			printf("�󳵵�����֮ǰС��0 ��ǰʱ����%d ����������%d ��ࣺ%f \n",time_step,veh_sum_left,car_left[i].gap);
		}
		if(car_left[i].x>LA+LB-veh_length) {
			LCsum_left++;
		}
	}
	//printf("����������%d LC��������%d LB�����һ������λ�ã�%d �ٶȣ�%d\n",veh_sum_left,LCsum_left,car_left[LCsum_left].x,car_left[LCsum_left].v);

	for (i = 0; i < veh_sum_right; i++) {
		if (i == 0) {//��ǰ��һ������
			car_right[i].gap = Vmax;//�ҳ���ǰ���б߽�
		} else {
			car_right[i].gap = car_right[i - 1].x - car_right[i].x - veh_length;
		}
		if(car_right[i].x>=LA) {
			LABlast++;//LB��������˳���
		}
		if(car_right[i].x>LA+LB-veh_length) {
			LCsum_right++;//LB�������Ҷ˳�������LC����������
		}
		if((int)car_right[i].gap<0) {
			printf("�ҳ�������֮ǰС��0����ǰʱ����%d ���Ϊ��%d λ��Ϊ��%f ���Ϊ��%f �ٶ�Ϊ��%f ǰ��λ��Ϊ��%f\n",time_step,i,car_right[i].x,car_right[i].gap,car_right[i].v,car_right[i-1].x);
		}
	}

	/*�����ҳ���gap_other, gap_back, gap, vob*/
	for (i = 0; i <= LABlast; i++) {
		if (car_right[i].x > car_left[0].x) {//��������1�����������ҳ����������󳵵���һ����ǰ��
			car_right[i].gap_other = Vmax;
			car_right[i].gap_back = car_right[i].x - car_left[0].x - veh_length;
			car_right[i].vob = car_left[0].v;
			//printf("abfirst:%d other:%d back:%d vob:%d\n", LABfirst, car_right[i].gap_other, car_right[i].gap_back, car_right[i].vob);
		} else if (car_right[i].x < car_left[veh_sum_left - 1].x) { //��������2�����������ҳ����������󳵵����һ��������
			car_right[i].gap_other = car_left[veh_sum_left - 1].x - car_right[i].x - veh_length;
			car_right[i].gap_back = car_right[i].x-LA;
			car_right[i].vob = -1;
		} else {
			for (j = 0; j < veh_sum_left; j++) {
				if (car_right[i].x == car_left[j].x) {//��������3�������ڳ����г�����ֱ�ӷ���-1
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
	/*�жϳ�����Ҫ�����ĳ���*/
	light_time=time_step%110;
	for (i = 0; i <= veh_sum_right; i++) {
		if (LA <= car_right[i].x && car_right[i].x < LA+LB) { //�ҳ�LB�����ҳ������㻻��������ʹ�ò���isinsert==1����ʾ

			double Vmin_right=getDoubleMin2(car_right[i].v+1, Vmax);

			double Dsafe_right=getDoubleMin2(Vmax,car_right[i].vob+amax);

			if (car_right[i].isAuto==1&&(car_right[i].gap - car_right[i].gap_other) <= 1 && car_right[i].gap_back > car_right[i].vob+ getDoubleMin2(car_right[i].vob+1,Vmax) -Vmin_right) {

				car_right[i].isinsert = 1;//�Զ���ʻ��������

				printf("test");

			} else if(car_right[i].isAuto==0&&(car_right[i].gap - car_right[i].gap_other) <= 1 && car_right[i].gap_back > car_right[i].vob+ getDoubleMin2(car_right[i].vob+1,Vmax) -Vmin_right) {

				car_right[i].isinsert = 1;//�˹���ʻ��������

			} else {

				car_right[i].isinsert = 0;
			}

			if(light_time<50||light_time>=110) {//Ԥ�źŵƿ���

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
	/*�ҳ������󳵵�*/
	for (i = 0; i <= LABlast;) {//����sum_right��ֵӦ�øı�
		if (car_right[i].isinsert == 1) {

			if(time_step>time_start) { //ͳ��ʱ�俪ʼ����
				for(j=0; j<20; j++) {
					if(orbit[j]==(int)car_right[i].x&&isleft[j]==0) { //����λ���ҳ�������λ����ͬ
						isleft[j]=1;//��ʾ��������
					}
				}
			}

			LB_change_num++;//����������

			int rightNum = -2;
			if (car_right[i].x > car_left[0].x) {
				rightNum = 0 ;//����λ�����󳵵���һ����ǰ��
			} else if (car_right[i].x < car_left[veh_sum_left - 1].x) {
				rightNum = veh_sum_left;//����λ�����󳵵����һ��������
			} else {
				for (j = 0; j < veh_sum_left; j++) {
					if (car_right[i].x<car_left[j].x && car_right[i].x>car_left[j + 1].x) {//
						rightNum = j + 1;
					}
				}
			}
			/*�󳵵����*/
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
			//printf("�ҳ������뵽�󳵵�λ�ã�%d  ���복��һ��λ�ã�%d ���복ǰһ��λ�ã�%d\n", car_left[rightNum].x, car_left[rightNum+1].x,car_left[rightNum-1].x);
			veh_sum_left++;
			/*�ҳ������*/
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


	//����֮����е�ͷ����
	int index_turn=LCsum_left;//�󳵵�����
	while(index_turn<veh_sum_left) { //��ʾ������LA��LB����
		if(((double)rand()) / RAND_MAX<p_change) { //���㻻������
			if(time_step>time_start){
				printf("///////////////////////////////////////��ǰʱ����%d λ�ã�%d ������ͷ",time_step,(int)car_left[index_turn].x);
			}
			for (j = index_turn + 1; j <= veh_sum_left - 1; j++) {//����ÿһ������ǰ�ƶ�
				car_left[j - 1].x = car_left[j].x;
				car_left[j - 1].v = car_left[j].v;
				car_left[j - 1].isinsert = car_left[j].isinsert;
				car_left[j - 1].isAuto = car_left[j].isAuto;
			}
			if(veh_sum_left==1) { //��Ӵ˴����룬��ֹ����ֻ��һ������ʱ���뿪֮��������λ�ò��䵼�³������ܻ�������
				car_left[0].x=0;
				car_left[0].v=0;
			}
			veh_sum_left--;
		} else {
			index_turn++;//�����ʾ����û�е�ͷ����ֱ�Ӳ�����һ������������ͷ������indexֵ���䣬��Ϊ��ʱindexĿ�공�ĺ�
		}
	}



	//���㳵��������
	for (i = 0; i < veh_sum_left; i++) {
		if (i == 0) {
			car_left[i].gap = Vmax;

			car_left[i].type=3;//������

		} else {
			car_left[i].gap = car_left[i - 1].x - car_left[i].x - veh_length;
			var_x_left[i]=car_left[i-1].x-car_left[i].x;//������ǰ��λ�ò�
			front_v_left[i]=car_left[i-1].v;//ǰ���ٶ�

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
			printf("�󳵵�������2: �ҳ�������������%d �󳵵�����������%d ��ǰ����ţ�%d  ��λ�ã�%d ��λ�ã�%d ǰ��λ�ã�%d \n", veh_sum_right, veh_sum_left, i, car_left[i].x, car_left[i + 1].x, car_left[i - 1].x);
		}
	}

	for (i = 0; i < veh_sum_right; i++) {
		if (i == 0) {//��ǰ��һ������
			car_right[i].gap = Vmax;

			car_right[i].type=3;

		} else {
			car_right[i].gap = car_right[i - 1].x - car_right[i].x - veh_length;
			var_x_right[i]=car_right[i-1].x-car_right[i].x;
			front_v_right[i]=car_right[i-1].v;//ǰ���ٶ�

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
			printf("��ǰ�ҳ���������෢������2������������\n");
		}
	}


	/*step4*/
	/*�󳵵�NSģ��*/
	for (i = 0; i < veh_sum_left; i++) {
		if(light_time<65&&i==0) {
			car_left[0].gap=road_long_right-car_left[0].x-veh_length;//�󳵵�LC�Ҷ˲�����
		}

		if(time_step>time_start) {
			osave_left[i]=(int)car_left[i].x;
		}


		if(car_left[i].type==0) {

			safev_left=car_left[i].v;//������һʱ�������ٶ�

			Ti=0.8;

			double genhao=bmax*bmax*Ti*Ti+bmax*(2*(var_x_left[i]-veh_length-s0)-car_left[i].v*Ti+front_v_left[i]*front_v_left[i]/(bmax));//////���ﲻ��Ҫ����2
			Dsafe=car_left[i].v*Ti+car_left[i].v*car_left[i].v/(bmax*2)-front_v_left[i]*front_v_left[i]/(bmax*2);//�˹���ʻ������ȫ����


			if(genhao<0) {
				Vsafe=0;
			} else {
				Vsafe=getDoubleMax2(-bmax* Ti+sqrt(genhao),0);//�˹���ʻ������ȫ�ٶ�
			}


			if(car_left[i].gap>Dsafe) {
				car_left[i].v= getDoubleMin4(car_left[i].v+amax,Vmax,Vsafe,car_left[i].gap);

			} else if(car_left[i].gap==Dsafe) {
				//(2)����
				car_left[i].v=getDoubleMin3(car_left[i].v,Vsafe,car_left[i].gap);
			}


			//(3)�������
			if(((double)rand())/RAND_MAX<p_slow) {
				car_left[i].v=getDoubleMax2(car_left[i].v-b_ease,0);
			}

			//(4)ȷ���Լ���
			if(car_left[i].gap<Dsafe) {
				car_left[i].v=getDoubleMax2(getDoubleMin2(Vsafe,car_left[i].gap),0);
			}

		} else if(car_left[i].type==1) { //ACC

			var_v_left[i]=safev_left-car_left[i].v;//ǰ���뵱ǰ���ٶȲǰ��Ϊ�˹���ʻ���ٶ�Ӧ������һʱ���ٶȣ�

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

			var_v_left[i]=car_left[i-1].v-car_left[i].v;//ǰ���뵱ǰ���ٶȲ�

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
			printf("���б������:%d\n",car_left[i].type);
		}

	}

	/*�ҳ���NSģ��*/
	for (i = 0; i < veh_sum_right; i++) {
		//�źŵ�
		if(light_time<65&&i==0) {
			time_red++;
			car_right[0].gap=road_long_right-car_right[0].x-veh_length;//�ҳ���LC�Ҷ˲�����
		}

		if(time_step>time_start) {
			osave_right[i]=(int)car_right[i].x;
		}

		if(car_right[i].type==0) {

			safev_right= car_right[i].v;

			Ti=0.8;

			double genhao=bmax*bmax*Ti*Ti+bmax*(2*(var_x_right[i]-veh_length-s0)-car_right[i].v*Ti+front_v_right[i]*front_v_right[i]/(bmax));//////���ﲻ��Ҫ����2
			Dsafe=car_right[i].v*Ti+car_right[i].v*car_right[i].v/(bmax*2)-front_v_right[i]*front_v_right[i]/(bmax*2);//�˹���ʻ������ȫ����


			if(genhao<0) {
				Vsafe=0;
			} else {
				Vsafe=getDoubleMax2(-bmax* Ti+sqrt(genhao),0);//�˹���ʻ������ȫ�ٶ�
				//Vsafe=-bmax* Ti+sqrt(genhao);//�˹���ʻ������ȫ�ٶ�
			}
			if(car_right[i].gap>Dsafe) {
				car_right[i].v= getDoubleMin4(car_right[i].v+amax,Vmax,Vsafe,car_right[i].gap);

			} else if(car_right[i].gap==Dsafe) {
				//(2)����
				car_right[i].v=getDoubleMin3(car_right[i].v,Vsafe,car_right[i].gap);
			}


			//(3)�������
			if(((double)rand())/RAND_MAX<p_slow) {
				car_right[i].v=getDoubleMax2(car_right[i].v-b_ease,0);
			}

			//(4)ȷ���Լ���
			if(car_right[i].gap<Dsafe) {
				car_right[i].v=getDoubleMax2(getDoubleMin2(Vsafe,car_right[i].gap),0);
			}


		}	else if(car_right[i].type==1) {
			var_v_right[i]=safev_right-car_right[i].v;//ǰ���뵱ǰ���ٶȲ�

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
			var_v_right[i]=car_right[i-1].v-car_right[i].v;//ǰ���뵱ǰ���ٶȲ�

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
			printf("���б�����ͣ�") ;
		}
	}


	for (i = 0; i < veh_sum_left; i++) {
		if (i == 0) {
			car_left[i].gap = Vmax;
		} else {
			car_left[i].gap = car_left[i - 1].x - car_left[i].x - veh_length;
		}
		if ((int)car_left[i].gap < 0) {
			printf("�󳵵�������3: �ҳ�������������%d �󳵵�����������%d ��ǰ����ţ�%d  ��λ�ã�%d ��λ�ã�%d ǰ��λ�ã�%d \n", veh_sum_right, veh_sum_left, i, car_left[i].x, car_left[i + 1].x, car_left[i - 1].x);
		}
	}
	for (i = 0; i < veh_sum_right; i++) {
		if (i == 0) {//��ǰ��һ������
			car_right[i].gap = Vmax;//�ҳ���ǰ���б߽�
			//printf("%d\n", car_right[0].gap);//���������Ϊ0
		} else {
			car_right[i].gap = car_right[i - 1].x - car_right[i].x - veh_length;
		}
		if ((int)car_right[i].gap < 0) {
			printf("��ǰ�ҳ���������෢������3������������\n");
		}
	}


	if (time_step >= total_timestep - count_timestep) {
		for (j = 0; j < veh_sum_left; j++) {
			if ((car_left[j].x < test_cell) && ((car_left[j].x + car_left[j].v) >= test_cell)) {//��ʾ������������ʱ������ĳ���㡣
				coil_veh_sum_left++;//��Ȧ��������һ
				coil_velosity_sum_left += car_left[j].v;//�ٶ��ۼ�

			}
		}
		for (j = 0; j < veh_sum_right; j++) {
			if ((car_right[j].x < test_cell) && ((car_right[j].x + car_right[j].v) >= test_cell)) {//��ʾ������������ʱ������ĳ���㡣
				coil_veh_sum_right++;//��Ȧ��������һ
				coil_velosity_sum_right += car_right[j].v;//�ٶ��ۼ�

			}
		}
	}


//λ�ø���
	for (i = 0; i < veh_sum_left; i++) {
		/*�󳵵�*/
		car_left[i].x = car_left[i].x + car_left[i].v;

	}
	for (i = 0; i < veh_sum_right; i++) {
		/*�ҳ���*/
		car_right[i].x = car_right[i].x + car_right[i].v;
	}

	for (i = 0; i < veh_sum_left; i++) {
		if (i == 0) {
			car_left[i].gap = Vmax;
		} else {
			car_left[i].gap = car_left[i - 1].x - car_left[i].x - veh_length;
		}
		if ((int)car_left[i].gap < 0) {
			printf("�󳵵�������4: �ҳ�������������%d �󳵵�����������%d ��ǰ����ţ�%d  ��λ�ã�%f ��λ�ã�%f ǰ��λ�ã�%f \n", veh_sum_right, veh_sum_left, i, car_left[i].x, car_left[i + 1].x, car_left[i - 1].x);
		}
	}
	for (i = 0; i < veh_sum_right; i++) {
		if (i == 0) {//��ǰ��һ������
			car_right[i].gap = Vmax;//�ҳ���ǰ���б߽�
			//printf("%d\n", car_right[0].gap);//���������Ϊ0
		} else {
			car_right[i].gap = car_right[i - 1].x - car_right[i].x - veh_length;
		}
		if ((int)car_right[i].gap < 0) {
			printf("�ҳ���������4: �ҳ�������������%d �󳵵�����������%d ��ǰ����ţ�%d  ��λ�ã�%f ��λ�ã�%f ǰ��λ�ã�%f \n", veh_sum_right, veh_sum_left, i, car_right[i].x, car_right[i + 1].x, car_right[i - 1].x);

		}
	}

	return 0;
}


//�������
void car_out(CARL* car_left,CARR* car_right) {
	while (car_left[0].x > road_long_right-veh_length){
	

		//printf("��ǰ����������%d,ͷ��λ�ã�%d,ͷ���ٶȣ�%d\n", veh_sum_left,car_left[0].x,car_left[0].v);
		for (i = 1; i < veh_sum_left; i++) {
			car_left[i - 1].v = car_left[i].v;
			car_left[i - 1].x = car_left[i].x;
			car_left[i - 1].isAuto = car_left[i].isAuto;//�˴������
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
		//printf("��ǰ����������%d,ͷ��λ�ã�%d,ͷ���ٶȣ�%d\n", veh_sum_left,car_left[0].x,car_left[0].v);
		for (i = 1; i < veh_sum_right; i++) {
			car_right[i - 1].v = car_right[i].v;
			car_right[i - 1].x = car_right[i].x;
			car_right[i - 1].isAuto = car_right[i].isAuto;//�˴������
		}
		if(veh_sum_right>0) {
			if(veh_sum_right==1) {
				car_right[0].x=0;
			}
			veh_sum_right--;
		}

	}

}


//������
int main() {
	CARL* car_left;
	car_left = (CARL*)malloc(sizeof(CARL) * 11000);
	CARR* car_right;
	car_right = (CARR*)malloc(sizeof(CARR) * 11000);

	r_pro=0;//�Ȳ������Զ���ʻ����

	sample=1;

	p_change=0;

	Qin=0.4;

	FILE* fp1;
	if ((fp1 = fopen("5Qin=0.40 �󳵵� ͳ��λ�ã�1300.txt", "a+")) == NULL) {
		printf("cannot open this file.\n");
		exit(0);
	}
	FILE* fp2;
	if ((fp2 = fopen("5Qin=0.40 �ҳ��� ͳ��λ�ã�1300.txt", "a+")) == NULL) {
		printf("cannot open this file.\n");
		exit(0);
	}

	srand((unsigned)time(NULL));		//���������


	printf("��ǰ�복���ʣ�%f ���ڼ��㡣����\n",Qin);

	coil_veh_sum_right = 0;
	coil_velosity_sum_right = 0;
	coil_veh_sum_left = 0;
	coil_velosity_sum_left = 0;
	LB_change_num=0;
	time_red=0;
	outNum_left=0;
	outNum_right=0;



	//printf("��ǰ������%d ���ڼ���...\n",sam);


	//��·��ʼ��
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

		//���ɵ�����
		enterMainRoad(car_left, car_right);

		//���ٸ���
		velocitySum(car_left, car_right);

		if(time_step==time_start) { //����ͳ��ʱ��

			while(car_right[ro].x>position_start) {
				ro++;//��������7000�ĵ�һ����
			}
			for(i=0; i<20; i++) {
				orbit[i]=(int)car_right[ro+i].x;
				printf("0ʱ����%d  �켣ͳ�Ƶ�%d����λ�ã�%d ro:%d �Ƿ��󳵵�:%d\n",time_step,i,orbit[i],ro,isleft[i]);
			}

		} else if(time_step>time_start) {

			for(i=0; i<20; i++) {
				if(isleft[i]==1) {
					for(j=0; j<veh_sum_left; j++) {
						if(orbit[i]==osave_left[j]) { //jΪ��ǰ���
							//printf("�����ҵ� ʱ����%d �󳵵��켣���ӵ�%d��λ�ã�%d ��·��ţ�%d ��·��λ�ã�%d �ٶȣ�%d\n",time_step,i,orbit[i],j,osave_left[j],(int)car_left[j].v);
							orbit[i]= (int)car_left[j].x;
							
							if(car_left[j].x<road_long_right-veh_length){
								fprintf(fp1,"%d, %d\n",time_step,(int)car_left[j].x);
							}
						}
					}
				} else if(isleft[i]==0) {
					for(j=0; j<veh_sum_right; j++) {
						if(orbit[i]==osave_right[j]) { //jΪ��ǰ���
							//printf("�����ҵ� ʱ����%d �ҳ����켣���ӵ�%d��λ�ã�%d ��·��ţ�%d �ٶȣ�%d\n",time_step,i,orbit[i],j,(int)car_right[j].v);
							orbit[i]= (int)car_right[j].x;
							if(car_right[j].x<road_long_right-veh_length){
								fprintf(fp2,"%d, %d\n",time_step,(int)car_right[j].x);
							}
						}
					}
				} else {
					printf("���б������");
				}
			}

			//������������֮��
			if(time_step>time_start) { //ͳ��ʱ�俪ʼ����
				for(i=0; i<20; i++) { //�󳵵���ǳ�������

					if(orbit[i]<1492) {
						//printf("3ʱ����%d  �켣ͳ�Ƶ�%d����λ�ã�%d  �Ƿ��󳵵�:%d �󳵵���������%d\n",time_step%110,i,orbit[i],isleft[i],veh_sum_left);

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

		//����ϵͳ
		car_out(car_left,car_right);


	}


	fclose(fp1);
	fclose(fp2);


	return 0;

}
