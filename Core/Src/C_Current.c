/*
 * C_Current.c
 *
 *  Created on: Apr 8, 2020
 *      Author: faiz
 */
#include "main.h"
#include "stm32f2xx_hal.h"
#include "Control_Init.h"
#include "can.h"
//#define Battery_Capacity 22

//float outNH=-0.05, outNB=-0.025, outNM=-0.0125, outNS=-0.0075, outZ=0, outPS=0.0075, outPM=0.0125, outPB=0.025, outPH=0.05;
float outNH=-0.05, outNB=-0.025, outNM=-0.01, outNS=-0.005, outZ=0, outPS=0.005, outPM=0.01, outPB=0.025, outPH=0.05;
extern float Current_Charger;
float 	SetPoint_CC;
float	CC_Value;

//test git2
void Constant_Current()
{
	CC_Value = 0.3*BPack_Capacity;

	if(Batt_SOC.m_uint16t <= 15)
		SetPoint_CC = 0.5*CC_Value;
	else {
		if(flag_Derating==1)
			SetPoint_CC = 0.5*CC_Value;
		else {
			if(BPack_Temp <= 10)
				SetPoint_CC = 0.5*CC_Value;

			if(BPack_Temp >10 && BPack_Temp < 20) // Temperature 10 ~ 20
				SetPoint_CC = 0.5*CC_Value;

			if(BPack_Temp >= 20 && BPack_Temp < 30) // Temperature 20 ~ 30
				SetPoint_CC = 0.7*CC_Value;

			if(BPack_Temp >= 30 && BPack_Temp <= 45)
				SetPoint_CC = CC_Value;

			if(BPack_Temp > 50)
				SetPoint_CC = 0.5*CC_Value;
		}
	}

	sp = SetPoint_CC;
	pv = Current_Charger;
	e = sp-pv;
	d = e-esblm;
	esblm = e;

	///////////////////////////fuzzifikasi error//////////////////////////////

	if(e<=-6)
	{ eNB=1;  eNM=eNS=eZ=ePS=ePM=ePB=0;}

	if(e>=-6&&e<=-4)
	{ eNB=(-(e+6)/2)+1;
	   eNM=(e+6)/2;
	   eNS=eZ=ePS=ePM=ePB=0;
	}

	else if(e>=-4&&e<=-2)
	{ eNM=(-(e+4)/2)+1;
	   eNS=(e+4)/2;
	   eNB=eZ=ePS=ePM=ePB=0;
	}

	else if(e>=-2&&e<=0)
	{ eNS=(-(e+2)/2)+1;
	   eZ=(e/2)+1;
	   eNB=eNM=ePS=ePM=ePB=0;
	}

	else if(e>=0&&e<=2)
	{ eZ=-(e/2)+1;
	   ePS=e/2;
	   eNB=eNM=eNS=ePM=ePB=0;
	}

	else if(e>=2&&e<=4)
	{ ePS=(-(e-2)/2)+1;
	   ePM=(e-2)/2;
	   eNB=eNM=eNS=eZ=ePB=0;
	}

	else if(e>=4&&e<=6)
	{ ePM=(-(e-4)/2)+1;
	   ePB=(e-4)/2;
	   eNB=eNM=eNS=eZ=ePS=0;
	}

	else if(e>=6)
	{ ePB=1; eNB=eNM=eNS=eZ=ePS=ePM=0;}

	/////////////////////fuzzifikasi delta error //////////////
	if(d<=-6)
	{ dNB=1;  dNM=dNS=dZ=dPS=dPM=dPB=0;}

	if(d>=-6&&d<=-4)
	{ dNB=(-(d+6)/2)+1;
	   dNM=(d+6)/2;
	   dNS=dZ=dPS=dPM=dPB=0;
	}

	else if(d>=-4&&d<=-2)
	{ dNM=(-(d+4)/2)+1;
	   dNS=(d+4)/2;
	   dNB=dZ=dPS=dPM=dPB=0;
	}

	else if(d>=-2&&d<=0)
	{ dNS=(-(d+2)/2)+1;
	   dZ=(d+2)/2;
	   dNB=dNM=dPS=dPM=dPB=0;
	}

	else if(d>=0&&d<=2)
	{ dZ=-(d/2)+1;
	   dPS=d/2;
	   dNB=dNM=dNS=dPM=dPB=0;
	}

	else if(d>=2&&d<=4)
	{ dPS=(-(d-2)/2)+1;
	   dPM=(d-2)/2;
	   dNB=dNM=dNS=dZ=dPB=0;
	}

	else if(d>=4&&d<=6)
	{ dPM=(-(d-4)/2)+1;
	   dPB=(d-4)/2;
	   dNB=dNM=dNS=dZ=dPS=0;
	}

	else if(d>=6)
	{ dPB=1; dNB=dNM=dNS=dZ=dPS=dPM=0;}

	/////////////////////inferensi///////////////////////

	r1=dPB; if(eNB<dPB) r1=eNB;		//max(dPB,eNB);
	r2=dPB; if(eNM<dPB) r2=eNM;
	r3=dPB; if(eNS<dPB) r3=eNS;
	r4=dPB; if(eZ<dPB) r4=eZ;
	r5=dPB; if(ePS<dPB) r5=ePS;
	r6=dPB; if(ePM<dPB) r6=ePM;
	r7=dPB; if(ePB<dPB) r7=ePB;


	r8=dPM; if(eNB<dPM) r8=eNB;		//r8=max(dPM,eNB);
	r9=dPM; if(eNM<dPM) r9=eNM;		//r9=max(dPM,eNM);
	r10=dPM; if(eNS<dPM) r10=eNS;	//r10=max(dPM,eNS);
	r11=dPM; if(eZ<dPM) r11=eZ;		//r11=max(dPM,eZ);
	r12=dPM; if(ePS<dPM) r12=ePS;	//r12=max(dPM,ePS);
	r13=dPM; if(ePM<dPM) r13=ePM;	//r13=max(dPM,ePM);
	r14=dPM; if(ePB<dPM) r14=ePB;	//r14=max(dPM,ePB);

	r15=dPS; if(eNB<dPS) r15=eNB;	//r15=max(dPS,eNB);
	r16=dPS; if(eNM<dPS) r16=eNM;	//r16=max(dPS,eNM);
	r17=dPS; if(eNS<dPS) r17=eNS;	//r17=max(dPS,eNS);
	r18=dPS; if(eZ<dPS) r18=eZ;		//r18=max(dPS,eZ);
	r19=dPS; if(ePS<dPS) r19=ePS;	//r19=max(dPS,ePS);
	r20=dPS; if(ePM<dPS) r20=ePM;	//r20=max(dPS,ePM);
	r21=dPS; if(ePB<dPS) r21=ePB;	//r21=max(dPS,ePB);

	r22=dZ; if(eNB<dZ) r22=eNB;		//r22=max(dZ,eNB);
	r23=dZ; if(eNM<dZ) r23=eNM;		//r23=max(dZ,eNM);
	r24=dZ; if(eNS<dZ) r24=eNS;		//r24=max(dZ,eNS);
	r25=dZ; if(eZ<dZ) r25=eZ;		//r25=max(dZ,eZ);
	r26=dZ; if(ePS<dZ) r26=ePS;		//r26=max(dZ,ePS);
	r27=dZ; if(ePM<dZ) r27=ePM;		//r27=max(dZ,ePM);
	r28=dZ; if(ePB<dZ) r28=ePB;		//r28=max(dZ,ePB);

	r29=dNS; if(eNB<dNS) r29=eNB;	//r29=max(dNS,eNB);
	r30=dNS; if(eNM<dNS) r30=eNM;	//r30=max(dNS,eNM);
	r31=dNS; if(eNS<dNS) r31=eNS;	//r31=max(dNS,eNS);
	r32=dNS; if(eZ<dNS) r32=eZ;		//r32=max(dNS,eZ);
	r33=dNS; if(ePS<dNS) r33=ePS;	//r33=max(dNS,ePS);
	r34=dNS; if(ePM<dNS) r34=ePM;	//r34=max(dNS,ePM);
	r35=dNS; if(ePB<dNS) r35=ePB;	//r35=max(dNS,ePB);

	r36=dNM; if(eNB<dNM) r36=eNB;	//r36=max(dNM,eNB);
	r37=dNM; if(eNM<dNM) r37=eNM;	//r37=max(dNM,eNM);
	r38=dNM; if(eNS<dNM) r38=eNS;	//r38=max(dNM,eNS);
	r39=dNM; if(eZ<dNM) r39=eZ;		//r39=max(dNM,eZ);
	r40=dNM; if(ePS<dNM) r40=ePS;	//r40=max(dNM,ePS);
	r41=dNM; if(ePM<dNM) r41=ePM;	//r41=max(dNM,ePM);
	r42=dNM; if(ePB<dNM) r42=ePB;	//r42=max(dNM,ePB);

	r43=dNB; if(eNB<dNB) r43=eNB;	//r43=max(dNB,eNB);
	r44=dNB; if(eNM<dNB) r44=eNM;	//r44=max(dNB,eNM);
	r45=dNB; if(eNS<dNB) r45=eNS;	//r45=max(dNB,eNS);
	r46=dNB; if(eZ<dNB) r46=eZ;		//r46=max(dNB,eZ);
	r47=dNB; if(ePS<dNB) r47=ePS;	//r47=max(dNB,ePS);
	r48=dNB; if(ePM<dNB) r48=ePM;	//r48=max(dNB,ePM);
	r49=dNB; if(ePB<dNB) r49=ePB;	//r49=max(dNB,ePB);

	///////////////////////////////DEFUZZIFIKASI///////////////////////////////////

	A=(r1*outZ)+(r2*outPS)+(r3*outPM)+(r4*outPB)+(r5*outPH)+(r6*outPH)+(r7*outPH);
	B=(r8*outNS)+(r9*outZ)+(r10*outPS)+(r11*outPM)+(r12*outPB)+(r13*outPH)+(r14*outPH);
	C=(r15*outNM)+(r16*outNS)+(r17*outZ)+(r18*outPS)+(r19*outPM)+(r20*outPB)+(r21*outPH);
	D=(r22*outNB)+(r23*outNM)+(r24*outNS)+(r25*outZ)+(r26*outPS)+(r27*outPM)+(r28*outPB);
	E=(r29*outNH)+(r30*outNB)+(r31*outNM)+(r32*outNS)+(r33*outZ)+(r34*outPS)+(r35*outPM);
	F=(r36*outNH)+(r37*outNH)+(r38*outNB)+(r39*outNM)+(r40*outNS)+(r41*outZ)+(r42*outPS);
	G=(r43*outNH)+(r44*outNH)+(r45*outNH)+(r46*outNB)+(r47*outNM)+(r48*outNS)+(r49*outZ);

	H=r1+r2+r3+r4+r5+r6+r7+r8+r9+r10+r11+r12+r13+r14+r15+r16+ r17+ r18+ r19+ r20+ r21+ r22+ r23+ r24+r25+r26+r27+r28+r29+r30+r31+r32+r33+r34+r35+r36+r37+r38+r39+r40+r41+r42+r43+r44+r45+r46+r47+r48+r49;

	step=(A+B+C+D+E+F+G)/H;

	dc=dc+step;

	if(dc>=90)
		dc=90;
	if(dc<=0)
		dc=0;

	duty = dc/100;

	if (HAL_GPIO_ReadPin(GPIOC, Button2_Pin)==1 && Eror_Code==0){
		Charger_Mode = 0;
		Handshaking = 0;
		identified = 0;
		UNIQUE_Code = 0;
	}
//	if (HAL_GPIO_ReadPin(GPIOC, Button2_Pin)==1 && Eror_Code==0){
//			  duty=duty+0.1;
//			  HAL_GPIO_WritePin(GPIOC, Buzzer_Pin,1);
//			  HAL_GPIO_TogglePin(GPIOB, Led1_Pin);
//			  uint32_t p = 1500000;
//			  	while(p>0)
//			  		p--;
//			  HAL_GPIO_WritePin(GPIOC, Buzzer_Pin,0);
//		}

	if(	flag_trip_overvoltage == 1		||
		flag_trip_overtemperature == 1	||
		flag_trip_undertemperature == 1	||
		flag_trip_overcurrentcharge == 1||
		flag_trip_SOCOverCharge == 1	||
		flag_trip_shortcircuit == 1		||
		flag_trip_systemfailure == 1	||
		Flag_ChargerShortCircuit == 1	||
		Flag_ChargerOverCurrent == 1	||
		Flag_ChargerOverTemperature == 1||
		Flag_ChargerOverVoltage == 1	||
		Flag_MiniPC_LostCommunication==1||
		Flag_BMS_LostCommunication == 1)
		{
			duty=0;
			Charger_Mode = 2;
		}

	TIM1->CCR1=duty*TIM1->ARR;

}

