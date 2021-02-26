/*
 * C_Voltage.c
 *
 *  Created on: Apr 8, 2020
 *      Author: faiz
 */

#include "main.h"
#include "stm32f2xx_hal.h"
#include "Control_Init.h"

//float oNH=-1, oNB=-0.5, oNM=-0.25, oNS=-0.15, oZ=0, oPS=0.15, oPM=0.25, oPB=0.5, oPH=1;
//float oNH=-0.05, oNB=-0.025, oNM=-0.01, oNS=-0.0075, oZ=0, oPS=0.0075, oPM=0.01, oPB=0.025, oPH=0.05;
float oNH=-0.1, oNB=-0.05, oNM=-0.025, oNS=-0.01, oZ=0, oPS=0.0075, oPM=0.01, oPB=0.025, oPH=0.05;
float SetPoint_CV = MAX_CHARGE_VOLTAGE-0.1;
extern float Voltage_Charger;

void Constant_Voltage(void)
{
	sp=SetPoint_CV;
	pv=Voltage_Charger;
	e=sp-pv;
	d=e-esblm;
	esblm=e;

	///////////////////////////fuzzifikasi error//////////////////////////////

		if(e<=-9)
		{ eNB=1;  eNM=eNS=eZ=ePS=ePM=ePB=0;}

		if(e>=-9&&e<=-6)
		{ eNB=(-(e+9)/3)+1;
		   eNM=(e+9)/3;
		   eNS=eZ=ePS=ePM=ePB=0;
		}

		else if(e>=-6&&e<=-3)
		{ eNM=(-(e+6)/3)+1;
		   eNS=(e+6)/3;
		   eNB=eZ=ePS=ePM=ePB=0;
		}

		else if(e>=-3&&e<=0)
		{ eNS=(-(e+3)/3)+1;
		   eZ=(e/3)+1;
		   eNB=eNM=ePS=ePM=ePB=0;
		}

		else if(e>=0&&e<=3)
		{ eZ=-(e/3)+1;
		   ePS=e/3;
		   eNB=eNM=eNS=ePM=ePB=0;
		}

		else if(e>=3&&e<=6)
		{ ePS=(-(e-3)/3)+1;
		   ePM=(e-3)/3;
		   eNB=eNM=eNS=eZ=ePB=0;
		}

		else if(e>=6&&e<=9)
		{ ePM=(-(e-6)/3)+1;
		   ePB=(e-6)/3;
		   eNB=eNM=eNS=eZ=ePS=0;
		}

		else if(e>=9)
		{ ePB=1; eNB=eNM=eNS=eZ=ePS=ePM=0;}

		/////////////////////fuzzifikasi delta error //////////////
		if(d<=-9)
		{ dNB=1;  dNM=dNS=dZ=dPS=dPM=dPB=0;}

		if(d>=-9&&d<=-6)
		{ dNB=(-(d+9)/3)+1;
		   dNM=(d+9)/3;
		   dNS=dZ=dPS=dPM=dPB=0;
		}

		else if(d>=-6&&d<=-3)
		{ dNM=(-(d+6)/3)+1;
		   dNS=(d+6)/3;
		   dNB=dZ=dPS=dPM=dPB=0;
		}

		else if(d>=-3&&d<=0)
		{ dNS=(-(d+3)/3)+1;
		   dZ=(d+3)/3;
		   dNB=dNM=dPS=dPM=dPB=0;
		}

		else if(d>=0&&d<=3)
		{ dZ=-(d/3)+1;
		   dPS=d/3;
		   dNB=dNM=dNS=dPM=dPB=0;
		}

		else if(d>=3&&d<=6)
		{ dPS=(-(d-3)/3)+1;
		   dPM=(d-3)/3;
		   dNB=dNM=dNS=dZ=dPB=0;
		}

		else if(d>=6&&d<=9)
		{ dPM=(-(d-6)/3)+1;
		   dPB=(d-6)/3;
		   dNB=dNM=dNS=dZ=dPS=0;
		}

		else if(d>=9)
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
	r14=dPM; if(ePB<dPM) r14=ePB;		//r14=max(dPM,ePB);

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

	///////////////////////////DEFUZZIFIKASI///////////////////////

	A=(r1*oZ)+(r2*oPS)+(r3*oPM)+(r4*oPB)+(r5*oPH)+(r6*oPH)+(r7*oPH);
	B=(r8*oNS)+(r9*oZ)+(r10*oPS)+(r11*oPM)+(r12*oPB)+(r13*oPH)+(r14*oPH);
	C=(r15*oNM)+(r16*oNS)+(r17*oZ)+(r18*oPS)+(r19*oPM)+(r20*oPB)+(r21*oPH);
	D=(r22*oNB)+(r23*oNM)+(r24*oNS)+(r25*oZ)+(r26*oPS)+(r27*oPM)+(r28*oPB);
	E=(r29*oNH)+(r30*oNB)+(r31*oNM)+(r32*oNS)+(r33*oZ)+(r34*oPS)+(r35*oPM);
	F=(r36*oNH)+(r37*oNH)+(r38*oNB)+(r39*oNM)+(r40*oNS)+(r41*oZ)+(r42*oPS);
	G=(r43*oNH)+(r44*oNH)+(r45*oNH)+(r46*oNB)+(r47*oNM)+(r48*oNS)+(r49*oZ);

	H=r1+r2+r3+r4+r5+r6+r7+r8+r9+r10+r11+r12+r13+r14+r15+r16+ r17+ r18+ r19+ r20+ r21+ r22+ r23+ r24+r25+r26+r27+r28+r29+r30+r31+r32+r33+r34+r35+r36+r37+r38+r39+r40+r41+r42+r43+r44+r45+r46+r47+r48+r49;

	step=(A+B+C+D+E+F+G)/H;

	dc=dc+step;

	if(dc>=85)	dc=85;
	if(dc<=0)	dc=0;

	duty = dc/100;

//	if (HAL_GPIO_ReadPin(GPIOC, Button2_Pin)==1 && Eror_Code==0){
//		  duty=duty+0.1;
//		  HAL_GPIO_WritePin(GPIOC, Buzzer_Pin,1);
//		  HAL_GPIO_TogglePin(GPIOB, Led1_Pin);
//		  uint32_t p = 1500000;
//		  	while(p>0)
//		  		p--;
//		  HAL_GPIO_WritePin(GPIOC, Buzzer_Pin,0);
//	}

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
		Flag_BMS_LostCommunication == 1  )
		{
			duty=0;
			Charger_Mode = 2;
		}
	TIM1->CCR1=duty*TIM1->ARR;
}


