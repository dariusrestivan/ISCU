#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include <math.h>

#define FS		2500
#define TS		0.0004
#define PWMPRD	30000	//15K*2/150M=2/10K=1/5K
#define DBLEN	450		//450/15M=3us

//sci-a c
#define SCIACANREV  (SciaRegs.SCIRXST.bit.RXRDY==1)
#define SCIACANSEND	(SciaRegs.SCICTL2.bit.TXRDY==1)
#define SCICCANREV  (ScicRegs.SCIRXST.bit.RXRDY==1)
#define SCICCANSEND	(ScicRegs.SCICTL2.bit.TXRDY==1)
//program control
#define PI						3.1415926
#define PI2						6.2831853
#define PI23					2.0943951
#define ABS(x) ((x)>0?(x):(-(x)))
#define LIMIT(x,min,max) if(1){if((x)<(min))(x)=(min);if((x)>(max))(x)=(max);}
#define MAKE0PI2(x)	if(1){while((x)<0){(x)+=PI2;}while((x)>PI2){(x)-=PI2;}}
#define FEQU(x,y)		((((x)-(y))<0.0001)&&(((y)-(x))<0.0001))
#define FNEQU(x,y)		((((x)-(y))>=0.0001)||(((y)-(x))>=0.0001))
#define BIGGERONE(x,y)  ((x)>(y)?(x):(y))
#define FTOI(x)			(int)((x)+0.5)
//IO
#define IO_LEDON				GpioDataRegs.GPACLEAR.bit.GPIO18=1;
#define IO_LEDOFF				GpioDataRegs.GPASET.bit.GPIO18=1;
//IN 0-5
#define IN0       	GpioDataRegs.GPBDAT.bit.GPIO58
#define IN1    		GpioDataRegs.GPBDAT.bit.GPIO57
#define IN2  	    GpioDataRegs.GPBDAT.bit.GPIO56
#define IN3			GpioDataRegs.GPBDAT.bit.GPIO55
#define IN4			GpioDataRegs.GPBDAT.bit.GPIO54
#define IN5			GpioDataRegs.GPBDAT.bit.GPIO53
//OUT 0-5
#define OUT0_LOW		GpioDataRegs.GPBCLEAR.bit.GPIO52=1;
#define OUT0_HIGH		GpioDataRegs.GPBSET.bit.GPIO52=1;
#define OUT1_LOW		GpioDataRegs.GPBCLEAR.bit.GPIO51=1;
#define OUT1_HIGH		GpioDataRegs.GPBSET.bit.GPIO51=1;
#define OUT2_LOW		GpioDataRegs.GPBCLEAR.bit.GPIO50=1;
#define OUT2_HIGH		GpioDataRegs.GPBSET.bit.GPIO50=1;
#define OUT3_LOW		GpioDataRegs.GPBCLEAR.bit.GPIO49=1;
#define OUT3_HIGH		GpioDataRegs.GPBSET.bit.GPIO49=1;
#define OUT4_LOW		GpioDataRegs.GPBCLEAR.bit.GPIO32=1;
#define OUT4_HIGH		GpioDataRegs.GPBSET.bit.GPIO32=1;
#define OUT5_LOW		GpioDataRegs.GPACLEAR.bit.GPIO27=1;
#define OUT5_HIGH		GpioDataRegs.GPASET.bit.GPIO27=1;
//vars
int gScriptToRun=0;
int gErrCode=0;
int gTsHappen=0;
int gnAdZeroCnt=0;
//
int gPureVarNum=0;		//how many pure vars based on the eeprom? such as gIa, gIb,
int gConstVarNum=0;		//how many const vars based on the eeprom? such as 100, 314,
int gScriptLineNum=0;	//how many script lines based on the eeprom?
float gVar[250];		//each var defined based on the eeprom
int   gScript[1500][10];
float gAdSampleValue[16];//original ad sample value
float gAdSampleOffset[16];
//int   gDI[6];
//int   gDO[6];
//global var reset
void ggResetAllVar()
{
	int i;
	int j;
	for(i=0;i<250;i++)
	{
		gVar[i]=0;
	}
	for(i=0;i<1500;i++)
	{
		for(j=0;j<10;j++)
		{
			gScript[i][j]=117;//117 is end function
		}
	}
	for(i=0;i<16;i++)
	{
		gAdSampleValue[i]=0;
		gAdSampleOffset[i]=0;
	}
	gnAdZeroCnt=0;
}
//
#pragma CODE_SECTION(ggPIController, "ramfuncs");
float ggPIController(float err,float * pErrAcc,float maxAbs,float kp,float ki)
{
	float newAcc;
	float newOut;

	newAcc=(*pErrAcc)+ki*err;
	if(newAcc>maxAbs)
	{
		newAcc=maxAbs;
	}
	if(newAcc<-maxAbs)
	{
		newAcc=-maxAbs;
	}
	*pErrAcc=newAcc;

	newOut=kp*err+newAcc;
	if(newOut>maxAbs)
	{
		newOut=maxAbs;
	}
	if(newOut<-maxAbs)
	{
		newOut=-maxAbs;
	}

	return newOut;
}
#pragma CODE_SECTION(ggPIRunAndReset, "ramfuncs");
float ggPIRunAndReset(int which, float err, float kp, float ki, float maxAbs)
{
	static float gAcc[10]={0,0,0,0,0,0,0,0,0,0};

	if(which>=0 && which<=9)//run
	{
		return ggPIController(err, &gAcc[which], maxAbs, kp, ki);
	}
	else if(which>=10 && which<=19)//reset
	{
		gAcc[which-10]=0;
	}
	return 0;
}
#pragma CODE_SECTION(ggEnablePwm, "ramfuncs");
int ggEnablePwm(int enable)
{
	static int RectifierEnableState=-1;

	if(enable==2)
	{
		return RectifierEnableState;
	}
	if(enable!=RectifierEnableState)
	{
		if(enable==1)
		{
			EALLOW;
			GpioCtrlRegs.GPAMUX1.bit.GPIO0=1;
			GpioCtrlRegs.GPAMUX1.bit.GPIO1=1;
			GpioCtrlRegs.GPAMUX1.bit.GPIO2=1;
			GpioCtrlRegs.GPAMUX1.bit.GPIO3=1;
			GpioCtrlRegs.GPAMUX1.bit.GPIO4=1;
			GpioCtrlRegs.GPAMUX1.bit.GPIO5=1;
			GpioCtrlRegs.GPAMUX1.bit.GPIO6=1;
			GpioCtrlRegs.GPAMUX1.bit.GPIO7=1;
			GpioCtrlRegs.GPAMUX1.bit.GPIO8=1;
			GpioCtrlRegs.GPAMUX1.bit.GPIO9=1;
			GpioCtrlRegs.GPAMUX1.bit.GPIO10=1;
			GpioCtrlRegs.GPAMUX1.bit.GPIO11=1;
			EDIS;
		}
		else
		{
			EALLOW;

			GpioDataRegs.GPASET.bit.GPIO0=1;//PWM1 is high
			GpioDataRegs.GPASET.bit.GPIO1=1;//PWM2 is high
			GpioDataRegs.GPASET.bit.GPIO2=1;//PWM3 is high
			GpioDataRegs.GPASET.bit.GPIO3=1;//PWM4 is high
			GpioDataRegs.GPASET.bit.GPIO4=1;//PWM5 is high
			GpioDataRegs.GPASET.bit.GPIO5=1;//PWM6 is high
			GpioDataRegs.GPASET.bit.GPIO6=1;//PWM1 is high
			GpioDataRegs.GPASET.bit.GPIO7=1;//PWM2 is high
			GpioDataRegs.GPASET.bit.GPIO8=1;//PWM3 is high
			GpioDataRegs.GPASET.bit.GPIO9=1;//PWM4 is high
			GpioDataRegs.GPASET.bit.GPIO10=1;//PWM5 is high
			GpioDataRegs.GPASET.bit.GPIO11=1;//PWM6 is high

		    GpioCtrlRegs.GPAMUX1.bit.GPIO0=0;//PWM1 as IO
		    GpioCtrlRegs.GPAMUX1.bit.GPIO1=0;//PWM2 as IO
		    GpioCtrlRegs.GPAMUX1.bit.GPIO2=0;//PWM3 as IO
		    GpioCtrlRegs.GPAMUX1.bit.GPIO3=0;//PWM4 as IO
		    GpioCtrlRegs.GPAMUX1.bit.GPIO4=0;//PWM5 as IO
		    GpioCtrlRegs.GPAMUX1.bit.GPIO5=0;//PWM6 as IO
		    GpioCtrlRegs.GPAMUX1.bit.GPIO6=0;//PWM1 as IO
		    GpioCtrlRegs.GPAMUX1.bit.GPIO7=0;//PWM2 as IO
		    GpioCtrlRegs.GPAMUX1.bit.GPIO8=0;//PWM3 as IO
		    GpioCtrlRegs.GPAMUX1.bit.GPIO9=0;//PWM4 as IO
		    GpioCtrlRegs.GPAMUX1.bit.GPIO10=0;//PWM5 as IO
		    GpioCtrlRegs.GPAMUX1.bit.GPIO11=0;//PWM6 as IO

			EDIS;
		}
		RectifierEnableState=enable;
	}

	return RectifierEnableState;
}

#pragma CODE_SECTION(ggChangePwmXRatio, "ramfuncs");
void ggChangePwmXRatio(int which,float ratio)//which=1-6, ratio=0~1
{
	int cmp;

	LIMIT(ratio,0.02,0.98);
	cmp=ratio*15000.0;

	switch(which)
	{
	case 1:EPwm1Regs.CMPA.half.CMPA =cmp;break;
	case 2:EPwm2Regs.CMPA.half.CMPA =cmp;break;
	case 3:EPwm3Regs.CMPA.half.CMPA =cmp;break;
	case 4:EPwm4Regs.CMPA.half.CMPA =cmp;break;
	case 5:EPwm5Regs.CMPA.half.CMPA =cmp;break;
	case 6:EPwm6Regs.CMPA.half.CMPA =cmp;break;
	default:break;
	}
}

#define FLASHADDRSTART 0X200000	// ex flash:0x200000//zone7
#define ggReadExFlash(addr) 		(*((int *)(FLASHADDRSTART+(addr))))		//read ex flash
int	ggWriteExFlash(int addr,  int data)//ret=1, ok
{
	int j;
	unsigned long timeOutCnt;
	int	tmpData;

	*((int *)(FLASHADDRSTART + 0x5555)) = 0x00AA;
	*((int *)(FLASHADDRSTART + 0x2AAA)) = 0x0055;
	*((int *)(FLASHADDRSTART + 0x5555)) = 0x00A0;
	*((int *)(FLASHADDRSTART + addr)) = data;
	timeOutCnt = 0;
	j=0;
	while(j<5)
	{
		tmpData = *((int*)(FLASHADDRSTART + addr));
		if(tmpData == data)
		{
			j+=1;
		}
		else
		{
			j=0;
		}

		timeOutCnt+=1;
		if(timeOutCnt>0x1000000)
		return 0;//failed

	}
	return 1;//succeed
}
int	ggEraseExFlash(void)//ret=1, ok
{
	int	Data;
	unsigned long TimeOut;
	unsigned long i;

	*((int*)(FLASHADDRSTART + 0x5555)) = 0xAAAA;
	*((int*)(FLASHADDRSTART + 0x2AAA)) = 0x5555;
	*((int*)(FLASHADDRSTART + 0x5555)) = 0x8080;
	*((int*)(FLASHADDRSTART + 0x5555)) = 0xAAAA;
	*((int*)(FLASHADDRSTART + 0x2AAA)) = 0x5555;
	*((int*)(FLASHADDRSTART + 0x5555)) = 0x1010;
	i = 0;
	TimeOut = 0;
	while(i<5)
	{
		Data=*((int*)(FLASHADDRSTART+0x3FFFF));
		if(Data == 0xFFFF)
		{
			i+=1;
		}
		else
		{
			i=0;
		}
		TimeOut+=1;
		if(TimeOut>0x1000000)return 0;
	}
	for(i=0;i<0x40000;i++)
	{
		Data = *((int*)(FLASHADDRSTART + i));
		if (Data !=0xFFFF)	return 2;
	}
	return  1;
}

#pragma CODE_SECTION(ggLoadExFlashCode, "ramfuncs");
void ggLoadExFlashCode()
{
	int i;
	int j;
	long int data1;
	long int data2;
	int startPos;
	static int head[20];
	//read the first 20 words, judge whether the scripts are legal
	for(i=0;i<20;i++)
	{
		head[i]=ggReadExFlash(i);
	}
	if(	head[0]=='G' && head[1]=='Z' && head[2]=='G' && head[3]=='B' && head[4]=='U' &&
		head[5]=='R' && head[6]=='N' && head[7]=='1' && head[8]=='2' && head[9]=='3' )
	{
		//read the number of the pureVars, constVars and scripts
		gPureVarNum=head[10];
		gConstVarNum=head[11];
		gScriptLineNum=head[12];
		for(i=0;i<gPureVarNum;i++)
		{
			startPos=20+i*30;
			//load the var[i] init value to gVar[i]
			data1=ggReadExFlash(startPos+28);
			data1&=0x0000ffff;
			data2=ggReadExFlash(startPos+29);
			data2&=0x0000ffff;
			data2=(data2<<16)|data1;
			gVar[i]=*((float*)&data2);
		}
		for(i=0;i<gConstVarNum;i++)
		{
			startPos=20+gPureVarNum*30+i*30;
			//load the var[i] init value to gVar[i]
			data1=ggReadExFlash(startPos+28);
			data1&=0x0000ffff;
			data2=ggReadExFlash(startPos+29);
			data2&=0x0000ffff;
			data2=(data2<<16)|data1;
			gVar[gPureVarNum+i]=*((float*)&data2);
		}
		for(i=0;i<gScriptLineNum;i++)
		{
			startPos=20+gPureVarNum*30+gConstVarNum*30+i*10;
			//load the script to gScript
			for(j=0;j<10;j++)
			{
				gScript[i][j]=ggReadExFlash(startPos+j);
				LIMIT(gScript[i][j],0,499);
			}
		}
		//by now,for every pureVar, we have 28 chars to store the name
		//we can report the name of the vars, but only 26*250 are supported
		//thus, we just remove the last two chars and report them to the computer
	}
	else
	{
		gPureVarNum=0;
		gConstVarNum=0;
		gScriptLineNum=0;
	}
	gScriptToRun=0;
}
//axis
#pragma CODE_SECTION(ABtoXY, "ramfuncs");
void ABtoXY(float ua,float ub,float * pUx,float * pUy)
{
	*pUx=ua;
	*pUy=ua*0.57735+1.1547*ub;
}
#pragma CODE_SECTION(XYtoAB, "ramfuncs");
void XYtoAB(float ux,float uy,float * pUa,float * pUb)
{
	*pUa=ux;
	*pUb=-0.5*ux+0.866*uy;
}
#pragma CODE_SECTION(ABtoDQ, "ramfuncs");
void ABtoDQ(float ua,float ub,float gama,float * pUd,float * pUq)
{
	float cosgama;
	float singama;

	cosgama=cos(gama);
	singama=sin(gama);

	*pUd=cosgama*ua+singama*ua*0.57735+singama*ub*1.1547;
	*pUq=-singama*ua+cosgama*ua*0.57735+cosgama*ub*1.1547;
}
#pragma CODE_SECTION(DQtoAB, "ramfuncs");
void DQtoAB(float ud,float uq,float gama,float * pUa,float * pUb)
{
	float cosgama;
	float singama;

	cosgama=cos(gama);
	singama=sin(gama);

	*pUa=cosgama*ud-singama*uq;
	*pUb=-cosgama*ud*0.5+singama*ud*0.866+singama*uq*0.5+cosgama*uq*0.866;

}
void ggUserParaSetup()
{
	EALLOW;

	if(1)//led
	{
		GpioCtrlRegs.GPAPUD.bit.GPIO18=0;    // Enable pull-up
		GpioCtrlRegs.GPAMUX2.bit.GPIO18=0; 		//as io
		GpioCtrlRegs.GPADIR.bit.GPIO18=1;
		GpioDataRegs.GPASET.bit.GPIO18=1;
	}
	if(1)//in0-5
	{
		GpioCtrlRegs.GPBPUD.bit.GPIO58=0;// Enable pull-up
		GpioCtrlRegs.GPBMUX2.bit.GPIO58=0;//as gpio
		GpioCtrlRegs.GPBDIR.bit.GPIO58=0;//input

		GpioCtrlRegs.GPBPUD.bit.GPIO57=0;// Enable pull-up
		GpioCtrlRegs.GPBMUX2.bit.GPIO57=0;//as gpio
		GpioCtrlRegs.GPBDIR.bit.GPIO57=0;//input

		GpioCtrlRegs.GPBPUD.bit.GPIO56=0;// Enable pull-up
		GpioCtrlRegs.GPBMUX2.bit.GPIO56=0;//as gpio
		GpioCtrlRegs.GPBDIR.bit.GPIO56=0;//input

		GpioCtrlRegs.GPBPUD.bit.GPIO55=0;// Enable pull-up
		GpioCtrlRegs.GPBMUX2.bit.GPIO55=0;//as gpio
		GpioCtrlRegs.GPBDIR.bit.GPIO55=0;//input

		GpioCtrlRegs.GPBPUD.bit.GPIO54=0;// Enable pull-up
		GpioCtrlRegs.GPBMUX2.bit.GPIO54=0;//as gpio
		GpioCtrlRegs.GPBDIR.bit.GPIO54=0;//input

		GpioCtrlRegs.GPBPUD.bit.GPIO53=0;// Enable pull-up
		GpioCtrlRegs.GPBMUX2.bit.GPIO53=0;//as gpio
		GpioCtrlRegs.GPBDIR.bit.GPIO53=0;//input
	}
	if(1)//out0-5
	{
		GpioCtrlRegs.GPBPUD.bit.GPIO52 = 0;    // Enable pull-up
		GpioCtrlRegs.GPBMUX2.bit.GPIO52=0; 		//as io
		GpioCtrlRegs.GPBDIR.bit.GPIO52=1;//as out
		GpioDataRegs.GPBSET.bit.GPIO52=1;//default = high vol

		GpioCtrlRegs.GPBPUD.bit.GPIO51 = 0;    // Enable pull-up
		GpioCtrlRegs.GPBMUX2.bit.GPIO51=0; 		//as io
		GpioCtrlRegs.GPBDIR.bit.GPIO51=1;//as out
		GpioDataRegs.GPBSET.bit.GPIO51=1;//default = high vol

		GpioCtrlRegs.GPBPUD.bit.GPIO50 = 0;    // Enable pull-up
		GpioCtrlRegs.GPBMUX2.bit.GPIO50=0; 		//as io
		GpioCtrlRegs.GPBDIR.bit.GPIO50=1;
		GpioDataRegs.GPBSET.bit.GPIO50=1;

		GpioCtrlRegs.GPBPUD.bit.GPIO49 = 0;    // Enable pull-up
		GpioCtrlRegs.GPBMUX2.bit.GPIO49=0; 		//as io
		GpioCtrlRegs.GPBDIR.bit.GPIO49=1;
		GpioDataRegs.GPBSET.bit.GPIO49=1;

		GpioCtrlRegs.GPBPUD.bit.GPIO32=0;    // Enable pull-up
		GpioCtrlRegs.GPBMUX1.bit.GPIO32=0; 		//as io
		GpioCtrlRegs.GPBDIR.bit.GPIO32=1;
		GpioDataRegs.GPBSET.bit.GPIO32=1;

		GpioCtrlRegs.GPAPUD.bit.GPIO27=0;    // Enable pull-up
		GpioCtrlRegs.GPAMUX2.bit.GPIO27=0; 		//as io
		GpioCtrlRegs.GPADIR.bit.GPIO27=1;
		GpioDataRegs.GPASET.bit.GPIO27=1;
	}
	if(1)//analog
	{
		//ad settings
		AdcRegs.ADCTRL1.bit.SUSMOD=3;
		AdcRegs.ADCTRL1.bit.ACQ_PS = 1;
		AdcRegs.ADCTRL1.bit.CPS = 0;//ADCLOCK=HISCLK/1
		AdcRegs.ADCTRL1.bit.SEQ_CASC = 1; //Cascaded mode
		AdcRegs.ADCTRL1.bit.CONT_RUN = 0;// start-stop mode (not continuous run mode)
		AdcRegs.ADCTRL1.bit.SEQ_OVRD=0;//

		AdcRegs.ADCTRL2.bit.EPWM_SOCB_SEQ=0;
		AdcRegs.ADCTRL2.bit.RST_SEQ1=1;
		AdcRegs.ADCTRL2.bit.SOC_SEQ1=0;
		AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 0x0;//no adc interrupt
		AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1=0;
		AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 0x0;//epwm enable adc
		AdcRegs.ADCTRL2.bit.EXT_SOC_SEQ1=0;
		AdcRegs.ADCTRL2.bit.RST_SEQ2=0;
		AdcRegs.ADCTRL2.bit.SOC_SEQ2=0;
		AdcRegs.ADCTRL2.bit.INT_ENA_SEQ2=0;
		AdcRegs.ADCTRL2.bit.INT_MOD_SEQ2=0;
		AdcRegs.ADCTRL2.bit.EPWM_SOCB_SEQ2=0;

		AdcRegs.ADCCHSELSEQ1.bit.CONV00=0;
		AdcRegs.ADCCHSELSEQ1.bit.CONV01=1;
		AdcRegs.ADCCHSELSEQ1.bit.CONV02=2;
		AdcRegs.ADCCHSELSEQ1.bit.CONV03=3;
		AdcRegs.ADCCHSELSEQ2.bit.CONV04=4;
		AdcRegs.ADCCHSELSEQ2.bit.CONV05=5;
		AdcRegs.ADCCHSELSEQ2.bit.CONV06=6;
		AdcRegs.ADCCHSELSEQ2.bit.CONV07=7;
		AdcRegs.ADCCHSELSEQ3.bit.CONV08=8;
		AdcRegs.ADCCHSELSEQ3.bit.CONV09=9;
		AdcRegs.ADCCHSELSEQ3.bit.CONV10=10;
		AdcRegs.ADCCHSELSEQ3.bit.CONV11=11;
		AdcRegs.ADCCHSELSEQ4.bit.CONV12=12;
		AdcRegs.ADCCHSELSEQ4.bit.CONV13=13;
		AdcRegs.ADCCHSELSEQ4.bit.CONV14=14;
		AdcRegs.ADCCHSELSEQ4.bit.CONV15=15;

		AdcRegs.ADCTRL3.bit.ADCCLKPS =0;//ADCLOCK/=1
		AdcRegs.ADCTRL3.bit.SMODE_SEL=0;
		AdcRegs.ADCMAXCONV.bit.MAX_CONV1 = 0xf;//conversion num =16
		AdcRegs.ADCTRL2.bit.RST_SEQ1=1;
	}
	if(1)//encoder
	{
		//eqep1
		EQep1Regs.QUPRD=1500000;			// Unit Timer for 100Hz at 150MHz SYSCLKOUT

		EQep1Regs.QDECCTL.bit.QSRC=0;		// Quadrature mode
		EQep1Regs.QDECCTL.bit.XCR=0;        // 4x resolution (cnt falling and rising edges)
		EQep1Regs.QDECCTL.bit.IGATE=0;		// disable index event

		EQep1Regs.QEPCTL.bit.FREE_SOFT=2;
		EQep1Regs.QEPCTL.bit.PCRM=0x01;		// QPOSCNT reset on max
		EQep1Regs.QEPCTL.bit.SEL=0;			//latch on rising edge
		EQep1Regs.QEPCTL.bit.UTE=1; 		// Unit Timer Enable
		EQep1Regs.QEPCTL.bit.QCLM=1; 		// Latch on unit time out
		EQep1Regs.QEPCTL.bit.QPEN=1; 		// QEP enable
		EQep1Regs.QEPCTL.bit.WDE=0;
		EQep1Regs.QEPCTL.bit.QPEN=1; 		// QEP enable

		EQep1Regs.QPOSCNT=0;
		EQep1Regs.QPOSMAX=80000;//

		EQep1Regs.QCAPCTL.bit.UPPS=2;   	// 1/4 for unit position at 150MHz SYSCLKOUT
		EQep1Regs.QCAPCTL.bit.CCPS=7;		// 1/128 for CAP clock,150M/128=1171875Hz
		EQep1Regs.QCAPCTL.bit.CEN=0; 		// QEP Capture Enable

		EQep1Regs.QEINT.all=0;//NO INT
		EQep1Regs.QCLR.all=0xffff;//

		//endr,GPIO20,21,23
		GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;   // Enable pull-up on (EQEP1A)
		GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 0;   // Sync to SYSCLKOUT (EQEP1A)
		GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 1;   // Configure as EQEP1A

		GpioCtrlRegs.GPAPUD.bit.GPIO21 = 0;   // Enable pull-up on (EQEP1B)
		GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 0;   // Sync to SYSCLKOUT (EQEP1B)
		GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 1;   // Configure as EQEP1B

		GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;   // Enable pull-up on
		GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 0;   // Sync to SYSCLKOUT
		GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 1;   // Configure as EQEP-index
		//endr2,GPIO24,25,26
		GpioCtrlRegs.GPAPUD.bit.GPIO24 = 0;   // Enable pull-up on (EQEP1A)
		GpioCtrlRegs.GPAQSEL2.bit.GPIO24 = 0;   // Sync to SYSCLKOUT (EQEP1A)
		GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 2;   // Configure as EQEP1A

		GpioCtrlRegs.GPAPUD.bit.GPIO25 = 0;   // Enable pull-up on (EQEP1B)
		GpioCtrlRegs.GPAQSEL2.bit.GPIO25 = 0;   // Sync to SYSCLKOUT (EQEP1B)
		GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 2;   // Configure as EQEP1B

		GpioCtrlRegs.GPAPUD.bit.GPIO26 = 0;   // Enable pull-up on (EQEP1B)
		GpioCtrlRegs.GPAQSEL2.bit.GPIO26 = 0;   // Sync to SYSCLKOUT (EQEP1B)
		GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 2;   // Configure as EQEP-index
	}
	if(1)//pwm
	{
		//EPWM-1
		EPwm1Regs.TBPRD = PWMPRD;
		EPwm1Regs.TBPHS.half.TBPHS = 0;//when syn signal occurs, load it
		EPwm1Regs.TBCTL.all=0x2012;//timer run sytle and et al.
		EPwm1Regs.CMPCTL.all=0x0100;//cmpr shadow and load when cnt=0
		EPwm1Regs.AQCTLA.all=0x0060;//action a
		EPwm1Regs.AQCTLB.all=0x0090;//action b
		EPwm1Regs.DBCTL.all=0x0007;//deat time config
		EPwm1Regs.DBFED = DBLEN;//dead time falling x/150M
		EPwm1Regs.DBRED = DBLEN;//dead time rising
		EPwm1Regs.ETSEL.bit.INTEN = 1;//allow pwm1 int
		EPwm1Regs.ETSEL.bit.INTSEL = 1;//interrupt when cnt=0
		EPwm1Regs.ETPS.bit.INTPRD = 1;//Generate INT every 1 time
		EPwm1Regs.ETCLR.bit.INT=1;//clear int flag
		//EPWM-2
		EPwm2Regs.TBPRD = PWMPRD;
		EPwm2Regs.TBPHS.half.TBPHS = 0;//when syn signal occurs, load it
		EPwm2Regs.TBCTL.all=0x2006;//timer run style
		EPwm2Regs.CMPCTL.all=0x1000;//cmpr shadow and load when cnt=0
		EPwm2Regs.AQCTLA.all=0x0060;//action a
		EPwm2Regs.AQCTLB.all=0x0090;//action b
		EPwm2Regs.DBCTL.all=0x0007;//dead time config
		EPwm2Regs.DBFED = DBLEN;//falling
		EPwm2Regs.DBRED = DBLEN;//rising
		//EPWM-3
		EPwm3Regs.TBPRD = PWMPRD;
		EPwm3Regs.TBPHS.half.TBPHS = 0;//when syn signal occurs, load it
		EPwm3Regs.TBCTL.all=0x2006;//timer run style
		EPwm3Regs.CMPCTL.all=0x1000;//cmpr shadow and load when cnt=0
		EPwm3Regs.AQCTLA.all=0x0060;//action a
		EPwm3Regs.AQCTLB.all=0x0090;//action b
		EPwm3Regs.DBCTL.all=0x0007;//dead time config
		EPwm3Regs.DBFED = DBLEN;//falling
		EPwm3Regs.DBRED = DBLEN;//rising
		//EPWM-4
		EPwm4Regs.TBPRD = PWMPRD;
		EPwm4Regs.TBPHS.half.TBPHS = 0;//when syn signal occurs, load it
		EPwm4Regs.TBCTL.all=0x2006;//timer run style
		EPwm4Regs.CMPCTL.all=0x1000;//cmpr shadow and load when cnt=0
		EPwm4Regs.AQCTLA.all=0x0060;//action a
		EPwm4Regs.AQCTLB.all=0x0090;//action b
		EPwm4Regs.DBCTL.all=0x0007;//dead time config
		EPwm4Regs.DBFED = DBLEN;//falling
		EPwm4Regs.DBRED = DBLEN;//rising
		//EPWM-5
		EPwm5Regs.TBPRD = PWMPRD;
		EPwm5Regs.TBPHS.half.TBPHS = 0;//when syn signal occurs, load it
		EPwm5Regs.TBCTL.all=0x2006;//timer run style
		EPwm5Regs.CMPCTL.all=0x1000;//cmpr shadow and load when cnt=0
		EPwm5Regs.AQCTLA.all=0x0060;//action a
		EPwm5Regs.AQCTLB.all=0x0090;//action b
		EPwm5Regs.DBCTL.all=0x0007;//dead time config
		EPwm5Regs.DBFED = DBLEN;//falling
		EPwm5Regs.DBRED = DBLEN;//rising
		//EPWM-6
		EPwm6Regs.TBPRD = PWMPRD;
		EPwm6Regs.TBPHS.half.TBPHS = 0;//when syn signal occurs, load it
		EPwm6Regs.TBCTL.all=0x2006;//timer run style
		EPwm6Regs.CMPCTL.all=0x1000;//cmpr shadow and load when cnt=0
		EPwm6Regs.AQCTLA.all=0x0060;//action a
		EPwm6Regs.AQCTLB.all=0x0090;//action b
		EPwm6Regs.DBCTL.all=0x0007;//dead time config
		EPwm6Regs.DBFED = DBLEN;//falling
		EPwm6Regs.DBRED = DBLEN;//rising

		//GPIO0-11 for epwm1-12 are output as high by default
		GpioCtrlRegs.GPAPUD.all	&=0xfffff000;  // PWM1-12 Enable pull-up
		GpioCtrlRegs.GPAMUX1.bit.GPIO0=0;//PWM as IO
		GpioCtrlRegs.GPADIR.bit.GPIO0=1;//dir is out
		GpioDataRegs.GPASET.bit.GPIO0=1;//is high

		GpioCtrlRegs.GPAMUX1.bit.GPIO1=0;//PWM as IO
		GpioCtrlRegs.GPADIR.bit.GPIO1=1;//dir is out
		GpioDataRegs.GPASET.bit.GPIO1=1;//is high

		GpioCtrlRegs.GPAMUX1.bit.GPIO2=0;//PWM as IO
		GpioCtrlRegs.GPADIR.bit.GPIO2=1;//dir is out
		GpioDataRegs.GPASET.bit.GPIO2=1;//is high

		GpioCtrlRegs.GPAMUX1.bit.GPIO3=0;//PWM as IO
		GpioCtrlRegs.GPADIR.bit.GPIO3=1;//dir is out
		GpioDataRegs.GPASET.bit.GPIO3=1;//is high

		GpioCtrlRegs.GPAMUX1.bit.GPIO4=0;//PWM as IO
		GpioCtrlRegs.GPADIR.bit.GPIO4=1;//dir is out
		GpioDataRegs.GPASET.bit.GPIO4=1;//is high

		GpioCtrlRegs.GPAMUX1.bit.GPIO5=0;//PWM as IO
		GpioCtrlRegs.GPADIR.bit.GPIO5=1;//dir is out
		GpioDataRegs.GPASET.bit.GPIO5=1;//is high

		GpioCtrlRegs.GPAMUX1.bit.GPIO6=0;//PWM as IO
		GpioCtrlRegs.GPADIR.bit.GPIO6=1;//dir is out
		GpioDataRegs.GPASET.bit.GPIO6=1;//is high

		GpioCtrlRegs.GPAMUX1.bit.GPIO7=0;//PWM as IO
		GpioCtrlRegs.GPADIR.bit.GPIO7=1;//dir is out
		GpioDataRegs.GPASET.bit.GPIO7=1;//is high

		GpioCtrlRegs.GPAMUX1.bit.GPIO8=0;//PWM as IO
		GpioCtrlRegs.GPADIR.bit.GPIO8=1;//dir is out
		GpioDataRegs.GPASET.bit.GPIO8=1;//is high

		GpioCtrlRegs.GPAMUX1.bit.GPIO9=0;//PWM as IO
		GpioCtrlRegs.GPADIR.bit.GPIO9=1;//dir is out
		GpioDataRegs.GPASET.bit.GPIO9=1;//is high

		GpioCtrlRegs.GPAMUX1.bit.GPIO10=0;//PWM as IO
		GpioCtrlRegs.GPADIR.bit.GPIO10=1;//dir is out
		GpioDataRegs.GPASET.bit.GPIO10=1;//is high

		GpioCtrlRegs.GPAMUX1.bit.GPIO11=0;//PWM as IO
		GpioCtrlRegs.GPADIR.bit.GPIO11=1;//dir is out
		GpioDataRegs.GPASET.bit.GPIO11=1;//is high
	}
	if(1)//sci-a
	{
		SciaRegs.SCIFFTX.all=0xA040;//1010 0000 0100 0000
		SciaRegs.SCIFFRX.all=0x004f;//0000 0000 0100 1111
		SciaRegs.SCIFFCT.all=0x0;	//0000 0000 0000 0000
		SciaRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
									   // Disable RX ERR, SLEEP, TXWAKE
		SciaRegs.SCICTL2.all =0x0;	   // disable rx and tx interrupt
		SciaRegs.SCIHBAUD    =0x0000;  // 115200 baud @LSPCLK = 37.5MHz.
		SciaRegs.SCILBAUD    =0x0028;
		SciaRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback,No parity,8 char bits
										// async mode, idle-line protocol
		SciaRegs.SCICTL1.all =0x0023;  	// Relinquish SCI from Reset

		//sci-a,R1,T1
		GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;    // Enable pull-up
		GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;    // Enable pull-up

		GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;   // Configure as SCIRX operation
		GpioCtrlRegs.GPAMUX2.bit.GPIO29=1;		//config as scitx operation

		GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input
	}
	if(1)//sci-b
	{
		//SCIB para
		ScibRegs.SCIFFTX.all=0xA040;//1010 0000 0100 0000
		ScibRegs.SCIFFRX.all=0x004f;//0000 0000 0100 1111
		ScibRegs.SCIFFCT.all=0x0;	//0000 0000 0000 0000
		ScibRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,// Disable RX ERR, SLEEP, TXWAKE
		ScibRegs.SCICTL2.all =0x0;	   // disable rx and tx interrupt
		if(1)
		{
			ScibRegs.SCIHBAUD=0x0000;//115200
			ScibRegs.SCILBAUD=0x0028;

		}
		else
		{
			ScibRegs.SCIHBAUD    =0x01;  // 9600 baud @LSPCLK = 37.5MHz.//37.5M/8/bps-1
			ScibRegs.SCILBAUD    =0xe7;
		}
		ScibRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback,No parity,8 char bits// async mode, idle-line protocol
		ScibRegs.SCICTL1.all =0x0023; 	 // Relinquish SCI from Reset
	}
	if(1)//sci-c
	{
		//SCIC para
		ScicRegs.SCIFFTX.all=0xA040;//1010 0000 0100 0000
		ScicRegs.SCIFFRX.all=0x004f;//0000 0000 0100 1111
		ScicRegs.SCIFFCT.all=0x0;	//0000 0000 0000 0000
		ScicRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,// Disable RX ERR, SLEEP, TXWAKE
		ScicRegs.SCICTL2.all =0x0;	   // disable rx and tx interrupt
		ScicRegs.SCIHBAUD=0x0000;
		ScicRegs.SCILBAUD=0x0028;
		ScicRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback,No parity,8 char bits// async mode, idle-line protocol
		ScicRegs.SCICTL1.all =0x0023; 	 // Relinquish SCI from Reset

		GpioCtrlRegs.GPBPUD.bit.GPIO62 = 0;    // Enable pull-up
		GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 1;   // Configure as SCIRX operation
		GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = 3;  // Asynch input
		GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;    // Enable pull-up
		GpioCtrlRegs.GPBMUX2.bit.GPIO63=1;		//config as scitx operation
	}
	if(1)//can, bps=100k
	{

	}
	if(1)//xintf
	{
		// All Zones---------------------------------
		// Timing for all zones based on XTIMCLK = 1/2 SYSCLKOUT
		XintfRegs.XINTCNF2.bit.XTIMCLK = 1;
		// No write buffering
		XintfRegs.XINTCNF2.bit.WRBUFF = 0;
		// XCLKOUT is enabled
		XintfRegs.XINTCNF2.bit.CLKOFF = 0;
		// XCLKOUT = XTIMCLK/2
		XintfRegs.XINTCNF2.bit.CLKMODE = 1;

		// Zone 0------------------------------------
		// When using ready, ACTIVE must be 1 or greater
		// Lead must always be 1 or greater
		// Zone write timing
		XintfRegs.XTIMING0.bit.XWRLEAD = 3;
		XintfRegs.XTIMING0.bit.XWRACTIVE = 7;
		XintfRegs.XTIMING0.bit.XWRTRAIL = 3;
		// Zone read timing
		XintfRegs.XTIMING0.bit.XRDLEAD = 3;
		XintfRegs.XTIMING0.bit.XRDACTIVE = 7;
		XintfRegs.XTIMING0.bit.XRDTRAIL = 3;
		// double all Zone read/write lead/active/trail timing
		XintfRegs.XTIMING0.bit.X2TIMING = 1;
		// Zone will NOT sample XREADY signal
		XintfRegs.XTIMING0.bit.USEREADY = 0;
		XintfRegs.XTIMING0.bit.READYMODE = 0;

		// Size must be either:
		// 0,1 = x32 or
		// 1,1 = x16 other values are reserved
		XintfRegs.XTIMING0.bit.XSIZE = 3;

		// Zone 6------------------------------------
		// When using ready, ACTIVE must be 1 or greater
		// Lead must always be 1 or greater
		// Zone write timing
		XintfRegs.XTIMING6.bit.XWRLEAD = 3;
		XintfRegs.XTIMING6.bit.XWRACTIVE = 7;
		XintfRegs.XTIMING6.bit.XWRTRAIL = 3;
		// Zone read timing
		XintfRegs.XTIMING6.bit.XRDLEAD = 3;
		XintfRegs.XTIMING6.bit.XRDACTIVE = 7;
		XintfRegs.XTIMING6.bit.XRDTRAIL = 3;

		// double all Zone read/write lead/active/trail timing
		XintfRegs.XTIMING6.bit.X2TIMING = 1;

		// Zone will NOT sample XREADY signal
		XintfRegs.XTIMING6.bit.USEREADY = 0;
		XintfRegs.XTIMING6.bit.READYMODE = 0;  // sample asynchronous

		// Size must be either:
		// 0,1 = x32 or
		// 1,1 = x16 other values are reserved
		XintfRegs.XTIMING6.bit.XSIZE = 3;

		// Zone 7------------------------------------
		// When using ready, ACTIVE must be 1 or greater
		// Lead must always be 1 or greater
		// Zone write timing
		XintfRegs.XTIMING7.bit.XWRLEAD = 3;
		XintfRegs.XTIMING7.bit.XWRACTIVE = 7;
		XintfRegs.XTIMING7.bit.XWRTRAIL = 3;
		// Zone read timing
		XintfRegs.XTIMING7.bit.XRDLEAD = 3;
		XintfRegs.XTIMING7.bit.XRDACTIVE = 7;
		XintfRegs.XTIMING7.bit.XRDTRAIL = 3;

		// double all Zone read/write lead/active/trail timing
		XintfRegs.XTIMING7.bit.X2TIMING = 1;

		// Zone will NOT sample XREADY signal
		XintfRegs.XTIMING7.bit.USEREADY = 0;
		XintfRegs.XTIMING7.bit.READYMODE = 0;

		// Size must be either:
		// 0,1 = x32 or
		// 1,1 = x16 other values are reserved
		XintfRegs.XTIMING7.bit.XSIZE = 3;

		// Bank switching
		// Zone 6 is slow, so add additional BCYC cycles
		// when ever switching from Zone 6 to another Zone.
		// This will help avoid bus contention.
		XintfRegs.XBANK.bit.BANK = 6;
		XintfRegs.XBANK.bit.BCYC = 6;

	   //Force a pipeline flush to ensure that the write to
	   //the last register configured occurs before returning.
		GpioCtrlRegs.GPCMUX1.bit.GPIO64 = 3;  // XD15
		GpioCtrlRegs.GPCMUX1.bit.GPIO65 = 3;  // XD14
		GpioCtrlRegs.GPCMUX1.bit.GPIO66 = 3;  // XD13
		GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 3;  // XD12
		GpioCtrlRegs.GPCMUX1.bit.GPIO68 = 3;  // XD11
		GpioCtrlRegs.GPCMUX1.bit.GPIO69 = 3;  // XD10
		GpioCtrlRegs.GPCMUX1.bit.GPIO70 = 3;  // XD19
		GpioCtrlRegs.GPCMUX1.bit.GPIO71 = 3;  // XD8
		GpioCtrlRegs.GPCMUX1.bit.GPIO72 = 3;  // XD7
		GpioCtrlRegs.GPCMUX1.bit.GPIO73 = 3;  // XD6
		GpioCtrlRegs.GPCMUX1.bit.GPIO74 = 3;  // XD5
		GpioCtrlRegs.GPCMUX1.bit.GPIO75 = 3;  // XD4
		GpioCtrlRegs.GPCMUX1.bit.GPIO76 = 3;  // XD3
		GpioCtrlRegs.GPCMUX1.bit.GPIO77 = 3;  // XD2
		GpioCtrlRegs.GPCMUX1.bit.GPIO78 = 3;  // XD1
		GpioCtrlRegs.GPCMUX1.bit.GPIO79 = 3;  // XD0

		GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 3;  // XA0/XWE1n
		GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 3;  // XA1
		GpioCtrlRegs.GPBMUX1.bit.GPIO42 = 3;  // XA2
		GpioCtrlRegs.GPBMUX1.bit.GPIO43 = 3;  // XA3
		GpioCtrlRegs.GPBMUX1.bit.GPIO44 = 3;  // XA4
		GpioCtrlRegs.GPBMUX1.bit.GPIO45 = 3;  // XA5
		GpioCtrlRegs.GPBMUX1.bit.GPIO46 = 3;  // XA6
		GpioCtrlRegs.GPBMUX1.bit.GPIO47 = 3;  // XA7

		GpioCtrlRegs.GPCMUX2.bit.GPIO80 = 3;  // XA8
		GpioCtrlRegs.GPCMUX2.bit.GPIO81 = 3;  // XA9
		GpioCtrlRegs.GPCMUX2.bit.GPIO82 = 3;  // XA10
		GpioCtrlRegs.GPCMUX2.bit.GPIO83 = 3;  // XA11
		GpioCtrlRegs.GPCMUX2.bit.GPIO84 = 3;  // XA12
		GpioCtrlRegs.GPCMUX2.bit.GPIO85 = 3;  // XA13
		GpioCtrlRegs.GPCMUX2.bit.GPIO86 = 3;  // XA14
		GpioCtrlRegs.GPCMUX2.bit.GPIO87 = 3;  // XA15
		GpioCtrlRegs.GPBMUX1.bit.GPIO39 = 3;  // XA16
		GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 3;  // XA17
		GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 3;  // XA18

		//GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 3;  // XREADY
		GpioCtrlRegs.GPBMUX1.bit.GPIO35 = 3;  // XRNW
		GpioCtrlRegs.GPBMUX1.bit.GPIO38 = 3;  // XWE0

		GpioCtrlRegs.GPBMUX1.bit.GPIO36 = 3;  // XZCS0
		GpioCtrlRegs.GPBMUX1.bit.GPIO37 = 3;  // XZCS7
		//GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 3;  // XZCS6

		asm(" RPT #7 || NOP");
	}
	EDIS;
}
#pragma CODE_SECTION(ggRevBufIsLegal, "ramfuncs");
int ggRevBufIsLegal(unsigned char * pSrc)
{
	int i=0;
	unsigned char sum=0;
	for(i=0;i<9;i++)
	{
		sum+=pSrc[i];
	}
	sum=sum&0x00ff;
	if(sum==pSrc[9])
	{
		return 1;
	}
	return 0;
}
////////////////////////////////////script explain system//////////////////////////////////////////////////////////////
static int nFun;
static int nTargetTrue;
static int nTargetFalse;
static int * para;
static int * p;
#pragma CODE_SECTION(ggFun0, "ramfuncs");
#pragma CODE_SECTION(ggFun1, "ramfuncs");
#pragma CODE_SECTION(ggFun2, "ramfuncs");
#pragma CODE_SECTION(ggFun3, "ramfuncs");
#pragma CODE_SECTION(ggFun4, "ramfuncs");
#pragma CODE_SECTION(ggFun5, "ramfuncs");
#pragma CODE_SECTION(ggFun6, "ramfuncs");
#pragma CODE_SECTION(ggFun7, "ramfuncs");
#pragma CODE_SECTION(ggFun8, "ramfuncs");
#pragma CODE_SECTION(ggFun9, "ramfuncs");
#pragma CODE_SECTION(ggFun10, "ramfuncs");
#pragma CODE_SECTION(ggFun11, "ramfuncs");
#pragma CODE_SECTION(ggFun12, "ramfuncs");
#pragma CODE_SECTION(ggFun13, "ramfuncs");
#pragma CODE_SECTION(ggFun14, "ramfuncs");
#pragma CODE_SECTION(ggFun15, "ramfuncs");
#pragma CODE_SECTION(ggFun16, "ramfuncs");
#pragma CODE_SECTION(ggFun17, "ramfuncs");
#pragma CODE_SECTION(ggFun18, "ramfuncs");
#pragma CODE_SECTION(ggFun19, "ramfuncs");
#pragma CODE_SECTION(ggFun20, "ramfuncs");
#pragma CODE_SECTION(ggFun21, "ramfuncs");
#pragma CODE_SECTION(ggFun22, "ramfuncs");
#pragma CODE_SECTION(ggFun23, "ramfuncs");
#pragma CODE_SECTION(ggFun24, "ramfuncs");
#pragma CODE_SECTION(ggFun25, "ramfuncs");
#pragma CODE_SECTION(ggFun26, "ramfuncs");
#pragma CODE_SECTION(ggFun27, "ramfuncs");
#pragma CODE_SECTION(ggFun28, "ramfuncs");
#pragma CODE_SECTION(ggFun29, "ramfuncs");
#pragma CODE_SECTION(ggFun30, "ramfuncs");
#pragma CODE_SECTION(ggFun31, "ramfuncs");
#pragma CODE_SECTION(ggFun32, "ramfuncs");
#pragma CODE_SECTION(ggFun33, "ramfuncs");
#pragma CODE_SECTION(ggFun34, "ramfuncs");
#pragma CODE_SECTION(ggFun35, "ramfuncs");
#pragma CODE_SECTION(ggFun36, "ramfuncs");
#pragma CODE_SECTION(ggFun37, "ramfuncs");
#pragma CODE_SECTION(ggFun38, "ramfuncs");
#pragma CODE_SECTION(ggFun39, "ramfuncs");
#pragma CODE_SECTION(ggFun40, "ramfuncs");
#pragma CODE_SECTION(ggFun41, "ramfuncs");
#pragma CODE_SECTION(ggFun42, "ramfuncs");
#pragma CODE_SECTION(ggFun43, "ramfuncs");
#pragma CODE_SECTION(ggFun44, "ramfuncs");
#pragma CODE_SECTION(ggFun45, "ramfuncs");
#pragma CODE_SECTION(ggFun46, "ramfuncs");
#pragma CODE_SECTION(ggFun47, "ramfuncs");
#pragma CODE_SECTION(ggFun48, "ramfuncs");
#pragma CODE_SECTION(ggFun49, "ramfuncs");
#pragma CODE_SECTION(ggFun50, "ramfuncs");
#pragma CODE_SECTION(ggFun51, "ramfuncs");
#pragma CODE_SECTION(ggFun52, "ramfuncs");
#pragma CODE_SECTION(ggFun53, "ramfuncs");
#pragma CODE_SECTION(ggFun54, "ramfuncs");
#pragma CODE_SECTION(ggFun55, "ramfuncs");
#pragma CODE_SECTION(ggFun56, "ramfuncs");
#pragma CODE_SECTION(ggFun57, "ramfuncs");
#pragma CODE_SECTION(ggFun58, "ramfuncs");
#pragma CODE_SECTION(ggFun59, "ramfuncs");
#pragma CODE_SECTION(ggFun60, "ramfuncs");
#pragma CODE_SECTION(ggFun61, "ramfuncs");
#pragma CODE_SECTION(ggFun62, "ramfuncs");
#pragma CODE_SECTION(ggFun63, "ramfuncs");
#pragma CODE_SECTION(ggFun64, "ramfuncs");
#pragma CODE_SECTION(ggFun65, "ramfuncs");
#pragma CODE_SECTION(ggFun66, "ramfuncs");
#pragma CODE_SECTION(ggFun67, "ramfuncs");
#pragma CODE_SECTION(ggFun68, "ramfuncs");
#pragma CODE_SECTION(ggFun69, "ramfuncs");
#pragma CODE_SECTION(ggFun70, "ramfuncs");
#pragma CODE_SECTION(ggFun71, "ramfuncs");
#pragma CODE_SECTION(ggFun72, "ramfuncs");
#pragma CODE_SECTION(ggFun73, "ramfuncs");
#pragma CODE_SECTION(ggFun74, "ramfuncs");
#pragma CODE_SECTION(ggFun75, "ramfuncs");
#pragma CODE_SECTION(ggFun76, "ramfuncs");
#pragma CODE_SECTION(ggFun77, "ramfuncs");
#pragma CODE_SECTION(ggFun78, "ramfuncs");
#pragma CODE_SECTION(ggFun79, "ramfuncs");
#pragma CODE_SECTION(ggFun80, "ramfuncs");
#pragma CODE_SECTION(ggFun81, "ramfuncs");
#pragma CODE_SECTION(ggFun82, "ramfuncs");
#pragma CODE_SECTION(ggFun83, "ramfuncs");
#pragma CODE_SECTION(ggFun84, "ramfuncs");
#pragma CODE_SECTION(ggFun85, "ramfuncs");
#pragma CODE_SECTION(ggFun86, "ramfuncs");
#pragma CODE_SECTION(ggFun87, "ramfuncs");
#pragma CODE_SECTION(ggFun88, "ramfuncs");
#pragma CODE_SECTION(ggFun89, "ramfuncs");
#pragma CODE_SECTION(ggFun90, "ramfuncs");
#pragma CODE_SECTION(ggFun91, "ramfuncs");
#pragma CODE_SECTION(ggFun92, "ramfuncs");
#pragma CODE_SECTION(ggFun93, "ramfuncs");
#pragma CODE_SECTION(ggFun94, "ramfuncs");
#pragma CODE_SECTION(ggFun95, "ramfuncs");
#pragma CODE_SECTION(ggFun96, "ramfuncs");
#pragma CODE_SECTION(ggFun97, "ramfuncs");
#pragma CODE_SECTION(ggFun98, "ramfuncs");
#pragma CODE_SECTION(ggFun99, "ramfuncs");
#pragma CODE_SECTION(ggFun100, "ramfuncs");
#pragma CODE_SECTION(ggFun101, "ramfuncs");
#pragma CODE_SECTION(ggFun102, "ramfuncs");
#pragma CODE_SECTION(ggFun103, "ramfuncs");
#pragma CODE_SECTION(ggFun104, "ramfuncs");
#pragma CODE_SECTION(ggFun105, "ramfuncs");
#pragma CODE_SECTION(ggFun106, "ramfuncs");
#pragma CODE_SECTION(ggFun107, "ramfuncs");
#pragma CODE_SECTION(ggFun108, "ramfuncs");
#pragma CODE_SECTION(ggFun109, "ramfuncs");
#pragma CODE_SECTION(ggFun110, "ramfuncs");
#pragma CODE_SECTION(ggFun111, "ramfuncs");
#pragma CODE_SECTION(ggFun112, "ramfuncs");
#pragma CODE_SECTION(ggFun113, "ramfuncs");
#pragma CODE_SECTION(ggFun114, "ramfuncs");
#pragma CODE_SECTION(ggFun115, "ramfuncs");
#pragma CODE_SECTION(ggFun116,"ramfuncs");
#pragma CODE_SECTION(ggFun117,"ramfuncs");
#pragma CODE_SECTION(ggFun118, "ramfuncs");
#pragma CODE_SECTION(ggFun119, "ramfuncs");
#pragma CODE_SECTION(ggFun120, "ramfuncs");
#pragma CODE_SECTION(ggFun121, "ramfuncs");
#pragma CODE_SECTION(ggFun122, "ramfuncs");
void ggFun0(){gScriptToRun=nTargetTrue;}
void ggFun1(){gVar[para[0]]=gVar[para[1]];gScriptToRun=nTargetTrue;}
void ggFun2(){gVar[para[0]]=gVar[para[1]]+gVar[para[2]];gScriptToRun=nTargetTrue;}
void ggFun3(){gVar[para[0]]=gVar[para[1]]-gVar[para[2]];gScriptToRun=nTargetTrue;}
void ggFun4(){gVar[para[0]]=gVar[para[1]]*gVar[para[2]];gScriptToRun=nTargetTrue;}
void ggFun5(){if(ABS(gVar[para[2]])>0.00001){gVar[para[0]]=gVar[para[1]]/gVar[para[2]];}gScriptToRun=nTargetTrue;}
void ggFun6(){gVar[para[0]]=sin(gVar[para[1]]);gScriptToRun=nTargetTrue;}
void ggFun7(){gVar[para[0]]=cos(gVar[para[1]]);gScriptToRun=nTargetTrue;}
void ggFun8(){gVar[para[0]]=tan(gVar[para[1]]);gScriptToRun=nTargetTrue;}
void ggFun9(){gVar[para[0]]=sqrt(gVar[para[1]]);gScriptToRun=nTargetTrue;}
void ggFun10(){gVar[para[0]]=atan(gVar[para[1]]);gScriptToRun=nTargetTrue;}
void ggFun11(){gVar[para[0]]=asin(gVar[para[1]]);gScriptToRun=nTargetTrue;}
void ggFun12(){gVar[para[0]]=acos(gVar[para[1]]);gScriptToRun=nTargetTrue;}
void ggFun13(){gVar[para[0]]=ABS(gVar[para[1]]);gScriptToRun=nTargetTrue;}
void ggFun14(){LIMIT(gVar[para[0]],gVar[para[1]],gVar[para[2]]);gScriptToRun=nTargetTrue;}
void ggFun15()
{
	if(gVar[para[0]]>gVar[para[1]])gVar[para[0]]=gVar[para[1]];
	if(gVar[para[0]]>gVar[para[2]])gVar[para[0]]=gVar[para[2]];
	if(gVar[para[0]]>gVar[para[3]])gVar[para[0]]=gVar[para[3]];
	if(gVar[para[0]]>gVar[para[4]])gVar[para[0]]=gVar[para[4]];
	if(gVar[para[0]]>gVar[para[5]])gVar[para[0]]=gVar[para[5]];
	gScriptToRun=nTargetTrue;
}
void ggFun16()
{
	if(gVar[para[0]]<gVar[para[1]])gVar[para[0]]=gVar[para[1]];
	if(gVar[para[0]]<gVar[para[2]])gVar[para[0]]=gVar[para[2]];
	if(gVar[para[0]]<gVar[para[3]])gVar[para[0]]=gVar[para[3]];
	if(gVar[para[0]]<gVar[para[4]])gVar[para[0]]=gVar[para[4]];
	if(gVar[para[0]]<gVar[para[5]])gVar[para[0]]=gVar[para[5]];
	gScriptToRun=nTargetTrue;
}
void ggFun17(){gScriptToRun=nTargetTrue;}
void ggFun18(){gVar[para[0]]=IN0;gScriptToRun=nTargetTrue;}
void ggFun19(){gVar[para[0]]=IN1;gScriptToRun=nTargetTrue;}
void ggFun20(){gVar[para[0]]=IN2;gScriptToRun=nTargetTrue;}
void ggFun21(){gVar[para[0]]=IN3;gScriptToRun=nTargetTrue;}
void ggFun22(){gVar[para[0]]=IN4;gScriptToRun=nTargetTrue;}
void ggFun23(){gVar[para[0]]=IN5;gScriptToRun=nTargetTrue;}
void ggFun24(){if(gVar[para[0]]>0.5){OUT0_HIGH;}else{OUT0_LOW;}gScriptToRun=nTargetTrue;}
void ggFun25(){if(gVar[para[0]]>0.5){OUT1_HIGH;}else{OUT1_LOW;}gScriptToRun=nTargetTrue;}
void ggFun26(){if(gVar[para[0]]>0.5){OUT2_HIGH;}else{OUT2_LOW;}gScriptToRun=nTargetTrue;}
void ggFun27(){if(gVar[para[0]]>0.5){OUT3_HIGH;}else{OUT3_LOW;}gScriptToRun=nTargetTrue;}
void ggFun28(){if(gVar[para[0]]>0.5){OUT4_HIGH;}else{OUT4_LOW;}gScriptToRun=nTargetTrue;}
void ggFun29(){if(gVar[para[0]]>0.5){OUT5_HIGH;}else{OUT5_LOW;}gScriptToRun=nTargetTrue;}
void ggFun30(){gVar[para[0]]=(gAdSampleValue[0]+gVar[para[1]])*gVar[para[2]];gScriptToRun=nTargetTrue;}
void ggFun31(){gVar[para[0]]=(gAdSampleValue[1]+gVar[para[1]])*gVar[para[2]];gScriptToRun=nTargetTrue;}
void ggFun32(){gVar[para[0]]=(gAdSampleValue[2]+gVar[para[1]])*gVar[para[2]];gScriptToRun=nTargetTrue;}
void ggFun33(){gVar[para[0]]=(gAdSampleValue[3]+gVar[para[1]])*gVar[para[2]];gScriptToRun=nTargetTrue;}
void ggFun34(){gVar[para[0]]=(gAdSampleValue[4]+gVar[para[1]])*gVar[para[2]];gScriptToRun=nTargetTrue;}
void ggFun35(){gVar[para[0]]=(gAdSampleValue[5]+gVar[para[1]])*gVar[para[2]];gScriptToRun=nTargetTrue;}
void ggFun36(){gVar[para[0]]=(gAdSampleValue[6]+gVar[para[1]])*gVar[para[2]];gScriptToRun=nTargetTrue;}
void ggFun37(){gVar[para[0]]=(gAdSampleValue[7]+gVar[para[1]])*gVar[para[2]];gScriptToRun=nTargetTrue;}
void ggFun38(){gVar[para[0]]=(gAdSampleValue[8]+gVar[para[1]])*gVar[para[2]];gScriptToRun=nTargetTrue;}
void ggFun39(){gVar[para[0]]=(gAdSampleValue[9]+gVar[para[1]])*gVar[para[2]];gScriptToRun=nTargetTrue;}
void ggFun40(){gVar[para[0]]=(gAdSampleValue[10]+gVar[para[1]])*gVar[para[2]];gScriptToRun=nTargetTrue;}
void ggFun41(){gVar[para[0]]=(gAdSampleValue[11]+gVar[para[1]])*gVar[para[2]];gScriptToRun=nTargetTrue;}
void ggFun42(){gVar[para[0]]=(gAdSampleValue[12]+gVar[para[1]])*gVar[para[2]];gScriptToRun=nTargetTrue;}
void ggFun43(){gVar[para[0]]=(gAdSampleValue[13]+gVar[para[1]])*gVar[para[2]];gScriptToRun=nTargetTrue;}
void ggFun44(){gVar[para[0]]=(gAdSampleValue[14]+gVar[para[1]])*gVar[para[2]];gScriptToRun=nTargetTrue;}
void ggFun45(){gVar[para[0]]=(gAdSampleValue[15]+gVar[para[1]])*gVar[para[2]];gScriptToRun=nTargetTrue;}
void ggFun46(){gVar[para[0]]=(gAdSampleValue[0]-gAdSampleOffset[0])*gVar[para[1]];gScriptToRun=nTargetTrue;}
void ggFun47(){gVar[para[0]]=(gAdSampleValue[1]-gAdSampleOffset[1])*gVar[para[1]];gScriptToRun=nTargetTrue;}
void ggFun48(){gVar[para[0]]=(gAdSampleValue[2]-gAdSampleOffset[2])*gVar[para[1]];gScriptToRun=nTargetTrue;}
void ggFun49(){gVar[para[0]]=(gAdSampleValue[3]-gAdSampleOffset[3])*gVar[para[1]];gScriptToRun=nTargetTrue;}
void ggFun50(){gVar[para[0]]=(gAdSampleValue[4]-gAdSampleOffset[4])*gVar[para[1]];gScriptToRun=nTargetTrue;}
void ggFun51(){gVar[para[0]]=(gAdSampleValue[5]-gAdSampleOffset[5])*gVar[para[1]];gScriptToRun=nTargetTrue;}
void ggFun52(){gVar[para[0]]=(gAdSampleValue[6]-gAdSampleOffset[6])*gVar[para[1]];gScriptToRun=nTargetTrue;}
void ggFun53(){gVar[para[0]]=(gAdSampleValue[7]-gAdSampleOffset[7])*gVar[para[1]];gScriptToRun=nTargetTrue;}
void ggFun54(){gVar[para[0]]=(gAdSampleValue[8]-gAdSampleOffset[8])*gVar[para[1]];gScriptToRun=nTargetTrue;}
void ggFun55(){gVar[para[0]]=(gAdSampleValue[9]-gAdSampleOffset[9])*gVar[para[1]];gScriptToRun=nTargetTrue;}
void ggFun56(){gVar[para[0]]=(gAdSampleValue[10]-gAdSampleOffset[10])*gVar[para[1]];gScriptToRun=nTargetTrue;}
void ggFun57(){gVar[para[0]]=(gAdSampleValue[11]-gAdSampleOffset[11])*gVar[para[1]];gScriptToRun=nTargetTrue;}
void ggFun58(){gVar[para[0]]=(gAdSampleValue[12]-gAdSampleOffset[12])*gVar[para[1]];gScriptToRun=nTargetTrue;}
void ggFun59(){gVar[para[0]]=(gAdSampleValue[13]-gAdSampleOffset[13])*gVar[para[1]];gScriptToRun=nTargetTrue;}
void ggFun60(){gVar[para[0]]=(gAdSampleValue[14]-gAdSampleOffset[14])*gVar[para[1]];gScriptToRun=nTargetTrue;}
void ggFun61(){gVar[para[0]]=(gAdSampleValue[15]-gAdSampleOffset[15])*gVar[para[1]];gScriptToRun=nTargetTrue;}
void ggFun62(){ABtoXY(gVar[para[0]],gVar[para[1]],&gVar[para[2]],&gVar[para[3]]);gScriptToRun=nTargetTrue;}
void ggFun63(){XYtoAB(gVar[para[0]],gVar[para[1]],&gVar[para[2]],&gVar[para[3]]);gScriptToRun=nTargetTrue;}
void ggFun64()
{
	gVar[para[2]]=sqrt(gVar[para[0]]*gVar[para[0]]+gVar[para[1]]*gVar[para[1]]);
	gVar[para[3]]=atan(gVar[para[1]]/gVar[para[0]]);
	gScriptToRun=nTargetTrue;
}
void ggFun65()
{
	gVar[para[2]]=gVar[para[0]]*cos(gVar[para[1]]);
	gVar[para[3]]=gVar[para[0]]*sin(gVar[para[1]]);
	gScriptToRun=nTargetTrue;
}
void ggFun66()
{
	ABtoDQ(gVar[para[0]],gVar[para[1]],gVar[para[2]],&gVar[para[3]],&gVar[para[4]]);
	gScriptToRun=nTargetTrue;
}
void ggFun67()
{
	DQtoAB(gVar[para[0]],gVar[para[1]],gVar[para[2]],&gVar[para[3]],&gVar[para[4]]);
	gScriptToRun=nTargetTrue;
}
void ggFun68()
{
	gVar[para[5]]=ggPIRunAndReset(0,gVar[para[0]]-gVar[para[1]],gVar[para[2]],gVar[para[3]],gVar[para[4]]);
	gScriptToRun=nTargetTrue;
}
void ggFun69()
{
	gVar[para[5]]=ggPIRunAndReset(1,gVar[para[0]]-gVar[para[1]],gVar[para[2]],gVar[para[3]],gVar[para[4]]);
	gScriptToRun=nTargetTrue;
}
void ggFun70()
{
	gVar[para[5]]=ggPIRunAndReset(2,gVar[para[0]]-gVar[para[1]],gVar[para[2]],gVar[para[3]],gVar[para[4]]);
	gScriptToRun=nTargetTrue;
}
void ggFun71()
{
	gVar[para[5]]=ggPIRunAndReset(3,gVar[para[0]]-gVar[para[1]],gVar[para[2]],gVar[para[3]],gVar[para[4]]);
	gScriptToRun=nTargetTrue;
}
void ggFun72()
{
	gVar[para[5]]=ggPIRunAndReset(4,gVar[para[0]]-gVar[para[1]],gVar[para[2]],gVar[para[3]],gVar[para[4]]);
	gScriptToRun=nTargetTrue;
}
void ggFun73()
{
	//pi5Run
	gVar[para[5]]=ggPIRunAndReset(5,gVar[para[0]]-gVar[para[1]],gVar[para[2]],gVar[para[3]],gVar[para[4]]);
	gScriptToRun=nTargetTrue;
}
void ggFun74()
{
	//pi6Run
	gVar[para[5]]=ggPIRunAndReset(6,gVar[para[0]]-gVar[para[1]],gVar[para[2]],gVar[para[3]],gVar[para[4]]);
	gScriptToRun=nTargetTrue;
}
void ggFun75()
{
	//pi7Run
	gVar[para[5]]=ggPIRunAndReset(7,gVar[para[0]]-gVar[para[1]],gVar[para[2]],gVar[para[3]],gVar[para[4]]);
	gScriptToRun=nTargetTrue;
}
void ggFun76()
{
	//pi8Run
	gVar[para[5]]=ggPIRunAndReset(8,gVar[para[0]]-gVar[para[1]],gVar[para[2]],gVar[para[3]],gVar[para[4]]);
	gScriptToRun=nTargetTrue;
}
void ggFun77()
{
	//pi9Run
	gVar[para[5]]=ggPIRunAndReset(9,gVar[para[0]]-gVar[para[1]],gVar[para[2]],gVar[para[3]],gVar[para[4]]);
	gScriptToRun=nTargetTrue;
}
void ggFun78()
{
	//pi0Reset
	ggPIRunAndReset(10,0,0,0,0);
	gScriptToRun=nTargetTrue;
}
void ggFun79()
{
	//pi1Reset
	ggPIRunAndReset(11,0,0,0,0);
	gScriptToRun=nTargetTrue;
}
void ggFun80()
{
	//pi2Reset
	ggPIRunAndReset(12,0,0,0,0);
	gScriptToRun=nTargetTrue;
}
void ggFun81()
{
	//pi3Reset
	ggPIRunAndReset(13,0,0,0,0);
	gScriptToRun=nTargetTrue;
}
void ggFun82()
{
	//pi4Reset
	ggPIRunAndReset(14,0,0,0,0);
	gScriptToRun=nTargetTrue;
}
void ggFun83()
{
	//pi5Reset
	ggPIRunAndReset(15,0,0,0,0);
	gScriptToRun=nTargetTrue;
}
void ggFun84()
{
	//pi6Reset
	ggPIRunAndReset(16,0,0,0,0);
	gScriptToRun=nTargetTrue;
}
void ggFun85()
{
	//pi7Reset
	ggPIRunAndReset(17,0,0,0,0);
	gScriptToRun=nTargetTrue;
}
void ggFun86()
{
	//pi8Reset
	ggPIRunAndReset(18,0,0,0,0);
	gScriptToRun=nTargetTrue;
}
void ggFun87()
{
	//pi9Reset
	ggPIRunAndReset(19,0,0,0,0);
	gScriptToRun=nTargetTrue;
}
void ggFun88(){ggEnablePwm(1);gScriptToRun=nTargetTrue;}
void ggFun89(){ggEnablePwm(0);gScriptToRun=nTargetTrue;}
void ggFun90(){gVar[para[0]]=ggEnablePwm(2);gScriptToRun=nTargetTrue;}
void ggFun91(){ggChangePwmXRatio(1,gVar[para[0]]);gScriptToRun=nTargetTrue;}
void ggFun92(){ggChangePwmXRatio(2,gVar[para[0]]);gScriptToRun=nTargetTrue;}
void ggFun93(){ggChangePwmXRatio(3,gVar[para[0]]);gScriptToRun=nTargetTrue;}
void ggFun94(){ggChangePwmXRatio(4,gVar[para[0]]);gScriptToRun=nTargetTrue;}
void ggFun95(){ggChangePwmXRatio(5,gVar[para[0]]);gScriptToRun=nTargetTrue;}
void ggFun96(){ggChangePwmXRatio(6,gVar[para[0]]);gScriptToRun=nTargetTrue;}
void ggFun97(){if(SCICCANSEND){ScicRegs.SCITXBUF=gVar[para[0]];gVar[para[1]]=1;}else{gVar[para[1]]=0;}gScriptToRun=nTargetTrue;}
void ggFun98(){if(SCICCANREV){gVar[para[0]]=ScicRegs.SCIRXBUF.all;gVar[para[1]]=1;}else{gVar[para[1]]=0;}gScriptToRun=nTargetTrue;}
void ggFun99(){long int addr;addr=gVar[para[0]];addr+=0x4000;gVar[para[1]]=*((int*)addr);gScriptToRun=nTargetTrue;}
void ggFun100(){long int addr;addr=gVar[para[0]];addr+=0x4000;*((int*)addr)=(int)gVar[para[1]];gScriptToRun=nTargetTrue;}
void ggFun101(){if(gVar[para[0]]>gVar[para[1]]){gScriptToRun=nTargetTrue;}else{gScriptToRun=nTargetFalse;}}
void ggFun102(){if(gVar[para[0]]<gVar[para[1]]){gScriptToRun=nTargetTrue;}else{gScriptToRun=nTargetFalse;}}
void ggFun103(){if(FEQU(gVar[para[0]],gVar[para[1]])){gScriptToRun=nTargetTrue;}else{gScriptToRun=nTargetFalse;}}
void ggFun104(){if(gVar[para[0]]<=gVar[para[1]]){gScriptToRun=nTargetTrue;}else{gScriptToRun=nTargetFalse;}}
void ggFun105(){if(gVar[para[0]]>=gVar[para[1]]){gScriptToRun=nTargetTrue;}else{gScriptToRun=nTargetFalse;}}
void ggFun106(){if(FNEQU(gVar[para[0]],gVar[para[1]])){gScriptToRun=nTargetTrue;}else{gScriptToRun=nTargetFalse;}}
void ggFun107(){gScriptToRun=nTargetTrue;}
void ggFun108(){gScriptToRun=nTargetTrue;}
void ggFun109(){if(gVar[para[0]]>gVar[para[1]]){gScriptToRun=nTargetTrue;}else{gScriptToRun=nTargetFalse;}}
void ggFun110(){if(gVar[para[0]]<gVar[para[1]]){gScriptToRun=nTargetTrue;}else{gScriptToRun=nTargetFalse;}}
void ggFun111(){if(FEQU(gVar[para[0]],gVar[para[1]])){gScriptToRun=nTargetTrue;}else{gScriptToRun=nTargetFalse;}}
void ggFun112(){if(gVar[para[0]]<=gVar[para[1]]){gScriptToRun=nTargetTrue;}else{gScriptToRun=nTargetFalse;}}
void ggFun113(){if(gVar[para[0]]>=gVar[para[1]]){gScriptToRun=nTargetTrue;}else{gScriptToRun=nTargetFalse;}}
void ggFun114(){if(FNEQU(gVar[para[0]],gVar[para[1]])){gScriptToRun=nTargetTrue;}else{gScriptToRun=nTargetFalse;}}
void ggFun115(){gScriptToRun=nTargetTrue;}
void ggFun116(){gScriptToRun=nTargetTrue;}
void ggFun117(){gScriptToRun=gScriptLineNum;}
void ggFun118(){gScriptToRun=nTargetTrue;}
void ggFun119(){gScriptToRun=nTargetTrue;}
void ggFun120(){gScriptToRun=nTargetTrue;}
void ggFun121()
{
	long int tmp=0;
	long int tmpLow=0;

	tmp=gVar[para[1]];
	tmpLow=gVar[para[2]];
	tmp=(tmp<<8)|tmpLow;

	gVar[para[0]]=tmp;

	gScriptToRun=nTargetTrue;
}
void ggFun122()
{
	long int tmp=0;
	long int tmpLow=0;

	tmp=gVar[para[0]];
	tmp&=0xffff;

	tmpLow=tmp>>8;
	gVar[para[1]]=tmpLow;

	tmpLow=tmp&0xff;
	gVar[para[2]]=tmpLow;
	gScriptToRun=nTargetTrue;
}
void (*gFunAddr[123])()=
{
	&ggFun0,&ggFun1,&ggFun2,&ggFun3,&ggFun4,&ggFun5,&ggFun6,&ggFun7,&ggFun8,&ggFun9,
	&ggFun10,&ggFun11,&ggFun12,&ggFun13,&ggFun14,&ggFun15,&ggFun16,&ggFun17,&ggFun18,&ggFun19,
	&ggFun20,&ggFun21,&ggFun22,&ggFun23,&ggFun24,&ggFun25,&ggFun26,&ggFun27,&ggFun28,&ggFun29,
	&ggFun30,&ggFun31,&ggFun32,&ggFun33,&ggFun34,&ggFun35,&ggFun36,&ggFun37,&ggFun38,&ggFun39,
	&ggFun40,&ggFun41,&ggFun42,&ggFun43,&ggFun44,&ggFun45,&ggFun46,&ggFun47,&ggFun48,&ggFun49,
	&ggFun50,&ggFun51,&ggFun52,&ggFun53,&ggFun54,&ggFun55,&ggFun56,&ggFun57,&ggFun58,&ggFun59,
	&ggFun60,&ggFun61,&ggFun62,&ggFun63,&ggFun64,&ggFun65,&ggFun66,&ggFun67,&ggFun68,&ggFun69,
	&ggFun70,&ggFun71,&ggFun72,&ggFun73,&ggFun74,&ggFun75,&ggFun76,&ggFun77,&ggFun78,&ggFun79,
	&ggFun80,&ggFun81,&ggFun82,&ggFun83,&ggFun84,&ggFun85,&ggFun86,&ggFun87,&ggFun88,&ggFun89,
	&ggFun90,&ggFun91,&ggFun92,&ggFun93,&ggFun94,&ggFun95,&ggFun96,&ggFun97,&ggFun98,&ggFun99,
	&ggFun100,&ggFun101,&ggFun102,&ggFun103,&ggFun104,&ggFun105,&ggFun106,&ggFun107,&ggFun108,&ggFun109,
	&ggFun110,&ggFun111,&ggFun112,&ggFun113,&ggFun114,&ggFun115,&ggFun116,&ggFun117,&ggFun118,&ggFun119,
	&ggFun120,&ggFun121,&ggFun122
};
//exe the scripts
#pragma CODE_SECTION(ggExeCurrentScript, "ramfuncs");
void ggExeCurrentScript()
{
	//vars related to this script line
	p=gScript[gScriptToRun];
	nFun=p[0];
	nTargetTrue=p[1];
	nTargetFalse=p[2];
	para=p+3;
	/*
	nFun=gScript[gScriptToRun][0];
	nTargetTrue=gScript[gScriptToRun][1];
	nTargetFalse=gScript[gScriptToRun][2];
	para[0]=gScript[gScriptToRun][3];
	para[1]=gScript[gScriptToRun][4];
	para[2]=gScript[gScriptToRun][5];
	para[3]=gScript[gScriptToRun][6];
	para[4]=gScript[gScriptToRun][7];
	para[5]=gScript[gScriptToRun][8];
	para[6]=gScript[gScriptToRun][9];
	*/
	if(nFun>=0 && nFun<123)
	{
		gFunAddr[nFun]();
	}
	else
	{
		gErrCode=1;
		gScriptToRun=gScriptLineNum;
	}
}
#pragma CODE_SECTION(ggSciaSendUntilOver, "ramfuncs");
void ggSciaSendUntilOver(int data)
{
	while(1)
	{
		if (SCIACANSEND)
		{
			SciaRegs.SCITXBUF=data;
			break;
		}
	}
}
#pragma CODE_SECTION(ggWriteScriptToFlash, "ramfuncs");
int ggWriteScriptToFlash(int purevnum, int constvnum, int scriptnum)//1 is ok, 0 is fail
{
	long int i;
	long int j;
	int k;
	long int totalWordNum;
	int oneInt;
	int intLow;
	int intHigh;
	#define MAXTRY 600000
	//1.erase the flash
	if(ggEraseExFlash()!=1)
	{
		return 0;
	}
	//2.write 3 crital numbers
	if(ggWriteExFlash(10,purevnum)!=1)
	{
		return 0;
	}
	if(ggWriteExFlash(11,constvnum)!=1)
	{
		return 0;
	}
	if(ggWriteExFlash(12,scriptnum)!=1)
	{
		return 0;
	}
	//3. send relay 55 to tell the software to continue
	ggSciaSendUntilOver(55);
	//4. get one int, program it, send it back
	totalWordNum=purevnum*30+constvnum*30+scriptnum*10;
	for(i=0;i<totalWordNum;i++)
	{
		for(k=0;k<2;k++)
		{
			for(j=0;j<MAXTRY;j++)//60000 tries for one receive
			{
				if(SCIACANREV)//new char comes
				{
					if(k==0)
					{
						intLow=SciaRegs.SCIRXBUF.all;
						ggSciaSendUntilOver(intLow);//send it back
					}
					else
					{
						intHigh=SciaRegs.SCIRXBUF.all;
						ggSciaSendUntilOver(intHigh);//send it back
						oneInt=(intHigh<<8)|intLow;
						if(ggWriteExFlash(20+i,oneInt)!=1)//program it
						{
							return 0;
						}
					}
					break;
				}
			}
			if(j==MAXTRY)
			{
				return 0;
			}
		}

	}
	//5. get the last 10 FLAGS to enable the flash script, write them at the beginning
	for(i=0;i<10;i++)
	{
		for(j=0;j<MAXTRY;j++)//60000 tries for one int
		{
			if(SCIACANREV)//new char comes
			{
				oneInt=SciaRegs.SCIRXBUF.all;//get one char
				if(ggWriteExFlash(i,oneInt)!=1)//program it
				{
					return 0;
				}
				ggSciaSendUntilOver(oneInt);//send it back
				break;
			}
		}
		if(j==MAXTRY)
		{
			return 0;
		}
	}
	return 1;//successful
}
#pragma CODE_SECTION(ggCommuWithPcSoftware, "ramfuncs");
void ggCommuWithPcSoftware(int flag)
{
	int i;
	static int sendingMode=2;//0-sending data; 1-sending var info; 2-silent;
	static unsigned char revBuf[10];//data rev
	static int reportCnt;//counter when reporting var info, max=250*26
	static int chnVar[10];//chn var source
	static float chnGain[10];//chn var gain
	static float chnOffset[10];//chn offset
	if(flag==0)//init
	{
		sendingMode=2;
		reportCnt=0;
		for(i=0;i<10;i++)
		{
			revBuf[i]=0;
			chnVar[i]=0;
			chnGain[i]=1;
			chnOffset[i]=0;
		}
	}
	if(flag==2)//normal run
	{
		static unsigned char r;
		static int chn;
		static long int dataWhole=0;
		static long int dataHigh=0;
		static long int dataLow=0;
		if(SCIACANREV)//new char comes
		{
			r=SciaRegs.SCIRXBUF.all;
			revBuf[0]=revBuf[1];
			revBuf[1]=revBuf[2];
			revBuf[2]=revBuf[3];
			revBuf[3]=revBuf[4];
			revBuf[4]=revBuf[5];
			revBuf[5]=revBuf[6];
			revBuf[6]=revBuf[7];
			revBuf[7]=revBuf[8];
			revBuf[8]=revBuf[9];
			revBuf[9]=r;
		}
		//machine run
		if(revBuf[0]==0xAA && revBuf[1]==0xBB && revBuf[2]==0xCC)//change var source
		{
			if(ggRevBufIsLegal(revBuf)==1)
			{
				chn=revBuf[4];
				dataHigh=revBuf[5];
				dataHigh=(dataHigh<<8)|revBuf[6];
				dataLow=revBuf[7];
				dataLow=(dataLow<<8)|revBuf[8];
				dataWhole=(dataHigh<<16)|dataLow;

				switch(revBuf[3])
				{
					case 0:
						chnVar[chn]=dataWhole;//change chn var source
						break;
					case 1:
						chnGain[chn]=(float)dataWhole*0.0001;//chang chn gain
						break;
					case 2:
						sendingMode=0;//sending data mode
						break;
					case 3:
						sendingMode=1;//sending var info
						reportCnt=0;//begin with reporting
						break;
					case 4:
						sendingMode=2;//silent
						break;
					case 5://wirte to the flash
						ggEnablePwm(0);
						if(ggWriteScriptToFlash(revBuf[5],revBuf[6],dataLow)==1)
						{
							//reset the system, that is equal to power up again
							ggResetAllVar();
							ggLoadExFlashCode();
							sendingMode=2;
							reportCnt=0;
							for(i=0;i<10;i++)
							{
								revBuf[i]=0;
								chnVar[i]=0;
								chnGain[i]=1;
								chnOffset[i]=0;
							}
						}
						break;
					case 6:
						gVar[chnVar[chn]]=(float)dataWhole*0.0001;//change value positive
						break;
					case 7:
						gVar[chnVar[chn]]=-(float)dataWhole*0.0001;//change value negative
						break;
					case 8:
						chnOffset[chn]=(float)dataWhole*0.0001;
						break;
					case 9:
						chnOffset[chn]=-(float)dataWhole*0.0001;
						break;
					default:
						break;
				}
				//clear rev buffer
				revBuf[0]=0;
				revBuf[1]=0;
				revBuf[2]=0;
				revBuf[3]=0;
				revBuf[4]=0;
				revBuf[5]=0;
				revBuf[6]=0;
				revBuf[7]=0;
				revBuf[8]=0;
				revBuf[9]=0;
			}
		}
		switch(sendingMode)
		{
			case 0://send datas
				if (SCIACANSEND)
				{
					static int chnToSend=0;
					if(chnToSend==0)
					{
						SciaRegs.SCITXBUF=201;
					}
					else
					{
						float rst=0;
						int tmp=0;
						rst=gVar[chnVar[chnToSend]];
						tmp=(rst+chnOffset[chnToSend])*chnGain[chnToSend];
						if(tmp>100)
						{
							tmp=100;
						}
						if(tmp<-100)
						{
							tmp=-100;
						}
						tmp+=100;
						SciaRegs.SCITXBUF=(char)tmp;
					}

					chnToSend+=1;
					if(chnToSend>=10)
					{
						chnToSend=0;
					}
				}
				break;
			case 1://send var informations
				if(reportCnt<6500)//250*26=6500 chars
				{
					if(SCIACANSEND)
					{
						int tmp1;
						int tmp2;
						int tmpData;

						tmp1=reportCnt/26;
						tmp2=reportCnt-tmp1*26;

						if(tmp1>=0 && tmp1<gPureVarNum && tmp2>=0 && tmp2<=25)
						{
							tmpData=ggReadExFlash(20+tmp1*30+tmp2);
						}
						else
						{
							tmpData=0;
						}
						SciaRegs.SCITXBUF=tmpData;
						reportCnt+=1;
					}
				}
				break;
			case 2://silent
				break;
			default:
				break;
		}
	}
}
//
#pragma CODE_SECTION(ggADSample, "ramfuncs");
void ggADSample()
{
	AdcRegs.ADCST.all=0xffff;
	AdcRegs.ADCTRL2.bit.SOC_SEQ1=1;//
	while(AdcRegs.ADCST.bit.INT_SEQ1==0)//wait for over
	{
	}
	AdcRegs.ADCST.bit.INT_SEQ1_CLR=1;
	gAdSampleValue[0]=(AdcRegs.ADCRESULT0)>>4;
	gAdSampleValue[1]=(AdcRegs.ADCRESULT1)>>4;
	gAdSampleValue[2]=(AdcRegs.ADCRESULT2)>>4;
	gAdSampleValue[3]=(AdcRegs.ADCRESULT3)>>4;
	gAdSampleValue[4]=(AdcRegs.ADCRESULT4)>>4;
	gAdSampleValue[5]=(AdcRegs.ADCRESULT5)>>4;
	gAdSampleValue[6]=(AdcRegs.ADCRESULT6)>>4;
	gAdSampleValue[7]=(AdcRegs.ADCRESULT7)>>4;
	gAdSampleValue[8]=(AdcRegs.ADCRESULT8)>>4;
	gAdSampleValue[9]=(AdcRegs.ADCRESULT9)>>4;
	gAdSampleValue[10]=(AdcRegs.ADCRESULT10)>>4;
	gAdSampleValue[11]=(AdcRegs.ADCRESULT11)>>4;
	gAdSampleValue[12]=(AdcRegs.ADCRESULT12)>>4;
	gAdSampleValue[13]=(AdcRegs.ADCRESULT13)>>4;
	gAdSampleValue[14]=(AdcRegs.ADCRESULT14)>>4;
	gAdSampleValue[15]=(AdcRegs.ADCRESULT15)>>4;
}

#pragma CODE_SECTION(ggLedSpark, "ramfuncs");
void ggLedSpark()
{
	static int secCnt=0;
	static int lightCnt=0;

	if(secCnt<(FS>>1))
	{
		secCnt+=1;
	}
	else
	{
		secCnt=0;
	}

	if(secCnt==(FS>>2))
	{
		lightCnt+=1;
	}

	if(lightCnt<=gErrCode)
	{
		if(secCnt<(FS>>2))
		{
			IO_LEDON;
		}
		else
		{
			IO_LEDOFF;
		}
	}
	else
	{
		IO_LEDOFF;
	}
	if(lightCnt>=gErrCode+6)
	{
		lightCnt=0;
	}
}
#pragma CODE_SECTION(epwm1CallBack, "ramfuncs");
interrupt void epwm1CallBack()
{
	gTsHappen=1;
	if(gnAdZeroCnt<10000)
	{
		if(gnAdZeroCnt>=5000)
		{
			ggADSample();
			gAdSampleOffset[0]+=gAdSampleValue[0]*TS;
			gAdSampleOffset[1]+=gAdSampleValue[1]*TS;
			gAdSampleOffset[2]+=gAdSampleValue[2]*TS;
			gAdSampleOffset[3]+=gAdSampleValue[3]*TS;
			gAdSampleOffset[4]+=gAdSampleValue[4]*TS;
			gAdSampleOffset[5]+=gAdSampleValue[5]*TS;
			gAdSampleOffset[6]+=gAdSampleValue[6]*TS;
			gAdSampleOffset[7]+=gAdSampleValue[7]*TS;
			gAdSampleOffset[8]+=gAdSampleValue[8]*TS;
			gAdSampleOffset[9]+=gAdSampleValue[9]*TS;
			gAdSampleOffset[10]+=gAdSampleValue[10]*TS;
			gAdSampleOffset[11]+=gAdSampleValue[11]*TS;
			gAdSampleOffset[12]+=gAdSampleValue[12]*TS;
			gAdSampleOffset[13]+=gAdSampleValue[13]*TS;
			gAdSampleOffset[14]+=gAdSampleValue[14]*TS;
			gAdSampleOffset[15]+=gAdSampleValue[15]*TS;
		}
		gnAdZeroCnt+=1;
	}
	else
	{
		//ad
		ggADSample();
		//led for the infomation
		ggLedSpark();
	}
	//commu
	ggCommuWithPcSoftware(2);

	//enable interrupt again
	EPwm1Regs.ETCLR.bit.INT=1;
	PieCtrlRegs.PIEACK.all |= PIEACK_GROUP3;
}
void ggEnableInt()
{
	EALLOW;
	//for pwm interrupt
	IER|=M_INT3;//global enable
	PieCtrlRegs.PIEIER3.bit.INTx1 = 1;// for the epwm1 interrupt
	PieVectTable.EPWM1_INT=&epwm1CallBack;
	//clear all pie flag
	PieCtrlRegs.PIEACK.all |= 0xffff;
	//clear global interrupt flag
	EINT;   // Enable Global interrupt INTM
	ERTM;   // Enable Global realtime interrupt DBGM
	EDIS;
}
#pragma CODE_SECTION(ggLoopForEver, "ramfuncs");
void ggLoopForEver()
{
 	for(;;)
	{
 		gTsHappen=0;
 		while(1){if(gTsHappen==1)break;}
 		if(gnAdZeroCnt>=10000)
 		{
 			gScriptToRun=0;
			while(gScriptToRun<gScriptLineNum)//exe the scripts
			{
				ggExeCurrentScript();//exe script
			}
 		}
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////main()/////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
extern unsigned int RamfuncsLoadStart;
extern unsigned int RamfuncsLoadEnd;
extern unsigned int RamfuncsRunStart;

void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr)
{
	while(SourceAddr < SourceEndAddr)
	{
	   *DestAddr++ = *SourceAddr++;
	}
	return;
}
void main()
{
	InitSysCtrl();//
	EALLOW;
	SysCtrlRegs.HISPCP.all = 2;// HSPCLK = SYSCLKOUT/2/2
	EDIS;
	DINT;//disable int
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
	InitFlash();
	InitPieCtrl();
	IER = 0x0000;
	IFR = 0x0000;
	InitPieVectTable();//server
	InitAdc();
	ggUserParaSetup();
	ggResetAllVar();
	DELAY_US(1000000);
	ggLoadExFlashCode();
	ggCommuWithPcSoftware(0);//0=means init the commu function
	ggEnableInt();
	ggLoopForEver();
}
