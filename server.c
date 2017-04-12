/**+**************************************************************************
* NAME           :  server.c                                                 *
* DESCRIPTION    :  IEC104 server program                                    *
*                                                                            *
* PROCESS        :                                                            *
*                                                                            *
*                                                                            *
*                                                                            *
*                                                                            *
* [C] Copyright Motorola Inc,  2006. All Rights Reserved                     *
*                                                                            *
* REV    DATE     PROGRAMMER         REVISION HISTORY                        *
* 1.0  2012.07.03 Gergely Zsolt       Original                                *
* 1.1  2012.07.13 Gergely Zsolt       Single command, select                  *  
* 1.2  2012.09.12 Gergely Zsolt       MAX_ASDU 50-re lett csökkentve          * 
* 1.3  2013.09.11 Gergely Zsolt       Receive number kezelése inf. transfernél is
*                                     ASDU puffer túlcsodulás kezelése
*                                     Receive number elcsúszás kezelése
* 1.4  2014.01.21. Gergely Zsolt      fnReadPar meghívása az fnTaskSpont-ból is
*                                     az adatok mentése miatt
* 1.5 2014.05.12. Gergely Zsolt       Új statistikák: általános lekérdezések, 
*                                     kapcsolat megszakadások száma     
* 1.6 2014.12.11. Gergely Zsolt       nRecSeqNum növelésének módosítása
* 1.7 2015.05.16. Gergely Zsolt       SRAM kezelés módosítása: több idõnek kell
* eltelnie az írás és az olvasás között                                  
* 1.8 2015.05.04					  Ha nem jó a dinamikus site table, nem
									  kommunikál           
* 1.9 2015.09.21					  Ha nem jó az RTU ideje, akkor a Front End
									  idejét használja									                     
***-*************************************************************************/

/*--------------------------------------------------*/
/* INCLUDES AND DEFINES                             */
/*--------------------------------------------------*/
#include "CAPPLIC.H"
#include "moscsock.h"
#include "stdio.h"
#include "string.h"

#define MAX_ASDU		80

#define MAX_CONN		2
/* #define MAX_MESS_NUM	100 */

#define BYTE			unsigned char
#define S_FORMAT		1
#define I_FORMAT		0
#define START_CHAR		0x68


#define FIRST_TIME 0
#define MAX_RX_BUFF 270
#define MAX_TX_BUFF 270
#define MAX_SP_EVNUM 20
#define MAX_DP_EVNUM 20
#define MAX_NM_EVNUM 20

#define MAX_NM_NUM 1200
#define MAX_SP_NUM 6250
#define MAX_DP_NUM 1000

#define	BOOL				int
#define	BYTE				unsigned char

#define	IEC_VAR_START		0x68
#define	IEC_VAR_END			0x16
#define	IEC_FIX_START		0x10
#define	IEC_FIX_END			0x16

#define	IEC_CTRL			0xE5
#define	IEC_ACK				0xE5

#define	CF_LINK_RESET		0
#define	CF_APP_RESET        1
#define	CF_LINK_TEST        2
#define CF_DATA_CONFIRM     3
#define CF_DATA_NOREPLY    	4
#define	CF_LINK_STATUS_REQ  9


#define	CF_ACK              0
#define	CF_NACK             1
#define	CF_LINK_STATUS_RES	11


#define	TR_REQ_STATUS              0
#define	TR_LINK_RESET              1


/********************************************
*
*	ASDU TI kodok
*
********************************************/
/* Process informacio - felugyeleti irany*/
#define	TI_M_SP_NA_1		1
#define TI_M_SP_TA_1        2
#define TI_M_DP_NA_1        3
#define TI_M_DP_TA_1        4
#define TI_M_ME_NA_1        9
#define TI_M_ME_TA_1        10
#define TI_M_ME_NC_1        13
#define TI_M_ME_TC_1        14
#define TI_M_IT_NA_1        15
#define TI_M_IT_TA_1        16

/* with long time tag (7 octets)*/
#define TI_M_SP_TB_1        30
#define TI_M_DP_TB_1        31
#define TI_M_ME_TD_1        34



/* Process informacio - vezerlo irany*/
#define	TI_C_SC_NA_1        45
#define TI_C_DC_NA_1        46
#define TI_C_RC_NA_1        47


/* Rendszer informacio - felugyeleti irany*/
#define TI_M_EI_NA_1        70

/* Rendszer informacio - vezerlo irany*/
#define TI_C_IC_NA_1        100
#define TI_C_CI_NA_1        101
#define TI_C_CS_NA_1        103
#define TI_C_TS_NA_1        104
#define TI_C_RP_NA_1        105

/* File transzfer tipuskodok*/
#define TI_F_FR_NA_1        120
#define TI_F_SR_NA_1        121
#define TI_F_SC_NA_1        122
#define TI_F_LS_NA_1        123
#define TI_F_AF_NA_1        124
#define TI_F_SG_NA_1        125
#define TI_F_DR_TA_1        126

/* Tombatvitel (specialis taviratok)*/
#define TI_S_AR_NA        	136
#define TI_S_AR_TA        	137
/********************************************
*
*	ASDU COT kodok
*
********************************************/
#define	COT_NOT_USED		0
#define	COT_PER_CYC			1
#define	COT_UPDATE			2
#define	COT_SPONT			3
#define	COT_INIT			4
#define	COT_REQ				5
#define	COT_ACT				6
#define	COT_ACTCON			7
#define	COT_DEACT			8
#define	COT_DEACTCON		9
#define	COT_ACTTERM			10
#define	COT_RETREM			11
#define	COT_RETLOC			12

#define COT_FILETRAN		13

#define	COT_INROGEN			20
#define	COT_REQCOGEN		37
#define	COT_REQCOSP			42

#define	IEC_CAOA_LEN_MAX	2
#define	IEC_IOA_LEN_MAX		3



#define	INDX0				0
#define	INDX1				1


/* Valtozo hosszusagu keret header szerkezete */
typedef	struct
	{
	BYTE			byStart1 ;
	BYTE			byLen1 ;
	BYTE			byLen2 ;
	BYTE			byStart2 ;
	}	IEC_VAR_HDR ;

typedef	struct
	{
	BYTE			byFnCode	;
	BYTE			byFCV_DFC	;
	BYTE 			byFCB_RES	;
	BYTE 			byPRM		;
	BYTE 			byDIR		;
	}	IEC_CTRL_FLD ;
/* DATA UNIT IDENTIFIER szerkezete*/
typedef	struct
	{
	BYTE			byTI ;
	BYTE			byDataNum ;
	BYTE			bySequence;
	BYTE			byCOT;
	BYTE			byCAOA[IEC_CAOA_LEN_MAX];
	}	IEC_DUI ;	
	
/* DATA UNIT IDENTIFIER szerkezete, IEC60870-5-104 szerint*/	
typedef	struct
	{
	BYTE			byTI ;
	BYTE			byDataNum ;
	BYTE			bySequence;
	BYTE			byCOT;
	BYTE			byOrigAddr;
	BYTE			byCAOA[IEC_CAOA_LEN_MAX];
	}	IEC_DUI_104 ;	

/* Egybites jelzes, idotaggal, SQ=0*/
typedef struct
	{
		BYTE 		byIOA[3];
		BYTE		bySP;
		BYTE		byTime[3];
	}	IEC_M_SP_TA_1 ;	

/* Ketbites jelzes, idotaggal, SQ=0*/
typedef struct
	{
		BYTE 		byIOA[3];
		BYTE		byDP;
		BYTE		byTime[3];
	}	IEC_M_DP_TA_1 ;		

/* Normalizalt mert ertek, SQ=0 */
typedef struct
	{
		BYTE 				byIOA[3];
		BYTE		 		byNM[2];
		BYTE				byQ;
	}	IEC_M_ME_NA_1;	
/* Ketszeres parancs*/
typedef struct
	{
	BYTE		byIOA[4];
	BYTE		byDC;
	BYTE		bySE;
	}	IEC_C_DC_NA_1 ;	
/* Egyszeres parancs*/
typedef struct
	{
	BYTE		byIOA[4];
	BYTE		bySC;
	BYTE		bySE;
	}	IEC_C_SC_NA_1 ;			

typedef	struct
	{
	BYTE			byMs[2] ;
	BYTE			byMin ;
	BYTE			byHour ;
	BYTE			byDay ;
	BYTE			byWeekDay;
	BYTE			byMon ;
	BYTE			byYear ;
	BYTE			bySummerTime;
	
	}	IEC_CP56Time ;
	
	
/* IEC60870-5-104 ------------------------------------------------------------------------------------------------------------------------*/			
/* CP56TIME2A ido szerkezete */
typedef	struct
	{
	BYTE			byMs[2] ;
	BYTE			byMin ;
	BYTE			byHour ;
	BYTE			byDayMonth_Week ;
	BYTE			byMon ;
	BYTE			byYear ;
	}	IEC_CP56Time2a ;

/* Egybites jelzes, idotaggal, SQ=0*/
typedef struct
	{
		BYTE 			byIOA[3];
		BYTE			bySP;
		IEC_CP56Time2a	sTime;
	}	IEC_M_SP_TB_1 ;	

/* Ketbites jelzes, idotaggal, SQ=0*/
typedef struct
	{
		BYTE 		byIOA[3];
		BYTE		byDP;
		IEC_CP56Time2a	sTime;
	} IEC_M_DP_TB_1 ;	
	
	
/* Normalizalt mert ertek, idõtaggal SQ=0 */
typedef struct
	{
		BYTE 				byIOA[3];
		BYTE		 		byNM[2];
		BYTE				byQ;
		IEC_CP56Time2a	sTime;		
	}	IEC_M_ME_TD_1;	
/* Ketszeres parancs*/



/* End of IEC60870-5-104 ------------------------------------------------------------------------------------------------------------------------*/			

/* SP tablak oszlopai */
typedef	struct
	{
	CB_TABLE_INFO   table_SP;
	short          *p_col_SP;
	short          *p_col_SP_MS1;
	short          *p_col_SP_MS2;
	short          *p_col_SP_MIN;
	short          *p_col_SP_CT;
	short          *p_col_SP_XOR;	
	short          *p_col_SP_STATUS;	
	
	}	strSP_TABLE ;	

/* NM tablak oszlopai */
typedef	struct
	{
	CB_TABLE_INFO   table_NM;
	short          *p_col_NM;
	short          *p_col_NM_LZ;
	short          *p_col_NM_Tx;
	short          *p_col_NM_STATUS;	
	}	strNM_TABLE ;	
	
/* DP tablak oszlopai */
typedef	struct
	{
	CB_TABLE_INFO   table_DP;
	short          *p_col_DPL;
	short          *p_col_DPH;
	short          *p_col_DP_MS1;
	short          *p_col_DP_MS2;
	short          *p_col_DP_MIN;
	short          *p_col_DP_CT;
	short          *p_col_DP_STATUS;
	}	strDP_TABLE ;	

/* Összes adat tárolásának struktúrája ***********************************************************************************************/
/* SP tablak oszlopai */
typedef	struct
	{
	short          SP;
	short          SP_MS1;
	short          SP_MS2;
	short          SP_MIN;
	short          SP_CT;
	short          SP_XOR;	
	short          SP_STATUS;	

	}	strSP ;

/* DP tablak oszlopai */
typedef	struct
	{
	short          DPL;
	short          DPH;
	short          DP_MS1;
	short          DP_MS2;
	short          DP_MIN;
	short          DP_CT;
	short          DP_STATUS;
	}	strDP ;
		
/* NM tablak oszlopai */
typedef	struct
	{
	
	short          NM;
	short          NM_LZ;
	short          NM_Tx;
	short          NM_STATUS;	
	}	strNM ;	



/* Összesített adatok */
typedef	struct
	{
	unsigned short nPAR[500];
	strSP          SP[MAX_SP_NUM];
	strDP          DP[MAX_DP_NUM];
	strNM          NM[MAX_NM_NUM];	
	}	strTotalData ;	
	
/* IEC táviratok eltárolása*/	
typedef struct			 	{
	char			sBuff[600];
	unsigned int	nLength;
	unsigned int	nSendNum;
							} strMessType;
							
							
/* 2012.04.26 */

typedef struct  {

	char			sBuff[400];
	unsigned int	nLength;
	unsigned int	nSendNum;
							} strASDUType;
 
				


/*************************************************************************************************************/


/*--------------------------------------------------*/
/* PROTOTYPES                                       */
/*--------------------------------------------------*/
void fnServer(unsigned short TableNumber1,unsigned short TableNumber2);
void fnReadSPDataTime(int nIEC_Offset, unsigned char *byData,  int *nMS1, int *nMS2, int *nMin, int *bTTime, int *nXOR);
void fnReadNMData(int nIEC_Offset, unsigned int *nData, unsigned int *nLiveZero, unsigned int *nStatus);
void fnReadDPDataTime(int nIEC_Offset, BYTE *byDP,  int *nMS1, int *nMS2, int *nMin, int *bTTime);
void fnReadDPData(int nIEC_Offset, BYTE byDP);
void fnReadSPData2(int nIEC_Offset, unsigned char *byData);

int fnBuildStartChar(BYTE *buf, BYTE byStartChar,int nNum);
int fnAPCISeqNums(BYTE *buf, unsigned int nSendSeqNum,unsigned int nRecSeqNum,int nFormat, int nNum);
int fnBuildMessLength(BYTE *buf, int nLength,int nNum);
void fnSpontTest(const int INDX);
int fnKliensSocket(const int INDX);
int fnKliensSocket_(const int INDX);
void fnRecSeqNum(int INDX, unsigned int nRecSeqNumClient);
void fnDisconnect(int INDX);


void fnSocketInit(int INDX);

void fnIEC_Init(void);

/* main_iec.c*/
/*extern void fnWriteNM( int nIECOffset,unsigned int nData);
extern void fnNMTblIndx(int nIECOffset, int *nNMTblIndx, int *nIndx);
extern void fnDPTblIndx(int nIECOffset, int *nDPTblIndx, int *nIndx);
extern void fnWriteSPData(int nIEC_Offset, int nData, int nMS1, int nMS2, int nMin, int nCTAct);
extern int fnReadSPData(int nIEC_Offset);
extern void fnDCTblIndx(int nIECOffset, int *nDCTblIndx, int *nOffset, short **p_col_DCAct);
extern void fnSCTblIndx(int nIECOffset, int *nSCTblIndx, int *nOffset, short **p_col_SCAct);
extern void fnReadSPData2(int nIEC_Offset, unsigned char *byData);
extern void fnReadSPDataTime(int nIEC_Offset, unsigned char *nData,  int *nMS1, int *nMS2, int *nMin, int *bTTime, int *nXOR);
extern void fnReadNMData(int nIEC_Offset, unsigned int *nData, unsigned int *nLiveZero, unsigned int *nStatus);
extern void fnReadDPDataTime(int nIEC_Offset, BYTE *byDP,  int *nMS1, int *nMS2, int *nMin, int *bTTime);
*/
void fnStart(void);
void IEC_DRV(unsigned short TableNumber1, unsigned short TableNumber2);
void fnIEC_Init(void);
void apl_task(void);
void rx_task(void);
void tx_task(void);
void apl_task(void);
void chk_task(void);
void fnEvents(IEC_DUI_104		duiRec, int INDX);
BYTE fnIEC_Csum(BYTE *byBuff, BYTE byNum);
void fnTxFCBInv(BOOL bFCV);
int fnBuildCF(BYTE *buf, BOOL bPrm, BOOL bFcv, int nCode, int nNum);
int fnBuildVarHeader(BYTE *buf, BYTE byLen, int nNum);
int fnBuildDUI(BYTE *buf, BYTE byTI, BOOL bSeq, BYTE byDataNum , BYTE byCOT, BYTE byOrigAddr, int nNum, IEC_DUI_104	*duiTransmit);
int fnBuildInfObj(BYTE *buf, BYTE *byIOA, BYTE *byData, int nNum,IEC_DUI_104	duiTransmit, int INDX);
void fnBuildVarStart(BYTE *buf, int nNum);
int fnBuildVarEnd(BYTE *buf, int nNum);
void fnBuildCP56Time2a(void);
void fnSetMOSCADTime(void);
void fnBuildCP56Time2aIEC(BYTE *buf);
void fnMemcpy(BYTE *dest, BYTE *src);
void fnReadPar(void);
void fnTx(void);
void fnRx(void);
void fnBuildIOA(BYTE *byAddr, unsigned long lIOA);
void fnLoHi(BYTE *byLo, BYTE *byHi, unsigned int nInt);
int  fnNorm(int nBe, int nLiveZero, BYTE *byNorm);
void fnSP_TABLE(int nNum);
void fnSetDCLogTime(int nDCIndex, BYTE byValue);
void fnNM_TABLE(int nNum);
void fnDP_TABLE(int nNum);
void fnSaveData(void);
void fnGetData(void);
void fnGetSPData(int nTableNum);
void fnGetDPData(int nTableNum);
void fnGetNMData(int nTableNum);
void fnSaveSPData(int nTableNum);
void fnSaveDPData(int nTableNum);
void fnSaveNMData(int nTableNum);
void fnResetSRAM(void);
void fnGetIntData(int nTableNum);
void fnSavePARData(int nTableNum);
void fnSendTESTFR_ACT(int INDX);



/* taskok */
void fnTaskCreateSocket(void);
void fnTaskServer0(void);
void fnTaskServer1(void);
void fnTaskSpont(void);



/*--------------------------------------------------*/
/* GLOBALS				                             */
/*--------------------------------------------------*/
char		message[812];

/*unsigned int	nSendSeqNumClient[MAX_CONN];
unsigned int	nRecSeqNumClient[MAX_CONN];*/

unsigned int	nAcknowledgedNum[MAX_CONN];
unsigned int	nSendSeqNum[MAX_CONN];
unsigned int	nRecSeqNum[MAX_CONN];

unsigned int	nStarted[MAX_CONN];

/*unsigned int	nSendSeqNumBIT_0[MAX_CONN];
unsigned int	nSendSeqNumBIT_1[MAX_CONN];*/

unsigned int	nTypeID[MAX_CONN];
unsigned int	nNumOfObj[MAX_CONN];
unsigned int	nCauseOfTx[MAX_CONN];
unsigned int	nCommand_SE[MAX_CONN];
MOSCAD_SOCKET 	newsocket[MAX_CONN];
unsigned int	nSI = 0;					/* Az aktuális socket indexe*/

/* ######################################################################################################################################################################### */
/* strMessType			strMess[MAX_CONN][MAX_MESS_NUM]; */	/* Üzenet puffer az összes socket-ra*/
/*unsigned int		nWritePtr[MAX_CONN];*/		/* Író pointer, új eseménytávirat esetén*/
/*unsigned int		nReadPtr[MAX_CONN];	*/		/* Olvasó pointer a kiküldõ függvénynek*/
/*unsigned int		nAckPtr[MAX_CONN];	*/		/* A visszanyugtázott távirat indexe */
/*unsigned int	  nSendSeqNum;
unsigned int	  nRecSeqNum;*/

unsigned int	nStarted[MAX_CONN];


MOSCAD_SOCKET 		sock;

/* ######################################################################################################################################################################### */
/* 2012.05.19  - Egy klienshez tartozó összes adat struktúrája ----------------------------------------------------------------*/
/* ######################################################################################################################################################################### */

typedef struct  {

MOSCAD_SOCKET 	newsocket;
/* strMessType			strMess[MAX_MESS_NUM]; */	    /* Üzenet puffer az összes socket-ra*/
unsigned int		nASDUWritePtr;		          /* Író pointer, új eseménytávirat esetén*/
unsigned int		nASDUSendPtr;			          /* Olvasó pointer a kiküldõ függvénynek*/
unsigned int	  nSendSeqNum;                /* A program saját belsõ számlálója */
unsigned int	  nRecSeqNum;                 /* A program saját belsõ számlálója */
unsigned int	  nStarted;				            /* Information transfer engedélyezett */		
              
unsigned long		lTickDC;                     /* Az elõkészítés parancs óta eltelt idõ*/              
int					    nInterrogationStep;
int					    nDCStep;
int					    nSCStep;
int					    nClockSyncStep;
              
/* Event képzés */              
int             nInterrogated;              /* Volt-e már teljes adat lekérdezés */                
int					    nSendSP ; 		              /* Elkuldott SP-k szama altalanos lekerdezeskor */
int					    nSendNM ;		                /* Elkuldott NM-k szama altalanos lekerdezeskor */
int					    nSendDP;		                /* Elkuldott DP-k szama altalanos lekerdezeskor */              
              
unsigned int		nPrNM[MAX_NM_NUM];            /* NM-k elõzõ értéke*/              
BYTE				    byPrSP[MAX_SP_NUM];           /* SP-k elõzõ értéke*/   
BYTE				    byPrDP[MAX_DP_NUM];           /* DP-k elõzõ értéke*/ 

IEC_M_DP_TB_1		strDPEventWT[MAX_DP_EVNUM];  
              } strKliensType;
/* ######################################################################################################################################################################### */
/* ------------------------------------------------------------- ----------------------------------------------------------------*/
/* ######################################################################################################################################################################### */
							
							
							
							
							
							

/*BYTE			sBuff[1000];
BYTE			sBuffTx[1000];*/

int       lTick_Rx; /* TESTFR ACT küldéshez */

/* IEC -----------------------------------------------------------------------------------------------------------------------*/

strSP_TABLE		sSPT[30];
strNM_TABLE		sNMT[10];
strDP_TABLE		sDPT[10];


/*--------------------------------------------------*/
/* GLOBAL VARIABLE 								    */
/*--------------------------------------------------*/
/* MOSCAD tabla leirok */
CB_TABLE_INFO   table_parBool;
CB_TABLE_INFO   table_parInt;
CB_TABLE_INFO   table_RxMon;
CB_TABLE_INFO   table_Stat;
CB_TABLE_INFO   table_SP;
CB_TABLE_INFO   table_SP2;
CB_TABLE_INFO   table_SP3;
CB_TABLE_INFO   table_SP4;
CB_TABLE_INFO   table_SP5;
CB_TABLE_INFO   table_SP6;
CB_TABLE_INFO   table_SP7;
CB_TABLE_INFO   table_SP8;
CB_TABLE_INFO   table_DP;
CB_TABLE_INFO   table_DP2;
CB_TABLE_INFO   table_DP3;
CB_TABLE_INFO   table_NM;
CB_TABLE_INFO   table_NM2;
CB_TABLE_INFO   table_NM3;
CB_TABLE_INFO   table_DC;
CB_TABLE_INFO   table_DC2;
CB_TABLE_INFO   table_DC3;
CB_TABLE_INFO   table_DC4;
CB_TABLE_INFO   table_DC5;

CB_TABLE_INFO   table_EVT;
CB_TABLE_INFO   table_SC;
CB_TABLE_INFO   table_SC2;
CB_TABLE_INFO   table_SC3;
CB_TABLE_INFO   table_SC4;
CB_TABLE_INFO   table_SC5;


CB_TABLE_INFO   table_DC_Event;


IEC_CTRL_FLD	cfRec;
IEC_CTRL_FLD	cfTransmit;
/*IEC_DUI_104		duiRec;*/
/*IEC_DUI_104		duiTransmit;*/
IEC_CP56Time	cp56Time;
IEC_C_DC_NA_1	ioDC;
IEC_C_DC_NA_1	ioDCSelection;
IEC_C_SC_NA_1	ioSC;



unsigned short	nTableNum1 = 1;
unsigned short	nTableNum2 = 2;

unsigned short	nTableNum;    
/*unsigned short	nTableNum;*/


/* MOSCAD tablak oszlopai */
short          *p_col_parBool;	
short          *p_col_parInt;
short          *p_col_RxMon;
short          *p_col_TxMon;
short          *p_col_Stat;


short          *p_col_DC_Index;
short          *p_col_DC_Value;
short          *p_col_DC_Year;
short          *p_col_DC_Month;
short          *p_col_DC_Day;
short          *p_col_DC_Hour;
short          *p_col_DC_Min;
short          *p_col_DC_Sec;


strSP_TABLE		sSPT[30];
strNM_TABLE		sNMT[10];
strDP_TABLE		sDPT[10];



short          *p_col_EVT;
short          *p_col_EVT2;

short          *p_col_DC;
short          *p_col_DC2;
short          *p_col_DC3;
short          *p_col_DC4;
short          *p_col_DC5;


short          *p_col_SC;
short          *p_col_SC2;
short          *p_col_SC3;
short          *p_col_SC4;
short          *p_col_SC5;


BYTE			byRowNumMon;

BYTE		byRxBuf[MAX_RX_BUFF];
BYTE		byTxBuf[MAX_TX_BUFF];
/*BYTE		byRecBuf[MAX_RX_BUFF]; */
BYTE		byTxBuf2[MAX_TX_BUFF];

int					nRxIndx;
int					nPrRxIndx;
int					nRecEnd;
int					nTxNum;
int					nTxNum2;

/* parameter tablabol kiolvasott ertekek */
int					nRxMon;
int					nDir;

int					nRxMonTblIndx;
int					nUPort;
int					nStatTblIndx;
unsigned int		nLinkTimeOut;
int					nMaxRptNum;
int					nLenCAOA;
int					nLenIOA;
int					nCAOA;
int					nSPTblIndx;
int					nSPTblIndx2;
int					nSPTblIndx3;
int					nSPTblIndx4;
int					nSPTblIndx5;
int					nSPTblIndx6;
int					nSPTblIndx7;
int					nSPTblIndx8;
int					nSPNum;
unsigned long		lSPStart;				
unsigned long		lSP2Start;				
unsigned long		lSP3Start;	
unsigned long		lSP4Start;	
unsigned long		lSP5Start;	
			
int					nDPTblIndx;
int					nDPTblIndx2;
int					nDPTblIndx3;
int					nDPNum;
unsigned long		lDPStart;	
int					nNMTblIndx;
int					nNMTblIndx2;
int					nNMTblIndx3;
int					nNMNum;
unsigned long		lNMStart;	
int					nDCTblIndx;
int					nDCTblIndx2;
int					nDCTblIndx3;
int					nDCTblIndx4;
int					nDCTblIndx5;


int					nDCNum;
unsigned long		lDCStart;	
int					nSCTblIndx;
int					nSCTblIndx2;
int					nSCTblIndx3;
int					nSCTblIndx4;
int					nSCTblIndx5;


int					nSCNum;
unsigned long		lSCStart;	
int					nLinkTestCycle;
int					nDelta;
unsigned int		nDCTimeOut;


BYTE		byCAOA_LO;
BYTE		byCAOA_HI;

/* Belso allapot valtozok */
int					nLinkInit1;
int					nLinkInit2;
int					nStart;
int					nLastTx2;
int					nActRptNum;
int					nTxFCB;
int					nRxFCB;
int					nActTransaction;
int					nPrTransaction;
int					nInterrogationStep[MAX_CONN];
int					nDCStep[MAX_CONN];
int					nSCStep[MAX_CONN];
int					nClockSyncStep[MAX_CONN];
unsigned int					nLinkInitStep;
int					nTxNum;
BOOL				bDPSentAll;
BOOL				bSPSentAll;
BOOL				bNMSentAll;
BOOL				bLostACK;
BOOL				bTimeOutActive;
int					nActTrCtr;
unsigned long		lTick;
unsigned long		lTickDC;
BOOL				bEnableCount;
BOOL				bEnableCountDC;
MOSCAD_DATE_TM      tm;
unsigned int		nRepeat;
int					nTickRec;
unsigned long		lIOASelection;
int					nSendSP[MAX_CONN]; 		/* Elkuldott SP-k szama altalanos lekerdezeskor */
int					nSendNM[MAX_CONN];		/* Elkuldott NM-k szama altalanos lekerdezeskor */
int					nSendDP[MAX_CONN];		/* Elkuldott DP-k szama altalanos lekerdezeskor */
unsigned long		lSPAct;
unsigned long		lTickEv;
/* Allapot es esemeny tombok */
BYTE				bySP[MAX_SP_NUM];
BYTE				byPrSP[MAX_CONN][MAX_SP_NUM];
IEC_M_SP_TB_1		strSPEventWT[MAX_CONN][MAX_SP_EVNUM];
int					nSPWrPtr[MAX_CONN];
int					nSPReadPtr[MAX_CONN];                                                              

BYTE				byDP[MAX_DP_NUM];
BYTE				byPrDP[MAX_CONN][MAX_DP_NUM];
IEC_M_DP_TB_1		strDPEventWT[MAX_CONN][MAX_DP_EVNUM];                             
int					nDPWrPtr[MAX_CONN];
int					nDPReadPtr[MAX_CONN];

unsigned int		nNM[MAX_NM_NUM];
unsigned int		nPrNM[MAX_CONN][MAX_NM_NUM];
unsigned int		nLiveZero[MAX_NM_NUM];
unsigned int		nStatus[MAX_NM_NUM];
unsigned int		nPrStatus[MAX_NM_NUM];

IEC_M_ME_TD_1		strNMEvent104[MAX_CONN][MAX_NM_EVNUM];
IEC_M_ME_NA_1		strNMEvent[MAX_CONN][MAX_NM_EVNUM];
int					nNMWrPtr[MAX_CONN];
int					nNMReadPtr[MAX_CONN];

int					nDCEventPtr;
unsigned int		nLinkTest;
unsigned int		nIECErr;

int					nMessReceived;
int					n20Msec;
int					nOldSec;


strTotalData		*TotalData;
int					nDelayedStart;
unsigned long		lLengthTotalData;

/* 2012.04.26 */
strASDUType   strASDU[MAX_CONN][MAX_ASDU];

unsigned int  nASDUWrPtr[MAX_CONN];
unsigned int  nASDUSendPtr[MAX_CONN];
/*int           bMessReceived = 0;*/
int           nInterrogated[MAX_CONN];  
int					nDisableWrite;

/* 2015.04.16 */
int             nDataSave = 0;
int             nFirstSave = 0;
/****************************************************************************/
/* Main program																*/
/****************************************************************************/
void fnServer(unsigned short TableNumber1,unsigned short TableNumber2)
{
TableNumber1 = 1;
TableNumber2 = 2;

	
 MOSCAD_message("Server fõprogram");	
	




}


/*--------------------------------------------------------------------------*/
/* The list of the functions included in this block                         */
/*--------------------------------------------------------------------------*/
CB_JUMPTBL user_jumptable[]=
{
	{"server", fnServer },
  	{"\0"     , 0  }
};

/*--------------------------------------------------------------------------*/
/* 'C' Block Initialization and Completion                                  */
/*--------------------------------------------------------------------------*/
void user_control_function(int control)
{




   switch(control)
   {
      /*---------------------------------------------------*/
      /* A new 'C' data file (without reset) is downloaded */
      /*---------------------------------------------------*/

      case CB_DATA :
         MOSCAD_message("\n A new C data file is downloaded");
         break;

         /*---------------------------------------------------------------------------*/
         /* A new system file (site config., network, phonebook, ...) is downloaded   */
         /*---------------------------------------------------------------------------*/

      case CB_FILE_DOWNLOAD :
         MOSCAD_message("\n A new system file is downloaded");
         break;

         /*---------------------------------------------*/
         /* PERFORM INITIALIZATIONS-WE START THE BLOCK. */
         /*---------------------------------------------*/
      case CB_INIT :
      	
         /* 2015.04.15 */
        /* MOSCAD_wait(3500); */

   			nTableNum1=1;
            nTableNum2=2;	
		    fnReadPar();


   			fnIEC_Init();
   			 


         break;

         
         
         break;

         /*----------------------------------------------*/
         /* FREE ALL; A NEW C BLOCK IS TO BE DOWNLOADED. */
         /*----------------------------------------------*/
      case CB_EXIT :
         break;

         /*----------------------------------------------*/
         /* Typically, there is nothing to be done,      */
         /* unless the application asked to be notified  */
         /*----------------------------------------------*/
      case CB_OTHER :
         break;

   }
}



/*--------------------------------------------------------------------------*/
/* fnTaskCreateSocket						                                       */
/* 	Server socket létrehozása												*/
/*	Kliens kérelem fogadása													*/
/*--------------------------------------------------------------------------*/
void fnTaskCreateSocket(void)
{


int				nNoRestarted;
int				nSaveTables;
long   lSRAMLength;
   

	MOSCAD_SOCKET_ADDR 	sockaddr;
	char 				msg[512];
	int					error;
	int					retval;
/*	int 				nonblock = 1;*/



	MOSCAD_SOCKET_ADDR 	peername;
	int 				peernamelen;
	char				sIPAddr[99];
	int         nI;




	MOSCAD_sprintf(msg, "TaskA created succesfully");
	MOSCAD_message(msg);

  MOSCAD_wait(100);

      nSI = 0; 
    
    for (nI=0;nI<MAX_CONN;nI++)
    {
         newsocket[nI] = -1;
    }

  MOSCAD_wait(5900);
  
  		nTableNum1=1;
      nTableNum2=2;	
  		fnReadPar();
  
	

  Cim1:

/* Socket létrehozása --------------------------------------------------------------------*/
sock = MOSCAD_socket(MOSCAD_SOCKET_AF_INET,MOSCAD_SOCKET_STREAM,MOSCAD_SOCKET_IPPROTO_IP);
if (sock == MOSCAD_SOCKET_INVALID_SOCKET)
{
	error = MOSCAD_socket_errno();
	MOSCAD_sprintf(msg, "Failed socket because error %d", error);
	MOSCAD_error(msg);
  MOSCAD_wait(5500);	
  goto Cim1;
	
}
else
{
/* Socket has been opened and ready to be used for TCP */
	MOSCAD_sprintf(msg, "Socket has been opened (blocking mode) and ready to be used for TCP, socket: %d", sock);
	MOSCAD_message(msg);
	/*MOSCAD_led_set(CB_ACE_LED_ID_USR1, 1000000, 0, 100);*/

}



/* Assume sock has been opened previously */
/*if ( (retval = MOSCAD_socket_ioctl(sock, MOSCAD_SOCKET_FIONBIO, &nonblock)) == 0)
{
	MOSCAD_sprintf(msg, "Nonblocking mode succesfully set");
	MOSCAD_message(msg);
	
}
else
{
	error = MOSCAD_socket_errno();
	MOSCAD_sprintf(msg, "Failed to set nonblocking modet because error %d", error);
	MOSCAD_error(msg);
	
}*/

/* Assume sock has been opened using MOSCAD_socket() */
/* Prepare a socket name with no physical IP address and port 2404 */

memset((void *)&sockaddr, 0, sizeof(sockaddr));
sockaddr.sin_family = MOSCAD_SOCKET_AF_INET;
sockaddr.sin_addr.s_addr = MOSCAD_htonl(0); /*INADDR_ANY: Mindegyik helyi ethernet socketre figyel*/
sockaddr.sin_port = MOSCAD_htons(2404); /*Logikai port szám*/

/* Bind socket ------------------------------------------------------------------*/
 MOSCAD_wait(900);
retval = MOSCAD_socket_bind(sock, &sockaddr, sizeof(sockaddr));
if (retval == ERR_MOSCAD_SOCKET_ERROR)
{
	error = MOSCAD_socket_errno();
	MOSCAD_sprintf(msg, "Failed socket bind because error %d", error);
	MOSCAD_error(msg);

	/* Close socket */
	MOSCAD_socket_close(sock);
	MOSCAD_wait(1500);	
	goto Cim1;
	  
	
}
else
{
	/*MOSCAD_led_set(CB_ACE_LED_ID_USR2, 1000000, 0, 100);*/
	
	
	/* Socket has been bound and is ready for receive on TCP port 2404 */
	MOSCAD_sprintf(msg, "Socket has been bound and is ready for receive on TCP port 2404");
	MOSCAD_message(msg);
	 MOSCAD_wait(1900);
	if (MOSCAD_socket_listen(sock, 4) == ERR_MOSCAD_SOCKET_ERROR)
	{
		error = MOSCAD_socket_errno();
		MOSCAD_sprintf(msg, "Failed socket listen error %d", error);
		MOSCAD_error(msg);
		MOSCAD_wait(1500);	
	  goto Cim1;


	}
	else
	{
		MOSCAD_led_set(CB_ACE_LED_ID_USR1, 400000000000, 0, 100);
	
	
		/* Socket go to listen mode  and is ready for receive on TCP port 2404 */
		MOSCAD_sprintf(msg, "Socket go to listen mode and is ready for receive on TCP port 2404");
		MOSCAD_message(msg);		
	}
}
   

    

   for(;;)
   {
      /*-----------------------------------------------------------------*/
      /* Task is create succesfully                             */
      /*-----------------------------------------------------------------*/
      /*MOSCAD_led_set(CB_ACE_LED_ID_USR1, 10000, 0, 100);*/
	
     
      
    MOSCAD_wait(1500);
      
		MOSCAD_sprintf(msg, " MOSCAD_socket_accept is called, newsocket[0]=%d,newsocket[1]=%d, sock=%d",newsocket[0],newsocket[1],sock);
		MOSCAD_message(msg);
      
 
      
		if (newsocket[0]<0)
		{
			nSI=0;
		/* 	MOSCAD_socket_close(newsocket[0]);   */
			
		}
		
		/* Egyelõre nem engedek új socket-t nyitni !!!!!!!!!!!!!!!!!!!!!!!!!!!!!! GZS 2009.06.25. */
		else if (newsocket[0]>=0  && newsocket[1]<0)
		{
			nSI=1;
		}
		
		
    	/* Várakozik egy kliens bejelentkezésére ----------------------------------------------------------------------------*/
		peernamelen = sizeof(peername);
    MOSCAD_wait(1500);
		newsocket[nSI] = MOSCAD_socket_accept(sock, &peername, &peernamelen);
		
				MOSCAD_sprintf(msg, " New socket is created by accept. New socket desc: %d, socket index: %d", newsocket[nSI],nSI);
				MOSCAD_message(msg);
		
    
        /* MOSCAD_wait(1500); */
				MOSCAD_socket_inet_socknametoa(&peername,sIPAddr);
				MOSCAD_sprintf(msg, " Kliens IP address: %s", sIPAddr);
				MOSCAD_message(msg);
		
		MOSCAD_led_set(CB_ACE_LED_ID_USR2, 400000000000, 0, 100);
		
    p_col_Stat[23] = p_col_Stat[23] + 1;  /* 2014.05.12 */
    
		/* Konnektáláskor nullázni kell a számlálókat*/
			nSendSeqNum[nSI] 		= 0;
			nRecSeqNum[nSI]  		= 0;
			nASDUSendPtr[nSI] = 0;
			nASDUWrPtr[nSI] = 0;
     	nStarted[nSI] = 0;
     	
     	
      nSendNM[nSI] = 0;
      nSendDP[nSI] = 0;
      nSendSP[nSI] = 0;
     	
      nInterrogationStep[nSI] = 0;
      nInterrogated[nSI] = 0;
      nClockSyncStep[nSI] = 0;       
      nSPReadPtr[nSI] = 0;
      nSPWrPtr[nSI] = 0;
      nDPReadPtr[nSI] = 0;                                                                                       
      nDPWrPtr[nSI] = 0;
      nNMReadPtr[nSI] = 0;
      nNMWrPtr[nSI] = 0;
      nASDUWrPtr[nSI] = 0;
      nASDUSendPtr[nSI] = 0;
     	
     	
     	
				MOSCAD_sprintf(msg, "Reset counters: nSendSeqNum[nSI]: %d, nRecSeqNum[nSI] : %d , nSI: %d   " ,  nSendSeqNum[nSI],nRecSeqNum[nSI],nSI );
				MOSCAD_message(msg);
		  
		
		
		if (newsocket[nSI] == MOSCAD_SOCKET_INVALID_SOCKET)
		{
			error = MOSCAD_socket_errno();
			if (error != ERR_MOSCAD_SOCKET_EWOULDBLOCK)
			{
				error = MOSCAD_socket_errno();
				MOSCAD_sprintf(msg, "Failed socket listen error %d", error);
				MOSCAD_error(msg);

				/* a problem exist in listensock – close it. */
				MOSCAD_socket_close(sock);
				nStarted[nSI] = 0;
				MOSCAD_wait(500);
		
			/* listensock need to be initialized again. */
			}
		} /*end if */  
         
         
         
         
         
         
         
   } /* end for(;;) */
         
         
         
         
}/*end  fnTaskInit(void)*/
/*-------------------------------------------------------------------------------------------------------------------------------------------------*/
         
         
         
         
         
         
         
/*--------------------------------------------------------------------------*/
/* Server0 task                                                                */
/*--------------------------------------------------------------------------*/
void fnTaskServer0(void)
{        
	
    /* MOSCAD_wait(4900); */
	 fnKliensSocket(0);   
 

} /*end fnTaskServer0*/



/*--------------------------------------------------------------------------*/
/* Server1 task                                                                */
/*--------------------------------------------------------------------------*/
void fnTaskServer1(void)
{        

   MOSCAD_wait(100);  
	fnKliensSocket(1);   

} /*end fnTaskServer1*/



/****************************************************************************/
/* Kliens socket kezelése  													*/
/* 	Bemenõ paraméter: a kilens socket-hez tartozó tömb index				*/
/****************************************************************************/
int fnKliensSocket(const int INDX)
{

	
BYTE				sBuff[1000]; 
BYTE				sBuffTx[1000]; 

	int					retval2;
	/*int					nNum;*/
	
	int					nI;
	char 				msg[1512];
	
	int					retval;
	int         nMessLength;
  unsigned int         nIndex;
  int         nCikl;
  unsigned long		lIOA;         
int					  nOffset;         
int           nRetVal;
BYTE          nSendSeqNumBIT_0;	
BYTE          nSendSeqNumBIT_1;	


unsigned int	nSendSeqNumClient;
unsigned int	nRecSeqNumClient;
unsigned int	nTypeID;
unsigned int	nNumOfObj;
unsigned int	nCauseOfTx;
IEC_DUI_104		duiRec;



	
/* fnEvents() */
char			message[1500];
/*int				nI;  */
MOSCAD_DATE_TM  mdt;
unsigned int	nMsec;
float			flDelta;
BOOL			bDevTime;
int				bTTime;
int				nMS1,nMS2,nMin,nXOR;

int				nInvalid;
int 			nSPTempPtr;
int				nDPTempPtr;
int				nNMTempPtr;
int				nNoRestarted;
int				nSaveTables;
long   lSRAMLength;
int       nNum;
int       byNum;
BYTE				byIOA[10];
BYTE      byData[100];
/*int       retval;*/
int					nDataNum;
IEC_DUI_104		duiTransmit;
long         lTimeRTU;
long         lTimeFIU;



/* Inicializálás */


  MOSCAD_wait(900);
	
 		MOSCAD_sprintf(msg, "_______STARTED: newsocket[INDX]=%d., INDX=%d,sock=%d",newsocket[INDX],INDX,sock);
		MOSCAD_message(msg);     
	
	
	
	
	

		nTableNum1=1;
    nTableNum2=2;	
		fnReadPar();

  
  
/*  		nTableNum1=1;
      nTableNum2=2;	
  		fnReadPar(); */
  
	
	
/* Végtelen ciklus----------------------------------------------------------------------------------------------------*/         
for (;;)
{         
      MOSCAD_wait(50);





      
/*    		MOSCAD_sprintf(msg, "_______newsocket[INDX]=%d., INDX=%d,sock=%d",newsocket[INDX],INDX,sock);
		MOSCAD_message(msg);     
   
      
 if (newsocket[INDX]<=0)
  {
		MOSCAD_sprintf(msg, "newsocket[INDX]=%d., INDX=%d,sock=%d",newsocket[INDX],INDX,sock);
		MOSCAD_message(msg);      
	     
  }          */
  
  
      
/*Ha konnektált és jó a dinamikus site tábla --------------------------------------------------------------------------------------------------*/	         
if (  (newsocket[INDX]>0)  && (p_col_Stat[16] == 0) )
{	

MOSCAD_wait(100);


		nTableNum1=1;
    nTableNum2=2;	
		fnReadPar(); 





	
fnSendTESTFR_ACT(INDX);		


/*Ha konnektált, akkor jöhet a táviratok adás-vétele -----------------------------------------------------*/
/*do
{            */

/*		MOSCAD_sprintf(msg, " %d. socket, Wait for kliens message... , nStarted[INDX]: %d, nDisableWrite: %d",INDX, nStarted[INDX], nDisableWrite);
		MOSCAD_message(msg); */
   
    /*  MOSCAD_wait(500);         */
/*Receive messages --------------------------------------------------------------------*/   
memset((void *)sBuff, 0, sizeof(sBuff));   
retval = 0;   
retval = MOSCAD_socket_recv(newsocket[INDX], sBuff, sizeof(sBuff), MOSCAD_SOCKET_MSG_DONTWAIT);
if (retval ==ERR_MOSCAD_SOCKET_ERROR)
{


  retval = MOSCAD_socket_errno();
  if (retval == ERR_MOSCAD_SOCKET_EWOULDBLOCK)
  {
  	/* MOSCAD_sprintf(msg, "1. Connection has been broken error newsocket[0]:%d, newsocket[1]: %d", newsocket[0],newsocket[1]);
  	MOSCAD_error(msg); */

    MOSCAD_wait(100);
    continue;
  }

	/* Socket connection has been broken. */
	MOSCAD_sprintf(msg, "Connection has been broken error %d, INDX: %d", MOSCAD_socket_errno(), INDX);
	MOSCAD_error(msg);
	
	MOSCAD_socket_close(newsocket[INDX]);
	newsocket[INDX]=-1;
	nStarted[INDX] = 0;

	MOSCAD_sprintf(msg, "Connection has been broken error newsocket[0]:%d, newsocket[1]: %d", newsocket[0],newsocket[1]);
	MOSCAD_error(msg);

	
}
else
{

	
/*		MOSCAD_sprintf(msg, " Kliens message length(retval): %d", retval);
		MOSCAD_message(msg); */
	
	/* Received chr */
 	if (retval == 0)
	{
  retval = MOSCAD_socket_errno();
  if (retval == ERR_MOSCAD_SOCKET_EWOULDBLOCK)
  {
    /* MOSCAD_sprintf(msg, "2. Connection has been broken error newsocket[0]:%d, newsocket[1]: %d", newsocket[0],newsocket[1]);
  	MOSCAD_error(msg); */

  
    MOSCAD_wait(100);
    continue;
  }

		/* Peer disconnected. */
		MOSCAD_sprintf(msg, "Peer disconnected error %d, INDX: %d", MOSCAD_socket_errno(), INDX);
		MOSCAD_error(msg);

		  MOSCAD_socket_close(newsocket[INDX]);
			newsocket[INDX]=-1;
	   nStarted[INDX] = 0;

	}
	else         /* Egy vagy több távirat érkezett */
	{
			MOSCAD_led_set(CB_ACE_LED_ID_USR3, 10, 1, 20);
		
		/* Received character */
		/*MOSCAD_sprintf(msg, " \n  %d. kliens message: %x  %x  %x  %x  %x  %x  %x  %x  %x  %x  %x  %x %x  %x  %x  %x  %x  %x  %x  %x  %x  %x  %x  %x", INDX,sBuff[0], sBuff[1],sBuff[2],sBuff[3],sBuff[4],sBuff[5],sBuff[6],sBuff[7],sBuff[8],sBuff[9],sBuff[10],sBuff[11],sBuff[12], sBuff[13],sBuff[14],sBuff[15],sBuff[16],sBuff[17],sBuff[18],sBuff[19],sBuff[20],sBuff[21],sBuff[22],sBuff[23]);
		MOSCAD_message(msg);*/
		  /*    MOSCAD_wait(100);    */
		
/* Elkezdi egy do-while ciklusban feldolgozni a beérkezett táviratokat */
nCikl=0;	
nMessLength = 6;
/*bMessReceived = 1;*/

do
{		
    nCikl++;
    nMessLength = 6;
/*		MOSCAD_sprintf(msg, " Do cikluson belul: kliens message length: %d, nCikl: %d", retval, nCikl);
		MOSCAD_message(msg); */

    /*MOSCAD_sprintf(msg, " \n Socket: %d, retval:  %d. sBuff[0] a cikluson belül: %x  %x  %x  %x  %x  %x  %x  %x  %x  %x  %x  %x %x  %x  %x  %x  %x  %x  %x  %x  %x  %x  %x  %x",INDX,  retval,sBuff[0], sBuff[1],sBuff[2],sBuff[3],sBuff[4],sBuff[5],sBuff[6],sBuff[7],sBuff[8],sBuff[9],sBuff[10],sBuff[11],sBuff[12], sBuff[13],sBuff[14],sBuff[15],sBuff[16],sBuff[17],sBuff[18],sBuff[19],sBuff[20],sBuff[21],sBuff[22],sBuff[23]);
		MOSCAD_message(msg); */ 


		/*Moscad send answer-----------------------------------------------------------------------------------------------------*/
		/*Ha a kezdõ karakter 0x68*/
		if (sBuff[0] == START_CHAR)
		{
			
			nSendSeqNumBIT_0 = sBuff[2] & 0x01;
			nSendSeqNumBIT_1 = (sBuff[2] & 0x02) >> 1;
			
		        /*MOSCAD_sprintf(message,"Socket: %d:  nSendSeqNumBIT_0: %d,   nSendSeqNumBIT_1: %d", INDX, nSendSeqNumBIT_0, nSendSeqNumBIT_1);			
    				MOSCAD_message(message );*/ 
			
							

			/* U format - Unnumbered control function -----------------------------------------------------------------------------------*/
			if (nSendSeqNumBIT_0 == 1 && nSendSeqNumBIT_1 == 1)
			{
			memcpy(sBuffTx,sBuff,6) ;   /* Bemásolja az adó pufferbe */
			
			
				/*Ha a funkció STARTDT act */
				if (sBuff[2] == 0x07)
				{
				
        lSRAMLength=MOSCAD_bspSRamLength(); 
				 MOSCAD_sprintf(message,"STARTDT ACT érkezett, lSRAMLength: %ld",lSRAMLength);			
				MOSCAD_message(message );

				
					sBuffTx[2] = 0x0B;
					nStarted[INDX] = 1;
					
				/* Elküldi a válasz táviratot */
				retval2 = MOSCAD_socket_send(newsocket[INDX], sBuffTx, 6,0);		
				MOSCAD_wait(100);	
				}
				
				/*Ha a funkció TESTFR con */
				if (sBuff[2] == 0x83)
				{					
				/*MOSCAD_sprintf(message,"Socket: %d: TESTFR CON érkezett",INDX);			
				MOSCAD_message(message );*/ 	
				}
				
				
				
				/*Ha a funkció TESTFR act */
				if (sBuff[2] == 0x43)
				{
				sBuffTx[2] = 0x83;
					
				/* MOSCAD_sprintf(message,"Socket: %d: TESTFR ACT érkezett",INDX);			
				MOSCAD_message(message ); */ 
					
				/* Elküldi a válasz táviratot */
				retval2 = MOSCAD_socket_send(newsocket[INDX], sBuffTx, 6,0);		
				}
				
				MOSCAD_wait(100);  
				
			
				/*Ha a funkció STIOPDT act */
				if (sBuff[2] == 0x13)
				{
					sBuffTx[2] = 0x23;
					nStarted[INDX] = 0;

				MOSCAD_sprintf(message,"Socket: %d:  STOPDT ACT érkezett", INDX);			
				MOSCAD_message(message ); 


				/* Elküldi a válasz táviratot */
				retval2 = MOSCAD_socket_send(newsocket[INDX], sBuffTx, 6,0);		
				}
				MOSCAD_wait(100);
        nMessLength = 6;


			   if (nStarted[INDX] == 1)
   				{
					MOSCAD_led_set(CB_ACE_LED_ID_USR4, 400000000000, 0, 100);  
   				}
   				else
   				{
				  MOSCAD_led_reset(CB_ACE_LED_ID_USR4);  
   				}
				
				
			} /* end U format*/
			
			/* S format - Numbered suoervisory function, NYUGTA  -----------------------------------------------------------------------------------*/
			if (nSendSeqNumBIT_0 == 1 && nSendSeqNumBIT_1 == 0)
			{
				nRecSeqNumClient  		= (sBuff[4] >> 1) + 128 * sBuff[5];		      /* lehet, hogy 128 ???? */
        
        /* Receive sequence number lekezelése, táviratok törlése a pufferbõl */
        fnRecSeqNum(INDX, nRecSeqNumClient);

        
        
			 
			} /* end S format*/
			
			
			/* I format - Information transfer -----------------------------------------------------------------------------------*/
			else if (nSendSeqNumBIT_0 == 0 )
			{
				/*Növelni kell a REC számlálót*/
        	/* 2014.12.11. */
 					if (nRecSeqNum[INDX]< 32767)
  				{
	          nRecSeqNum[INDX]++;
          }	
          else
          {
            nRecSeqNum[INDX]=0;
          }

          

				
				nTypeID = sBuff[6];
				nNumOfObj = sBuff[7];
				nCauseOfTx = sBuff[8];
				
				duiRec.byTI    = sBuff[6] ;	
				duiRec.byDataNum  = sBuff[7] & 0x7F ;	
				duiRec.bySequence = sBuff[7] & 0x80 ;	
				duiRec.byCOT      = sBuff[8] & 0x3F ;
				duiRec.byOrigAddr = sBuff[9];
				
				
								
				/* A kliens táviratában küldött számlálók*/
        

        
        
				nSendSeqNumClient 		= (sBuff[2] >> 1) + 128 * sBuff[3];
				nRecSeqNumClient  		= (sBuff[4] >> 1) + 128 * sBuff[5];

        /* Receive sequence number lekezelése, táviratok törlése a pufferbõl */
        fnRecSeqNum(INDX, nRecSeqNumClient);
				 
				/*MOSCAD_sprintf(msg, " \n    0. bit: %x,  1. bit  %x  ",nSendSeqNumBIT_0[INDX],nSendSeqNumBIT_1[INDX]);
				MOSCAD_message(msg);*/
				MOSCAD_sprintf(msg, " \nSocket: %d:   Information transfer: nSendSeqNumClient: %x,  nRecSeqNumClient  %x  , nRecSeqNum[INDX]: %x,nSendSeqNum[INDX]: %x,  Orig. addr: %d",INDX, nSendSeqNumClient,nRecSeqNumClient, nRecSeqNum[INDX],nSendSeqNum[INDX], duiRec.byOrigAddr);
				MOSCAD_message(msg);
				
				/* Ha a várt sorszámú távirat érkezett*/
				if ((nRecSeqNum[INDX] == nSendSeqNumClient+1)  /* && (nLinkTestCycle==190) */  )
				{
					/* Single command --------------------------------------------------------------------*/
					if ( nTypeID ==TI_C_SC_NA_1 && nCauseOfTx== COT_ACT)
						{
							 /*	nCommand_SE[INDX] = sBuff[1 + 1 + 4 + 4 + 2 + 3 ] & 0x80;;
							
						
							nNum = fnBuildStartChar(sBuffTx, START_CHAR,0);
							nNum = fnAPCISeqNums(sBuffTx, nSendSeqNum[INDX],nRecSeqNum[INDX],S_FORMAT, nNum+1);							
							fnBuildMessLength(sBuffTx, nNum-2,1);
														
							
							retval2 = MOSCAD_socket_send(newsocket[INDX], sBuffTx, nNum,0);
               MOSCAD_wait(100);		
                 */
			       	
							nMessLength = 16;
							
              /* Parancs kiírása a PLC táblájába */					    
							memcpy(&ioSC.byIOA[0],&sBuff[10 + nLenCAOA],nLenIOA);	
							ioSC.bySC = sBuff[10 + nLenCAOA + nLenIOA] & 0x01;
							ioSC.bySE = sBuff[10 + nLenCAOA + nLenIOA] & 0x80;	

							lIOA = ioSC.byIOA[0] + 	ioSC.byIOA[1]*256 + ioSC.byIOA[2]*256*256;	

							MOSCAD_sprintf(message,"Socket: %d: Single command received - Inf. Obj. Addr: %ld, végrehajtás: %d, nLenCAOA: %d, nLenIOA: %d",INDX, lIOA, ioSC.bySE, nLenCAOA, nLenIOA);
        			MOSCAD_message(message );
        									
									/* elõkészítés */							
									if (ioSC.bySE > 0)																
									{

										lIOA = ioSC.byIOA[0] + 	ioSC.byIOA[1]*256 + ioSC.byIOA[2]*256*256;	

										/* ervenyes a cim, parancs kiiras a tablaba*/
										if (lIOA >= lSCStart && lIOA < lSCStart + nSCNum && duiRec.byCOT == COT_ACT)
										{
											nOffset = lIOA - lSCStart;
											
											/*MOSCAD_sprintf(message,"Socket: %d:  Single command received, elõkészítés - SC nOffset: %d,ioSC.bySC: %d",INDX,nOffset,ioSC.bySC);
        							MOSCAD_message(message ); */
        									
											MOSCAD_sprintf(message,"Socket: %d:  Single command received, elõkészítés - SC nOffset: %d,ioSC.bySC: %d, p_col_SC: %p",INDX,nOffset,ioSC.bySC,p_col_SC);
        							MOSCAD_message(message );
																			


                      nSCStep[INDX] = 1;  /* Ideiglenes */
											/*(message,"lIOA %d",lIOA);
        									MOSCAD_error(message );*/
											

										}
											
									} /* end elõkészítés */																
							
							
									/* vegrehajtas */							
									if (ioSC.bySE == 0)																
									{

										lIOA = ioSC.byIOA[0] + 	ioSC.byIOA[1]*256 + ioSC.byIOA[2]*256*256;	

										/* ervenyes a cim, parancs kiiras a tablaba*/
										if (lIOA >= lSCStart && lIOA < lSCStart + nSCNum && duiRec.byCOT == COT_ACT)
										{
											nOffset = lIOA - lSCStart;
											
											/*MOSCAD_sprintf(message,"Socket: %d:  Single command received - SC nOffset: %d,ioSC.bySC: %d",INDX,nOffset,ioSC.bySC);
        							MOSCAD_message(message );*/
        									
											MOSCAD_sprintf(message,"Socket: %d:  Single command received - SC nOffset: %d,ioSC.bySC: %d, p_col_SC: %p",INDX,nOffset,ioSC.bySC,p_col_SC);
        							MOSCAD_message(message );
									
											
											if (nOffset<250)
											{
												p_col_SC[lIOA - lSCStart] = ioSC.bySC ;	
												nSCStep[INDX] = 1;

											}
											else if (nOffset>=250 && nOffset<500)
											{
												p_col_SC2[nOffset-250] = ioSC.bySC ;	
												nSCStep[INDX] = 1;
											}
											else if (nOffset>=500 && nOffset<750)
											{
												p_col_SC3[nOffset-500] = ioSC.bySC ;	
												nSCStep[INDX] = 1;
											}
											
											else if (nOffset>=750 && nOffset<1000)
											{
												p_col_SC4[nOffset-750] = ioSC.bySC ;	
												nSCStep[INDX] = 1;
											}
											else if (nOffset>=1000 && nOffset<1250)
											{
												p_col_SC5[nOffset-1000] = ioSC.bySC ;	
												nSCStep[INDX] = 1;
											}


                      nSCStep[INDX] = 1;  /* Ideiglenes */
											/*(message,"lIOA %d",lIOA);
        									MOSCAD_error(message );*/
											

										}
											
									} /* end vegrehajtas */																
							



					
						} /* nTypeID[INDX] ==TI_C_SC_NA_1 */
						
					/* Double command --------------------------------------------------------------------*/					
					if ( nTypeID ==TI_C_DC_NA_1 && nCauseOfTx== COT_ACT)
						{
									
									
									
    
									memcpy(&ioDC.byIOA[0],&sBuff[10 + nLenCAOA],nLenIOA);	
									ioDC.byDC = sBuff[10 + nLenCAOA + nLenIOA] & 0x03;
									ioDC.bySE = sBuff[10 + nLenCAOA + nLenIOA] & 0x80;	
    
    
    					
    					    nMessLength = 16;

													
									/*elokeszites */							
									if (ioDC.bySE > 0)			
									{	
										p_col_Stat[7] = 1;
										memcpy(&ioDCSelection.byIOA[0],&sBuff[10 + nLenCAOA],nLenIOA);	
										ioDCSelection.byDC = sBuff[10 + nLenCAOA + nLenIOA] & 0x03;
										ioDCSelection.bySE = sBuff[10 + nLenCAOA + nLenIOA] & 0x80;	
										
										lIOA = ioDCSelection.byIOA[0] + 	ioDCSelection.byIOA[1]*256 + ioDCSelection.byIOA[2]*256*256;
										lIOASelection = lIOA;	
										
										
							       MOSCAD_sprintf(message,"Socket: %d:  1. Double command received - Inf. Obj. Addr: %ld, elõkészítés: %d, nLenCAOA: %d, nLenIOA: %d",INDX,lIOA, ioDC.bySE, nLenCAOA, nLenIOA);
        			       MOSCAD_message(message );												
										
										
										/* ervenyes a cim */
										if ( lIOA >= lDCStart && lIOA < lDCStart + nDCNum )
										{
											lTickDC = 0;	
											nDCStep[INDX] = 1;
											bEnableCountDC = 1;	
											
							       MOSCAD_sprintf(message,"Socket: %d: Double command received - Inf. Obj. Addr: %ld, elõkészítés: %d, nLenCAOA: %d, nLenIOA: %d",INDX,lIOA, ioDC.bySE, nLenCAOA, nLenIOA);
        			       MOSCAD_message(message );												
										}																			
										 		
									} /* end elokeszites */
			
					    
					    
					    
					         /* Végrehajtás */
									 if (ioDC.bySE == 0)																
									{
											/*MOSCAD_sprintf(message,"lTickDC: %d",lTickDC);
        									MOSCAD_error(message );*/
										lIOA = ioDC.byIOA[0] + 	ioDC.byIOA[1]*256 + ioDC.byIOA[2]*256*256;	
										/* Idon belul erkezett */
										/*if ()
										{*/
																										
										p_col_Stat[6] = lTickDC  / 10;
										/* ervenyes a cim, parancs kiiras a tablaba*/
										if (lTickDC < nDCTimeOut && lIOA >= lDCStart && lIOA < lDCStart + nDCNum && lIOA == lIOASelection && ((ioDC.byDC ) == (ioDCSelection.byDC )))
										{

										
										
											lTickDC = 0;	
											nDCStep[INDX] = 1;
											bEnableCountDC = 0;
											
											nOffset = lIOA - lDCStart;
											
											
										 MOSCAD_sprintf(message,"Socket: %d: Double command received - Inf. Obj. Addr: %ld, végrehajtás: %d, nLenCAOA: %d, nLenIOA: %d, nOffset: %d, lTickDC: %ld",INDX,lIOA, ioDC.bySE, nLenCAOA, nLenIOA, nOffset, lTickDC);
        			       MOSCAD_message(message );
											
											

											fnSetDCLogTime(nOffset, ioDC.byDC);
											
											if (nOffset<250)
											{
												p_col_DC[nOffset] = ioDC.byDC ;	
											}
											else if (nOffset>=250 && nOffset<500)
											{
												p_col_DC2[nOffset-250] = ioDC.byDC ;	
											}
											else if (nOffset>=500 && nOffset<750)
											{
												p_col_DC3[nOffset-500] = ioDC.byDC ;	
											}
											else if (nOffset>=750 && nOffset<1000)
											{
												p_col_DC4[nOffset-750] = ioDC.byDC ;	
											}
											else if (nOffset>=1000 && nOffset<1250)
											{
												p_col_DC5[nOffset-1000] = ioDC.byDC ;	
											}
											
											
										}   /* if (lTickDC < nDCTimeOut && lIOA >= lDCStart && lIOA < lDCStart + nDCNum && lIOA == lIOASelection && ((ioDC.byDC ) == (ioDCSelection.byDC ))) */
					
					     }     /* 	 if (ioDC.bySE == 0) */
	
					    
					    
						} /* nTypeID[INDX] ==TI_C_SC_NA_1 */

					/* Interrogation command --------------------------------------------------------------------*/					
					if ( nTypeID ==TI_C_IC_NA_1 && nCauseOfTx== COT_ACT)
						{
							nCommand_SE[INDX] = sBuff[1 + 1 + 4 + 4+ 2 +3 ] & 0x80;;
							
					    nMessLength = 16;
					    
					    nInterrogationStep[INDX] = 1;
					    					    
               p_col_Stat[18] = p_col_Stat[18] + 1;  /* 2014.05.12 */             
                            
						} /* nTypeID[INDX] ==TI_C_SC_NA_1 */
					


          /* Clock syncron */
					if ( nTypeID ==TI_C_CS_NA_1 && nCauseOfTx== COT_ACT)
						{
							MOSCAD_sprintf(message, "Socket: %d: Clock syncron received, retval : %d",INDX, retval);
				      MOSCAD_message(message);
					    nMessLength = 22;
              nClockSyncStep[INDX] = 1;														


								fnBuildCP56Time2aIEC(&sBuff[10 + nLenCAOA + nLenIOA]);								
								fnSetMOSCADTime();
											    
					    					    
						} /* nTypeID[INDX] ==TI_C_SC_NA_1 */
																
				} /* end if ((nRecSeqNum[INDX] == nSendSeqNumClient+1)) */
        else
        {
        /* Nem a várt sorszámú távirat jött */
        p_col_Stat[19] = p_col_Stat[19] + 1;  /* 2014.05.12 */
        
        
        			MOSCAD_sprintf(message, "Socket: %d: Nem a várt sorszámú távirat jött : nRecSeqNum[INDX]: %d,    nSendSeqNumClient+1: %d",INDX, nRecSeqNum[INDX],nSendSeqNumClient+1);
				      MOSCAD_message(message);
              fnDisconnect(INDX);        
        }
				
			} /* end I format*/
			

		} /* end if sBuff[0] == 0x68*/
		else    /* Ha egyik se, törli a buffert */
		{
			 nMessLength = retval;
			 MOSCAD_message("Törli az sBuff tartalmát !!!!!!!!!!!!!!!!!!!!!");			
    }

		
  if (retval >= nMessLength)
  {		
    retval = retval - nMessLength;
    memcpy(&sBuff[0],&sBuff[nMessLength],sizeof(sBuff)-nMessLength	);	
  }
  else
  {
    MOSCAD_sprintf(message, "Socket: %d: nMessLength > retval !!!!!!!!!!!!!!!!!!!!!, retval: %d, nMessLength: %d",INDX, retval, nMessLength);		
    MOSCAD_message(message);
    retval = 0;
  }		
  
 /*MOSCAD_wait(100); */
}	
while (retval > 0 && nCikl<10 );
		
		
} /*end else if retval==0 */
	

	
} /* end else  if (retval ==ERR_MOSCAD_SOCKET_ERROR) */


			
		
		
					
			/*fnSpontTest(0);*/
			
			
/*					MOSCAD_sprintf(message,"Direct nDisableWrite: %d", nDisableWrite );			
					MOSCAD_message(message ); */

/* ====================================================================================================================================================== */			   
/*			  fnEvents(duiRec);  */



/* Ha már feldolgozta a bejövõ táviratokat */






	lTickEv = 0;
	
			
						
		/*if (nDelayedStart == 0)
		{
			return;
		}*/


		MOSCAD_get_datetime(&mdt);
		
		
		if (mdt.seconds>nOldSec || mdt.seconds <=1 )
		{
			n20Msec = 0;
		}
		
		nMsec = 1000 * mdt.seconds + mdt.minutes + mdt.seconds + mdt.hours;
		
		nOldSec = mdt.seconds;
		
   	if (MOSCAD_get_table_info (2,&table_parInt)!=0 )
   		{
        MOSCAD_sprintf(message,"IEC drv: 1. No valid information in table: %d",2);
        MOSCAD_error(message );
        return;
   		}
	p_col_parInt = (short *)(table_parInt.ColDataPtr[0]);	
		
	
/* Ha már meg volt az általános lekérdezés ...............................................................................................................................*/
if (nInterrogated[INDX] == 1)
{  
  	
/* Egybites jelzesek ---------------------------------------------------------------------------*/	
/*Ha ures a puffer----------------------------------*/
if (nSPWrPtr[INDX]==0)
{
		nSPTempPtr = 0;

/*		MOSCAD_sprintf(message,"IECDRV104: fnEvents running..., newsocket[0]: %d, nStarted[0]: %d, nSPWrPtr[INDX]: %d, nSPReadPtr[INDX]: %d, nSPTempPtr: %d, nSPNum: %d",newsocket[0],nStarted[0], nSPWrPtr[INDX], nSPReadPtr[INDX],nSPTempPtr,nSPNum );			
		MOSCAD_message(message );            */



	for (nI=0;nI<nSPNum && nI<MAX_SP_NUM && (nSPTempPtr < MAX_SP_EVNUM) ;nI++)
	{	
	
				
	fnReadSPData2(nI, &bySP[nI]);
	
		/* Ha van SP event */
		if ( bySP[nI] != byPrSP[INDX][nI] ) 
		{			
			
			
				/* MOSCAD_sprintf(message, " New SP event, offset: %d, value: %d", nI,bySP[nI] );
				MOSCAD_message(message);
		
		MOSCAD_sprintf(message,"IECDRV2: fnEvents running..., newsocket[0]: %d, nStarted[0]: %d, nSPWrPtr[INDX]: %d, nSPReadPtr[INDX]: %d, nSPTempPtr: %d",newsocket[0],nStarted[0], nSPWrPtr[INDX], nSPReadPtr[INDX],nSPTempPtr );			
		MOSCAD_message(message );       */
		

			
			fnReadSPDataTime(nI,&bySP[nI], &nMS1, &nMS2, &nMin, &bTTime, &nXOR);
			
			/* Information object address beirasa */
			fnBuildIOA(&strSPEventWT[INDX][nSPTempPtr].byIOA[0], lSPStart+nI);
			/* SP data beirasa*/
			strSPEventWT[INDX][nSPTempPtr].bySP = bySP[nI];
			/* Msec es perc beirasa */
			/* Van kivulrol kapott ido */
			bDevTime = bTTime;
			
      lTimeFIU = mdt.minutes * 60000 + nMsec;
      lTimeRTU = nMin * 60000 + nMS2 * 256 + nMS1;
      
      
			

			
			if ((bDevTime == 0) || (lTimeFIU-lTimeRTU > 120000) ||  (nMin > mdt.minutes)  )
			{
      
        if (bDevTime == 1)
         {
 			   MOSCAD_sprintf(message," lTimeFIU : %d,  lTimeRTU: %d, mdt.minutes: %d, nMsec: %d, nMin: %d, nMS2: %d, nMS1: %d ", lTimeFIU, lTimeRTU, mdt.minutes, nMsec, nMin, nMS2, nMS1 );			
			   MOSCAD_message(message );            
         }
				
				/*strSPEventWT[INDX][nSPTempPtr].byTime[1] = nMsec / 256;
				strSPEventWT[INDX][nSPTempPtr].byTime[0] = nMsec - (nMsec / 256) * 256;
				strSPEventWT[INDX][nSPTempPtr].byTime[2] = mdt.minutes;*/
				strSPEventWT[INDX][nSPTempPtr].sTime.byMs[0] 			= nMsec - (nMsec / 256) * 256;
				strSPEventWT[INDX][nSPTempPtr].sTime.byMs[1] 			= nMsec / 256;
				strSPEventWT[INDX][nSPTempPtr].sTime.byMin			= mdt.minutes;
				strSPEventWT[INDX][nSPTempPtr].sTime.byHour			= mdt.hours;
				strSPEventWT[INDX][nSPTempPtr].sTime.byDayMonth_Week	= mdt.date + mdt.wday*64;
				strSPEventWT[INDX][nSPTempPtr].sTime.byMon			= mdt.month;
				strSPEventWT[INDX][nSPTempPtr].sTime.byYear			= mdt.year;
			}
			else 
			{
				/*strSPEventWT[INDX][nSPTempPtr].byTime[0] = nMS1;
				strSPEventWT[INDX][nSPTempPtr].byTime[1] = nMS2;
				strSPEventWT[INDX][nSPTempPtr].byTime[2] = nMin;*/	
				
				strSPEventWT[INDX][nSPTempPtr].sTime.byMs[0] 			= nMS1 + nMin* 3 + mdt.hours * 2;
				strSPEventWT[INDX][nSPTempPtr].sTime.byMs[1] 			= nMS2;
				strSPEventWT[INDX][nSPTempPtr].sTime.byMin			= nMin;
				strSPEventWT[INDX][nSPTempPtr].sTime.byHour			= mdt.hours;
				strSPEventWT[INDX][nSPTempPtr].sTime.byDayMonth_Week	= mdt.date + mdt.wday*64;
				strSPEventWT[INDX][nSPTempPtr].sTime.byMon			= mdt.month;
				strSPEventWT[INDX][nSPTempPtr].sTime.byYear			= mdt.year;							
			}
			
			nSPTempPtr++;
			byPrSP[INDX][nI] = bySP[nI];
			
		} /* end if esemeny volt */		
	} /* end for */
		
			/* Beiro pointer aktualizalasa */	 			
			if (nSPTempPtr>0)
			{
				nSPWrPtr[INDX]=nSPTempPtr;			
          /* ASDU összeállítása */				
					byNum = nSPWrPtr[INDX];	
					nNum = fnAPCISeqNums(strASDU[INDX][nASDUWrPtr[INDX]].sBuff, 0,0,I_FORMAT, 2); /* csak a hely miatt, nem a véglelges értékek. A kezdõ karakterek helye is ki van hagyva */															
					nNum = fnBuildDUI(strASDU[INDX][nASDUWrPtr[INDX]].sBuff,TI_M_SP_TB_1 , 0,byNum , COT_SPONT, 0,nNum, &duiTransmit);
					nNum = fnBuildInfObj(strASDU[INDX][nASDUWrPtr[INDX]].sBuff, byIOA, (BYTE*)strSPEventWT[INDX], nNum,duiTransmit, INDX);     /* byIOA nincs használva!!! */      

          strASDU[INDX][nASDUWrPtr[INDX]].nLength = nNum;       /* 2 + 4 + ASDU hossz */
          
          /* Beíró pointer növelése */
          
          
  				/* MOSCAD_sprintf(message,"Socket: %d:  Elküldendõ SP ASDU: nASDUWrPtr[INDX]: %d, length: %d", INDX,nASDUWrPtr[INDX], strASDU[INDX][nASDUWrPtr[INDX]].nLength);			
					MOSCAD_message(message ); */
        
          if (nASDUWrPtr[INDX]<MAX_ASDU-5)
          {
              nASDUWrPtr[INDX]++;
          }
          else
          {
           p_col_Stat[20] = p_col_Stat[20] + 1;  /* 2014.05.12 */
   				MOSCAD_sprintf(message,"Socket: %d:  Túl sok az elküldendõ üzenet, SP,  [0]: %d, [1]: %d, [2]: %d, [3]: %d, [42]: %d , [43]: %d , [44]: %d, [45]: %d, [46]: %d , [47]: %d  " ,INDX, strASDU[INDX][0].nSendNum, strASDU[INDX][1].nSendNum,strASDU[INDX][2].nSendNum,strASDU[INDX][3].nSendNum,strASDU[INDX][42].nSendNum,strASDU[INDX][43].nSendNum,  strASDU[INDX][44].nSendNum, strASDU[INDX][45].nSendNum, strASDU[INDX][46].nSendNum, strASDU[INDX][47].nSendNum);			
					MOSCAD_message(message );
          }
        											
			}
} /*end if nSPWrPtr[INDX]==0 */	
	
/* Ketbites jelzesek --------------------------------------------------------------------------------------------*/	
/*Ha ures a puffer-------------------------*/
if (nDPWrPtr[INDX]==0)
{
	
	nDPTempPtr = 0;
	for (nI=0;nI<nDPNum && nI<MAX_DP_NUM && (nDPTempPtr < MAX_DP_EVNUM);nI++)
	{
		
	fnReadDPDataTime(nI, &byDP[nI], &nMS1, &nMS2, &nMin, &bTTime);				
			
		if ( byDP[nI] != byPrDP[INDX][nI] ) 
		{					
			/* Information object address beirasa */
			fnBuildIOA(&strDPEventWT[INDX][nDPTempPtr].byIOA[0], lDPStart+nI);
			/* DP data beirasa*/
			strDPEventWT[INDX][nDPTempPtr].byDP = byDP[nI];
			
			bDevTime = bTTime;
							
      lTimeFIU = mdt.minutes * 60000 + nMsec;
      lTimeRTU = nMin * 60000 + nMS2 * 256 + nMS1;
              
              
			if ((bDevTime == 0) || (lTimeFIU-lTimeRTU > 120000) ||  (nMin > mdt.minutes)  )
			{
         if (bDevTime == 1)
         {
 			   MOSCAD_sprintf(message," lTimeFIU : %d,  lTimeRTU: %d, mdt.minutes: %d, nMsec: %d, nMin: %d, nMS2: %d, nMS1: %d ", lTimeFIU, lTimeRTU, mdt.minutes, nMsec, nMin, nMS2, nMS1 );			
			   MOSCAD_message(message );            
         }

				/* Msec es perc beirasa */
				strDPEventWT[INDX][nDPTempPtr].sTime.byMs[0] 			= nMsec - (nMsec / 256) * 256;
				strDPEventWT[INDX][nDPTempPtr].sTime.byMs[1]			= nMsec / 256;
				strDPEventWT[INDX][nDPTempPtr].sTime.byMin 			= mdt.minutes;	
				strDPEventWT[INDX][nDPTempPtr].sTime.byHour			= mdt.hours;
				strDPEventWT[INDX][nDPTempPtr].sTime.byDayMonth_Week	= mdt.date + mdt.wday*64;
				strDPEventWT[INDX][nDPTempPtr].sTime.byMon			= mdt.month;
				strDPEventWT[INDX][nDPTempPtr].sTime.byYear			= mdt.year;
						
			}
			else
			{
								/* Msec es perc beirasa */
				strDPEventWT[INDX][nDPTempPtr].sTime.byMs[0]			= nMS1;
				strDPEventWT[INDX][nDPTempPtr].sTime.byMs[1]			= nMS2;
				strDPEventWT[INDX][nDPTempPtr].sTime.byMin			= nMin;			
				strDPEventWT[INDX][nDPTempPtr].sTime.byHour			= mdt.hours;
				strDPEventWT[INDX][nDPTempPtr].sTime.byDayMonth_Week	= mdt.date + mdt.wday*64;
				strDPEventWT[INDX][nDPTempPtr].sTime.byMon			= mdt.month;
				strDPEventWT[INDX][nDPTempPtr].sTime.byYear			= mdt.year;
			}
		
			nDPTempPtr++;	
			
			byPrDP[INDX][nI] = byDP[nI];
			
		} /* end if esemeny volt */
			
	} /* end for */
	
			/* Beiro pointer aktualizalasa */	 			
			if (nDPTempPtr > 0)
			{
				nDPWrPtr[INDX]=nDPTempPtr;			
          /* ASDU összeállítása */				
					byNum = nDPWrPtr[INDX];	
					nNum = fnAPCISeqNums(strASDU[INDX][nASDUWrPtr[INDX]].sBuff, 0,0,I_FORMAT, 2); /* csak a hely miatt, nem a véglelges értékek. A kezdõ karakterek helye is ki van hagyva */															
					nNum = fnBuildDUI(strASDU[INDX][nASDUWrPtr[INDX]].sBuff,TI_M_DP_TB_1 , 0,byNum , COT_SPONT, 0,nNum,&duiTransmit);
					nNum = fnBuildInfObj(strASDU[INDX][nASDUWrPtr[INDX]].sBuff, byIOA, (BYTE*)strDPEventWT[INDX], nNum,duiTransmit, INDX);     /* byIOA nincs használva!!! */      

          strASDU[INDX][nASDUWrPtr[INDX]].nLength = nNum;       /* 2 + 4 + ASDU hossz */
          
          /* Beíró pointer növelése */
          
          
  				/* MOSCAD_sprintf(message,"Socket: %d: Elküldendõ DP ASDU: nASDUWrPtr[INDX]: %d, length: %d", INDX, nASDUWrPtr[INDX], strASDU[INDX][nASDUWrPtr[INDX]].nLength);			
					MOSCAD_message(message ); */
				
          if (nASDUWrPtr[INDX]<MAX_ASDU-5)
          {
              nASDUWrPtr[INDX]++;
          }
          else
          {
           p_col_Stat[21] = p_col_Stat[21] + 1;  /* 2014.05.12 */
   				MOSCAD_sprintf(message,"Socket: %d:  Túl sok az elküldendõ üzenet, DP,  [0]: %d, [1]: %d, [2]: %d, [3]: %d, [42]: %d , [43]: %d , [44]: %d, [45]: %d, [46]: %d , [47]: %d  " ,INDX, strASDU[INDX][0].nSendNum, strASDU[INDX][1].nSendNum,strASDU[INDX][2].nSendNum,strASDU[INDX][3].nSendNum,strASDU[INDX][42].nSendNum,strASDU[INDX][43].nSendNum,  strASDU[INDX][44].nSendNum, strASDU[INDX][45].nSendNum, strASDU[INDX][46].nSendNum, strASDU[INDX][47].nSendNum);			
					MOSCAD_message(message );
          }
				
				
			}	
} /* end if (nDPWrPtr[INDX]==0) */	
		
/* Normalt mert ertekek valtozas figyelese -----------------------------------------------------------------*/	
/*Ha ures a puffer-------------------------*/
if (nNMWrPtr[INDX]==0)
{	
	
	flDelta = (float)nDelta;
	nNMTempPtr = 0;
	for (nI=0;nI<nNMNum && nI<MAX_NM_NUM && (nNMTempPtr < MAX_NM_EVNUM-1);nI++)
	{

	fnReadNMData(nI, &nNM[nI], &nLiveZero[nI], &nStatus[nI]);

		
		if ( (nNM[nI] > nPrNM[INDX][nI] * (1.04 )) || (nNM[nI] < nPrNM[INDX][nI] * (0.96 )) || nStatus[nI]!=nPrStatus[nI]) 
		{		
				
			/* Information object address beirasa */
			fnBuildIOA(&strNMEvent104[INDX][nNMTempPtr].byIOA[0], lNMStart+nI);

			/* NM data beirasa*/
			/*nLiveZero = p_col_NM_LZ[nI]; */
			fnNorm(nNM[nI], nLiveZero[nI], &strNMEvent104[INDX][nNMTempPtr].byNM[0]);
			
			/*nTemp = byTemp[0] + byTemp[1]*256;
			strNMEvent[nNMWrPtr[INDX]].nNM = nTemp;*/
			/* Qualifier */
			nInvalid = 0;
			if (nStatus[nI]  == 1)
					{
						nInvalid = 64;
					}
					
			strNMEvent104[INDX][nNMTempPtr].byQ = nInvalid;
			nPrNM[INDX][nI] = nNM[nI];
			nPrStatus[nI] = nStatus[nI];
		
			nNMTempPtr++;	

		} /* end if esemeny volt */
			
	} /* end for */
	
				/* Ha volt esemény */	 	
				if (nNMTempPtr > 0)
				{
								
				
				
  				nNMWrPtr[INDX]=nNMTempPtr;			
          /* ASDU összeállítása */				
					byNum = nNMWrPtr[INDX];	
					nNum = fnAPCISeqNums(strASDU[INDX][nASDUWrPtr[INDX]].sBuff, 0,0,I_FORMAT, 2); /* csak a hely miatt, nem a véglelges értékek. A kezdõ karakterek helye is ki van hagyva */															
					nNum = fnBuildDUI(strASDU[INDX][nASDUWrPtr[INDX]].sBuff,TI_M_ME_NA_1 , 0,byNum , COT_SPONT, 0,nNum,&duiTransmit);
					nNum = fnBuildInfObj(strASDU[INDX][nASDUWrPtr[INDX]].sBuff, byIOA, (BYTE*)strNMEvent104[INDX], nNum,duiTransmit, INDX);     /* byIOA nincs használva!!! */      

          strASDU[INDX][nASDUWrPtr[INDX]].nLength = nNum;       /* 2 + 4 + ASDU hossz */
          
          
          /* Beíró pointer növelése */
  				/* MOSCAD_sprintf(message,"Socket: %d: Elküldendõ NM ASDU: nASDUWrPtr[INDX]: %d, length: %d",INDX, nASDUWrPtr[INDX], strASDU[INDX][nASDUWrPtr[INDX]].nLength);			
					MOSCAD_message(message ); */
				
          if (nASDUWrPtr[INDX]<MAX_ASDU-5)
          {
              nASDUWrPtr[INDX]++;
          }
          else
          {
           p_col_Stat[22] = p_col_Stat[22] + 1;  /* 2014.05.12 */
   				MOSCAD_sprintf(message,"Socket: %d:  Túl sok az elküldendõ üzenet, NM,  [0]: %d, [1]: %d, [2]: %d, [3]: %d, [42]: %d , [43]: %d , [44]: %d, [45]: %d, [46]: %d , [47]: %d  " ,INDX, strASDU[INDX][0].nSendNum, strASDU[INDX][1].nSendNum,strASDU[INDX][2].nSendNum,strASDU[INDX][3].nSendNum,strASDU[INDX][42].nSendNum,strASDU[INDX][43].nSendNum,  strASDU[INDX][44].nSendNum, strASDU[INDX][45].nSendNum, strASDU[INDX][46].nSendNum, strASDU[INDX][47].nSendNum);			
					MOSCAD_message(message );
          }
				
				
				
				}
	
	
	
} /*end if (nNMWrPtr[INDX]==0) ----------------------------------------------------------------------------------*/	
	
		/*MOSCAD_sprintf(message,"lTickEv: %d",lTickEv);
    	MOSCAD_error(message );*/
    	

} /* end if nInterrogated[INDX] == 1 */
/* .......................................................................................................................................................................... */

if (nInterrogationStep[INDX] == 5)  /* Activation termination */
{

			nNum = fnAPCISeqNums(strASDU[INDX][nASDUWrPtr[INDX]].sBuff, 0,0,I_FORMAT, 2); /* csak a hely miatt, nem a véglelges értékek. A kezdõ karakterek helye is ki van hagyva */															
			nNum = fnBuildDUI(strASDU[INDX][nASDUWrPtr[INDX]].sBuff,TI_C_IC_NA_1 , 0,1 , COT_ACTTERM,duiRec.byOrigAddr, nNum,&duiTransmit);			
			nNum = fnBuildInfObj(strASDU[INDX][nASDUWrPtr[INDX]].sBuff, byIOA, byData, nNum,duiTransmit, INDX);        /* byData nincs használva */
			
      strASDU[INDX][nASDUWrPtr[INDX]].nLength = nNum;       /* 2 + 4 + ASDU hossz */			
			
			MOSCAD_sprintf(message,"Socket: %d: nInterrogationStep[INDX]=5, nNum: %d", INDX, nNum);			
			MOSCAD_message(message );
			
          if (nASDUWrPtr[INDX]<MAX_ASDU-1)
          {
              nASDUWrPtr[INDX]++;
          }
          else
          {
					   MOSCAD_message("Túl sok az elküldendõ üzenet, interrogation" );
             fnDisconnect(INDX);
          }


            nInterrogationStep[INDX] = 0;
            nInterrogated[INDX] = 1;
                        
            /*fnSendTESTFR_ACT();	*/		          
          			
}   /*  end nInterrogationStep[INDX] == 5) */

if (nInterrogationStep[INDX] == 4)   /* Send SP values */
{

			if (nSPNum < 126)
			{
				nDataNum = nSPNum;			
			}							
			else
			{
				if ( (nSPNum - nSendSP[INDX]) > 125)
				{
				nDataNum = 125;
				}
				else
				{
					nDataNum= nSPNum - nSendSP[INDX];
				}
			}
			           

			nNum = fnAPCISeqNums(strASDU[INDX][nASDUWrPtr[INDX]].sBuff, 0,0,I_FORMAT, 2); /* csak a hely miatt, nem a véglelges értékek. A kezdõ karakterek helye is ki van hagyva */															
 			fnBuildIOA(byIOA, lSPStart + nSendSP[INDX]);	     			
			nNum = fnBuildDUI(strASDU[INDX][nASDUWrPtr[INDX]].sBuff,TI_M_SP_NA_1 , 1,nDataNum , COT_INROGEN,duiRec.byOrigAddr, nNum,&duiTransmit);			
			nNum = fnBuildInfObj(strASDU[INDX][nASDUWrPtr[INDX]].sBuff, byIOA, byData, nNum,duiTransmit, INDX);

      strASDU[INDX][nASDUWrPtr[INDX]].nLength = nNum;       /* 2 + 4 + ASDU hossz */			
			
			MOSCAD_sprintf(message,"Socket:%d: nInterrogationStep[INDX]=4, nNum: %d, nSPNum: %d, SP offset: %d, nDataNum: %d",INDX, nNum, nSPNum, nSendSP[INDX], nDataNum);			
			MOSCAD_message(message );
	
			nSendSP[INDX] = nSendSP[INDX] + nDataNum;
					
          if (nASDUWrPtr[INDX]<MAX_ASDU-1)
          {
              nASDUWrPtr[INDX]++;
          }
          else
          {
					   MOSCAD_message("Túl sok az elküldendõ üzenet, interrogation" );
             fnDisconnect(INDX);             
          }

          if (nSendSP[INDX] == nSPNum)
          {
            nInterrogationStep[INDX] = 5;
          }
          
          /*fnSendTESTFR_ACT();*/
          			
}     /*  end nInterrogationStep[INDX] == 4) */
  
if (nInterrogationStep[INDX] == 3)   /* Send DP values */
{

			if (nDPNum < 126)
			{
				nDataNum = nDPNum;			
			}							
			else
			{
				if ( (nDPNum - nSendDP[INDX]) > 125)
				{
				nDataNum = 125;
				}
				else
				{
					nDataNum= nDPNum - nSendDP[INDX];
				}
			}
			           

			nNum = fnAPCISeqNums(strASDU[INDX][nASDUWrPtr[INDX]].sBuff, 0,0,I_FORMAT, 2); /* csak a hely miatt, nem a véglelges értékek. A kezdõ karakterek helye is ki van hagyva */															
 			fnBuildIOA(byIOA, lDPStart + nSendDP[INDX]);	     			
			nNum = fnBuildDUI(strASDU[INDX][nASDUWrPtr[INDX]].sBuff,TI_M_DP_NA_1 , 1,nDataNum , COT_INROGEN,duiRec.byOrigAddr, nNum,&duiTransmit);			
			nNum = fnBuildInfObj(strASDU[INDX][nASDUWrPtr[INDX]].sBuff, byIOA, byData, nNum,duiTransmit, INDX);

      strASDU[INDX][nASDUWrPtr[INDX]].nLength = nNum;       /* 2 + 4 + ASDU hossz */			
			
			MOSCAD_sprintf(message,"Socket: %d: nInterrogationStep[INDX]=3, nNum: %d, DP offset: %d", INDX,nNum, nSendDP[INDX]);			
			MOSCAD_message(message );
	
			nSendDP[INDX] = nSendDP[INDX] + nDataNum;
					
          if (nASDUWrPtr[INDX]<MAX_ASDU-1)
          {
              nASDUWrPtr[INDX]++;
          }
          else
          {
					   MOSCAD_message("Túl sok az elküldendõ üzenet, interrogation" );
             fnDisconnect(INDX);             
          }

          if (nSendDP[INDX] == nDPNum)
          {
            nInterrogationStep[INDX] = 4;
          }
          
          /*fnSendTESTFR_ACT(); */
          			
}     /*  end nInterrogationStep[INDX] == 2) */


if (nInterrogationStep[INDX] == 2)   /* Send NM values */
{
		/* NM-k elkuldese */
			if (nNMNum < 76)
			{
				nDataNum = nNMNum;
			}
			else
			{
				if ( (nNMNum - nSendNM[INDX]) > 75)
				{
					nDataNum = 75;
				}
				else
				{
					nDataNum= nNMNum - nSendNM[INDX];
				}
			}			

			nNum = fnAPCISeqNums(strASDU[INDX][nASDUWrPtr[INDX]].sBuff, 0,0,I_FORMAT, 2); /* csak a hely miatt, nem a véglelges értékek. A kezdõ karakterek helye is ki van hagyva */															
 			fnBuildIOA(byIOA, lNMStart + nSendNM[INDX]);	     			
			nNum = fnBuildDUI(strASDU[INDX][nASDUWrPtr[INDX]].sBuff,TI_M_ME_NA_1 , 1,nDataNum , COT_INROGEN,duiRec.byOrigAddr, nNum,&duiTransmit);			
			nNum = fnBuildInfObj(strASDU[INDX][nASDUWrPtr[INDX]].sBuff, byIOA, byData, nNum,duiTransmit, INDX);

			
      strASDU[INDX][nASDUWrPtr[INDX]].nLength = nNum;       /* 2 + 4 + ASDU hossz */			
			
			MOSCAD_sprintf(message,"Socket: %d:  nInterrogationStep[INDX]=2, nNum: %d, NM offset: %d", INDX, nNum, nSendNM[INDX]);			
			MOSCAD_message(message );


		
			nSendNM[INDX] = nSendNM[INDX] + nDataNum;
					
          if (nASDUWrPtr[INDX]<MAX_ASDU-1)
          {
              nASDUWrPtr[INDX]++;
          }
          else
          {
					   MOSCAD_message("Túl sok az elküldendõ üzenet, interrogation" );
             fnDisconnect(INDX);             
          }

          if (nSendNM[INDX] == nNMNum)
          {
            nInterrogationStep[INDX] = 3;
          }
          
          /*fnSendTESTFR_ACT(); */
          			
}     /*  end nInterrogationStep[INDX] == 2) */

	
if (nInterrogationStep[INDX] == 1)  /* Activation confirmation */
{

			nNum = fnAPCISeqNums(strASDU[INDX][nASDUWrPtr[INDX]].sBuff, 0,0,I_FORMAT, 2); /* csak a hely miatt, nem a véglelges értékek. A kezdõ karakterek helye is ki van hagyva */															
			nNum = fnBuildDUI(strASDU[INDX][nASDUWrPtr[INDX]].sBuff,TI_C_IC_NA_1 , 0,1 , COT_ACTCON,duiRec.byOrigAddr, nNum,&duiTransmit);			 /* COT_ACTCON helyett COT_ACT */
			nNum = fnBuildInfObj(strASDU[INDX][nASDUWrPtr[INDX]].sBuff, byIOA, byData, nNum,duiTransmit, INDX);        /* byData nincs használva */
			
      strASDU[INDX][nASDUWrPtr[INDX]].nLength = nNum;       /* 2 + 4 + ASDU hossz */			
			
			MOSCAD_sprintf(message,"Socket: %d: nInterrogationStep[INDX]=1, nNum: %d",INDX, nNum);			
			MOSCAD_message(message );
			
          if (nASDUWrPtr[INDX]<MAX_ASDU-1)
          {
              nASDUWrPtr[INDX]++;
          }
          else
          {
					   MOSCAD_message("Túl sok az elküldendõ üzenet, interrogation" );
             fnDisconnect(INDX);             
          }

          nInterrogationStep[INDX] = 2;			
          nSendNM[INDX] = 0;
          nSendDP[INDX] = 0;
          nSendSP[INDX] = 0;


         /*fnSendTESTFR_ACT();	*/		

          
 
                    
}   /*  end nInterrogationStep[INDX] == 1) */
       	

		if (nSCStep[INDX] == 1) /*Activation confirm */
		{
			/*MOSCAD_sprintf(message,"Activation confirm, single command");						
			MOSCAD_error(message);				*/

			nNum = fnAPCISeqNums(strASDU[INDX][nASDUWrPtr[INDX]].sBuff, 0,0,I_FORMAT, 2); /* csak a hely miatt, nem a véglelges értékek. A kezdõ karakterek helye is ki van hagyva */		      													
			nNum = fnBuildDUI(strASDU[INDX][nASDUWrPtr[INDX]].sBuff,TI_C_SC_NA_1 , 0,1 , COT_ACTCON,				duiRec.byOrigAddr, nNum,&duiTransmit);
			nNum = fnBuildInfObj(strASDU[INDX][nASDUWrPtr[INDX]].sBuff, byIOA, byData, nNum,duiTransmit, INDX);        /* byData nincs használva */
			
			
			
      strASDU[INDX][nASDUWrPtr[INDX]].nLength = nNum;       /* 2 + 4 + ASDU hossz */		
      
			MOSCAD_sprintf(message,"Socket: %d ---------------------------------------------------------------------IECDRV2: nSCStep[INDX]=1, nNum: %d, nLenIOA: %d, nASDUWrPtr[INDX]: %d", INDX,nNum, nLenIOA, nASDUWrPtr[INDX]);	
      MOSCAD_message(message);
      
      	
			
			
          if (nASDUWrPtr[INDX]<MAX_ASDU-1)
          {
              nASDUWrPtr[INDX]++;
          }
          else
          {
					   MOSCAD_message("Túl sok az elküldendõ üzenet, SC ACTCON" );
             fnDisconnect(INDX);
          }

    nSCStep[INDX] = 0;
						
		}	 /* end if  (nSCStep[INDX] == 1) */
		
		
		if (nDCStep[INDX] == 1) /*Activation confirm */
		{
			/*MOSCAD_sprintf(message,"Activation confirm, single command");						
			MOSCAD_error(message);				*/

			nNum = fnAPCISeqNums(strASDU[INDX][nASDUWrPtr[INDX]].sBuff, 0,0,I_FORMAT, 2); /* csak a hely miatt, nem a véglelges értékek. A kezdõ karakterek helye is ki van hagyva */		      													
			nNum = fnBuildDUI(strASDU[INDX][nASDUWrPtr[INDX]].sBuff,TI_C_DC_NA_1 , 0,1 , COT_ACTCON,				duiRec.byOrigAddr, nNum,&duiTransmit);
			nNum = fnBuildInfObj(strASDU[INDX][nASDUWrPtr[INDX]].sBuff, byIOA, byData, nNum,duiTransmit, INDX);        /* byData nincs használva */
			
			
			
      strASDU[INDX][nASDUWrPtr[INDX]].nLength = nNum;       /* 2 + 4 + ASDU hossz */		
      
			MOSCAD_sprintf(message,"Socket: %d ---------------------------------------------------------------------IECDRV2: nDCStep[INDX]=1, nNum: %d, nLenIOA: %d, nASDUWrPtr[INDX]: %d",INDX, nNum, nLenIOA, nASDUWrPtr[INDX]);	
      MOSCAD_message(message);
      
      	
			
			
          if (nASDUWrPtr[INDX]<MAX_ASDU-1)
          {
              nASDUWrPtr[INDX]++;
          }
          else
          {
					   MOSCAD_message("Túl sok az elküldendõ üzenet, DC ACTCON" );
             fnDisconnect(INDX);             
          }



    nDCStep[INDX] = 0;
						
		}		 /* end if  (nDCStep[INDX] == 1) */	
		
		

if (nClockSyncStep[INDX] == 1)  /* Activation confirmation */
{

			nNum = fnAPCISeqNums(strASDU[INDX][nASDUWrPtr[INDX]].sBuff, 0,0,I_FORMAT, 2); /* csak a hely miatt, nem a véglelges értékek. A kezdõ karakterek helye is ki van hagyva */															
			nNum = fnBuildDUI(strASDU[INDX][nASDUWrPtr[INDX]].sBuff,TI_C_CS_NA_1 , 0,1 , COT_ACTCON,duiRec.byOrigAddr, nNum,&duiTransmit);			 /* COT_ACTCON helyett COT_ACT */
			nNum = fnBuildInfObj(strASDU[INDX][nASDUWrPtr[INDX]].sBuff, byIOA, byData, nNum,duiTransmit, INDX);        /* byData nincs használva */
			
 			MOSCAD_sprintf(message,"Socket: %d: IECDRV2: nClockSyncStep[INDX]=1, nNum: %d",INDX, nNum);			
			MOSCAD_message(message );
						
      strASDU[INDX][nASDUWrPtr[INDX]].nLength = nNum;       /* 2 + 4 + ASDU hossz */			
			
			
          if (nASDUWrPtr[INDX]<MAX_ASDU-1)
          {
              nASDUWrPtr[INDX]++;
          }
          else
          {
					   MOSCAD_message("Túl sok az elküldendõ üzenet, clock sync" );
             fnDisconnect(INDX);             
          }

          nClockSyncStep[INDX] = 0;			
 

         /*fnSendTESTFR_ACT();	*/		

                    
}   /*  end nClockSycStep == 1) */




			  
			  
        

/* ====================================================================================================================================================== */			   
			
        /* Ha van küldendõ távirat */
				if (nASDUWrPtr[INDX] > nASDUSendPtr[INDX])
				{			
								
				  /* APCI kitöltése a send és receive sorszámokkal */
				  fnAPCISeqNums(strASDU[INDX][nASDUSendPtr[INDX]].sBuff, nSendSeqNum[INDX],nRecSeqNum[INDX],I_FORMAT, 2);
					/*strASDU[INDX][nASDUSendPtr[INDX]].nLength = fnBuildVarEnd(strASDU[INDX][nASDUSendPtr[INDX]].sBuff,strASDU[INDX][nASDUSendPtr[INDX]].nLength);*/				
 					fnBuildVarStart(strASDU[INDX][nASDUSendPtr[INDX]].sBuff, strASDU[INDX][nASDUSendPtr[INDX]].nLength)	;		

          strASDU[INDX][nASDUSendPtr[INDX]].nSendNum = nSendSeqNum[INDX];   

					/* MOSCAD_sprintf(message,"Socket: %d: Elküldendõ távirat: nASDUSendPtr[INDX]: %d, nASDUWrPtr[INDX]: %d, Length: %d,  strASDU[INDX][nASDUSendPtr[INDX]].nSendNum: %d, nDisableWrire: %d", INDX,nASDUSendPtr[INDX], nASDUWrPtr[INDX],strASDU[INDX][nASDUSendPtr[INDX]].nLength,strASDU[INDX][nASDUSendPtr[INDX]].nSendNum, nDisableWrite );			
					MOSCAD_message(message ); */


					/* távirat elküldése*/
					nRetVal = MOSCAD_socket_send(newsocket[INDX], strASDU[INDX][nASDUSendPtr[INDX]].sBuff, strASDU[INDX][nASDUSendPtr[INDX]].nLength,0);	
					nASDUSendPtr[INDX]++;
					
					/* 2014.12.11. */
					if (nSendSeqNum[INDX]< 32767)
					{
	        nSendSeqNum[INDX]++;
          }	
          else
          {
          nSendSeqNum[INDX]=0;
          /* nRecSeqNum[INDX]=0; */
          }				

        }	/* end   */
  					

			



} /*End if konnektált ---------------------------------------------------------------------------------------------*/

	
} /*end for neverend---------------------------------------------------------------------------------------------------------*/





}


/****************************************************************************/
/* Build start character of send message 			   													*/
/****************************************************************************/
int fnBuildStartChar(BYTE *buf, BYTE byStartChar,int nNum)
{
		
	buf[nNum] = byStartChar;
		
	return (nNum + 1);

} /* end fnBuildStartChar()*/
/****************************************************************************/
/* Build message length of send message 			   													*/
/****************************************************************************/
int fnBuildMessLength(BYTE *buf, int nLength,int nNum)
{
		
	buf[nNum] = nLength;
		
	return (nNum + 1);

} /* end fnBuildMessLength()*/
/****************************************************************************/
/* Build APCI sequence numbers			   													*/
/****************************************************************************/
int fnAPCISeqNums(BYTE *buf, unsigned int nSendSeqNum,unsigned int nRecSeqNum,int nFormat, int nNum)
{
	if (nFormat == I_FORMAT)
	{
		buf[nNum] 	= (nSendSeqNum - (nSendSeqNum/256)*256) << 1;
		buf[nNum+1] = nSendSeqNum/128;
		buf[nNum+2] = (nRecSeqNum - (nRecSeqNum/256)*256) << 1;
		buf[nNum+3] = nRecSeqNum/128;	
	}
	else if (nFormat == S_FORMAT)
	{
		buf[nNum] 	= 1; /* S format*/
		buf[nNum+1] = 0;
		buf[nNum+2] = (nRecSeqNum - (nRecSeqNum/256)*256) << 1;
		buf[nNum+3] = nRecSeqNum/128;	
	}
		
	
	return (nNum + 4);

} /* end fnAPCISeqNums*/

/****************************************************************************/
/* Send spontaneous data, if connected		   													*/
/****************************************************************************/
void fnTaskSpont(void)
{


	MOSCAD_sprintf(message, "TaskSpont created succesfully");
	MOSCAD_message(message);

  MOSCAD_wait(100);


   for(;;)
   {


    MOSCAD_wait(1000);


		nTableNum1=1;
    nTableNum2=2;	
		fnReadPar(); 


    }

} /* end fnTaskSpont*/

/****************************************************************************/
/* Globális változók inicializálása	   										*/
/****************************************************************************/
void fnSocketInit(int INDX)
{
	int nI;
	
	
	
	for (nI=0;nI<MAX_CONN;nI++)
	{    
		
		/* socket leírók inicializálása*/
/*		newsocket[nI]=-1;

		nWritePtr[nI]=0;
		nReadPtr[nI]=0;
		nAckPtr[nI]=0;
		
    nStarted[nI]=0;*/		


		     
		
	} /* end for*/
	
} /* end fnInit*/

/****************************************************************************/
/* cprog2.c függvényei				   										*/
/****************************************************************************/
/****************************************************************************/
/* Kiolvas egy adatot a SP adatok kozul, a VALID/INVALID statuszt figyelembe veve 	*/
/*																			*/
/****************************************************************************/
void fnReadSPDataTime(int nIEC_Offset, unsigned char *byData,  int *nMS1, int *nMS2, int *nMin, int *bTTime, int *nXOR)
{
short          *p_col_SPAct;
short          *p_col_SP_CTAct;
short          *p_col_SP_MS1Act;
short          *p_col_SP_MS2Act;
short          *p_col_SP_MINAct;
short          *p_col_SP_XORAct;
short          *p_col_SP_STATUS;


int				nIndx;
int				nTblIndx;
int				nStatus;

int				nInvalid;

nTblIndx = nIEC_Offset/250;
nIndx    = nIEC_Offset - nTblIndx *250;
						
					/*Elõállítja a tábla indexet, és offstet */	
					/*fnSPTblIndx(nIEC_Offset, &nSPTblIndx, &nIndx);*/
						
   					
				   	p_col_SPAct     = sSPT[nTblIndx].p_col_SP;
				   	p_col_SP_MS1Act = sSPT[nTblIndx].p_col_SP_MS1;
				   	p_col_SP_MS2Act = sSPT[nTblIndx].p_col_SP_MS2;
				   	p_col_SP_MINAct = sSPT[nTblIndx].p_col_SP_MIN;				   
				   	p_col_SP_CTAct  = sSPT[nTblIndx].p_col_SP_CT; 	
				   	p_col_SP_XORAct = sSPT[nTblIndx].p_col_SP_XOR; 	
				   	p_col_SP_STATUS = sSPT[nTblIndx].p_col_SP_STATUS;	

					
					*nMS1   = p_col_SP_MS1Act[nIndx];
					*nMS2   = p_col_SP_MS2Act[nIndx];
					*nMin   = p_col_SP_MINAct[nIndx];					
					*bTTime = p_col_SP_CTAct[nIndx];
					*nXOR   = p_col_SP_XORAct[nIndx];
					
					nStatus = p_col_SP_STATUS[nIndx];
					
					nInvalid = 0;
					if (nStatus  == 0)
					{
						nInvalid = 64;
					}
				
				*byData  = p_col_SPAct[nIndx] + nInvalid;	
				
			/*	*byData  = p_col_SPAct[nIndx];*/
				



} /* end fnReadSPData()*/


/****************************************************************************/
/* Kiolvas egy adatot a DP adatok kozul, ido adattal, a VALID/INVALID statuszt figyelembe veve	*/
/*																			*/
/****************************************************************************/
void fnReadDPDataTime(int nIEC_Offset, BYTE *byDP,  int *nMS1, int *nMS2, int *nMin, int *bTTime)
{
short          *p_col_DPLAct;
short          *p_col_DPHAct;
short          *p_col_DP_CTAct;
short          *p_col_DP_MS1Act;
short          *p_col_DP_MS2Act;
short          *p_col_DP_MINAct;

short          *p_col_DP_STATUS;

int				nIndx;
int				nTblIndx;
int				nStatus;
int				nInvalid;

nTblIndx = nIEC_Offset/250;
nIndx    = nIEC_Offset - nTblIndx *250;
						
					/*Elõállítja a tábla indexet, és offstet */	
					/*fnSPTblIndx(nIEC_Offset, &nSPTblIndx, &nIndx);*/
						
   					
				   	p_col_DPLAct    = sDPT[nTblIndx].p_col_DPL;
				   	p_col_DPHAct    = sDPT[nTblIndx].p_col_DPH;
				   	p_col_DP_MS1Act = sDPT[nTblIndx].p_col_DP_MS1;
				   	p_col_DP_MS2Act = sDPT[nTblIndx].p_col_DP_MS2;
				   	p_col_DP_MINAct = sDPT[nTblIndx].p_col_DP_MIN;				   
				   	p_col_DP_CTAct  = sDPT[nTblIndx].p_col_DP_CT; 	
				   	p_col_DP_STATUS = sDPT[nTblIndx].p_col_DP_STATUS; 	

					
					*nMS1   = p_col_DP_MS1Act[nIndx];
					*nMS2   = p_col_DP_MS2Act[nIndx];
					*nMin   = p_col_DP_MINAct[nIndx];					
					*bTTime = p_col_DP_CTAct[nIndx];
					
					nStatus = p_col_DP_STATUS[nIndx];
					
					nInvalid = 0;
					if (nStatus  == 0)
					{
						nInvalid = 64;
					}
					
				
			*byDP = ((BYTE)p_col_DPLAct[nIndx] | ((BYTE)p_col_DPHAct[nIndx] * 2)) + nInvalid;



} /* end fnReadDPDataTime()*/


/****************************************************************************/
/* Kiolvas egy adatot a SP adatok kozul, a VALID/INVALID statuszt figyelembe veve */
/*																			*/
/****************************************************************************/
void fnReadSPData2(int nIEC_Offset, unsigned char *byData)
{
short          *p_col_SPAct;

short          *p_col_SP_STATUS;

int				nIndx;
int				nTblIndx;
int				nStatus;
int				nInvalid;


nTblIndx = nIEC_Offset/250;
nIndx    = nIEC_Offset - nTblIndx *250;
						
					/*Elõállítja a tábla indexet, és offstet */	
					/*fnSPTblIndx(nIEC_Offset, &nSPTblIndx, &nIndx);*/
						
   					
				   	p_col_SPAct     = sSPT[nTblIndx].p_col_SP;
				   	p_col_SP_STATUS = sSPT[nTblIndx].p_col_SP_STATUS;
				   	
				   	nStatus = p_col_SP_STATUS[nIndx];
				   	
				   	nInvalid = 0;
				   	if (nStatus  == 0)
					{
						nInvalid = 64;
					}

				   	
				   	*byData  		= p_col_SPAct[nIndx] + nInvalid;
				   	


} /* end fnReadSPData2()*/

/****************************************************************************/
/* Kiolvas egy adatot az NM adatok kozul, a VALID/INVALID statuszt figyelembe veve	*/
/*																			*/
/****************************************************************************/
void fnReadNMData(int nIEC_Offset, unsigned int *nData, unsigned int *nLiveZero, unsigned int *nStatus)
{

short          *p_col_NMAct;
short          *p_col_NM_LZ_Act;
short          *p_col_NM_STATUS;


int				nIndx;
int				nTblIndx;

nTblIndx = nIEC_Offset/240;
nIndx    = nIEC_Offset - nTblIndx *240;
						
					/*Elõállítja a tábla indexet, és offstet */	
					/*fnSPTblIndx(nIEC_Offset, &nSPTblIndx, &nIndx);*/
						
   					
				   	p_col_NMAct    	= sNMT[nTblIndx].p_col_NM;
				   	p_col_NM_LZ_Act = sNMT[nTblIndx].p_col_NM_LZ;
				   	p_col_NM_STATUS = sNMT[nTblIndx].p_col_NM_STATUS;
				   	
				   	
				   	
				   	*nData  		= p_col_NMAct[nIndx];
				   	*nLiveZero		= p_col_NM_LZ_Act[nIndx];
				   	/* *nStatus		= p_col_NM_STATUS[nIndx];*/
				   	
				   	if (p_col_NM_STATUS[nIndx] == 0)
				   	{
				   		*nStatus = 1;
				   	}
				   	else
				   	{
				   		*nStatus = 0;
				   	}
				   	


} /* end fnReadNMData()*/

/****************************************************************************/
/* main_iec.c függvényei				   										*/
/****************************************************************************/

/****************************************************************************/
/* Inicializalas															*/
/****************************************************************************/
void fnIEC_Init(void)
{

unsigned long		lLargest;
unsigned long		lSRAMLength;

nStart = 0;	
nRxIndx = 0;	
nPrRxIndx = 0;	
nRecEnd = 0;
nLinkInit1 = 0;
nLinkInit2 = 0;
nTxFCB = 0;
nRxFCB = 0;
nActRptNum = 0;
nActTransaction = 0;
/*nInterrogationStep = 0;*/
/*nInterrogated[INDX] = 0;*/
/*nClockSyncStep = 0;*/
bDPSentAll = 1;
bSPSentAll = 1;
bNMSentAll = 1;
nLinkInitStep = 0;
/*nSPReadPtr[INDX] = 0;
nSPWrPtr[INDX] = 0;*/
/*nDPReadPtr[INDX] = 0;                                                                                       
nDPWrPtr[INDX] = 0;
nNMReadPtr[INDX] = 0;
nNMWrPtr[INDX] = 0;*/
/*nASDUWrPtr[INDX] = 0;*/
/*nASDUSendPtr[INDX] = 0;*/



MOSCAD_led_off(CB_ACE_LED_ID_USR1);
MOSCAD_led_off(CB_ACE_LED_ID_USR2);
MOSCAD_led_off(CB_ACE_LED_ID_USR3);
MOSCAD_led_off(CB_ACE_LED_ID_USR4);



/*	
           if (MOSCAD_run_task(CB_TaskA, rx_task, NULL) !=0 )
            {
               MOSCAD_error("Can't run rx_task");
            }
            if (MOSCAD_run_task(CB_TaskB, tx_task, NULL) !=0 )
            {
               MOSCAD_error("Can't run tx_task");
            }
             if (MOSCAD_run_task(CB_TaskC, apl_task, NULL) !=0 )
            {
               MOSCAD_error("Can't run apl_task");
            }
           if (MOSCAD_run_task(CB_TaskD, event_task, NULL) !=0 )
            {
               MOSCAD_error("Can't run event_task");
            }
 */
 
 
      	     MOSCAD_largest_available_free_mem(&lLargest);
	   		 MOSCAD_sprintf(message,"2015.09.21 IEC104 server  program. Ver1.9: Largest available free memory: %ld",lLargest);
   			 MOSCAD_message(message );

         p_col_Stat[25] = 19;  /* Version 1.9  2015.09.21. */


 			lSRAMLength=MOSCAD_bspSRamLength();   

       	if ( lSRAMLength > 0)    /* 2015.05.16. */
				{
     			TotalData = (strTotalData *)MOSCAD_bspSRamStart();
         	lLengthTotalData = sizeof(strTotalData);
          sprintf(message,"SRAM length: %ld, TotalData length: %ld",lSRAMLength,lLengthTotalData);
    			MOSCAD_message(message );         	          	

           p_col_Stat[26] = 1;
        }
        else
				{
       	 sprintf(message,"No SRAM found !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ");
   			 MOSCAD_message(message );         	          	
  
          p_col_Stat[26] = 0;
        }        
        



 
 
         if (MOSCAD_run_task(CB_TaskD, fnTaskCreateSocket, NULL) !=0 )
         {
            MOSCAD_error("Can't run fnTasCreateSocket");
         }
         
         if (MOSCAD_run_task(CB_TaskC, fnTaskServer0, NULL) !=0 )
         {
            MOSCAD_error("Can't run fnTaskServer0");
         }
        
         if (MOSCAD_run_task(CB_TaskB, fnTaskServer1, NULL) !=0 )
         {
            MOSCAD_error("Can't run fnTaskServer1");
         }  
        
         if (MOSCAD_run_task(CB_TaskA, fnTaskSpont, NULL) !=0 )
         {
            MOSCAD_error("Can't run fnTaskSpont"); /* 2014.01.21. GZS : csak az fnReadPar() miatt */ 
         } 


 
} /* end fnIEC_init()*/

/****************************************************************************/
/* Check summat szamol															*/
/****************************************************************************/
BYTE fnIEC_Csum(BYTE *byBuff, BYTE byNum)
{
BYTE	byI;
BYTE 	byCsum;

byCsum = 0;
for (byI = 0; byI<byNum; byI++)
{
	byCsum = byCsum + byBuff[byI];
}


return byCsum;

} /* end fnIEC_Csum()*/
/****************************************************************************/
/* Parameter tabla kiolvasas												*/
/****************************************************************************/
void fnReadPar(void)
{
char 				message[300];
int					nNoRestarted;
int					nResetSRAM; 
int				nSaveTables;
long     lSRAMLength;



/* parameter tabla feldolgozas */
if ((nStart == 0) )
	{
	/* Bool parameterek */	
	  if (MOSCAD_get_table_info (nTableNum1,&table_parBool)!=0 )
	   {
    	MOSCAD_sprintf(message,"2. No valid information in table: %d",nTableNum1);
      	MOSCAD_error(message );
        return;
   		}
	p_col_parBool = (short *)(table_parBool.ColDataPtr[0]);

	/* Egesz parameterek */
   	if (MOSCAD_get_table_info (nTableNum2,&table_parInt)!=0 )
   		{
        MOSCAD_sprintf(message,"3. No valid information in table: %d",nTableNum2);
        MOSCAD_error(message );
        return;
   		}
	p_col_parInt = (short *)(table_parInt.ColDataPtr[0]);	
	
	/* Rx, Tx monitor */
	nRxMonTblIndx = p_col_parInt[0];	
	if (MOSCAD_get_table_info (nRxMonTblIndx,&table_RxMon)!=0 )
   		{
        MOSCAD_sprintf(message,"4. No valid information in table: %d",nRxMonTblIndx);
        MOSCAD_error(message );
        return;
   		}
	p_col_RxMon = (short *)(table_RxMon.ColDataPtr[0]);		
	p_col_TxMon = (short *)(table_RxMon.ColDataPtr[1]);		
	
	
	byRowNumMon = table_RxMon.NumOfRows;
			
	/* Statisztikak */			
	nStatTblIndx = p_col_parInt[2];	
	if (MOSCAD_get_table_info (nStatTblIndx,&table_Stat)!=0 )
   		{
        MOSCAD_sprintf(message,"5. No valid information in table: %d",nStatTblIndx);
        MOSCAD_error(message );
        return;
   		}
	p_col_Stat = (short *)(table_Stat.ColDataPtr[0]);				

	 		
	/* Double point */
/*	nDPTblIndx = p_col_parInt[13];	
	if (MOSCAD_get_table_info (nDPTblIndx,&table_DP)!=0 )
   		{
        MOSCAD_sprintf(message,"No valid information in table: %d",nDPTblIndx);
        MOSCAD_error(message );
        return;
   		}
	p_col_DPL = (short *)(table_DP.ColDataPtr[1]);
	p_col_DPH = (short *)(table_DP.ColDataPtr[0]);
	p_col_DP_MS1 = (short *)(table_DP.ColDataPtr[2]);
	p_col_DP_MS2 = (short *)(table_DP.ColDataPtr[3]);
	p_col_DP_MIN = (short *)(table_DP.ColDataPtr[4]);
	p_col_DP_CT  = (short *)(table_DP.ColDataPtr[5]);
*/
	/* Double point2 */
/*	nDPTblIndx2 = p_col_parInt[43];	
	if (MOSCAD_get_table_info (nDPTblIndx2,&table_DP2)!=0 )
   		{
        MOSCAD_sprintf(message,"No valid information in table: %d",nDPTblIndx2);
        MOSCAD_error(message );
        return;
   		}
	p_col_DP2L = (short *)(table_DP2.ColDataPtr[1]);
	p_col_DP2H = (short *)(table_DP2.ColDataPtr[0]);
	p_col_DP2_MS1 = (short *)(table_DP2.ColDataPtr[2]);
	p_col_DP2_MS2 = (short *)(table_DP2.ColDataPtr[3]);
	p_col_DP2_MIN = (short *)(table_DP2.ColDataPtr[4]);
	p_col_DP2_CT  = (short *)(table_DP2.ColDataPtr[5]);
*/
	/* Double point3 */
/*	nDPTblIndx3 = p_col_parInt[55];	
	if (MOSCAD_get_table_info (nDPTblIndx3,&table_DP3)!=0 )
   		{
        MOSCAD_sprintf(message,"No valid information in table: %d",nDPTblIndx3);
        MOSCAD_error(message );
        return;
   		}
	p_col_DP3L = (short *)(table_DP3.ColDataPtr[1]);
	p_col_DP3H = (short *)(table_DP3.ColDataPtr[0]);
	p_col_DP3_MS1 = (short *)(table_DP3.ColDataPtr[2]);
	p_col_DP3_MS2 = (short *)(table_DP3.ColDataPtr[3]);
	p_col_DP3_MIN = (short *)(table_DP3.ColDataPtr[4]);
	p_col_DP3_CT  = (short *)(table_DP3.ColDataPtr[5]);
*/ 			
	/* Normalt mert ertek */		
/*	nNMTblIndx = p_col_parInt[18];	
	if (MOSCAD_get_table_info (nNMTblIndx,&table_NM)!=0 )
   		{
        MOSCAD_sprintf(message,"No valid information in table: %d",nNMTblIndx);
        MOSCAD_error(message );
        return;
   		}
	p_col_NM    = (short *)(table_NM.ColDataPtr[0]);
	p_col_NM_LZ = (short *)(table_NM.ColDataPtr[1]);
	p_col_NM_Tx = (short *)(table_NM.ColDataPtr[2]);*/

	/* Normalt mert ertek 2.*/		
/*	nNMTblIndx2 = p_col_parInt[44];	
	if (MOSCAD_get_table_info (nNMTblIndx2,&table_NM2)!=0 )
   		{
        MOSCAD_sprintf(message,"No valid information in table: %d",nNMTblIndx2);
        MOSCAD_error(message );
        return;
   		}
	p_col_NM2    = (short *)(table_NM2.ColDataPtr[0]);
	p_col_NM2_LZ = (short *)(table_NM2.ColDataPtr[1]);
	p_col_NM2_Tx = (short *)(table_NM2.ColDataPtr[2]);*/
	
	/* Normalt mert ertek 3.*/		
/*	nNMTblIndx3 = p_col_parInt[56];	
	if (MOSCAD_get_table_info (nNMTblIndx3,&table_NM3)!=0 )
   		{
        MOSCAD_sprintf(message,"No valid information in table: %d",nNMTblIndx3);
        MOSCAD_error(message );
        return;
   		}
	p_col_NM3    = (short *)(table_NM3.ColDataPtr[0]);
	p_col_NM3_LZ = (short *)(table_NM3.ColDataPtr[1]);
	p_col_NM3_Tx = (short *)(table_NM3.ColDataPtr[2]);*/

        MOSCAD_sprintf(message,"=====================fnReadPar step1");
        MOSCAD_error(message );


	/* Double command */
	nDCTblIndx = p_col_parInt[23];	
	if (MOSCAD_get_table_info (nDCTblIndx,&table_DC)!=0 )
   		{
        MOSCAD_sprintf(message,"6. No valid information in table: %d",nDCTblIndx);
        MOSCAD_error(message );
        return;
   		}
	p_col_DC = (short *)(table_DC.ColDataPtr[0]);

	/* Double command 2. */
	nDCTblIndx2 = p_col_parInt[45];	
	if (MOSCAD_get_table_info (nDCTblIndx2,&table_DC2)!=0 )
   		{
        MOSCAD_sprintf(message,"7. No valid information in table: %d",nDCTblIndx2);
        MOSCAD_error(message );
        return;
   		}
	p_col_DC2 = (short *)(table_DC2.ColDataPtr[0]);

	/* Double command 3. */
	nDCTblIndx3 = p_col_parInt[57];	
	if (MOSCAD_get_table_info (nDCTblIndx3,&table_DC3)!=0 )
   		{
        MOSCAD_sprintf(message,"8. No valid information in table: %d",nDCTblIndx3);
        MOSCAD_error(message );
        return;
   		}
	p_col_DC3 = (short *)(table_DC3.ColDataPtr[0]);

	/* Double command 4. */
	nDCTblIndx4 = p_col_parInt[68];	
	if (MOSCAD_get_table_info (nDCTblIndx4,&table_DC4)!=0 )
   		{
        MOSCAD_sprintf(message,"9. No valid information in table: %d",nDCTblIndx4);
        MOSCAD_error(message );
        return;
   		}
	p_col_DC4 = (short *)(table_DC4.ColDataPtr[0]);

	/* Double command 5. */
	nDCTblIndx5 = p_col_parInt[77];	
	if (MOSCAD_get_table_info (nDCTblIndx5,&table_DC5)!=0 )
   		{
        MOSCAD_sprintf(message,"No valid information in table: %d",nDCTblIndx5);
        MOSCAD_error(message );
        return;
   		}
	p_col_DC5 = (short *)(table_DC5.ColDataPtr[0]);

        MOSCAD_sprintf(message,"=====================fnReadPar step2");
        MOSCAD_error(message );

	/* Single command */
	nSCTblIndx = p_col_parInt[31];	
	if (MOSCAD_get_table_info (nSCTblIndx,&table_SC)!=0 )
   		{
        MOSCAD_sprintf(message,"10. No valid information in table: %d",nSCTblIndx);
        MOSCAD_error(message );
        return;
   		}
	p_col_SC = (short *)(table_SC.ColDataPtr[0]);

	/* Single command 2. */
	nSCTblIndx2 = p_col_parInt[46];	
	if (MOSCAD_get_table_info (nSCTblIndx2,&table_SC2)!=0 )
   		{
        MOSCAD_sprintf(message,"11. No valid information in table: %d",nSCTblIndx2);
        MOSCAD_error(message );
        return;
   		}
	p_col_SC2 = (short *)(table_SC2.ColDataPtr[0]);

	/* Single command 3. */
	nSCTblIndx3 = p_col_parInt[58];	
	if (MOSCAD_get_table_info (nSCTblIndx3,&table_SC3)!=0 )
   		{
        MOSCAD_sprintf(message,"12. No valid information in table: %d",nSCTblIndx3);
        MOSCAD_error(message );
        return;
   		}
	p_col_SC3 = (short *)(table_SC3.ColDataPtr[0]);

	/* Single command 4. */
	nSCTblIndx4 = p_col_parInt[69];	
	if (MOSCAD_get_table_info (nSCTblIndx4,&table_SC4)!=0 )
   		{
        MOSCAD_sprintf(message,"13. No valid information in table: %d",nSCTblIndx4);
        MOSCAD_error(message );
        return;
   		}
	p_col_SC4 = (short *)(table_SC4.ColDataPtr[0]);

	/* Single command 5. */
	nSCTblIndx5 = p_col_parInt[78];	
	if (MOSCAD_get_table_info (nSCTblIndx5,&table_SC5)!=0 )
   		{
        MOSCAD_sprintf(message,"No valid information in table: %d",nSCTblIndx5);
        MOSCAD_error(message );
        return;
   		}
	p_col_SC5 = (short *)(table_SC5.ColDataPtr[0]);



        MOSCAD_sprintf(message,"=====================fnReadPar step3,p_col_SC: %p, p_col_SC2: %p, p_col_SC3: %p",p_col_SC,p_col_SC2,p_col_SC3);
        MOSCAD_error(message );


	} /* end if nStart==1 parameter tabla feldolgozas ---------------------------------------------------------------------------------------------------*/
	 
 
   
   	if (MOSCAD_get_table_info (nTableNum2,&table_parInt)!=0 )
   		{
        MOSCAD_sprintf(message,"14. No valid information in table: %d",2);
        MOSCAD_error(message );
        return;
   		}
	p_col_parInt = (short *)(table_parInt.ColDataPtr[0]);	
	 
	 
nStart = 1;
                                                                                                
	/* DC parancs log */
   	if (MOSCAD_get_table_info (p_col_parInt[64],&table_DC_Event)!=0 )
   		{
        MOSCAD_sprintf(message,"15. No valid information in table: %d",p_col_parInt[64]);
        MOSCAD_error(message );
        return;
   		}
	p_col_DC_Index = (short *)(table_DC_Event.ColDataPtr[0]);	
	p_col_DC_Value = (short *)(table_DC_Event.ColDataPtr[1]);	
	p_col_DC_Year  = (short *)(table_DC_Event.ColDataPtr[2]);	
	p_col_DC_Month = (short *)(table_DC_Event.ColDataPtr[3]);	
	p_col_DC_Day   = (short *)(table_DC_Event.ColDataPtr[4]);	
	p_col_DC_Hour  = (short *)(table_DC_Event.ColDataPtr[5]);	
	p_col_DC_Min   = (short *)(table_DC_Event.ColDataPtr[6]);		
	p_col_DC_Sec   = (short *)(table_DC_Event.ColDataPtr[7]);	


/* Bool */
if (p_col_parBool[0] == 0)
{
	nRxMon = 0; 	
}
else
{
	nRxMon = 1; 	
}
if (p_col_parBool[1] == 0)
{
	nDir = 0;
	cfTransmit.byDIR=0x00; 	
}
else
{
	nDir = 1; 	
	cfTransmit.byDIR=0x80;	
}

/* Integer */
nUPort = p_col_parInt[1];
nLinkTimeOut = p_col_parInt[3];
nMaxRptNum   = p_col_parInt[4];
nLenCAOA     = p_col_parInt[5];
nLenIOA      = p_col_parInt[6];


nCAOA 		 = p_col_parInt[7];
byCAOA_LO	 = (BYTE) (nCAOA & 0x00FF);
byCAOA_HI	 = (BYTE) (nCAOA / 256);
nLinkTestCycle = p_col_parInt[28];
nDelta			= p_col_parInt[29];
nDCTimeOut		= p_col_parInt[30];


lSPStart = 65536 * p_col_parInt[12] + 256 * p_col_parInt[11] + p_col_parInt[10];
lDPStart = 65536 * p_col_parInt[17] + 256 * p_col_parInt[16] + p_col_parInt[15];
lNMStart = 65536 * p_col_parInt[22] + 256 * p_col_parInt[21] + p_col_parInt[20];
lDCStart = 65536 * p_col_parInt[27] + 256 * p_col_parInt[26] + p_col_parInt[25];
lSCStart = 65536 * p_col_parInt[35] + 256 * p_col_parInt[34] + p_col_parInt[33];


nSPNum = p_col_parInt[9];
nDPNum = p_col_parInt[14];
nNMNum = p_col_parInt[19];  
nDCNum = p_col_parInt[24];	
nSCNum = p_col_parInt[32];	


nNoRestarted = p_col_parInt[88];
nDelayedStart =p_col_parInt[89];
nDisableWrite =p_col_parInt[91];
nResetSRAM 	  =p_col_parInt[92];	
nSaveTables =p_col_parInt[90];



 


		fnSP_TABLE(30);
		fnNM_TABLE(6);
		fnDP_TABLE(6);
    
    
if (nLenIOA > 3 ) 
{
	nLenIOA = 3;
}

p_col_Stat[8] = nLinkInitStep;



if (nResetSRAM ==1)
{ 
	     p_col_parInt[92] = 0;

			lSRAMLength=MOSCAD_bspSRamLength(); 
			
			/* Ha van benne SRAM*/
			if ( lSRAMLength > 0)
				{
 				MOSCAD_sprintf(message, "RESET SRAM !!!");
		    MOSCAD_message(message);     

					fnResetSRAM();    

				} 
}  /* end if reset sram*/




/*if (2<1)
{ */
   /* Write data to SRAM ************************************************************************************/		
		if ((nNoRestarted == 1) && (nSaveTables ==1) && (p_col_parInt[8] ==5) && (p_col_parInt[13] ==6)  )
			{	
			
			lSRAMLength=MOSCAD_bspSRamLength(); 
			
			/* Ha van benne SRAM*/
			if ( (lSRAMLength > 0) && (nDataSave > 10) )
				{
					fnSaveData(); /* MOSCAD tábla adatainak kimentése SRAM-ba*/
				p_col_parInt[90]=0;
				
        /* MOSCAD_sprintf(message, "Save tables to SRAM, lSRAMLength: %ld", lSRAMLength);
		    MOSCAD_message(message);  */
  				
            if ( nFirstSave == 1 )
            {
	 				  MOSCAD_sprintf(message, "Write to SRAM, lSRAMLength: %ld, nSPNUM: %d ",lSRAMLength, p_col_parInt[9]);
		        MOSCAD_message(message);
            nFirstSave = 2;
            }
             
        nDataSave = 0;
			    
				
				} 
        nDataSave = nDataSave + 1;
        
			}  /* end if   ((nNoRestarted == 1) && (nSaveTables ==1) && (p_col_parInt[8] ==5) && (p_col_parInt[13] ==6)  )    */
	
  
nNoRestarted = p_col_parInt[88];
nDelayedStart =p_col_parInt[89];
nDisableWrite =p_col_parInt[91];
nResetSRAM 	  =p_col_parInt[92];	
nSaveTables =p_col_parInt[90];

  
/* SRAM adatainak kiírása a MOSCAD CPU tábláiba *************************************************************************************** */  
if ((nNoRestarted == 0) && (nDelayedStart ==1) && (nDisableWrite ==0)  && (TotalData->nPAR[8] ==5) && (TotalData->nPAR[13] ==6))
{
  nFirstSave = 1;
	p_col_parInt[88] = 1;
	
	
			lSRAMLength=MOSCAD_bspSRamLength(); 
      
 				MOSCAD_sprintf(message, "1. step: Copy data from SRAM to MOSCAD tables,lSRAMLength: %ld ",lSRAMLength);
		    MOSCAD_message(message);     

      
			
			/* Ha van benne SRAM*/
			if ( lSRAMLength > 0)
				{
 				MOSCAD_sprintf(message, "2. step: Copy data from SRAM to MOSCAD tables,lSRAMLength: %ld, nSPNUM: %d ",lSRAMLength, TotalData->nPAR[9]);
		    MOSCAD_message(message);     

		  MOSCAD_wait(800); 	

				fnGetData(); /* SRAM adatainak tábla adatainak kimentése MOSCAD  táblákba*/
        
        MOSCAD_wait(400); 
        
				p_col_parInt[90]=0;

				} 
} /* end if ((nNoRestarted == 0) && (nDelayedStart ==1) && (nDisableWrite ==0)  && (TotalData->nPAR[8] ==5) && (TotalData->nPAR[13] ==6)) */







/*}*/  /* end if 2<1 */

} /* end fnReadPar()*/

/****************************************************************************/
/* Build var. length header 			   									*/
/****************************************************************************/
/*int fnBuildVarHeader(BYTE *buf, BYTE byLen, int nNum)
{
	return (nNum+4);

} *//* end fnBuildVarHeader()*/
/****************************************************************************/
/* Build control field 			   											*/
/****************************************************************************/
int fnBuildCF(BYTE *buf, BOOL bPrm, BOOL bFcv, int nCode, int nNum)
{
	if ( nDir == 1 )
	{
		cfTransmit.byDIR = 0x80;
	}
	else
	{
		cfTransmit.byDIR = 0x00;
	}
	
	if ( bPrm == 1 )
	{
		cfTransmit.byPRM = 0x40;
	}
	else
	{
		cfTransmit.byPRM = 0x00;
	}
	
	fnTxFCBInv(bFcv);
	
	if ( bFcv == 1 )
	{
		cfTransmit.byFCV_DFC = 0x10;
	}
	else
	{
		cfTransmit.byFCV_DFC = 0x00;
	}	
	cfTransmit.byFnCode = nCode & 0x0F;
	
	buf[nNum] = cfTransmit.byDIR | cfTransmit.byPRM | cfTransmit.byFCB_RES | cfTransmit.byFCV_DFC | cfTransmit.byFnCode;
	return (nNum+1);

} /* end fnBuildCF()*/
/****************************************************************************/
/* Build data unit identifier             */
/* Globális változó: duiTransmit          */ 			   													
/****************************************************************************/
int fnBuildDUI(BYTE *buf, BYTE byTI, BOOL bSeq, BYTE byDataNum , BYTE byCOT, BYTE byOrigAddr, int nNum, IEC_DUI_104	*duiTransmit)
{
	
			
	duiTransmit->byTI       = byTI;
	duiTransmit->byDataNum  = byDataNum;
	
	if (bSeq == 1)
	{
		duiTransmit->bySequence = 0x80;
	}
	else
	{
		duiTransmit->bySequence = 0x00;
	}
	
	duiTransmit->byCOT       = byCOT;
	
	duiTransmit->byOrigAddr  = byOrigAddr;
	
	duiTransmit->byCAOA[0] = byCAOA_LO;
	duiTransmit->byCAOA[1] = byCAOA_HI;
	
	buf[nNum] = duiTransmit->byTI;
	buf[nNum+1] = duiTransmit->bySequence | duiTransmit->byDataNum;
	buf[nNum+2] = duiTransmit->byCOT;
	buf[nNum+3] = duiTransmit->byOrigAddr;
	
	memcpy(&buf[nNum+4],&duiTransmit->byCAOA[0],nLenCAOA);
		
	return (nNum + 4 + nLenCAOA);

} /* end fnBuildDUI()*/
/****************************************************************************/
/* Build information object 			   									*/
/* a Data Unit Identifier alapján dolgozik !!!        */
/* Globális változók: strSPEventWT[INDX], nSPWrPtr[INDX], nSPReadPtr[INDX] */
/* 	*buf: ahova masolni kell												*/
/*  *byIOA: IEC cimet tartalmazo tomb										*/
/*	*byData: nincs hasznalva												*/
/*	nNum: adatok szama														*/
/****************************************************************************/
int fnBuildInfObj(BYTE *buf, BYTE *byIOA, BYTE *byData, int nNum,IEC_DUI_104	duiTransmit, int INDX)
{
	int  			nI;

	int				nObjLen;



	BYTE			byT[3];
	unsigned long	lSPSt;
	unsigned long	lNMSt;
	unsigned long	lDPSt;
	int				nOffset;


	int				nMS1,nMS2,nMin,bTTime,nXOR;
	unsigned int	nNMVal;
	unsigned int	nLiveZ;
	unsigned int	nIECStatus;
	unsigned int	nInvalid;
	BYTE			byDPVal;
	
/*IEC_M_DP_TB_1		**strDPEventWT[INDX];*/  	
	
	

	/* Clock sync.activation confirm */
	if (duiTransmit.byTI == TI_C_CS_NA_1)
	{



		fnBuildCP56Time2a();
		
		
		byT[0] = byData[0];
		
		byT[0] = 0x00;
		byT[1] = 0x00;
		byT[2] = 0x00;
		
		memcpy(&buf[nNum],&byT[0],nLenIOA);
			
		/*buf[nNum]   = 0x00;
		buf[nNum+1] = 0x00;
		buf[nNum+2] = 0x00;*/
		
		/*memcpy(&cp56Time.byMs[0],&buf[nNum+nLenIOA],2);*/
		
		buf[nNum + nLenIOA + 0] = cp56Time.byMs[0];		
		buf[nNum + nLenIOA + 1] = cp56Time.byMs[1];
		buf[nNum + nLenIOA + 2] = cp56Time.byMin;
		buf[nNum + nLenIOA + 3] = cp56Time.byHour | cp56Time.bySummerTime;
		buf[nNum + nLenIOA + 4] = cp56Time.byDay | cp56Time.byWeekDay * 32;
		buf[nNum + nLenIOA + 5] = cp56Time.byMon;
		buf[nNum + nLenIOA + 6] = cp56Time.byYear;

        /*MOSCAD_sprintf(message,"fnBuildInfObj: nNum: %d",nNum);
        MOSCAD_message(message ); */

		
	return (nNum + nLenIOA + 7);			
	}
	/* Activation confirmation of general interrogation */	
	else if (duiTransmit.byTI == TI_C_IC_NA_1 && duiTransmit.byCOT == COT_ACTCON)
	{
		byT[0] = 0x00;
		byT[1] = 0x00;
		byT[2] = 0x00;
						
		/*buf[nNum]   = 0x00;
		buf[nNum+1] = 0x00;
		buf[nNum+2] = 0x00;*/
		
		memcpy(&buf[nNum],&byT[0],nLenIOA);
		buf[nNum+nLenIOA] = 20;
		
	return (nNum + nLenIOA + 1);	
		
	}	/* end activation confirmation of general interrogation */
		/* Activation termination of general interrogation */	
	else if (duiTransmit.byTI == TI_C_IC_NA_1 && duiTransmit.byCOT == COT_ACTTERM)
	{
		byT[0] = 0x00;
		byT[1] = 0x00;
		byT[2] = 0x00;
						
		/*buf[nNum]   = 0x00;
		buf[nNum+1] = 0x00;
		buf[nNum+2] = 0x00;*/
		
		memcpy(&buf[nNum],&byT[0],nLenIOA);
		buf[nNum+nLenIOA] = 20;
		
	return (nNum + nLenIOA + 1);	
		
	}	/* end activation confirmation of general interrogation */
	/* Activation confirmation of DC */	
	else if (duiTransmit.byTI == TI_C_DC_NA_1 && duiTransmit.byCOT == COT_ACTCON)
	{
		memcpy(&buf[nNum], &ioDC.byIOA[0],nLenIOA);		
		buf[nNum+nLenIOA] = ioDC.byDC | ioDC.bySE;
		
	return (nNum + nLenIOA +1);	
		
	}	/* end activation confirmation of DC */	
	
	/* Activation confirmation of SC */		
	else if (duiTransmit.byTI == TI_C_SC_NA_1 && duiTransmit.byCOT == COT_ACTCON)
	{
		memcpy(&buf[nNum], &ioSC.byIOA[0],nLenIOA);		
		buf[nNum+nLenIOA] = ioSC.bySC | ioSC.bySE;
		
	return (nNum + nLenIOA +1);	
		
	}	/* end activation confirmation of SC */		
	
	/* Activation termination of DC */	
	else if (duiTransmit.byTI == TI_C_DC_NA_1 && duiTransmit.byCOT == COT_ACTTERM)
	{
		memcpy(&buf[nNum], &ioDC.byIOA[0],nLenIOA);		
		buf[nNum+nLenIOA] = ioDC.byDC | ioDC.bySE;
		
	return (nNum + nLenIOA +1);	
		
	}	/* end activation confirmation of general interrogation */	

	/* Altalanos lekerdezesre feladott NM ertekek */	
	else if (duiTransmit.byTI == TI_M_ME_NA_1 && duiTransmit.byCOT == COT_INROGEN && duiTransmit.bySequence == 0x80)
	{	
		byT[0] = byIOA[0]; /*(BYTE)p_col_parInt[20];*/
		byT[1] = byIOA[1]; /*(BYTE)p_col_parInt[21];*/
		byT[2] = byIOA[2]; /*(BYTE)p_col_parInt[22];*/

		lNMSt = byIOA[0] + 256 * byIOA[1] + 65536 * byIOA[2];
		nOffset = lNMSt - lNMStart;
		
		
		memcpy(&buf[nNum], &byT[0],nLenIOA);
		
		
			

		
		for (nI=0;nI<duiTransmit.byDataNum;nI++)
		{
			
			fnReadNMData(nI + nOffset, &nNMVal, &nLiveZ, &nIECStatus);
			fnNorm(nNMVal, nLiveZ, &buf[nNum+nLenIOA+nI*3]);
			
			nInvalid = 0;
			if (nIECStatus  == 1)
					{
						nInvalid = 64;
					}
			
			
			buf[nNum+nLenIOA+nI*3+2] = nInvalid;			

/*			
			if (nSendNM[INDX] < 240)	
			{		
				nLiveZero = p_col_NM_LZ[nI +nOffset];
				p_col_NM_Tx[nI] = fnNorm(p_col_NM[nI + nOffset], nLiveZero, &buf[nNum+nLenIOA+nI*3]);
				buf[nNum+nLenIOA+nI*3+2] = 0;			
			}
			else if (nSendNM[INDX] >= 240 && nSendNM[INDX] < 480)	
			{		
				nLiveZero = p_col_NM2_LZ[nI +nOffset-240];
				p_col_NM2_Tx[nI-240+nOffset] = fnNorm(p_col_NM2[nI + nOffset-240], nLiveZero, &buf[nNum+nLenIOA+nI*3]);
				buf[nNum+nLenIOA+nI*3+2] = 0;			
			}
			else if (nSendNM[INDX] >= 480 && nSendNM[INDX] < 720)	
			{		
				nLiveZero = p_col_NM3_LZ[nI +nOffset-480];
				p_col_NM3_Tx[nI-480+nOffset] = fnNorm(p_col_NM3[nI + nOffset-480], nLiveZero, &buf[nNum+nLenIOA+nI*3]);
				buf[nNum+nLenIOA+nI*3+2] = 0;			
			}
*/			
			

		}
		
	return (nNum + 3 + duiTransmit.byDataNum*3);	
		
	}	
	/*  Altalanos lekerdezesre feladott DP ertekek*/
	else if (duiTransmit.byTI == TI_M_DP_NA_1 && duiTransmit.byCOT == COT_INROGEN && duiTransmit.bySequence == 0x80)
	{
		lDPSt = byIOA[0] + 256 * byIOA[1] + 65536 * byIOA[2];
		nOffset = lDPSt - lDPStart;
				
		buf[nNum]   = byIOA[0]; 
		buf[nNum+1] = byIOA[1]; 
		buf[nNum+2] = byIOA[2]; 
		
		
		
		
		for (nI=0;nI<duiTransmit.byDataNum;nI++)
		{
			
			
				fnReadDPDataTime(nI+nOffset, &byDPVal, &nMS1, &nMS2, &nMin, &bTTime);	
				buf[nNum+3+nI] = byDPVal;
			
/*			if (nSendDP[INDX] < 250 )
			{			
				buf[nNum+3+nI] = (BYTE)p_col_DPL[nI+nOffset] | (BYTE)p_col_DPH[nI+nOffset] * 2 ;			
			}
			else if (nSendDP[INDX] >= 250 && nSendDP[INDX] < 500 )
			{
				buf[nNum+3+nI] = (BYTE)p_col_DP2L[nI+nOffset-250] | (BYTE)p_col_DP2H[nI+nOffset-250] * 2 ;
			}
			else if (nSendDP[INDX] >= 500 && nSendDP[INDX] < 750 )
			{
				buf[nNum+3+nI] = (BYTE)p_col_DP3L[nI+nOffset-500] | (BYTE)p_col_DP3H[nI+nOffset-500] * 2 ;
			}*/
			
		}
		
	return (nNum + 3 + duiTransmit.byDataNum);	
		
	}		
	/* Altalanos lekerdezesre feladott SP ertekek */	
	else if (duiTransmit.byTI == TI_M_SP_NA_1 && duiTransmit.byCOT == COT_INROGEN && duiTransmit.bySequence == 0x80)
	{
		
		lSPSt = byIOA[0] + 256 * byIOA[1] + 65536 * byIOA[2];
		nOffset = lSPSt - lSPStart;
		
		buf[nNum]   = byIOA[0]; /*(BYTE)p_col_parInt[10];*/
		buf[nNum+1] = byIOA[1]; /*(BYTE)p_col_parInt[11];*/
		buf[nNum+2] = byIOA[2]; /*(BYTE)p_col_parInt[12];*/
		
		for (nI=0;nI<duiTransmit.byDataNum;nI++)
		{
			
			fnReadSPDataTime(nI + nOffset, &buf[nNum+3+nI],  &nMS1, &nMS2, &nMin, &bTTime, &nXOR);
		}
			
		
	return (nNum + 3 + duiTransmit.byDataNum);	
		
	}
		
	/* SP event feladas */	
	else if (duiTransmit.byTI == TI_M_SP_TB_1  && duiTransmit.byCOT == COT_SPONT && duiTransmit.bySequence == 0x00)
	{			
		nI  = 0;
		nObjLen = nLenIOA + 1 + 7; /* nem 4 */
		nSPReadPtr[INDX] = 0;  /* 2012.04.27 */
		
    	while (nSPReadPtr[INDX] != nSPWrPtr[INDX])
      	{
      		memcpy(&buf[nNum +  nI * nObjLen],&strSPEventWT[INDX][nSPReadPtr[INDX]].byIOA[0],nLenIOA);
    		      		
      		buf[nNum + nI * nObjLen + nLenIOA] = strSPEventWT[INDX][nSPReadPtr[INDX]].bySP;
      		
      		/* IEC60870-5-104 szerintt 7 byte-os*/
      		memcpy(&buf[nNum +  nI * nObjLen + nLenIOA + 1],&strSPEventWT[INDX][nSPReadPtr[INDX]].sTime.byMs[0],7);
      					
			   /* Olvaso pointer novelese */	 			
			   if (nSPReadPtr[INDX] < MAX_SP_EVNUM-1)
			     {
				    nSPReadPtr[INDX]++;			
			     }
			   else
			     {
				    nSPReadPtr[INDX] = 0; 
				    nSPWrPtr[INDX] = 0;     		
			     }
			   nI++;
      	} /* end while */
      	nSPReadPtr[INDX] = 0; 
				nSPWrPtr[INDX] = 0;     		
      	
      	return (nNum + nI * nObjLen);
	} /* end SP event */
	
	/* DP event feladas */	
	else if (duiTransmit.byTI == TI_M_DP_TB_1  && duiTransmit.byCOT == COT_SPONT && duiTransmit.bySequence == 0x00)
	{			
		nI  = 0;
		nObjLen = nLenIOA + 1 + 7;  /* nem 4 */
		nDPReadPtr[INDX] = 0;  
		
		  
		
		
    	while (nDPReadPtr[INDX] != nDPWrPtr[INDX])
      	{
      		memcpy(&buf[nNum +  nI * nObjLen],&strDPEventWT[INDX][nDPReadPtr[INDX]].byIOA[0],nLenIOA);
    		      		
      		buf[nNum + nI * nObjLen + nLenIOA] = strDPEventWT[INDX][nDPReadPtr[INDX]].byDP;
      		
      		/* IEC60870-5-104 szerintt 7 byte-os*/      		
      		memcpy(&buf[nNum +  nI * nObjLen + nLenIOA + 1],&strDPEventWT[INDX][nDPReadPtr[INDX]].sTime.byMs[0],7);
      					
			/* Olvaso pointer novelese */	 			
			if (nDPReadPtr[INDX] < MAX_DP_EVNUM-1)
			{
				nDPReadPtr[INDX]++;			
			}
			else
			{
				nDPReadPtr[INDX] = 0;    
				nDPWrPtr[INDX] = 0;  		
			}
			nI++;
			/*MOSCAD_sprintf(message,"nDPReadPtr[INDX], nDPWrPtr[INDX]: %d  %d",nDPReadPtr[INDX], nDPWrPtr[INDX]);
        	MOSCAD_error(message );*/
      	} /* end while */
		nDPReadPtr[INDX] = 0;    
		nDPWrPtr[INDX] = 0;  		
      	
      	return (nNum + nI * nObjLen);
	}	/* end DP event */
	
	/* NM event feladas */	
	else if (duiTransmit.byTI == TI_M_ME_NA_1  && duiTransmit.byCOT == COT_SPONT && duiTransmit.bySequence == 0x00)
	{			
		nI  = 0;
		nObjLen = nLenIOA +3;
		nNMReadPtr[INDX] = 0;
		
    	while (nNMReadPtr[INDX] != nNMWrPtr[INDX])
      	{
      		memcpy(&buf[nNum +  nI * nObjLen],&strNMEvent104[INDX][nNMReadPtr[INDX]].byIOA[0],nLenIOA);
      		memcpy(&buf[nNum +  nI * nObjLen + nLenIOA],&strNMEvent104[INDX][nNMReadPtr[INDX]].byNM[0],2);
      		
   			/*fnLoHi(&byLow, &byHigh, strNMEvent[nNMReadPtr[INDX]].nNM);
      		buf[nNum + nI * nObjLen + nLenIOA + 1] = byLow;
      		buf[nNum + nI * nObjLen + nLenIOA + 2] = byHigh; */     		
      		buf[nNum + nI * nObjLen + nLenIOA + 2] = strNMEvent104[INDX][nNMReadPtr[INDX]].byQ;
      					
			/* Olvaso pointer novelese */	 			
			if (nNMReadPtr[INDX] < MAX_NM_EVNUM-1)
			{
				nNMReadPtr[INDX]++;			
			}
			else
			{
				nNMReadPtr[INDX] = 0;
				nNMWrPtr[INDX] = 0;      		
			}
			nI++;
      	} /* end while */
   		
    nNMReadPtr[INDX] = 0;    
		nNMWrPtr[INDX] = 0;  		
   	      			
      	      			
	return (nNum + nI * nObjLen);	
	} /* end NM event */	
		
	return (nNum + 3 + nLenCAOA);
} /* end fnBuildDUI()*/
/****************************************************************************/
/* Build variable end 			   									*/
/****************************************************************************/
int fnBuildVarEnd(BYTE *buf, int nNum)
{
BYTE		byCsum;	

	byCsum = fnIEC_Csum(&buf[2], nNum-2);
	buf[nNum] = byCsum;
	buf[nNum+1] = IEC_VAR_END;
	return (nNum + 2);	

} /* end fnBuildVarEnd()*/
/****************************************************************************/
/* Build variable start 			   										*/
/****************************************************************************/
void fnBuildVarStart(BYTE *buf, int nNum)
{

	buf[0] = IEC_VAR_START;
	buf[1] = (BYTE)nNum-2;
/*	buf[2] = (BYTE)nNum-6;
	buf[3] = IEC_VAR_START;*/
	
} /* end fnBuildVarStart()*/

/****************************************************************************/
/* FCB invertalas 			   													*/
/****************************************************************************/
void fnTxFCBInv(BOOL bFCV)
{
if (bFCV == 1)	
{
	if (nTxFCB == 0)	
	{
		nTxFCB = 1;
		cfTransmit.byFCB_RES = 0x20;	
	}
	else
	{
		nTxFCB = 0;
		cfTransmit.byFCB_RES = 0x00;		
	}
}

	
} /* end fnTxFCBInv() */
/****************************************************************************/
/* Build 7 octet binary time from MOSCAD time	   									*/
/****************************************************************************/
void fnBuildCP56Time2a(void)
{
unsigned int 	nMsec;
MOSCAD_DATE_TM	mdt;

		MOSCAD_get_datetime(&mdt);
		nMsec = 1000 * mdt.seconds;
		cp56Time.byMs[1] = nMsec / 256;
		cp56Time.byMs[0] = nMsec - (nMsec / 256) * 256;
		cp56Time.byMin = mdt.minutes;
		cp56Time.byHour = mdt.hours;
		cp56Time.byDay = mdt.date;
		cp56Time.byWeekDay = mdt.wday;
		cp56Time.byMon = mdt.month;
		cp56Time.byYear = mdt.year;
			
} /* end fnBuildCP56Time2a()*/

/****************************************************************************/
/* Set DC log time from MOSCAD	   									*/
/****************************************************************************/
void fnSetDCLogTime(int nDCIndex, BYTE byValue)
{
MOSCAD_DATE_TM	mdt;

	MOSCAD_get_datetime(&mdt);
	
	p_col_DC_Index[nDCEventPtr] = nDCIndex;	
	p_col_DC_Value[nDCEventPtr] = (int)byValue;	
	p_col_DC_Year[nDCEventPtr]  = mdt.year;;	
	p_col_DC_Month[nDCEventPtr] = mdt.month;	
	p_col_DC_Day[nDCEventPtr]   = mdt.date;	
	p_col_DC_Hour[nDCEventPtr]  = mdt.hours;
	p_col_DC_Min[nDCEventPtr]   = mdt.minutes;
	p_col_DC_Sec[nDCEventPtr]   = mdt.seconds;	

if (nDCEventPtr <= 249)
{
	nDCEventPtr++;
}
else
{
	nDCEventPtr = 0;
}


} /*end fnSetDCLogTime() */
/****************************************************************************/
/* Build 7 octet binary time from IEC	   									*/
/****************************************************************************/
void fnBuildCP56Time2aIEC(BYTE *buf)
{

		cp56Time.byMs[0]   = buf[0];
		cp56Time.byMs[1]   = buf[1];		
		cp56Time.byMin     = buf[2] & 0x3F;
		cp56Time.byHour    = buf[3] & 0x1F;
		cp56Time.byDay     = buf[4] & 0x1F;
		cp56Time.byWeekDay = (buf[4] & 0xE0) / 32;
		cp56Time.byMon     = buf[5] & 0x0F;
		cp56Time.byYear    = buf[6] & 0x7F;
		cp56Time.bySummerTime = buf[3] & 0x80;
		
			
} /* end fnBuildCP56Time2aIEC()*/
/****************************************************************************/
/* Set MOSCAD time	   									*/
/****************************************************************************/
void fnSetMOSCADTime(void)
{
unsigned int nMsec;

if (cp56Time.byYear>0 && cp56Time.byYear<99)
{
	tm.year = cp56Time.byYear ;
}
if (cp56Time.byMon>0 && cp56Time.byMon<13)
{
	tm.month = cp56Time.byMon;
}
if (cp56Time.byWeekDay>0 && cp56Time.byWeekDay<8)
{
	tm.wday = cp56Time.byWeekDay;
}
if (cp56Time.byDay>0 && cp56Time.byWeekDay<32)
{
	tm.date = cp56Time.byDay;
}
if (cp56Time.byHour>=0 && cp56Time.byHour<24)
{
	tm.hours = cp56Time.byHour;
}
if (cp56Time.byMin>=0 && cp56Time.byMin<60)
{
	tm.minutes = cp56Time.byMin;
}
nMsec = cp56Time.byMs[0] + cp56Time.byMs[1] * 256;
if (nMsec<60000)
{
	
	tm.seconds = nMsec / 1000 + 1;
}



MOSCAD_set_datetime(&tm);

			
} /* end fnSetMoscadTime()*/
/****************************************************************************/
/* MOTOROLA memcpy 2 byte-ra			   									*/
/****************************************************************************/
void fnMemcpy(BYTE *dest, BYTE *src)
{
BYTE by[2];

by[0] = src[1];
by[1] = src[0];

memcpy(dest,by,2);
			
} /* end fnMemcpy()*/


/****************************************************************************/
/* Build inf. obj. address from unsigned long			   									*/
/****************************************************************************/
void fnBuildIOA(BYTE *byAddr, unsigned long lIOA)
{

if (nLenIOA==1)
{
	byAddr[0] = lIOA;
}
else if (nLenIOA==2)
{
	byAddr[1] = lIOA / 256;
	byAddr[0] = lIOA - byAddr[1]*256;	
}
else if (nLenIOA==3)
{
	byAddr[2] = lIOA / 65536;	
	byAddr[1] = (lIOA - byAddr[2] * 65536) / 256;
	byAddr[0] = lIOA - byAddr[2] * 65536 - byAddr[1] * 256;	
}		
} /* end fnMemcpy()*/
/****************************************************************************/
/* Egy integer LOW byte-jat allitja elo										*/
/****************************************************************************/
void fnLoHi(BYTE *byLo, BYTE *byHi, unsigned int nInt)
{

byHi[0] = nInt / 256;
byLo[0] = nInt - byHi[0] * 256;
			
} /* end fnLoHi()*/
/****************************************************************************/
/* Normalt ertek eloallitasa												*/
/* A MOSCAD-bol 0..3200 vagy 0..4000 kozotti erteket var					*/
/****************************************************************************/
int fnNorm(int nBe, int nLiveZero, BYTE *byNorm)
{
int				nTemp,nTemp2;
int				nOffset;
int				nProp;


if (nLiveZero == 1)
{
	nOffset = 0;
	nProp = 4000;
}
else
{
	nOffset = 0;
	nProp = 3200;
}

nTemp = nBe - nOffset;
if (nTemp<0)
{
	nTemp = 0;
}


if (nTemp >= 4000)
{
	nTemp = 4000 - 1;
}

/*
lTemp = nTemp;
lTemp = lTemp * 32768;
lTemp = lTemp / nProp;

nTemp2  = (int)lTemp;
byNorm[1] = nTemp2 / 256;
byNorm[0] = nTemp2 - 256 * byNorm[1];
*/


nTemp2 = (nTemp * 4096) / nProp;
if (nTemp2 > 4095)
{
	nTemp2 = 4095;
}

byNorm[1] = nTemp2 / 256;
byNorm[0] = nTemp2 - 256 * byNorm[1];

return nTemp2;
			
} /* end fnLoHi()*/
/********************************************************************************/
/* Elloallitja az SP tablak pointereit										    */
/********************************************************************************/
void fnSP_TABLE(int nNum)
{

int 	nIndxTbl[40];	
char	message[300];
int		nI;

	/* Egesz parameterek */
   	if (MOSCAD_get_table_info (2,&table_parInt)!=0 )
   		{
        MOSCAD_sprintf(message,"16. No valid information in table: %d",2);
        MOSCAD_error(message );
        return;
   		}
	p_col_parInt = (short *)(table_parInt.ColDataPtr[0]);	


nIndxTbl[0]  = p_col_parInt[8];
nIndxTbl[1]  = p_col_parInt[36];
nIndxTbl[2]  = p_col_parInt[37];
nIndxTbl[3]  = p_col_parInt[38];
nIndxTbl[4]  = p_col_parInt[39];
nIndxTbl[5]  = p_col_parInt[40];
nIndxTbl[6]  = p_col_parInt[41];
nIndxTbl[7]  = p_col_parInt[42];
nIndxTbl[8]  = p_col_parInt[47];
nIndxTbl[9]  = p_col_parInt[48];
nIndxTbl[10] = p_col_parInt[49];
nIndxTbl[11] = p_col_parInt[50];
nIndxTbl[12] = p_col_parInt[51];
nIndxTbl[13] = p_col_parInt[52];
nIndxTbl[14] = p_col_parInt[53];
nIndxTbl[15] = p_col_parInt[54];
nIndxTbl[16] = p_col_parInt[60];
nIndxTbl[17] = p_col_parInt[61];
nIndxTbl[18] = p_col_parInt[62];
nIndxTbl[19] = p_col_parInt[63];
nIndxTbl[20] = p_col_parInt[70];
nIndxTbl[21] = p_col_parInt[71];
nIndxTbl[22] = p_col_parInt[72];
nIndxTbl[23] = p_col_parInt[73];
nIndxTbl[24] = p_col_parInt[74];
nIndxTbl[25] = p_col_parInt[79];
nIndxTbl[26] = p_col_parInt[80];
nIndxTbl[27] = p_col_parInt[81];
nIndxTbl[28] = p_col_parInt[82];
nIndxTbl[29] = p_col_parInt[83];




for (nI=0; nI<nNum;nI++)
	{

					if (MOSCAD_get_table_info (nIndxTbl[nI],&sSPT[nI].table_SP)!=0 )
   					{
					       MOSCAD_sprintf(message,"No valid information in table: %d",nIndxTbl[nI]);
					       MOSCAD_error(message );
					       return;
   					}
   					
				   	sSPT[nI].p_col_SP     		= (short *)(sSPT[nI].table_SP.ColDataPtr[0]);
				   	sSPT[nI].p_col_SP_MS1 		= (short *)(sSPT[nI].table_SP.ColDataPtr[1]);
				   	sSPT[nI].p_col_SP_MS2 		= (short *)(sSPT[nI].table_SP.ColDataPtr[2]);
				   	sSPT[nI].p_col_SP_MIN		= (short *)(sSPT[nI].table_SP.ColDataPtr[3]);				   
				   	sSPT[nI].p_col_SP_CT  		= (short *)(sSPT[nI].table_SP.ColDataPtr[4]); 	
				   	sSPT[nI].p_col_SP_XOR 		= (short *)(sSPT[nI].table_SP.ColDataPtr[5]); 
				   	sSPT[nI].p_col_SP_STATUS 	= (short *)(sSPT[nI].table_SP.ColDataPtr[6]);	
	
	} /* end for */


} /* end fnSP_TABLE() */

/********************************************************************************/
/* Elloallitja az NM tablak pointereit										    */
/********************************************************************************/
void fnNM_TABLE(int nNum)
{

int 	nIndxTbl[40];	
char	message[300];
int		nI;

	/* Egesz parameterek */
   	if (MOSCAD_get_table_info (2,&table_parInt)!=0 )
   		{
        MOSCAD_sprintf(message,"17. No valid information in table: %d",2);
        MOSCAD_error(message );
        return;
   		}
	p_col_parInt = (short *)(table_parInt.ColDataPtr[0]);	


nIndxTbl[0]  = p_col_parInt[18];
nIndxTbl[1]  = p_col_parInt[44];
nIndxTbl[2]  = p_col_parInt[56];
nIndxTbl[3]  = p_col_parInt[65];
nIndxTbl[4]  = p_col_parInt[75];
nIndxTbl[5]  = p_col_parInt[84];



for (nI=0; nI<nNum;nI++)
	{

					if (MOSCAD_get_table_info (nIndxTbl[nI],&sNMT[nI].table_NM)!=0 )
   					{
					       MOSCAD_sprintf(message,"18. No valid information in table: %d",nIndxTbl[nI]);
					       MOSCAD_error(message );
					       return;
   					}
   					
				   	sNMT[nI].p_col_NM     = (short *)(sNMT[nI].table_NM.ColDataPtr[0]);
				   	sNMT[nI].p_col_NM_LZ  = (short *)(sNMT[nI].table_NM.ColDataPtr[1]);
				   	sNMT[nI].p_col_NM_Tx  = (short *)(sNMT[nI].table_NM.ColDataPtr[2]);
				   	sNMT[nI].p_col_NM_STATUS  = (short *)(sNMT[nI].table_NM.ColDataPtr[3]);
	

	
	} /* end for */


} /* end fnNM_TABLE() */

/********************************************************************************/
/* Elloallitja az DP tablak pointereit										    */
/********************************************************************************/
void fnDP_TABLE(int nNum)
{

int 	nIndxTbl[40];	
char	message[300];
int		nI;

	/* Egesz parameterek */
   	if (MOSCAD_get_table_info (2,&table_parInt)!=0 )
   		{
        MOSCAD_sprintf(message,"19. No valid information in table: %d",2);
        MOSCAD_error(message );
        return;
   		}
	p_col_parInt = (short *)(table_parInt.ColDataPtr[0]);	


nIndxTbl[0]  = p_col_parInt[13];
nIndxTbl[1]  = p_col_parInt[43];
nIndxTbl[2]  = p_col_parInt[55];
nIndxTbl[3]  = p_col_parInt[67];
nIndxTbl[4]  = p_col_parInt[76];
nIndxTbl[5]  = p_col_parInt[85];




for (nI=0; nI<nNum;nI++)
	{

					if (MOSCAD_get_table_info (nIndxTbl[nI],&sDPT[nI].table_DP)!=0 )
   					{
					       MOSCAD_sprintf(message,"20. No valid information in table: %d",nIndxTbl[nI]);
					       MOSCAD_error(message );
					       return;
   					}
   					
				   	sDPT[nI].p_col_DPH     = (short *)(sDPT[nI].table_DP.ColDataPtr[0]);
				   	sDPT[nI].p_col_DPL     = (short *)(sDPT[nI].table_DP.ColDataPtr[1]);
				   	sDPT[nI].p_col_DP_MS1  = (short *)(sDPT[nI].table_DP.ColDataPtr[2]);
				   	sDPT[nI].p_col_DP_MS2  = (short *)(sDPT[nI].table_DP.ColDataPtr[3]);
				   	sDPT[nI].p_col_DP_MIN  = (short *)(sDPT[nI].table_DP.ColDataPtr[4]);				   
				   	sDPT[nI].p_col_DP_CT   = (short *)(sDPT[nI].table_DP.ColDataPtr[5]); 		
				   	sDPT[nI].p_col_DP_STATUS  = (short *)(sDPT[nI].table_DP.ColDataPtr[6]); 	
	
	} /* end for */


} /* end fnDP_TABLE() */



/****************************************************************************/
/* Esemeny kepzo, parameter feldolgozo függvény, és a táviratbuffer kezelése is itt történik								*/
/* 	- Globális bemenet: a MOSCAD táblái										*/
/*                      bySP, byDP, nNM tömbök                                              */
/*	- Globális kimenet: nSPWrPtr[INDX] pointer, strSPEventWT[INDX] struktúra (IEC101 szerint kitöltve) 	*/
/*	- 					        nDPWrPtr[INDX] pointer, strDPEventWT[INDX] struktúra (IEC101 szerint kitöltve)	*/
/*	-					          nNMWrPtr[INDX] pointer, strNMEvent   struktúra	(IEC101 szerint kitöltve)	*/					
/*                      A struktúra tömbbe az informatiom object-ek vannak feltöltve        */
/****************************************************************************/
void fnEvents(IEC_DUI_104		duiRec, int INDX)
{


} /* end fnEvents */

/****************************************************************************/
/* SRAM írását, olvasását végzõ fuggvények										*/
/****************************************************************************/
/****************************************************************************/
/* Elmenti az adatokat az SRAM-ba										*/
/****************************************************************************/
void fnSaveData(void)
{

fnSaveSPData(25);
fnSaveDPData(5);
fnSaveNMData(5);
fnSavePARData(2);

			
} /* end fnSaveData()*/
/****************************************************************************/
/* Kiírja az adatokat az SRAM-ból a táblákba										*/
/****************************************************************************/
void fnGetData(void)
{
char			message[300];

fnGetSPData(25);
MOSCAD_wait(350);

fnGetDPData(4);
MOSCAD_wait(250);
fnGetNMData(5);
MOSCAD_wait(250);
fnGetIntData(2);
MOSCAD_wait(150);

MOSCAD_sprintf(message,"IECDRV2: database initialised, data read from SRAM and write to tables, nSPNum: %d ---------------------", TotalData->nPAR[9]);
MOSCAD_message(message );

			
} /* end fnGetData()*/
/****************************************************************************/
/*																			*/
/* Elmenti az SP adatokat az SRAM-ba										*/
/*																			*/
/****************************************************************************/
void fnSaveSPData(int nTableNum)
{
short          *p_col_SPAct;
short          *p_col_SP_CTAct;
short          *p_col_SP_MINAct;
short          *p_col_SP_MS1Act;
short          *p_col_SP_MS2Act;
short          *p_col_SP_XORAct;
short          *p_col_SP_STATUS;
int				nTblIndx;
int				nI,nJ;
int				i;


for (nI=0;nI<nTableNum;nI++)
{					
				nTblIndx = nI;
				
				p_col_SPAct     = sSPT[nTblIndx].p_col_SP;
				p_col_SP_MS1Act = sSPT[nTblIndx].p_col_SP_MS1;
				p_col_SP_MS2Act = sSPT[nTblIndx].p_col_SP_MS2;
				p_col_SP_MINAct = sSPT[nTblIndx].p_col_SP_MIN;				   
				p_col_SP_CTAct  = sSPT[nTblIndx].p_col_SP_CT; 	
				p_col_SP_XORAct = sSPT[nTblIndx].p_col_SP_XOR; 	
				p_col_SP_STATUS = sSPT[nTblIndx].p_col_SP_STATUS;	

				for (nJ=0;nJ<250;nJ++)
					{
   					
   					i = nTblIndx*250 + nJ;
   					TotalData->SP[i].SP 	= p_col_SPAct[nJ];
   					TotalData->SP[i].SP_MS1 = p_col_SP_MS1Act[nJ];
   					TotalData->SP[i].SP_MS2 = p_col_SP_MS2Act[nJ];
   					TotalData->SP[i].SP_MIN = p_col_SP_MINAct[nJ];
   					TotalData->SP[i].SP_CT 	= p_col_SP_CTAct[nJ];
   					TotalData->SP[i].SP_XOR = p_col_SP_XORAct[nJ];
   					TotalData->SP[i].SP_STATUS = p_col_SP_STATUS[nJ];
   												
						} 
				} 


} /* end fnWriteSPData()*/


/****************************************************************************/
/*																			*/
/*  Elmenti a DP adatokat az SRAM-ba										*/
/*																			*/
/****************************************************************************/
void fnSaveDPData(int nTableNum)
{

int				nTblIndx;
int				nI,nJ;
int				i;

	short          *p_col_DPL;
	short          *p_col_DPH;
	short          *p_col_DP_MS1;
	short          *p_col_DP_MS2;
	short          *p_col_DP_MIN;
	short          *p_col_DP_CT;
	short          *p_col_DP_STATUS;


for (nI=0;nI<nTableNum;nI++)
{					
				nTblIndx = nI;
																		
				p_col_DPL	     = sDPT[nTblIndx].p_col_DPL;
				p_col_DPH	     = sDPT[nTblIndx].p_col_DPH;
				p_col_DP_MS1     = sDPT[nTblIndx].p_col_DP_MS1;
				p_col_DP_MS2     = sDPT[nTblIndx].p_col_DP_MS2;
				p_col_DP_MIN     = sDPT[nTblIndx].p_col_DP_MIN;
				p_col_DP_CT      = sDPT[nTblIndx].p_col_DP_CT;
				p_col_DP_STATUS  = sDPT[nTblIndx].p_col_DP_STATUS;

				for (nJ=0;nJ<250;nJ++)
					{
   					
   					 i = nTblIndx*250 + nJ;
   					 TotalData->DP[i].DPL		= p_col_DPL[nJ];
   					 TotalData->DP[i].DPH		= p_col_DPH[nJ];
   					 TotalData->DP[i].DP_MS1	= p_col_DP_MS1[nJ];
   					 TotalData->DP[i].DP_MS2	= p_col_DP_MS2[nJ];
   					 TotalData->DP[i].DP_CT		= p_col_DP_CT[nJ];
   					 TotalData->DP[i].DP_STATUS = p_col_DP_STATUS[nJ];;
   					   									
					}  
				}

} /* end fnSaveDPData()*/

/****************************************************************************/
/*																			*/
/* Elmenti az NM adatokat az SRAM-ba		*/
/*																			*/
/****************************************************************************/
void fnSaveNMData(int nTableNum)
{

int				nTblIndx;
int				nI,nJ;
int				i;

	short          *p_col_NM;
	short          *p_col_NM_LZ;
	short          *p_col_NM_Tx;
	short          *p_col_NM_STATUS;	


for (nI=0;nI<nTableNum;nI++)
{					
				nTblIndx = nI;
																		
				p_col_NM	     = sNMT[nTblIndx].p_col_NM;
				p_col_NM_LZ	     = sNMT[nTblIndx].p_col_NM_LZ;
				p_col_NM_Tx	     = sNMT[nTblIndx].p_col_NM_Tx;
				p_col_NM_STATUS  = sNMT[nTblIndx].p_col_NM_STATUS;

				for (nJ=0;nJ<250;nJ++)
					{
   					
   					 i = nTblIndx*250 + nJ;
   					  TotalData->NM[i].NM 		= p_col_NM[nJ];
   					  TotalData->NM[i].NM_LZ 	= p_col_NM_LZ[nJ];
   					  TotalData->NM[i].NM_Tx 	= p_col_NM_Tx[nJ];
   					  TotalData->NM[i].NM_STATUS= p_col_NM_STATUS[nJ];
   					    					   									
					}  
				}

} /* end fnSaveNMData()*/
/****************************************************************************/
/*																			*/
/* Elmenti az INT paraméter adatokat az SRAM-ba		*/
/*																			*/
/****************************************************************************/
void fnSavePARData(int nTableNum)
{

int				nJ;





short          *p_col_parInt;
CB_TABLE_INFO   table_parInt;
	
	
	/* Egesz parameterek */
   	if (MOSCAD_get_table_info (nTableNum,&table_parInt)!=0 )
   		{
        MOSCAD_sprintf(message,"3. No valid information in table: %d",nTableNum);
        MOSCAD_error(message );
        return;
   		}
	p_col_parInt = (short *)(table_parInt.ColDataPtr[0]);	


				for (nJ=0;nJ<88;nJ++)
					{   					
   					 TotalData->nPAR[nJ] =  p_col_parInt[nJ];  					   									
					}  
				

} /* end fnSavePARData()*/
/****************************************************************************/
/* Kiolvassa az SP adatokat az SRAM-ból és beírja a MOSCAD táblákba			*/
/*																			*/
/****************************************************************************/
void fnGetSPData(int nTableNum)
{
short          *p_col_SPAct;
short          *p_col_SP_CTAct;
short          *p_col_SP_MINAct;
short          *p_col_SP_MS1Act;
short          *p_col_SP_MS2Act;
short          *p_col_SP_XORAct;
short          *p_col_SP_STATUS;

int				nTblIndx;
int				nI,nJ;
int				i;


for (nI=0;nI<nTableNum;nI++)
{					
				nTblIndx = nI;
								
				p_col_SPAct     = sSPT[nTblIndx].p_col_SP;
				p_col_SP_MS1Act = sSPT[nTblIndx].p_col_SP_MS1;
				p_col_SP_MS2Act = sSPT[nTblIndx].p_col_SP_MS2;
				p_col_SP_MINAct = sSPT[nTblIndx].p_col_SP_MIN;				   
				p_col_SP_CTAct  = sSPT[nTblIndx].p_col_SP_CT; 	
				p_col_SP_XORAct = sSPT[nTblIndx].p_col_SP_XOR; 	
				p_col_SP_STATUS = sSPT[nTblIndx].p_col_SP_STATUS;	


				for (nJ=0;nJ<250;nJ++)
					{
   					
   					 i = nTblIndx*250 + nJ;
   					 p_col_SPAct[nJ] 	= TotalData->SP[i].SP;
   					 p_col_SP_MS1Act[nJ]= TotalData->SP[i].SP_MS1;
   					 p_col_SP_MS2Act[nJ]= TotalData->SP[i].SP_MS2;
   					 p_col_SP_MINAct[nJ]= TotalData->SP[i].SP_MIN;
   					 p_col_SP_CTAct[nJ] = TotalData->SP[i].SP_CT;
   				 	 p_col_SP_XORAct[nJ]= TotalData->SP[i].SP_XOR;
   					 p_col_SP_STATUS[nJ]= TotalData->SP[i].SP_STATUS;
   									
					 /*p_col_SPAct[nJ] = 12;*/
					}  
				}


} /* end fnWriteSPData()*/
/****************************************************************************/
/*																			*/
/* Kiolvassa a DP adatokat az SRAM-ból és beírja a MOSCAD táblákba			*/
/*																			*/
/****************************************************************************/
void fnGetDPData(int nTableNum)
{

int				nTblIndx;
int				nI,nJ;
int				i;

	short          *p_col_DPL;
	short          *p_col_DPH;
	short          *p_col_DP_MS1;
	short          *p_col_DP_MS2;
	short          *p_col_DP_MIN;
	short          *p_col_DP_CT;
	short          *p_col_DP_STATUS;


for (nI=0;nI<nTableNum;nI++)
{					
				nTblIndx = nI;
																		
				p_col_DPL	     = sDPT[nTblIndx].p_col_DPL;
				p_col_DPH	     = sDPT[nTblIndx].p_col_DPH;
				p_col_DP_MS1     = sDPT[nTblIndx].p_col_DP_MS1;
				p_col_DP_MS2     = sDPT[nTblIndx].p_col_DP_MS2;
				p_col_DP_MIN     = sDPT[nTblIndx].p_col_DP_MIN;
				p_col_DP_CT      = sDPT[nTblIndx].p_col_DP_CT;
				p_col_DP_STATUS  = sDPT[nTblIndx].p_col_DP_STATUS;

				for (nJ=0;nJ<250;nJ++)
					{
   					
   					 i = nTblIndx*250 + nJ;
   					 p_col_DPL[nJ] 		= TotalData->DP[i].DPL;
   					 p_col_DPH[nJ] 		= TotalData->DP[i].DPH;
   					 p_col_DP_MS1[nJ]	= TotalData->DP[i].DP_MS1;
   					 p_col_DP_MS2[nJ]	= TotalData->DP[i].DP_MS2;
   					 p_col_DP_CT[nJ]	= TotalData->DP[i].DP_CT;
   					 p_col_DP_STATUS[nJ]= TotalData->DP[i].DP_STATUS;
   					   									
					}  
				}

} /* end fnWriteDPData()*/
/****************************************************************************/
/*																			*/
/* Kiolvassa az NM adatokat az SRAM-ból és beírja a MOSCAD táblákba			*/
/*																			*/
/****************************************************************************/
void fnGetNMData(int nTableNum)
{

int				nTblIndx;
int				nI,nJ;
int				i;

	short          *p_col_NM;
	short          *p_col_NM_LZ;
	short          *p_col_NM_Tx;
	short          *p_col_NM_STATUS;	


for (nI=0;nI<nTableNum;nI++)
{					
				nTblIndx = nI;
																		
				p_col_NM	     = sNMT[nTblIndx].p_col_NM;
				p_col_NM_LZ	     = sNMT[nTblIndx].p_col_NM_LZ;
				p_col_NM_Tx	     = sNMT[nTblIndx].p_col_NM_Tx;
				p_col_NM_STATUS  = sNMT[nTblIndx].p_col_NM_STATUS;

				for (nJ=0;nJ<250;nJ++)
					{
   					
   					 i = nTblIndx*250 + nJ;
   					 p_col_NM[nJ] 			= TotalData->NM[i].NM;
   					 p_col_NM_LZ[nJ] 		= TotalData->NM[i].NM_LZ;
   					 p_col_NM_Tx[nJ] 		= TotalData->NM[i].NM_Tx;
   					 p_col_NM_STATUS[nJ]	= TotalData->NM[i].NM_STATUS;
   					    					   									
					}  
				}

} /* end fnWriteNMData()*/
/****************************************************************************/
/*																			*/
/* Kiolvassa az IEC par int táblát adatai az SRAM-ból és beírja a MOSCAD táblába			*/
/*																			*/
/****************************************************************************/
void fnGetIntData(int nTableNum)
{


int				nJ;


short          *p_col_parInt;
CB_TABLE_INFO   table_parInt;
	
	
	/* Egesz parameterek */
   	if (MOSCAD_get_table_info (nTableNum,&table_parInt)!=0 )
   		{
        MOSCAD_sprintf(message,"3. No valid information in table: %d",nTableNum);
        MOSCAD_error(message );
        return;
   		}
	p_col_parInt = (short *)(table_parInt.ColDataPtr[0]);	

		for (nJ=0;nJ<88;nJ++)
			{
				 p_col_parInt[nJ] = TotalData->nPAR[nJ];    					    					   									
			}  

} /* end fnGetIntData()*/
/****************************************************************************/
/* Torli az adatokat az SRAM-ban											*/
/*																			*/
/****************************************************************************/
void fnResetSRAM(void)
{
unsigned long			nJ;
char					*ch;

ch = (char *)MOSCAD_bspSRamStart();

				for (nJ=0;nJ<lLengthTotalData;nJ++)
					{
						
   					ch[nJ]=0;
 			
					} 
fnGetData();

					       MOSCAD_sprintf(message,"IECDRV2: SRAM erased");
					       MOSCAD_message(message );

} /* end fnResetSRAM()*/

/****************************************************************************/
/*	Elküld egy TESTFR ACT táviratot										*/
/*																			*/
/****************************************************************************/
void fnSendTESTFR_ACT(int INDX)
{
int     retval;
BYTE    sBuff[300];


			
        sBuff[0] = START_CHAR;
				sBuff[1] = 0x04;				
				sBuff[2] = 0x43;  
				sBuff[3] = 0x00;
				sBuff[4] = 0x00;
				sBuff[5] = 0x00;				
        retval = MOSCAD_socket_send(newsocket[INDX], sBuff, 6,0);		


} /* end fnSendTESTFR_ACT()*/

/****************************************************************************/
/*																			*/
/*	Receive sequence number kezelése										*/
/*																			*/
/****************************************************************************/
void fnRecSeqNum(int INDX, unsigned int nRecSeqNumClient)
{

int           nI;
int           nIndex;
int           nMessLength;




		        
/*            MOSCAD_sprintf(message,"Socekt: %d: !!!!!!!!!!!!!!!!!!!!!!!!! Nyugta érkezett: nRecSeqNumClient: %d, nASDUSendPtr[INDX]: %d, nASDUWrPtr[INDX]: %d, nSendSeqNum[INDX]: %d", INDX,nRecSeqNumClient,nASDUSendPtr[INDX], nASDUWrPtr[INDX], nSendSeqNum[INDX]);			
    				MOSCAD_message(message );
		        MOSCAD_sprintf(message," [0]: %d, [1]: %d, [2]: %d, [3]: %d, [4]: %d , [5]: %d , [6]: %d, [7]: %d, [8]: %d , [9]: %d   ", strASDU[INDX][0].nSendNum, strASDU[INDX][1].nSendNum,strASDU[INDX][2].nSendNum,strASDU[INDX][3].nSendNum,strASDU[INDX][4].nSendNum,strASDU[INDX][5].nSendNum,  strASDU[INDX][6].nSendNum, strASDU[INDX][7].nSendNum, strASDU[INDX][8].nSendNum, strASDU[INDX][9].nSendNum); 			
    				MOSCAD_message(message );  */

					
				nMessLength = 6;

          if   (strASDU[INDX][0].nSendNum == 0 && strASDU[INDX][1].nSendNum == 0) /* Ha csak úgy jött egy nyugta, várunk még egy kicsit - 2012.06.21 !!! */
          {
            /*MOSCAD_sprintf(message,"Socket: %d: Csak úgy jött egy nyugta !!!, nRecSeqNumClient: %d",INDX,nRecSeqNumClient); 			
    			  MOSCAD_message(message );*/ 

            /* MOSCAD_wait(2800);                                                                                           
            fnSendTESTFR_ACT(INDX);	                    
            MOSCAD_wait(2800); */          
          }
          else
          {

		 
		 
		    /* Fel kell szabadítani az strASDU[INDX] struktúrában a helyeket ---------------------------------*/
		    /* Meg kell keresni a kapott Receive Seguence numberhez tartozó tömbindexet az strASDU[INDX] struktúrában */
		    for (nI=0;nI<MAX_ASDU-5;nI++)
		    {
		      if (strASDU[INDX][nI].nSendNum == nRecSeqNumClient-1)
		      {
		        nIndex = nI;
		      /*  MOSCAD_sprintf(message,"Socket: %d: ************************** Nyugta érkezett: nIndex: %d,strASDU[INDX][nI].nSendNum: %d,   nRecSeqNumClient: %d, nASDUSendPtr[INDX]: %d, nASDUWrPtr[INDX]: %d, nSendSeqNum[INDX]: %d",INDX, nIndex,strASDU[INDX][nI].nSendNum, nRecSeqNumClient,nASDUSendPtr[INDX], nASDUWrPtr[INDX], nSendSeqNum[INDX]);			
    				MOSCAD_message(message );
    				
		        MOSCAD_sprintf(message," [0]: %d, [1]: %d, [2]: %d, [3]: %d, [4]: %d , [5]: %d , [6]: %d, [7]: %d, [8]: %d , [9]: %d   ", strASDU[INDX][0].nSendNum, strASDU[INDX][1].nSendNum,strASDU[INDX][2].nSendNum,strASDU[INDX][3].nSendNum,strASDU[INDX][4].nSendNum,strASDU[INDX][5].nSendNum,  strASDU[INDX][6].nSendNum, strASDU[INDX][7].nSendNum, strASDU[INDX][8].nSendNum, strASDU[INDX][9].nSendNum); 			
    				MOSCAD_message(message ); */
    				



    				

            if ( nASDUSendPtr[INDX] > nIndex )
            {
              /* nASDUSendPtr[INDX] = nASDUSendPtr[INDX] - nIndex-1;  Ez a jó */
              nASDUSendPtr[INDX] = nASDUSendPtr[INDX] - nIndex-1;  
            }
            else
            {
               MOSCAD_message("nIndex > nASDUSendPtr[INDX] !!!!!!!!!!!!!!!!!!!!!");	
            }
            
            if ( nASDUWrPtr[INDX] > nIndex )
            {
              nASDUWrPtr[INDX] = nASDUWrPtr[INDX] - nIndex-1;
            }
            else
            {
               MOSCAD_message("nIndex > nASDUWrPtr[INDX] !!!!!!!!!!!!!!!!!!!!!");	
            }
            

            memcpy(&strASDU[INDX][0], &strASDU[INDX][nIndex+1], sizeof(strASDU[INDX][0])*(MAX_ASDU-nIndex-5));
 
/*		        MOSCAD_sprintf(message,"Socket: %d:  Total size: %d, 1 member size: %d, copyied size: %d", INDX,sizeof(strASDU[INDX]), sizeof(strASDU[INDX][0]),sizeof(strASDU[INDX][0])*(MAX_ASDU-nIndex-3) ); 			
    				MOSCAD_message(message );
 
            
		        MOSCAD_sprintf(message,"Socket: %d:  [0]: %d, [1]: %d, [2]: %d, [3]: %d, [4]: %d , [5]: %d , [6]: %d, [7]: %d, [8]: %d , [9]: %d   ", INDX, strASDU[INDX][0].nSendNum, strASDU[INDX][1].nSendNum,strASDU[INDX][2].nSendNum,strASDU[INDX][3].nSendNum,strASDU[INDX][4].nSendNum,strASDU[INDX][5].nSendNum,  strASDU[INDX][6].nSendNum, strASDU[INDX][7].nSendNum, strASDU[INDX][8].nSendNum, strASDU[INDX][9].nSendNum); 			
    				MOSCAD_message(message );  */
            } /* end if (strASDU[INDX][nI].nSendNum == nRecSeqNumClient-1) */
          } /*  end for        */
        } /* end else */


} /* end fnRecSeqNum(int INDX) */


/****************************************************************************/
/*																			*/
/*	Ha túlcsordul az ASDU puffer										*/
/*																			*/
/****************************************************************************/
void fnDisconnect(int INDX)
{

              p_col_Stat[24] = p_col_Stat[24] + 1;  /* 2014.05.12 */
              
					   	newsocket[INDX]=-1;  /*  ~~~ */
              MOSCAD_message("Disconnect from client" );




} /* fnDisconnect*/






