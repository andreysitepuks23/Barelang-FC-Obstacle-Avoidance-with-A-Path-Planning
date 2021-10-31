//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////// Bismillahirrahmanirrahim Tahun ini Juara 1 //////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>

#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <string.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netdb.h>

#include "BarelangFC-Vision.h"
#include "BarelangFC-Receiver.h"
#include "BarelangFC-Strategy.h"
#include "BarelangFC-RobotCoordination.h"
#include "BarelangFC-GameControllerClient.h"
#include "BarelangFC-Odometry.h"
#include "BarelangFC-Monitor.h"

//-----------------------config Parameter------------------------
#include "minIni.h"
#define INVALID_VALUE   -1024.0
minIni* ini = new minIni("configure.ini");
//---------------------------------------------------------------

#define	PI	3.1415926535897932384626433832795
#define	PORT	3838
#define	BUFLEN	4096
#define BUFSIZE	1024

#define PORTM	5000	//motion
#define PORTH	5001	//head
#define PORTL   5005    //lokalisasi
#define HOST	"localhost"

//#define SERVER		"192.168.123.30" //IP Address laptop juri
#define SERVER		"192.168.1.1" //IP Address laptop juri
#define TEAM		1 //1 //2 //3 //4 //5 //nomor tim di GameController
#define	robotID		robotNumber //1 //2 //3 //4 //5

#define BARELANG_COLOR	TEAM //inter = TEAM, nasional (0 = cyan, magenta = 1).
#define DROPBALL	128 //128 = inter, 2 = nasional

using namespace BarelangFC;
	pthread_t threadStrategy, threadVision, threadGC_Read, threadGC_Return, Lokalisasi, threadReceiverVision, threadReceiverLocalization, threadPID, threadMonitor;
	BarelangVision kamera;
	BarelangReceiver receler;
	BarelangStrategy strategy;
	RobotCoordination koordinasi;
	GameControllerClient gameController;
	BarelangOdometry odometry;
	BarelangMonitor monitor;

void * sendMonitoring( void * uMonitor) { monitor.sendingMonitor();}
void * updateStrategy( void * uStrategy) { strategy.checkStrategyControl(); }
//void * camera( void * uCamera) { kamera.processing(); }
//void * camera( void * uCamera) { kamera.pidTuner(); }
void * receiverVision( void * uReceiver) { receler.processing_vision(); }
void * receiverLocalization( void * uReceiver) { receler.processing_localization(); }
void * gcReadData(void * argument) { gameController.readGameControllerData(); }
void * gcReturnData(void * argument) { gameController.returnGameControllerData(); }
void runLuaProgram() { system("cd ../Player; screen -S dcm lua run_dcm.lua; screen -S player lua walk_server.lua;");}
//void runLuaProgram() { system("cd ../Player; screen -S dcm lua run_dcm.lua;");}


/////////////////////////////////////////////////////////
////////////////////Fungsi Global////////////////////////
/////////////////////////////////////////////////////////
void	resetCase1();
void	resetCase2();
void	resetCase3();
void	resetCase4();
void	resetCase5();
void	resetCase6();
void	resetCase7();
void	resetCase8();
void	resetOdometry();
void	resetKoordinasiRobotBalik();
void	resetAllLokalisasiVariable();
void	resetAllVariable();
void    refreshMoveLokalisasi();
void 	refreshMoveGrid();
void	jalanDirection(double, double, double);
void	saveSudutImu();
double 	abs(double);

/////////////////////////////////////////////////////////
///////////////////Variable Global///////////////////////
/////////////////////////////////////////////////////////
int	robotNumber,
	stateCondition = 0,
	firstStateCondition = 0,	//switch strategy

	stateGameController = 0,
	lastStateGameController = 0,
	stateChange = 0,

	kickOff = 0,			//parsing tambahan ketika gameController bermasalah
	lastKickOff = 0,		//parsing tambahan ketika gameController bermasalah
	kickOffChange = 0,		//parsing tambahan ketika gameController bermasalah

	secRemaining = 0,
	lastSecRemaining = 0,
	secRemainingChange = 0,

	state,				//kill n run
	lastState,			//kill n run
	wait = 0,

	delay = 0,			//search goal case 4
	delayWaitBall = 0,		//search ball case 0
	countBearing = 0,		//Imu erorr
	countDribble = 0,		//lama dribble
	tunda = 0,
	tunggu = 0,
	waiting = 0,
	waitTracking = 0,
	reset = 0,
	delayTrackBall = 0,
	matte = 0,
	chotto = 0,

	modeKick = 1,

	saveAngle = 0,
	lastDirection = 0,

	countHilang = 0,

	confirmsBall = 0,
	countTilt = 0,
	sumTilt = 0,

	Strategi = 0,

	countInitPos = 0,	//lock initial pos untuk pickup
	robotStatus = 1,	//Robot Aktif

	varCount = 0,		//variable counting untuk case 1001
	countDef = 0,		//variable counting untuk case 130 & 140

	//New Variable 2020
	modePlay = 0,		//set initial posisi 0 = nasional, 1 = internasional
	varCount1 = 0, varCount2 = 0, varCount3 = 0, varCount4 = 0, varCount5 = 0,	//variable counting void moveGrid
	BackIn = 0,		//koordinasi robot balik
	//initialPos_X = 0, initialPos_Y = 0,
	//robotPos_X = 0,	robotPos_Y = 0,
	Grid = 0,		//grid posisi robot
	initGrid = 0, offsetX = 0, offsetY = 0,
	ballPos_X = 0, ballPos_Y = 0,	//koordinat posisi bola dilapangan
	GridBall = 0;			//grid posisi bola

double	headPan,	//f.Kepala
	headTilt,	//f.Kepala
	posPan,
	posTilt,
	errorPan,	//f.Kepala
	errorTilt,	//f.Kepala
	PPan,		//f.Kepala
	PTilt,		//f.Kepala

	ball_panKP,	//PID trackBall
	ball_panKD,	//PID trackBall
	ball_tiltKP,	//PID trackBall
	ball_tiltKD,	//PID trackBall

	goal_panKP,	//PID trackGoal
	goal_panKD,	//PID trackGoal
	goal_tiltKP,	//PID trackGoal
	goal_tiltKD;	//PID trackGoal

double	ballPositioningSpeed,
	pTiltTendang,
	pPanTendang,
	pTiltOper,
	pPanOper,
	cSekarang,
	cAktif,
	posTiltLocal,
	posTiltGoal,
	erorrXwalk,
	erorrYwalk,
	erorrAwalk,
	jalan,
	lari,
	kejar,
	kejarMid,
	kejarMax,
	tinggiRobot,
	outputSudutY1,
	inputSudutY1,
	outputSudutY2,
	inputSudutY2,
	outputSudutX1,
	inputSudutX1,
	outputSudutX2,
	inputSudutX2,
	frame_X,
	frame_Y,
	rotateGoal_x,
	rotateGoal_y,
	rotateGoal_a,

	sendAngle,
	angle,

	robotPos_X,	//om
	initialPos_X,	//ov
	robotPos_Y,	//om
	initialPos_Y,	//om

	kurama = 0;

int	sudutTengah,
	sudutKanan,
	sudutKiri,
	tendangJauh,
	tendangDekat,
	tendangSamping;

bool	play = false,
	firstTimes = true,	//first strategy player
	zeroState = false,	//send ball is not found (for coordinations)
	tracked = false,	//search ball
	pickUp = false,		//strategi setelah pick up
	searchRectangle = false,
	FollowGoal = false,

	masukKiri = false,
	masukKanan = false,
	backPosition = false,
	switched = false,
	manual = false,
	signIn = false,

	kanan = false,
	kiri = false,

	followSearchAktif = false,
	Activated = false,

	jatuh = false,
	exeCutor = false,

	robot1exeCutor = false,		//koordinasi
	robot2exeCutor = false,		//koordinasi
	robot3exeCutor = false,		//koordinasi
	robot4exeCutor = false,		//koordinasi
	robot5exeCutor = false,		//koordinasi

	useVision = true,		//f.setting
	useSocket = true,		//f.setting
	useGameController = true,	//f.setting
	useCoordination = true,		//f.setting
	useLocalization = true,		//f.setting
	useFollowSearchGoal = true,	//f.setting
	useImu = true,			//f.setting
	useDribble = true,		//f.setting
	dribbleOnly = true,		//f.setting
	useSearchGoal = true,		//f.setting
	useSideKick = true,		//f.setting
	useLastDirection = true,	//f.setting
	useNearFollowSearchGoal = true,	//f.setting
	useUpdateCoordinate = true,	//f.setting
	usePenaltyStrategy = true;	//f.setting

//Socket LUA head&motion
int	sockmotion;
int	sockhead;
struct	sockaddr_in addrmotion;
struct	sockaddr_in addrhead;
struct	hostent *localserver;
char	dataMotion[20];
char	dataHead[20];
char	line[10];

//Socket Lokalisasi
int     socklokalisasi;
struct  sockaddr_in addrlokalisasi;
char    dataLokalisasi[30];
char    param[15];

///////////////////////////////////////////////////////////////////
///////////////////////open port LUA///////////////////////////////
///////////////////////////////////////////////////////////////////
void initSendDataMotion() {
	sockmotion = socket(AF_INET, SOCK_DGRAM, 0);
	localserver = gethostbyname(HOST);
	bzero((char *) &addrmotion, sizeof(addrmotion));
	addrmotion.sin_family = AF_INET;
	addrmotion.sin_port = htons(PORTM);
}

void initSendDataHead() {
	sockhead = socket(AF_INET, SOCK_DGRAM, 0);
	localserver = gethostbyname(HOST);
	bzero((char *) &addrhead, sizeof(addrhead));
	addrhead.sin_family = AF_INET;
	addrhead.sin_port = htons(PORTH);
}


///////////////////////////////////////////////////////////////////
////////////////////open port Lokalisasi///////////////////////////
///////////////////////////////////////////////////////////////////
void initSendDataLokalisasi() {
        socklokalisasi = socket(AF_INET, SOCK_DGRAM, 0);
        localserver = gethostbyname(HOST);
        bzero((char *) &addrlokalisasi, sizeof(addrlokalisasi));
        addrlokalisasi.sin_family = AF_INET;
        addrlokalisasi.sin_port = htons(PORTL);
}


// Function To Set Timer =====================================================================
struct timeval	t1,
		t2;
int	attends;
double	elapsedTime,
	second;
bool	timer = false;
void setWaktu() {
	elapsedTime	=
	second		=
	attends		= 0;
	timer		= false;

	gettimeofday(&t1, NULL);
}
// Function For Check Timer
void cekWaktu(double detik) {
	if (attends > 10) {
		gettimeofday(&t2, NULL);

		// compute and print the elapsed time in millisec
		elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
		elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;
		second = elapsedTime / 1000.0;
		//printf ("  waktu berlangsung = %.f detik \n\n\n\n", second);

		if (second >= detik) {
			timer = true;
		} else {
			timer = false;
		}
	} else { attends++; }
}

// Absolute ==================================================================================
double abs(double x) {
	if (x < 0) { x = -x; }
	return x;
}

// Sensor Accelero, Gyro, Angle Value ========================================================
double	accrX, accrY, accrZ,
	gyroX, gyroY, gyroZ,
	angleX, angleY, angleZ,
	myAccrX, myAccrY;
bool	robotJatuh = false,
	robotBergerak = false;
int	countParse = 0,
	countMove = 0;
void getSensor() {
	char line[100];
	char * spr;

	FILE *outputfp;
	outputfp = fopen("../Player/SensorNote", "r");
	fscanf(outputfp, "%s", &line);
	countParse = 0;
	spr = strtok (line, ";");
	while (spr != NULL) {
		if (countParse == 0)	  { sscanf(spr, "%lf", &accrX); } //x					#dpn-blkng
		else if (countParse == 1) { sscanf(spr, "%lf", &accrY); } //y					#kiri-kanan
		else if (countParse == 2) { sscanf(spr, "%lf", &accrZ); } //z
		else if (countParse == 3) { sscanf(spr, "%lf", &gyroX); } //roll				#kiri-kanan
		else if (countParse == 4) { sscanf(spr, "%lf", &gyroY); } //pitch				#dpn-blkng
		else if (countParse == 5) { sscanf(spr, "%lf", &gyroZ); } //yaw
		else if (countParse == 6) { sscanf(spr, "%lf", &angleX); } //
		else if (countParse == 7) { sscanf(spr, "%lf", &angleY); } //
		else if (countParse == 8) { sscanf(spr, "%lf", &angleZ); countParse = -1; } //
		spr = strtok (NULL,";");
		countParse++;
	}
	fclose(outputfp);
	//printf("  accr(%.2lf, %.2lf, %.2lf)", accrX, accrY, accrZ);
	//printf("  gyro(%.2lf, %.2lf, %.2lf)", gyroX, gyroY, gyroZ);
	//printf("  angle(%.lf, %.lf, %.lf)", angleX, angleY, angleZ);

	// menunjukkan bahwa robot sedang dalam kondisi jatuh -> back to case 0
	if ((accrX >= -0.5) && (accrX <= 0.9) && (accrY >= -0.9) && (accrY <= 0.9)) { //printf("  robot berdiri...................\n\n");
		robotJatuh = false;
	} else { //printf("  ...................robot jatuh.\n\n");
		robotJatuh = true;
	}

	if (gyroX != 0) { //printf("  robot bergerak............\n");
		countMove = 0;
		robotBergerak = true;
	} else { //printf("  ............robot TDK bergerak\n");
		if (countMove > 10) {
			robotBergerak = false;
		} else {
			countMove++;
		}
	}
}

int	acclrX;
bool	robotFall = false;
void getImuSensor() {
	acclrX = strategy.pitch;
	if (acclrX >= -50 && acclrX <= 50) { //printf("  robot berdiri...................\n\n");
		robotFall = false;
	} else { //printf("  ...................robot jatuh.\n\n");
		robotFall = true;
	}
}

// Sudut 20 Servo ===========================================================================
double	head1, head2,
	Larm1, Larm2, Larm3,
	Lleg1, Lleg2, Lleg3, Lleg4, Lleg5, Lleg6,
	Rleg1, Rleg2, Rleg3, Rleg4, Rleg5, Rleg6,
	Rarm1, Rarm2, Rarm3;
int	countParses = 0;
void getServPos() {
	char line[200];
	char * sprs;

	FILE *outputfp;
	outputfp = fopen("../Player/PosNote", "r");
	fscanf(outputfp, "%s", &line);
	countParses = 0;
	sprs = strtok (line, ";");
	while (sprs != NULL) {
		if (countParses == 0)	  { sscanf(sprs, "%lf", &head1); }			//ID=19
		else if (countParses == 1) { sscanf(sprs, "%lf", &head2); }			//ID=20
		else if (countParses == 2) { sscanf(sprs, "%lf", &Larm1); }			//ID=2
		else if (countParses == 3) { sscanf(sprs, "%lf", &Larm2); }			//ID=4
		else if (countParses == 4) { sscanf(sprs, "%lf", &Larm3); }			//ID=6
		else if (countParses == 5) { sscanf(sprs, "%lf", &Lleg1); odometry.L_leg1 = Lleg1;}			//ID=8
		else if (countParses == 6) { sscanf(sprs, "%lf", &Lleg2); odometry.L_leg2 = Lleg2;}			//ID=10
		else if (countParses == 7) { sscanf(sprs, "%lf", &Lleg3); odometry.L_leg3 = Lleg3;}			//ID=12
		else if (countParses == 8) { sscanf(sprs, "%lf", &Lleg4); odometry.L_leg4 = Lleg4;}			//ID=14
		else if (countParses == 9) { sscanf(sprs, "%lf", &Lleg5); odometry.L_leg5 = Lleg5;}			//ID=16
		else if (countParses == 10) { sscanf(sprs, "%lf", &Lleg6); odometry.L_leg6 = Lleg6;}			//ID=18
		else if (countParses == 11) { sscanf(sprs, "%lf", &Rleg1); odometry.R_leg1 = Rleg1;}			//ID=7
		else if (countParses == 12) { sscanf(sprs, "%lf", &Rleg2); odometry.R_leg2 = Rleg2;}			//ID=9
		else if (countParses == 13) { sscanf(sprs, "%lf", &Rleg3); odometry.R_leg3 = Rleg3;}			//ID=11
		else if (countParses == 14) { sscanf(sprs, "%lf", &Rleg4); odometry.R_leg4 = Rleg4;}			//ID=13
		else if (countParses == 15) { sscanf(sprs, "%lf", &Rleg5); odometry.R_leg5 = Rleg5;}			//ID=15
		else if (countParses == 16) { sscanf(sprs, "%lf", &Rleg6); odometry.R_leg6 = Rleg6;}			//ID=17
		else if (countParses == 17) { sscanf(sprs, "%lf", &Rarm1); }			//ID=1
		else if (countParses == 18) { sscanf(sprs, "%lf", &Rarm2); }			//ID=3
		else if (countParses == 19) { sscanf(sprs, "%lf", &Rarm3); countParses = -1; }	//ID=5
		sprs = strtok (NULL,";");
		countParses++;
	}
	fclose(outputfp);
	//printf("  head(%.lf, %.lf)", head1, head2);
	//printf("  Larm(%.lf, %.lf, %.lf)", Larm1, Larm2, Larm3);
	//printf("  Rarm(%.lf, %.lf, %.lf)", Rarm1, Rarm2, Rarm3);
	//printf("  Lleg(%.lf, %.lf, %.lf, %.lf, %.lf, %.lf)", Lleg1, Lleg2, Lleg3, Lleg4, Lleg5, Lleg6);
	//printf("  Rleg(%.lf, %.lf, %.lf, %.lf, %.lf, %.lf)", Rleg1, Rleg2, Rleg3, Rleg4, Rleg5, Rleg6);
}

// Walk Controller ===========================================================================
double  walkX = 0.0,
        walkY = 0.0,
        walkA = 0.0;
double Walk(double x, double y, double a) {

	char line[50];

	walkX = x;
	walkY = y;
        walkA = a;

	odometry.RobotWalk_X = walkX; //+ erorrXwalk;
	odometry.RobotWalk_Y = walkY; //+ erorrYwalk;
	odometry.RobotWalk_A = walkA; //+ erorrAwalk;

	if (robotID != 2 || robotID != 5) {
		if (odometry.RobotWalk_X >= 0.08) { odometry.RobotWalk_X = 0.08; }
		else if (odometry.RobotWalk_X <= -0.03) { odometry.RobotWalk_X = -0.03; }

		if (odometry.RobotWalk_Y >= 0.03) { odometry.RobotWalk_Y = 0.03; }
		else if (odometry.RobotWalk_Y <= -0.03) { odometry.RobotWalk_Y = -0.03; }

		if (odometry.RobotWalk_A >= 0.4) { odometry.RobotWalk_A = 0.4; }
		else if (odometry.RobotWalk_A <= -0.4) { odometry.RobotWalk_A = -0.4; }
	} else {
		if (odometry.RobotWalk_X >= 0.06) { odometry.RobotWalk_X = 0.06; }
		else if (odometry.RobotWalk_X <= -0.03) { odometry.RobotWalk_X = -0.03; }

		if (odometry.RobotWalk_Y >= 0.03) { odometry.RobotWalk_Y = 0.03; }
		else if (odometry.RobotWalk_Y <= -0.03) { odometry.RobotWalk_Y = -0.03; }

		if (odometry.RobotWalk_A >= 0.4) { odometry.RobotWalk_A = 0.4; }
		else if (odometry.RobotWalk_A <= -0.4) { odometry.RobotWalk_A = -0.4; }
	}

	if (useSocket) { // Socket
		strcpy(line, "walk");
		sprintf(dataMotion, "%s,%f,%f,%f", line, x+erorrXwalk, y+erorrYwalk, a+erorrAwalk);
		sendto(sockmotion, dataMotion, strlen(dataMotion), 0,(struct sockaddr *) &addrmotion, sizeof(addrmotion));
		//printf("  data walk = %s\n", dataMotion);
	} else { // NotePad
		FILE *outputfp;
		strcpy(line, "walk");
		outputfp = fopen("../Player/WalkNote", "wb");
		fprintf(outputfp, "%s,%.2lf,%.2lf,%.2lf", &line[0], x+erorrXwalk, y, a);//setting jalan ditempat default
		fclose(outputfp);
		//printf("walk : %g,%g,%g\n",x,y,a);
	}

	return (walkX,walkY,walkA);
}

// Robot Movement ============================================================================
void motion(char line[2]) {
	//Tendang Jauh Kiri		= 1
	//Tendang Jauh Kanan		= 2

	//Tendang Pelan Kiri		= 3
	//Tendang Pelan Kanan		= 4

	//Tendang Ke Samping Kiri	= 5
	//Tendang Ke Samping Kanan	= 6

	//Tendang WalkKick Kiri		= t
	//Tendang WalkKick Kanan	= y

	//Duduk				= 7
	//Berdiri			= 8
	//Play				= 9
	//Stop				= 0

	if (useSocket) { // Socket
		char awalan[50];
		strcpy(awalan, "motion");
		sprintf(dataMotion, "%s,%s",awalan, line);
		sendto(sockmotion, dataMotion, strlen(dataMotion), 0, (struct sockaddr *) &addrmotion, sizeof(addrmotion));
		//printf("  data motion = %s\n", dataMotion);
	} else { // NotePad
		FILE *outputfp;
		outputfp = fopen("../Player/WalkNote", "wb");
		fprintf(outputfp, "%s", &line[0]);
		fclose(outputfp);
	}

	// untuk kondisi offset imu
	if (line == "9") {
		strategy.robotGerak = true;
	} else if (line == "0") {
		strategy.robotGerak = false;
	}
}

// Checking Lost Ball ========================================================================
int	countBallLost  = 0,
	countBallFound = 0,
	returnBallVal;
int ballLost(int threshold) {
	if (useVision) {
		if (receler.Ball_X == -1 && receler.Ball_Y == -1) {
			countBallFound = 0;
			countBallLost++;
			if (countBallLost >= threshold) {
				returnBallVal = 1;
			}
		} else {
			countBallLost = 0;
			countBallFound++;
			if (countBallFound > 1) {
				returnBallVal = 0;
			}
		}
	} else {
		countBallFound = 0;
		countBallLost++;
		if (countBallLost >= threshold) {
			returnBallVal = 1;
		}
	} return returnBallVal;
}

// Head Movement =============================================================================
void headMove(double pan,double tilt) {
	if (useSocket) { // Socket
		if (robotFall) { sprintf(dataHead, "%.2f,%.2f", 0.0, -2.1); }
		else { sprintf(dataHead, "%.2f,%.2f", pan, tilt); }
		sendto(sockhead, dataHead, strlen(dataHead), 0,(struct sockaddr *) &addrhead, sizeof(addrhead));
		//printf("  data head = %s\n", dataHead);
	} else { // NotePad
		FILE *fp, *outputfp;
		outputfp = fopen("../Player/HeadNote", "wb");
		fprintf(outputfp, "%.2lf,%.2lf", pan,tilt);
		fclose(outputfp);
	}

	headPan = posPan = pan;
	headTilt = posTilt = tilt;
	//printf("headPan = %.2f \t headTilt = %.2f\n", pan, tilt);
}

// Search ball ===============================================================================
double	tiltRate = -0.05,
	panRate  = -0.05,
	searchKe = 0,

	batasKanan = -1.6,
	batasKiri  =  1.6,
	batasAtas  = -2.0,
	batasBawah = -0.6;

void tiltSearchBall(double tempPosPan) { //printf("  tiltSearchBall\n\n");
	posPan = tempPosPan;
	posTilt += tiltRate;

	if (posTilt <= batasAtas || posTilt >= batasBawah) {
		tiltRate *= -1;
	}

	if	(posTilt <= batasAtas)  { posTilt = batasAtas; }
	else if	(posTilt >= batasBawah) { posTilt = batasBawah; }

	headMove(posPan, posTilt); //printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
}

void panSearchBall(double tempPosTilt) { //printf("  panSearchBall\n\n");
	posTilt = tempPosTilt;
	posPan += panRate;

	if (posPan <= batasKanan || posPan >= batasKiri) {
		panRate *= -1;
	}

	if (headPan <= (0.0 + panRate) && headPan >= (0.0 - panRate)) {
		//searchKe += 0.5;
	}

	if	(posPan >= batasKiri)  { posPan = batasKiri; }
	else if	(posPan <= batasKanan) { posPan = batasKanan; }

	headMove(posPan, posTilt); //printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
}

int	i = 1,
	panKe = 2,
	tiltKe = 1;
double	panSearch[5] = {1.6, 0.8, 0.0, -0.8, -1.6},
	//tiltSearch1[3] = {-0.6, -1.2, -1.8},
	tiltSearch1[3] = {-0.8, -1.4, -1.8},
	tiltSearch2[2] = {-0.8, -1.4};
void SearchBall(int mode) { //printf("  normalSearchBall\n\n");
	if (mode == 1) { // (atas-bawah)
		posTilt += tiltRate;
		if (posTilt <= batasAtas || posTilt >= batasBawah) {
			if (panKe == 2) { searchKe += 1; }

			tiltRate *= -1;
			panKe += i;
			if (panKe >=  4 || panKe <= 0) {
				i = -i;
			}
		} posPan = panSearch[panKe]; //printf("count pan = %d\n", panKe);

	} else if (mode == 2) { // (kiri-kanan)
		posPan += panRate;
		if (posPan <= batasKanan || posPan >= batasKiri) {
			if (tiltKe == 1) { searchKe += 1; }
			panRate *= -1;
			tiltKe += i;

			if (tiltKe >=  2 || tiltKe <= 0) {
				i = -i;
			}
		} posTilt = tiltSearch1[tiltKe]; //printf("count tilt = %d\n", tiltKe);

	} else if (mode == 3) { // muter-muter
		posPan += panRate;
		if(posPan <= batasKanan || posPan >= batasKiri) {
			panRate *= -1;
			countTilt++;
			if (countTilt > 1) countTilt = 0;
		}
		posTilt = tiltSearch2[countTilt]; //printf("count tilt = %d\n", countTilt);
	}

	if	(posPan >= batasKiri)   { posPan = batasKiri; }
	else if	(posPan <= batasKanan)  { posPan = batasKanan; }
	if	(posTilt <= batasAtas)  { posTilt = batasAtas; }
	else if	(posTilt >= batasBawah) { posTilt = batasBawah; }

	headMove(posPan, posTilt); //printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
}

int	sabar = 0,
	tiltPos = 0;
void threeSearchBall() {
	if (sabar > 7) {
		posPan += panRate;
		if (posPan <= batasKanan || posPan >= batasKiri) {
			if (tiltPos == 2 && posPan <= -1.5) { searchKe += 1; }
			panRate *= -1;
			tiltPos += i;
			if (tiltPos >=  2 || tiltPos <= 0) {
				i = -i;
			}
		} posTilt = tiltSearch1[tiltPos]; //printf("count tilt = %d\n", tiltPos);
	} else {
		posPan = 1.45;
		posTilt = -0.8;
		tiltPos = 0;
		searchKe = 0;
		i = 1;
		tiltRate = -0.05;
	        panRate  = -0.05;
		sabar++;
	}

	if	(posPan >= batasKiri)   { posPan = batasKiri; }
	else if	(posPan <= batasKanan)  { posPan = batasKanan; }
	if	(posTilt <= batasAtas)  { posTilt = batasAtas; }
	else if	(posTilt >= batasBawah) { posTilt = batasBawah; }

	headMove(posPan, posTilt); //printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
}

void rightSearchBall() { //printf("  rightSearchBall\n\n");
	posTilt += tiltRate;
	if (posTilt <= batasAtas || posTilt >= batasBawah) {
		tiltRate *= -1;
		posPan += panRate;
		if (posPan >= 0.0 || posPan <= batasKanan) {
			panRate *= -1;
		}
	}

	if	(posPan >= 0.0)   { posPan = 0.0; }
	else if	(posPan <= batasKanan)  { posPan = batasKanan; }
	if	(posTilt <= batasAtas)  { posTilt = batasAtas; }
	else if	(posTilt >= batasBawah) { posTilt = batasBawah; }

	headMove(posPan, posTilt);
}

void leftSearchBall() { //printf("  leftSearchBall\n\n");
	posTilt += tiltRate;
	if (posTilt <= batasAtas || posTilt >= batasBawah) {
		tiltRate *= -1;
		posPan += panRate;
		if (posPan >= batasKiri || posPan <= 0.0) {
			panRate *= -1;
		}
	}

	if	(posPan <= 0.0)  { posPan = 0.0; }
	else if	(posPan >= batasKiri)   { posPan = batasKiri; }
	if	(posTilt <= batasAtas)  { posTilt = batasAtas; }
	else if	(posTilt >= batasBawah) { posTilt = batasBawah; }

	headMove(posPan, posTilt);
}

double	invPan;
void searchBallPan(double uPan, double uTilt) {
	posTilt = uTilt;
	invPan = uPan*-1;

	posPan += panRate;

	if	(posPan <= invPan) { posPan = invPan; panRate *= -1; }
	else if	(posPan >= uPan  ) { posPan = uPan;   panRate *= -1; }

	headMove(posPan, posTilt);
}

bool neckX;
void searchBallRectang(double atas, double kanan, double bawah, double kiri) {
	if (neckX) {
		posPan += panRate;
		if (posPan >= kiri || posPan <= kanan) {
			panRate *= -1;
			neckX = false;
		}
	} else {
	        posTilt += tiltRate;
		if (posTilt <= atas || posTilt >= bawah) {
			tiltRate *= -1;
			neckX = true;
		}
	}

	if	(posPan >= kiri)	{ posPan = kiri; }
	else if (posPan <= kanan)	{ posPan = kanan; }
	if	(posTilt <= atas)	{ posTilt = atas; }
	else if (posTilt >= bawah)	{ posTilt = bawah; }

	headMove(posPan, posTilt); //printf("pan = %f, tilt = %f\n",posPan,posTilt);
}

double	ballPan = 0,
	ballTilt = 0;
void saveBallLocation() {
//	trackBall();
	ballPan = posPan;
	ballTilt = posTilt;
}

void loadBallLocation(double tilt) {
	//posPan = 0.0;
	//posTilt = -0.8;
	posPan = ballPan;
	posTilt = ballTilt + tilt;
	headMove(posPan, posTilt);
}

int	semeh;
int koordinasiJarak() {
	semeh = (int)(headTilt*-100);
	return semeh;
}

// Ball Tracking =============================================================================
double	intPanB = 0, dervPanB = 0, errorPanB = 0, preErrPanB = 0,
	PPanB = 0, IPanB = 0, DPanB = 0,
	intTiltB = 0, dervTiltB = 0, errorTiltB = 0, preErrTiltB = 0,
	PTiltB = 0, ITiltB = 0, DTiltB = 0,
	dtB = 0.04;
double	B_Pan_err_diff, B_Pan_err, B_Tilt_err_diff, B_Tilt_err, B_PanAngle, B_TiltAngle,
	pOffsetB, iOffsetB, dOffsetB,
	errorPanBRad, errorTiltBRad,
	offsetSetPointBall;
void trackBall() {
	if (useVision) {
		if (receler.Ball_X != -1 && receler.Ball_Y != -1) { //printf("Tracking");
			ballPos_X = receler.BallCoor_X;
			ballPos_Y = receler.BallCoor_Y;
			//mode 1 ######################################################################
				/*// PID pan ==========================================================
				errorPanB  = (double)Ball_x - (frame_X / 2);//160
				PPanB  = errorPanB  * 0.00010; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya

				intPanB += errorPanB * dtB;
				IPanB = intPanB * 0.0;

				dervPanB = (errorPanB - preErrPanB) / dtB;
				DPanB = dervPanB * 0.00001;

				preErrPanB = errorPanB;

				//posPan += PPanB*-1; //dikali -1 kalau receler terbalik dalam pemasangan
				posPan += (PPanB + IPanB + DPanB) * -1;


				// PID tilt ==========================================================
				errorTiltB = (double)Ball_y - (frame_Y / 2);//120
				PTiltB = errorTiltB * 0.00010; // Tune in Kp Tilt 0.00030

				intTiltB += errorTiltB * dtB;
				ITiltB = intTiltB * 0.0;

				dervTiltB = (errorTiltB - preErrTiltB) / dtB;
				DTiltB = dervTiltB * 0.00001;

				preErrTiltB = errorTiltB;

				//posTilt += PTiltB;
				posTilt += (PTiltB + ITiltB + DTiltB);*/

			//mode 2 ######################################################################
				offsetSetPointBall = (int)((posTilt * 30) + 54);
				if (offsetSetPointBall > 36) offsetSetPointBall = 36;
				else if (offsetSetPointBall < 0) offsetSetPointBall = 0;

				errorPanB  = (double)receler.Ball_X - ((frame_X / 2) + offsetSetPointBall);//160
				errorTiltB = (double)receler.Ball_Y - (frame_Y / 2);//120
				errorPanB *= -1;
				errorTiltB *= -1;
				errorPanB *= (90 / (double)frame_X); // pixel per angle
				errorTiltB *= (60 / (double)frame_Y); // pixel per angle
				//errorPanB *= (77.32 / (double)frame_X); // pixel per angle
				//errorTiltB *= (61.93 / (double)frame_Y); // pixel per angle

				errorPanBRad = (errorPanB * PI)/ 180;
				errorTiltBRad = (errorTiltB * PI) /180;
				//printf("errorPan = %.2f \t errorTilt = %.2f\n", errorPanB, errorTiltB);
				//printf("RadrrorPan = %.2f \t RaderrorTilt = %.2f\n", errorPanBRad, errorTiltBRad);
				//printf("KPPan = %f \t KDPan = %f\t", kamera.panKP, kamera.panKD); printf("KPTilt = %f \t KDTilt = %f\n", kamera.tiltKP, kamera.tiltKD);

				B_Pan_err_diff = errorPanBRad - B_Pan_err;
				B_Tilt_err_diff = errorTiltBRad - B_Tilt_err;

				// PID pan ==========================================================
				//PPanB  = B_Pan_err  * kamera.panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
				PPanB  = B_Pan_err  * ball_panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
				intPanB += B_Pan_err * dtB;
				IPanB = intPanB * 0.0;
				dervPanB = B_Pan_err_diff / dtB;
				//DPanB = dervPanB * kamera.panKD;
				DPanB = dervPanB * ball_panKD;
				B_Pan_err = errorPanBRad;
				posPan += (PPanB + IPanB + DPanB);

				// PID tilt ==========================================================
				//PTiltB = B_Tilt_err * kamera.tiltKP; // Tune in Kp Tilt 0.00030
				PTiltB = B_Tilt_err * ball_tiltKP; // Tune in Kp Tilt 0.00030

				intTiltB += B_Tilt_err * dtB;
				ITiltB = intTiltB * 0.0;

				dervTiltB = B_Tilt_err_diff / dtB;
				//DTiltB = dervTiltB * kamera.tiltKD;
				DTiltB = dervTiltB * ball_tiltKD;

				preErrTiltB = errorTiltB;
				B_Tilt_err = errorTiltBRad;
				posTilt += (PTiltB + ITiltB + DTiltB) * -1;


			if      (posPan  >=  1.6)  { posPan  =  1.6; }
			else if (posPan  <= -1.6)  { posPan  = -1.6; }
			if      (posTilt <= -2.0)  { posTilt = -2.0; }
			else if (posTilt >= -0.4)  { posTilt = -0.4; }

			headMove(posPan, posTilt); //printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);

			koordinasiJarak();
			saveBallLocation();
		}
	}
}

// Body Tracking Ball ========================================================================
double	errorBodyPosition,
	bodyP_Controller;
int	bodyTrue = 0,
	delayTrue = 0;
int bodyTrackingBall(int threshold) {
	errorBodyPosition = 0 - headPan;
	bodyP_Controller = errorBodyPosition * -0.5;//-0.5

	if (ballLost(20)) {
		Walk(0.0, 0.0, 0.0);
		bodyP_Controller = bodyTrue = delayTrue = 0;
	} else {
		if (errorBodyPosition >= -0.3 && errorBodyPosition <= 0.3) {//untuk hasil hadap 0.8
			//motion("0");
			Walk(0.0, 0.0, 0.0);
			delayTrue++;
		} else {
			trackBall();
			//motion("9");

			if (bodyP_Controller < 0) { bodyP_Controller = -0.2;  } //kanan 0.15
			else { bodyP_Controller = 0.2; } //kiri 0.15

			bodyTrue = delayTrue = 0;
			Walk(0.0, 0.0, bodyP_Controller);
		}

		if (delayTrue >= threshold) {
			bodyTrue = 1;
		} else {
			bodyTrue = 0;
		}
	} //printf("Body Error = %.2f\t Body P Controller = %.2f\n",errorBodyPosition,bodyP_Controller);
	return bodyTrue;
}

// Hitung Jarak Bola berdasarkan headTilt ==============================================================
double	alphaY,		//hasil derajat ketika headTilt
	betaY,
	inputY,		//nilai realtime headTilt

	alphaX,		//hasil derajat ketika headPan
	betaX,
	inputX,		//nilai realtime headPan

	jarakBola_Y,	//hasil jarak(cm) dari kalkulasi headTilt
	jarakBola_X,	//hasil jarak(cm) dari kalkulasi headPan
	jarakBola;

void kalkulasiJarakBola() {
	inputY	= headTilt;
	inputX	= headPan;

	//alphaY = -57.29 * headTilt; //(metode 1)
	alphaY = outputSudutY1 + ((outputSudutY2 - outputSudutY1) / (inputSudutY2 - inputSudutY1)) * (inputY - inputSudutY1);  //(metode 2) //printf("  alphaY = %.2f,", alphaY);
	betaY = 180 - (90 + alphaY);  //printf("  betaY = %.2f,", betaY);

	//alphaX = -57.29 * headPan; //(metode 1)
	alphaX = outputSudutX1 + ((outputSudutX2 - outputSudutX1) / (inputSudutX2 - inputSudutX1)) * (inputX - inputSudutX1);  //(metode 2) //printf("  alphaX = %.2f,", alphaX);
	betaX = 180 - (90 + alphaX);  //printf("  betaX = %.2f,", betaX);

	//sin & cos dalam c++ adalah radian, oleh karena itu harus : sin( ... * PI / 180)
	jarakBola_Y = (tinggiRobot * sin(alphaY * PI / 180)) / sin(betaY * PI / 180);  //printf("  jarakBola_Y = %.f,", jarakBola_Y);
	jarakBola_X = (jarakBola_Y * sin(alphaX * PI / 180)) / sin(betaX * PI / 180);  //printf("  jarakBola_X = %.f\n\n\n\n", jarakBola_X);
	jarakBola = sqrt((jarakBola_Y * jarakBola_Y) + (jarakBola_X * jarakBola_X));

	//regresi (metode 3)
	//jarakBola_Y = (-933.9*(pow(posTilt,5))) + (-5340.8*(pow(posTilt,4))) + (-12018*(pow(posTilt,3))) + (-13183*(pow(posTilt,2))) + (-7050.2*posTilt) - 1454.3;
}

// Untuk Kalkulasi Posisi P1
double 	P1_X, P1_Y;
void hitungKoordinatBolaP1() {
	//mode1--------------
		//kalkulasiJarakBola();
		//P1_X = jarakBola_X;
		//P1_Y = jarakBola_Y;
	//mode2--------------
		P1_X = receler.Ball_Y;
		P1_Y = receler.Ball_X;
	//printf("  P1_X = %.2f,  P1_Y = %.2f,", P1_X, P1_Y);
}

// Untuk Kalkulasi Posisi P2
double 	P2_X, P2_Y;
void hitungKoordinatBolaP2() {
	//mode1--------------
		//kalkulasiJarakBola();
		//P2_X = jarakBola_X;
		//P2_Y = jarakBola_Y;
	//mode2--------------
		P2_X = receler.Ball_Y;
		P2_Y = receler.Ball_X;
	//printf("  P2_X = %.2f,  P2_Y = %.2f,", P2_X, P2_Y);
}

// Untuk penyebut dan pembilang gradient
double 	deltaY, deltaX;
void hitungDeltaY() {
	deltaY = P2_Y - P1_Y; //mendekat positif
	//deltaY = P1_Y - P2_Y; //mendekat positif
	//printf("  deltaY = %.2f,", deltaY);
}

void hitungDeltaX() {
	deltaX = P2_X - P1_X; //mendekat negatif
	//deltaX = P1_X - P2_X; //mendekat positif
	//printf("  deltaX = %.2f,\n\n", deltaX);
}

// Untuk Gradient
double 	gradient;
void hitungGradient() {
	gradient = deltaY / deltaX;
	//printf("  gradient = %.2f,", gradient);
}

int	cntOke1,
	cntOke2,
	cntUlang,
	kondisiBola = 0;
bool	oke     = true,
	ulang   = false;
void hitungGerakBola() { // Bagian pengecekan pergerakan bola
	if (receler.Ball_X == -1 && receler.Ball_Y == -1) { //bola hilang
		deltaY		=
		deltaX		=
		jarakBola_X	=
		jarakBola_Y	= 0;
	}

	if (ulang) { //printf("ulang \n");
		cntOke1 =
		cntOke2 = 0;
		if (cntUlang > 5) {
			ulang = false;
			oke = true;
		} else {
			deltaY		=
			deltaX		=
			jarakBola_X	=
			jarakBola_Y	= 0;

			cntUlang++;
		}
	}

	if (oke) { //printf("oke \n");
		cntUlang = 0;
		if (cntOke1 > 5) {
			hitungKoordinatBolaP2();

			hitungDeltaY();
			hitungDeltaX();
			//hitungGradient();

			if (cntOke2 > 10) {
				oke = false;
				ulang = true;
			} else {
				cntOke2++;
			}
		} else {
			hitungKoordinatBolaP1();
			cntOke1++;
		}
	}

	//if ((deltaY >= 20 && deltaX <= -20) || (deltaY >= 20 && deltaX >= 20)) { //0.5 //7.0
	if (deltaY >= 30) { //0.5 //7.0
		//printf("  deltaY = %.f,", deltaY);
		//printf("  Bola Menjauh\n");
		kondisiBola = 1;
	//} else if((deltaY <= -20 && deltaX <= -20) || (deltaY <= -20 && deltaX >= 20)) { //-2 //-1.4
	} else if(deltaY <= -30) { //-2 //-1.4
		//printf("  deltaY = %.f,", deltaY);
		//printf("  Bola Mendekat\n");
		kondisiBola = -1;
	} else if(deltaY >= -5 && deltaY <= 5 && deltaX >= -5 && deltaX <= 5) { //-2 //-1.4
		//printf("  Bola Diam");
		kondisiBola = 0;
	}
}

// Follow Ball ===============================================================================
int	countReadyKick;
double	SetPointPan = 0,
	SetPointTilt = -0.8,//-0.08
	errorfPan,
	errorfTilt,
	PyMove = 0,
	PxMove = 0,
	PaMove = 0;
void followBall(int mode){ //0 normal, 1 sambil belok
	trackBall();

	if	(posTilt >= SetPointTilt) { posTilt = SetPointTilt; }
	else if	(posTilt < -2.0) { posTilt = -2.0; }

	errorfPan  = posPan - SetPointPan;
	errorfTilt = posTilt - SetPointTilt;

	if (posTilt >= SetPointTilt && posPan < 0.4 && posPan > -0.4 && receler.Ball_X != -1 && receler.Ball_Y != -1) { //Stop(bola sudah dekat)
		countReadyKick++;
	} else { //Kejar Bola(bola masih jauh)
		countReadyKick = 0;
	}

	if (countReadyKick >= 1) { //5
		PxMove = 0.0; //jalan ditempat
		PyMove = errorfPan * 0.040; //0.045
		PaMove = errorfPan * 0.20; //0.30; //0.045
	} else {
		if (headTilt < -1.5) {
			PxMove = kejarMax; //0.08
		} else if (headTilt >= -1.5 && headTilt < -1.4) {
			PxMove = kejarMid; //0.07
		} else if (headTilt > -1.0) {
			PxMove = lari; //0.05
		} else {
			PxMove = kejar; //0.06
		}
		//PxMove = errorfTilt * 0.1 * -13; //Robot besar 0.13, robot kecil 0.1
		//PxMove = 0.06 / -1.6 * posTilt; //0.04-0.06
		PyMove = errorfPan * 0.40;//0.125; //0.045
		PaMove = errorfPan * 0.30;//0.25; //0.35; //0.045
	}

	if (mode == 0) { // Mode differential walking
		if (errorfPan > -0.4 && errorfPan < 0.4) { 	//printf("AAAAAAAA\n");
			Walk(PxMove, 0.0, PaMove);
		} else { 					//printf("BBBBBBBB\n");
			Walk(0.0, 0.0, PaMove);
		}
	}
	else if (mode == 1) { // Mode omnidirectional walking
		if (errorfPan > -0.4 && errorfPan < 0.4) { 	//printf("CCCCCCCC\n");
			Walk(PxMove, PyMove, PaMove);
		} else { 					//printf("DDDDDDDD\n");
			Walk(0.0, 0.0, PaMove);
		}
	}
}

// Get Imu Value =================================================================
void sudut() {
	angle = strategy.bearingValue;

	odometry.RobotAngle = angle;
	if (angle > 0){
		sendAngle = angle;
	} else if (angle < 0){
		sendAngle = angle + 360;
	} else if (angle == 0){
		sendAngle =  0;
	}
	//printf("  Sudut = %.f", angle);
}

// IMU ===========================================================================
int	setPoint1,
	setPoint2;

double	errorCPosPan,
	errorCPosTilt,
	alfaImu,
	bodyYImu,
	bodyXImu;

//	X+ = maju
//	X- = mundur
//	Y+ = samping kiri
//	Y- = samping kanan
//	A+ = putar kiri
//	A- = putar kanan

void rotateDirec(int arah, double jarakTilt) {
	errorCPosPan = posPan;// adalah nilai tengah pan, dan menjadi titik berhenti jika telah tepenuhi
	errorCPosTilt = posTilt - (jarakTilt);//-0.45 adalah nilai tengah tilt, robot akan jalan ditempat(tidak maju/mundur) jika nilai terpenuhi

	bodyXImu 	= errorCPosTilt * (-0.1);	//nilai pengali ini harus tetap bernilai negatif //besarnya kalkulasi maju/mundur yg dibutuhkan tehadap posTilt
	bodyYImu 	= abs(errorCPosPan/100) + 0.017;//0.017;
	alfaImu 	= errorCPosPan * 0.7;		//0.7; nilai pengali ini harus tetap bernilai positif //besarnya kalkulasi rotate yg dibutuhkan tehadap posPan

	if (arah <= 0) { //rotate ke kanan
		//alfaImu = -0.15; bodyYImu = 0.017;
		if (bodyYImu < 0) { bodyYImu = -bodyYImu; }
	} else { //rotate ke kiri
		//alfaImu = 0.15; bodyYImu = -0.017;
		if (bodyYImu > 0) { bodyYImu = -bodyYImu; }
	}
}

void rotateParabolic(int arah, double jarakTilt) {
	rotateDirec(arah, jarakTilt);
	Walk(bodyXImu, bodyYImu, alfaImu);
}

bool	robotDirection = false;
int	eleh = 0,
	btsRotate = 0,
	btsSetPoint = 0;
void Imu(int gawang, double jarakTilt) {// opsi 2, menggunakan "mode" untuk mengecek robot direction
	trackBall();

	if (gawang > 180) { gawang = 180; }
	else if (gawang < -180) { gawang = -180; }

	sudut();

	setPoint1 =  10 + gawang;//10 //15 //20 //60
	setPoint2 = -10 + gawang;//10 //15 //20 //60

	if (setPoint1 > 180 || setPoint2 < -180) { // jika arah imu dibelakang
		if (setPoint1 > 180) { // nilai setpoint1 diubah jd negatif
			btsSetPoint = setPoint1 - 360;

			if (angle >= setPoint2 || angle <= btsSetPoint) {
				bodyXImu = bodyYImu = alfaImu = 0.0;
				if (eleh > 10) { robotDirection = true;	}
				else { eleh++; }
			} else  {
				if (gawang >= 0) {
					btsRotate = gawang - 180;
					if ((angle <= gawang) && (angle >= btsRotate)) {
						rotateDirec(-1, jarakTilt); //printf("  rotate ke kanan\n\n");
					} else {
						rotateDirec(1, jarakTilt); //printf("  rotate ke kiri\n\n");
					}
				} else {
					btsRotate = gawang + 180;
					if ((angle >= gawang) && (angle <= btsRotate)) {
						rotateDirec(1, jarakTilt); //printf("  rotate ke kiri\n\n");
					} else {
						rotateDirec(-1, jarakTilt); //printf("  rotate ke kanan\n\n");
					}
				}

				eleh = 0;
				robotDirection = false;
			} Walk(bodyXImu, bodyYImu, alfaImu);
		} else { // nilai setPoint2 diubah jadi positif
			btsSetPoint = setPoint2 + 360;

			if (angle >= btsSetPoint || angle <= setPoint1) {
				bodyXImu = bodyYImu = alfaImu = 0.0;
				if (eleh > 10) { robotDirection = true;	}
				else { eleh++; }
			} else {
				if (gawang >= 0) {
					btsRotate = gawang - 180;
					if ((angle <= gawang) && (angle >= btsRotate)) {
						rotateDirec(-1, jarakTilt); //printf("  rotate ke kanan\n\n");
					} else {
						rotateDirec(1, jarakTilt); //printf("  rotate ke kiri\n\n");
					}
				} else {
					btsRotate = gawang + 180;
					if ((angle >= gawang) && (angle <= btsRotate)) {
						rotateDirec(1, jarakTilt); //printf("  rotate ke kiri\n\n");
					} else {
						rotateDirec(-1, jarakTilt); //printf("  rotate ke kanan\n\n");
					}
				}

				eleh = 0;
				robotDirection = false;
			} Walk(bodyXImu, bodyYImu, alfaImu);
		}
	} else { // arah imu kedepan
		if (angle >= setPoint2 && angle <= setPoint1) {
			bodyXImu = bodyYImu = alfaImu = 0.0;
			if (eleh > 10) { robotDirection = true;	}
			else { eleh++; }
		} else {
			if (gawang >= 0) {
				btsRotate = gawang - 180;
				if ((angle <= gawang) && (angle >= btsRotate)) {
					rotateDirec(-1, jarakTilt); //printf("  rotate ke kanan\n\n");
				} else {
					rotateDirec(1, jarakTilt); //printf("  rotate ke kiri\n\n");
				}
			} else {
				btsRotate = gawang + 180;
				if ((angle >= gawang) && (angle <= btsRotate)) {
					rotateDirec(1, jarakTilt); //printf("  rotate ke kiri\n\n");
				} else {
					rotateDirec(-1, jarakTilt); //printf("  rotate ke kanan\n\n");
				}
			}

			eleh = 0;
			robotDirection = false;
		} Walk(bodyXImu, bodyYImu, alfaImu);
	}
}

// Grid Coordinate ================================================================
double outGrid;
void gridCoor(){
	if (Grid >= 1 && Grid <= 12){ outGrid = 0; }
	else if (Grid >= 13 && Grid <= 42){ outGrid = odometry.ArchSinEnemy; }
	else if (Grid == 43) { outGrid = 70; }
	else if (Grid == 44) { outGrid = 45; }
	else if (Grid == 45) { outGrid = 25; }
	else if (Grid == 46) { outGrid = -25; }
	else if (Grid == 47) { outGrid = -45; }
	else if (Grid == 48) { outGrid = -70; }
	else if (Grid == 49) { outGrid = 75; }
	else if (Grid == 50) { outGrid = 60; }
	else if (Grid == 51) { outGrid = 20; }
	else if (Grid == 52) { outGrid = -20; }
	else if (Grid == 53) { outGrid = -60; }
	else if (Grid == 54) { outGrid = -75; }

	//untuk kondisi followSearchGoal
	if (Activated) {
		if (Grid >= 43 && Grid <= 54) followSearchAktif = true;
		else followSearchAktif = false;
	}
}

// Rotate Body with IMU ===========================================================
bool	posRotate = false;

void rotateBodyImu(int rotate) {
	trackBall();

	sudut();

	setPoint1 =  5 + rotate; //20 bisa kemungkinan 180 keatas
	setPoint2 = -5 + rotate; //20 bisa kemungkinan -180 bawah

	if (angle > setPoint2 && angle < setPoint1 ) {
		posRotate = true;
	} else {
		if (angle > rotate) { //putar Kiri
			Walk(0.0, 0.0, 0.2);
		}  else { //putar Kanan
			Walk(0.0, 0.0, -0.2);
		}
		//posRotate = false;
	}
}

// Rotate Body with IMU ===========================================================
bool	posRotateNew = false;
double	IaMove, errorImuNew,
	putar;
void rotateBodyImuNew(int rotate) {
	trackBall();

	sudut();

	setPoint1 =  5 + rotate;//10 //15 //20 //60
	setPoint2 = -5 + rotate;//10 //15 //20 //60

	if (setPoint1 > 180 || setPoint2 < -180) { // jika arah imu dibelakang
		if (setPoint1 > 180) { // nilai setpoint1 diubah jd negatif
			btsSetPoint = setPoint1 - 360;

			if (angle >= setPoint2 || angle <= btsSetPoint) { //misal 170 ke -170
				PaMove = 0.00;
				posRotateNew = true;
			} else  {
				btsRotate = rotate - 180;
				if ((angle <= rotate) && (angle >= btsRotate)) { //misal di range 0 - 180, maka putar kanan
					putar = (rotate - angle) * 0.0065; //0.0033
					PaMove = -putar;
				} else { //putar kiri
					if (angle > rotate) { putar = (angle - rotate) * 0.0065; } //0.0033
					else { putar = ((180 - rotate) + (180 + angle)) * 0.0065; } //0.0033
					PaMove = putar;
				}
			}
		} else { // nilai setPoint2 diubah jadi positif
			btsSetPoint = setPoint2 + 360;

			if (angle >= btsSetPoint || angle <= setPoint1) {
				PaMove = 0.00;
				posRotateNew = true;
			} else {
				btsRotate = rotate + 180;
				if ((angle >= rotate) && (angle <= btsRotate)) { //misal di range -180 - 0, maka putar kiri
					putar = abs(rotate - angle) * 0.0065; //0.0033
					PaMove = putar;
				} else { //putar kanan
					if (angle < rotate) { putar = (rotate - angle) * 0.0065; } //0.0033
					else { putar = ((180 + rotate) + (180 - angle)) * 0.0065; } //0.0033
					PaMove = -putar;
				}
			}
		}
	} else { // arah imu kedepan
		if (angle >= setPoint2 && angle <= setPoint1) {
			PaMove = 0.00;
			posRotateNew = true;
		} else {
			if (rotate >= 0) {
				btsRotate = rotate - 180;
				if ((angle <= rotate) && (angle >= btsRotate)) { //putar kanan
					putar = (rotate - angle) * 0.0065; //0.0033
					PaMove = -putar;
				} else { //putar kiri
					if (angle > rotate) { putar = (angle - rotate) * 0.0065; } //0.0033
					else { putar = ((180 - rotate) + (180 + angle)) * 0.0065; } //0.0033
					PaMove = putar;
				}
			} else {
				btsRotate = rotate + 180;
				if ((angle >= rotate) && (angle <= btsRotate)) {//maka putar kiri
					putar = abs(rotate - angle) * 0.0065; //0.0033
					PaMove = putar;
				} else { //putar kanan
					if (angle < rotate) { putar = (rotate - angle) * 0.0065; } //0.0033
					else { putar = ((180 + rotate) + (180 - angle)) * 0.0065; } //0.0033
					PaMove = -putar;
				}
			}
		}
	}
	if(!posRotateNew){
		Walk(0.0,0.0,PaMove);
	}

}

//	A+ = putar kiri
//	A- = putar kanan
void jalanDirection(double Xwalk, double Ywalk, double rotate) {
	if (rotate > 180) { rotate = 180; }
	else if (rotate < -180) { rotate = -180; }
	//-175 sampai -185
	setPoint1 =  5 + rotate;//10 //15 //20 //60
	setPoint2 = -5 + rotate;//10 //15 //20 //60

	if (setPoint1 > 180 || setPoint2 < -180) { // jika arah imu dibelakang
		if (setPoint1 > 180) { // nilai setpoint1 diubah jd negatif
			btsSetPoint = setPoint1 - 360;

			if (angle >= setPoint2 || angle <= btsSetPoint) { //misal 170 ke -170
				PaMove = 0.00;
			} else  {
				btsRotate = rotate - 180;
				if ((angle <= rotate) && (angle >= btsRotate)) { //misal di range 0 - 180, maka putar kanan
					putar = (rotate - angle) * 0.0065; //0.0033
					PaMove = -putar;
				} else { //putar kiri
					if (angle > rotate) { putar = (angle - rotate) * 0.004; } //0.0033
					else { putar = ((180 - rotate) + (180 + angle)) * 0.004; } //0.0033
					PaMove = putar;
				}
			}
		} else { // nilai setPoint2 diubah jadi positif
			btsSetPoint = setPoint2 + 360;

			if (angle >= btsSetPoint || angle <= setPoint1) {
				PaMove = 0.00;
			} else {
				btsRotate = rotate + 180;
				if ((angle >= rotate) && (angle <= btsRotate)) { //misal di range -180 - 0, maka putar kiri
					putar = abs(rotate - angle) * 0.004; //0.0033
					PaMove = putar;
				} else { //putar kanan
					if (angle < rotate) { putar = (rotate - angle) * 0.004; } //0.0033
					else { putar = ((180 + rotate) + (180 - angle)) * 0.004; } //0.0033
					PaMove = -putar;
				}
			}
		}
	} else { // arah imu kedepan
		if (angle >= setPoint2 && angle <= setPoint1) {
			PaMove = 0.00;
		} else {
			if (rotate >= 0) {
				btsRotate = rotate - 180;
				if ((angle <= rotate) && (angle >= btsRotate)) { //putar kanan
					putar = (rotate - angle) * 0.004; //0.0033
					PaMove = -putar;
				} else { //putar kiri
					if (angle > rotate) { putar = (angle - rotate) * 0.004; } //0.0033
					else { putar = ((180 - rotate) + (180 + angle)) * 0.004; } //0.0033
					PaMove = putar;
				}
			} else {
				btsRotate = rotate + 180;
				if ((angle >= rotate) && (angle <= btsRotate)) {//maka putar kiri
					putar = abs(rotate - angle) * 0.004; //0.0033
					PaMove = putar;
				} else { //putar kanan
					if (angle < rotate) { putar = (rotate - angle) * 0.004; } //0.0033
					else { putar = ((180 + rotate) + (180 - angle)) * 0.004; } //0.0033
					PaMove = -putar;
				}
			}
		}
	}

	if (PaMove > 0.3) { PaMove = 0.3; }
	else if (PaMove < -0.3) { PaMove = -0.3; }

	Walk(Xwalk, Ywalk, PaMove);
}

// Ball Positioning Using P Controller =======================================================
double	errorPosX,
	errorPosY,
	PxMoveBallPos,
	PyMoveBallPos,
	PaMoveBallPos;
bool	ballPos = false;
void ballPositioning(double setPointX, double setPointY, double speed) {
	errorPosX = headPan - setPointX;
	errorPosY = headTilt - setPointY;

	if ((errorPosX > -0.10 && errorPosX < 0.10) && (errorPosY > -0.10)) { //&& errorPosY < 0.10)) { //sudah sesuai
		PyMoveBallPos = 0.00;
		PxMoveBallPos = 0.00;
		ballPos = true;
	} else { //belum sesuai
		ballPos = false;
		if ((headPan >= 1.0 && headTilt >= -1.2) || (headPan <= -1.0 && headTilt >= -1.2)) { //bola disamping //pan tilt kircok (polar)
			PxMoveBallPos = -0.03;
			PyMoveBallPos = errorPosX * 0.08;//0.12;
		} else {
			//Xmove
			if (headTilt > setPointY) { //> (setPointY + 0.1)) { //kelebihan
				PxMoveBallPos = -0.03;
			} else if (headTilt >= (setPointY - 0.1) && headTilt <= setPointY) { //<= (setPointY + 0.1)) { //sudah dalam range
				PxMoveBallPos = 0.00;
			} else if (headTilt >= (setPointY - 0.3) && headTilt < (setPointY - 0.1)) { //bola sudah dekat
				PxMoveBallPos = errorPosY * -speed;
				if (PxMoveBallPos >= 0.015) { PxMoveBallPos = 0.015; }
				else if (PxMoveBallPos <= 0.00) { PxMoveBallPos = 0.00; }
			} else { //bola masih jauh
				PxMoveBallPos = headTilt * (0.08 / -1.6); //0.05
			}

			//Ymove
			if (headTilt >= (setPointY - 0.03)) { //> (setPointY + 0.1)) { //kelebihan
				PyMoveBallPos = 0.00;
			} else {
				if (headPan >= (setPointX - 0.1) && headPan <= (setPointX + 0.1)) { //sudah dalam range
					PyMoveBallPos = 0.00;
				} else { //belum dalam range
					PyMoveBallPos = errorPosX * 0.08;//0.08;//0.12;
				}
			}
		}
	} Walk(PxMoveBallPos, PyMoveBallPos, 0.0);
}

// Dribble Ball ======================================================================
int	bawaBola;
double	setPointFootY, setPointFootY1, setPointFootY2;
void dribble(int gawang, double speed) {
	trackBall();

	sudut();

	//setPoint1 =  20 + gawang;//20
	//setPoint2 = -20 + gawang;//20

	//if (angle >= setPoint2 && angle <= setPoint1) {
	//	robotDirection = true;
	//} else  { robotDirection = false; }

	//if (robotDirection) { //printf("arah imu sudah benar\n");
		//if (headTilt <= -0.7 || bawaBola >= 300) {
		//	bawaBola = 0;
		//	robotDirection = false;
		//	stateCondition = 5;
		//} bawaBola++;

		// Ball Positioning ======================================
		//if (posPan >= 0) { setPointFootY = 0.25;  }//kiri
		//else             { setPointFootY = -0.25; }//kanan
		//errorPosX = headPan - setPointFootY;

		setPointFootY1 =  0.2; //0.22
		setPointFootY2 = -0.2; //-0.22
		errorPosY = headTilt + 0.5;//0.04;//0.05;//0.08;

		//if(errorPosX >= -0.1 && errorPosX <= 0.1) {
		if(headPan <= setPointFootY1 && headPan >= setPointFootY2) { // -0.2 > x < 0.2
			//if (headTilt >= -0.8) {
				PxMoveBallPos = 0.3*speed;
			//} else {
			//	PxMoveBallPos = errorPosY*speed*-1;
			//}
			Walk(PxMoveBallPos, 0.0, 0.0);
		} else { // x < -0.2 || x > 0.2
			if (headTilt >= pTiltTendang) {
				PxMoveBallPos = errorPosY * -speed;
				//PxMoveBallPos = -0.02;//-0.03;
				//Walk(-0.03, 0.0, 0.0);
			} else {
				PxMoveBallPos = 0.0;
			}

			if (headPan > setPointFootY1) { //printf("kiri bos\n");
				PyMoveBallPos = (headPan - 0.1) * 0.06;
			} else if (headPan < setPointFootY2) { //printf("kanan bos\n");
				PyMoveBallPos = (headPan + 0.1) * 0.06;
			}

			Walk(PxMoveBallPos, PyMoveBallPos, 0.0);

			//if (headTilt >= (cSekarang - 0.2)) {
			//	Walk(-0.03, 0.0, 0.0);
			//} else {
			//	if (headPan > setPointFootY1) { //printf("kiri bos\n");
			//		PyMoveBallPos = (headPan - 0.1) * 0.06;
			//		Walk(0.0, PyMoveBallPos, 0.0);
			//	} else if (headPan < setPointFootY2) { //printf("kanan bos\n");
			//		PyMoveBallPos = (headPan + 0.1) * 0.06;
			//		Walk(0.0, PyMoveBallPos, 0.0);
			//	}
			//}

			//if(errorPosY >= -0.1) { // XmoveBackWard
			//	Walk(-0.03, 0.0, 0.0);
			//} else { // Ymove
			//	PyMoveBallPos = errorPosX*0.06;
			//	Walk(0.0, PyMoveBallPos, 0.0);
			//}
		}
	//} else { //printf("cari arah imu\n");
	//	bawaBola = 0;
	//	if (posTilt > -0.8 && posPan > -0.5 && posPan < 0.5) {//bola dekat
	//		Imu(gawang, cSekarang);
	//	} else {//bola masih jauh
	//		followBall(0);
	//	}
	//}
}

// Checking Lost Goal ========================================================================
int	countGoalLost  = 0,
	countGoalFound = 0,
	returnGoalVal;
int goalLost(int threshold) {
	if (useVision) {
		if (receler.Goal_X == -1 && receler.Goal_Y == -1) {
			countGoalFound = 0;
			countGoalLost++;
			if (countGoalLost >= threshold) {
				returnGoalVal = 1;
			}
		} else {
			if (headTilt < -1.0) {
				countGoalLost = 0;
				countGoalFound++;
				if (countGoalFound > 1) {
					returnGoalVal = 0;
				}
			}
		}
	} else {
		countGoalFound = 0;
		countGoalLost++;
		if (countGoalLost >= threshold) {
			returnGoalVal = 1;
		}
	} return returnGoalVal;
}

// Goal Tracking =============================================================================
double	intPanG = 0, dervPanG = 0, errorPanG = 0, preErrPanG = 0,
	PPanG = 0, IPanG = 0, DPanG = 0,
	intTiltG = 0, dervTiltG = 0, errorTiltG = 0, preErrTiltG = 0,
	PTiltG = 0, ITiltG = 0, DTiltG = 0,
	dtG = 0.04;
double	G_Pan_err_diff, G_Pan_err, G_Tilt_err_diff, G_Tilt_err, G_PanAngle, G_TiltAngle,
	pOffsetG, iOffsetG, dOffsetG,
	errorPanGRad, errorTiltGRad;
int	offsetSetPointGoal;
void trackGoal() {
	if (useVision) {
		if (receler.Goal_X != -1 && receler.Goal_Y != -1) { //printf("Tracking");
			//mode 1 ######################################################################
				//PID pan ==========================================================
				/*errorPanG  = (double)receler.Goal_X - (frame_X / 2);//160
				PPanG  = errorPanG  * 0.00010; //Tune in Kp Pan  0.00035 //kalau kepala msh goyang2, kurangin nilainya

				intPanG += errorPanG * dtG;
				IPanG = intPanG * 0.0;

				dervPanG = (errorPanG - preErrPanG) / dtG;
				DPanG = dervPanG * 0.00001;

				preErrPanG = errorPanG;

				//posPan += PPanG*-1; //dikali -1 kalau receler terbalik dalam pemasangan
				posPan += (PPanG + IPanG + DPanG) * -1;


				//PID tilt ==========================================================
				errorTiltG = (double)receler.Goal_Y - (frame_Y / 2);//120
				PTiltG = errorTiltG * 0.00010; //Tune in Kp Tilt 0.00030

				intTiltG += errorTiltG * dtG;
				ITiltG = intTiltG * 0.0;

				dervTiltG = (errorTiltG - preErrTiltG) / dtG;
				DTiltG = dervTiltG * 0; //0.00001;

				preErrTiltG = errorTiltG;

				//posTilt += PTiltG;
				posTilt += (PTiltG + ITiltG + DTiltG);*/

			//mode 2 ######################################################################
				offsetSetPointGoal = (int)((posTilt * 30) + 54);
				if (offsetSetPointGoal > 36) offsetSetPointGoal = 36;
				else if (offsetSetPointGoal < 0) offsetSetPointGoal = 0;

				errorPanG  = (double)receler.Goal_X - ((frame_X / 2) + offsetSetPointGoal);//160
				errorTiltG = (double)receler.Goal_Y - (frame_Y / 2);//120
				errorPanG *= -1;
				errorTiltG *= -1;
				errorPanG *= (90 / (double)frame_X); // pixel per angle
				errorTiltG *= (60 / (double)frame_Y); // pixel per angle
				errorPanG *= (77.32 / (double)frame_X); // pixel per angle
				errorTiltG *= (61.93 / (double)frame_Y); // pixel per angle

				errorPanGRad = (errorPanG * PI)/ 180;
				errorTiltGRad = (errorTiltG * PI) /180;
				//printf("errorPan = %.2f \t errorTilt = %.2f\n", errorPanG, errorTiltG);
				//printf("RadrrorPan = %.2f \t RaderrorTilt = %.2f\n", errorPanGRad, errorTiltGRad);
				//printf("KPPan = %f \t KDPan = %f\t", kamera.panKP, kamera.panKD); printf("KPTilt = %f \t KDTilt = %f\n", kamera.tiltKP, kamera.tiltKD);

				G_Pan_err_diff = errorPanGRad - G_Pan_err;
				G_Tilt_err_diff = errorTiltGRad - G_Tilt_err;

				// PID pan ==========================================================
				//PPanG  = G_Pan_err  * kamera.panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
				PPanG  = G_Pan_err  * goal_panKP; // Tune in Kp Pan 0.00035 //kalau kepala msh goyang2, kurangin nilainya
				intPanG += G_Pan_err * dtG;
				IPanG = intPanG * 0.0;
				dervPanG = G_Pan_err_diff / dtG;
				//DPanG = dervPanG * kamera.panKD;
				DPanG = dervPanG * goal_panKD;
				G_Pan_err = errorPanGRad;
				posPan += (PPanG + IPanG + DPanG);

				// PID tilt ==========================================================
				//PTiltG = G_Tilt_err * kamera.tiltKP; // Tune in Kp Tilt 0.00030
				PTiltG = G_Tilt_err * goal_tiltKP; // Tune in Kp Tilt 0.00030

				intTiltG += G_Tilt_err * dtG;
				ITiltG = intTiltG * 0.0;

				dervTiltG = G_Tilt_err_diff / dtG;
				//DTiltG = dervTiltG * kamera.tiltKD;
				DTiltG = dervTiltG * goal_tiltKD;

				preErrTiltG = errorTiltG;
				G_Tilt_err = errorTiltGRad;
				posTilt += (PTiltG + ITiltG + DTiltG) * -1;


			if      (posPan  >=  1.6)  { posPan  =  1.6; }
			else if (posPan  <= -1.6)  { posPan  = -1.6; }
			if      (posTilt <= -2.0)  { posTilt = -2.0; }
			else if (posTilt >= -0.4)  { posTilt = -0.4; }

			headMove(posPan, posTilt); //printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
		}

	}
}

// Follow Goal ===============================================================================
int	countReadyStop = 0;
void followGoal(double Xwalk, double SetPan, int mode){ //0 normal, 1 sambil belok
	errorfPan  = posPan - SetPan;

	if (posTilt < -2.0 && posPan < 0.4 && posPan > -0.4 && receler.Goal_X != -1 && receler.Goal_Y != -1) { //Stop
		countReadyStop++;
	} else { //Follow
		countReadyStop = 0;
	}

	if (countReadyStop >= 5) {
		PxMove = 0.00; //jalan ditempat
		PyMove = errorfPan * 0.040; //0.045
		PaMove = errorfPan * 0.20; //0.30; //0.045
	} else {
		PxMove = Xwalk; //0.08
		PyMove = errorfPan * 0.040; //0.045
		PaMove = errorfPan * 0.20; //0.35; //0.045
	}

	if (mode == 0) { // pake alfa
		if (errorfPan > -0.4 && errorfPan < 0.4) { 	//printf("AAAAAAAA\n");
			Walk(PxMove, 0.0, PaMove);
		} else { 					//printf("BBBBBBBB\n");
			Walk(0.0, 0.0, PaMove);
		}
	}
	else if (mode == 1) { // tanpa alfa
		if (errorfPan > -0.4 && errorfPan < 0.4) { 	//printf("CCCCCCCC\n");
			Walk(PxMove, PyMove, 0.0);
		} else { 					//printf("DDDDDDDD\n");
			Walk(0.0, PyMove, 0.0);
		}
	}
}

// Body Tracking Goal ========================================================================
double  errorBodyPositionG,
        bodyP_ControllerG;
int     bodyTrueG = 0,
        delayTrueG = 0;
int bodyTrackingGoal(int threshold) {
//	trackGoal();

	errorBodyPositionG = 0 - headPan;
	bodyP_ControllerG = errorBodyPositionG * -0.5;//-0.5

	if (errorBodyPositionG >= -0.1 && errorBodyPositionG <= 0.1) {//untuk hasil hadap 0.8
		//motion("0");
		Walk(0.0, 0.0, 0.0);
		delayTrueG++;
	} else {
		trackGoal();
		//motion("9");

		bodyTrueG       =
		delayTrueG      = 0;

		if (bodyP_ControllerG < 0) { //kanan
			//Walk(rotateGoal_x, abs(rotateGoal_y), -abs(rotateGoal_a));
			Walk(rotateGoal_x, rotateGoal_y, -rotateGoal_a);
		} else { //kiri
			//Walk(rotateGoal_x, -abs(rotateGoal_y), abs(rotateGoal_a));
			Walk(rotateGoal_x, -rotateGoal_y, rotateGoal_a);
		}
	}

	if (delayTrueG >= threshold) {
		bodyTrueG = 1;
	} else {
		bodyTrueG = 0;
	}
	return bodyTrueG;
}

// Search Goal =====================================================================================================
double	goalPan = 0;
void saveGoalLocation() {
//	trackGoal();
	goalPan = headPan;
}

void saveSudutImu() {
//	sudut();
	saveAngle = angle;
}

int	prediksiGoalPan = 0;
double	Rotate = 0;
void prediksiArahGoal() {
	prediksiGoalPan = (int)(57.29 * headPan);
	Rotate = headPan * (8 / 1.57); //90 derajat = 8 detik //waktu rotate
}

int	count = 0;
bool	goalSearch = false,
	searchGoalFinish = false;
void panSearchGoal(double arah) { //printf("  panSearchBall\n\n");
	if (panRate < 0) { panRate = -panRate; }

	if (arah < 0) {
		headPan += panRate;
		if (headPan >= batasKiri) {
			prediksiGoalPan = 0;
			saveAngle = 0;
			Rotate = 0;
//			goalSearch = true;
		}
	} else {
		headPan -= panRate;
		if (headPan <= batasKanan) {
			prediksiGoalPan = 0;
			saveAngle = 0;
			Rotate = 0;
//			goalSearch = true;
		}
	}

	if	(headPan >= batasKiri)  { headPan = batasKiri; }
	else if	(headPan <= batasKanan) { headPan = batasKanan; }

	headMove(headPan, -2.0); //printf("posPan = %.2f \t posTilt = %.2f\n", posPan, posTilt);
}

// +predictPanGoal = kiri
// -predictPanGoal = kanan

//	A+ = putar kiri
//	A- = putar kanan
double	predictGoalPan;
void predictGoal(double alpha, double tilt) {
	predictGoalPan = alpha / 57.29; //sudut * nilai per satu sudut(nilai servo)

	if (predictGoalPan <= -1.6) { predictGoalPan = -1.6; }
	else if (predictGoalPan >= 1.6) { predictGoalPan = 1.6; }
	//printf("  predict = %f\n\n",predictGoalPan);

	headMove(predictGoalPan, tilt);
}


void predictGoalTeam(double alpha, double tilt) {
	int 	setPointPan1, setPointPan2,
		btsSetPointPan, btsRotatePan;
	double	putarPan;

	if (alpha > 180) { alpha = 180; }
	else if (alpha < -180) { alpha = -180; }

	setPointPan1 =  5 + alpha;
	setPointPan2 = -5 + alpha;

	if (setPointPan1 > 180 || setPointPan2 < -180) { // jika arah imu dibelakang
		if (setPointPan1 > 180) { // nilai setPointPan1 diubah jd negatif
			btsSetPointPan = setPointPan1 - 360;

			if (angle >= setPointPan2 || angle <= btsSetPointPan) { //misal 170 ke -170
				predictGoalPan = 0.00;
			} else  {
				btsRotatePan = alpha - 180;
				if ((angle <= alpha) && (angle >= btsRotatePan)) { //misal di range 0 - 180, maka putarPan kanan
					putarPan = (alpha - angle) / 57.29;
					predictGoalPan = -putarPan;
				} else { //putarPan kiri
					if (angle > alpha) { putarPan = (angle - alpha) / 57.29; } //0.0033
					else { putarPan = ((180 - alpha) + (180 + angle)) / 57.29; } //0.0033
					predictGoalPan = putarPan;
				}
			}
		} else { // nilai setPointPan2 diubah jadi positif
			btsSetPointPan = setPointPan2 + 360;

			if (angle >= btsSetPointPan || angle <= setPointPan1) {
				predictGoalPan = 0.00;
			} else {
				btsRotatePan = alpha + 180;
				if ((angle >= alpha) && (angle <= btsRotatePan)) { //misal di range -180 - 0, maka putarPan kiri
					putarPan = abs(alpha - angle) / 57.29; //0.0033
					predictGoalPan = putarPan;
				} else { //putarPan kanan
					if (angle < alpha) { putarPan = (alpha - angle) / 57.29; } //0.0033
					else { putarPan = ((180 + alpha) + (180 - angle)) / 57.29; } //0.0033
					predictGoalPan = -putarPan;
				}
			}
		}
	} else { // arah imu kedepan
		if (angle >= setPointPan2 && angle <= setPointPan1) {
			predictGoalPan = 0.00;
		} else {
			if (alpha >= 0) {
				btsRotatePan = alpha - 180;
				if ((angle <= alpha) && (angle >= btsRotatePan)) { //putarPan kanan
					putarPan = (alpha - angle) / 57.29; //0.0033
					predictGoalPan = -putarPan;
				} else { //putarPan kiri
					if (angle > alpha) { putarPan = (angle - alpha) / 57.29; } //0.0033
					else { putarPan = ((180 - alpha) + (180 + angle)) / 57.29; } //0.0033
					predictGoalPan = putarPan;
				}
			} else {
				btsRotatePan = alpha + 180;
				if ((angle >= alpha) && (angle <= btsRotatePan)) {//maka putarPan kiri
					putarPan = abs(alpha - angle) / 57.29;; //0.0033
					predictGoalPan = putarPan;
				} else { //putarPan kanan
					if (angle < alpha) { putarPan = (alpha - angle) / 57.29; } //0.0033
					else { putarPan = ((180 + alpha) + (180 - angle)) / 57.29; } //0.0033
					predictGoalPan = -putarPan;
				}
			}
		}
	}

	if (predictGoalPan <= -1.6) { predictGoalPan = -1.6; }
	else if (predictGoalPan >= 1.6) { predictGoalPan = 1.6; }

	headMove(predictGoalPan, tilt);
}


int	goalSide = 0;
double	arahPandang = 0;
int cekArah() {
	arahPandang = angle - (57.29 * headPan);
	if (arahPandang >= -90 && arahPandang <= 90) { //gawang lawan
		goalSide = 0;
	} else { //gawang sendiri
		goalSide = 1;
	}
	return goalSide;
}

int	errorGoal = 0;
double	predictRotate;
void searchGoal(int mode) {
	if (goalSearch) { //printf("\n\ncari bola\n\n"); //kedua
		if (tunggu < 10) {
			loadBallLocation(0.0);
			elapsedTime = 0;
		        second = 0;
			reset = 0;
			robotDirection = false;
			timer = false;
			waiting = 0;
		} else if (tunggu > 50) {
			if (ballLost(20)) {
				if (waiting > 40) {
					resetCase1();
					stateCondition = 1;
				} else {
					panSearchBall(cSekarang);
					//tiltSearchBall(0.0);
					Walk(0.0, 0.0, 0.0);
				} waiting++;
			} else {
				trackBall();
				waiting = 0;

				if (mode == 1) { //rotate dgn trackgoal
					searchGoalFinish = true;
				} else if (mode == 2) { //rotate dgn timer
					if (reset > 5) {
						cekWaktu(abs(Rotate));
						if (timer) {
							Walk(0.0, 0.0, 0.0);
							searchGoalFinish = true;
						} else {
							if (Rotate < 0) { //kiri
								rotateParabolic(1, cSekarang);
							} else { //kanan
								rotateParabolic(-1, cSekarang);
							}
						}
					} else {
						setWaktu();
						reset++;
					}
				} else if (mode == 3) { //rotate dgn offset Imu
					if (robotDirection && headPan >= -0.4 && headPan <= 0.4) {
						Walk(0.0, 0.0, 0.0);
						searchGoalFinish = true;
					} else {
						if (headTilt >= cAktif && headPan >= -0.4 && headPan <= 0.4) {
							if (reset > 5) {
								cekWaktu(20);
								if (timer) {
									Walk(0.0, 0.0, 0.0);
									searchGoalFinish = true;
								} else {
									if (((saveAngle - prediksiGoalPan) > -30) && ((saveAngle - prediksiGoalPan) < 30)) {
										if (prediksiGoalPan > 0) { errorGoal = -abs((saveAngle-prediksiGoalPan) * 0.4); } //0.3 //0.33 //0.4
										else { errorGoal = abs((saveAngle-prediksiGoalPan) * 0.4); } //0.3 //0.33 //0.4
										if (errorGoal >= 20) { errorGoal = 20; }
									} else if (((saveAngle - prediksiGoalPan) >= 30) && ((saveAngle - prediksiGoalPan) < 60) || ((saveAngle - prediksiGoalPan) > -60) && ((saveAngle - prediksiGoalPan) <= -30)) {
										if (prediksiGoalPan > 0) { errorGoal = -abs((saveAngle-prediksiGoalPan) * 0.3); } //0.3 //0.33 //0.4
										else { errorGoal = abs((saveAngle-prediksiGoalPan) * 0.3); } //0.3 //0.33 //0.4
										if (errorGoal >= 20) { errorGoal = 20; }
									} else {
										errorGoal = 0;
									}

									if (useSideKick) {
										if (prediksiGoalPan >= 45) { //kiri
											modeKick = 4; //tendangSamping
											Imu(90 + (saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
										} else if (prediksiGoalPan <= -45) { //kanan
											modeKick = 3; //tendangSamping
											Imu(-90 + (saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
										} else {
											modeKick = tendangJauh;
											Imu((saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
											lastDirection = angle;
										}
									} else {
										modeKick = tendangJauh;
										Imu((saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
										lastDirection = angle;
									}
								}
							} else {
								setWaktu();
								robotDirection = false;
								reset++;
							}
						} else {
							reset = 0;
							followBall(0);
						}
					}
				}
			}
		} else {
			elapsedTime = 0;
		        second = 0;
			reset = 0;
			robotDirection = false;
			timer = false;
			waiting = 0;

			if (ballLost(20)) {
				panSearchBall(cSekarang);
				//tiltSearchBall(0.0);
			} else {
				trackBall();
			} Walk(0.0, 0.0, 0.0);
		} tunggu++;
	} else { //pertama
		tunggu = 0;

		if (goalLost(20)) { //printf("\n\n  cari gawang\n\n");//fungsi yg cari gawang disini
			Walk(0.0, 0.0, 0.0);

			if (delay > 5) {
				cekWaktu(2.5);
				if (!timer) {
					if (second <= 2) {
						if (strategy.strategyNumber == 3 || strategy.strategyNumber == 4 || strategy.strategyNumber == 9 || strategy.strategyNumber == 10 || strategy.strategyNumber == 11 || strategy.strategyNumber == 12) {
							headMove(0.0, -2.0);
						} else {
							predictGoal(angle, -2.0);
						}
					} else if (second > 2 && second <= 2.5) {
						if (angle < 0) {
							headMove(-1.6, -2.0);
						} else {
							headMove(1.6, -2.0);
						}
					}
				} else {
					panSearchGoal(angle);
				}
			} else {
				setWaktu();
				count = 0;
				delay++;
			}

		} else { //printf("\n\n  rotate gawang\n\n");
			trackGoal();

			if (count > 50) { //50 //60
				if (((saveAngle - prediksiGoalPan) >= -90) && ((saveAngle - prediksiGoalPan) <= 90)) {
					if (mode == 1) {
						if (bodyTrueG == 1) {
							Walk(0.0, 0.0, 0.0);
							goalSearch = true;
						} else {
							bodyTrackingGoal(10); //20
						}
					} else if (mode == 2) {
						goalSearch = true;
					} else if (mode == 3) {
						goalSearch = true;
					}
				} else { //gawang sendiri
					Walk(0.0, 0.0, 0.0);

					prediksiGoalPan = 0;
					saveAngle = 0;
					Rotate = 0;

					goalSearch = true;
				}
			} else {
				Walk(0.0, 0.0, 0.0);

				prediksiArahGoal();
				saveSudutImu();
				//saveGoalLocation();

				count++;
			}
		}
	}
}

bool	rotateGoal = false,
	ballposTracked = false;
void rotateToGoal(int mode) {
	if (!searchGoalFinish) { //pertama
		if (ballposTracked) { //printf("  searchgoal,"); //second
			searchGoal(3); //1 2 3
		} else { //printf("  positioning,"); //first
			if (ballLost(20)) { //20
				resetCase1();
				stateCondition = 1;
			} else {
				trackBall();
				if (mode == 1) {
					if (headTilt >= (cSekarang - 0.2) && headPan >= -0.4 && headPan <= 0.4) {
//						saveBallLocation();
						ballposTracked = true;
					} else {
						followBall(0);
					}
				} else if (mode == 2) { //
					if (ballPos) { //positioning sebelum search goal
//						saveBallLocation();
						ballposTracked = true;
					} else {
						ballPositioning(0.0, cSekarang, ballPositioningSpeed); //0.15
					}
				}
			}
		}
	} else { //printf("  finish.........................................,"); //kedua
		trackBall();
		rotateGoal = true;
	}
}

// Tendang ===================================================================================
bool	tendang = false;
void kick(int mode) {
//	trackBall();
	if (mode == 3 || mode == 4) {
		if (mode == 3) { kanan = true; kiri = false; } //arah kanan
		else if (mode == 4) { kiri = true; kanan = false; } //arah kiri
	} else {
		if (posPan >= 0 && kanan == false && kiri == false) { //kiri
			kiri = true; kanan = false;
		} else if (posPan <= 0 && kanan == false && kiri == false) { //kanan
			kanan = true; kiri = false;
		}
	}

	if (kiri) { //kiri
	//if (posPan >= 0) { //kiri
		if (ballPos) { //printf("ball pos left true\n");
			motion("0");
			if (mode == 1 || mode == 2) {
				usleep(700000); //6
				motion("1");
			} else if (mode == 3 || mode == 4) {
				sleep(1); //10
				//motion("3");
				motion("4");
			} else if (mode == 5 || mode == 6) {
				usleep(700000);
				motion("5");
			}
			tendang = true;
		} else {
			ballPositioning(-pPanTendang, pTiltTendang, ballPositioningSpeed); //0.15
		}
	} if (kanan) { //kanan
	//} else { //kanan
		if (ballPos) { //printf("ball pos right true\n");
			motion("0");
			if (mode == 1 || mode == 2) {
				usleep(700000); //7
				motion("2");
			} else if (mode == 3 || mode == 4) {
				sleep(1); //10
				//motion("4");
				motion("3");
			} else if (mode == 5 || mode == 6) {
				usleep(700000); //7
				motion("6");
			}
			tendang = true;
		} else {
			ballPositioning(pPanTendang, pTiltTendang, ballPositioningSpeed); //0.15
		}
	}
}

void rotateKickOffImu(int sudut, int mode) {
	if (headTilt >= -1.4) {
		if (robotDirection && headPan >= -0.4 && headPan <= 0.4) {
			if (sudut >= 0) { //kiri
				if (ballPos) { //printf("ball pos left true\n");
					motion("0");
					if (mode == 1 || mode == 2) {
						usleep(700000); //6
						motion("1");
					} else if (mode == 3 || mode == 4) {
						//usleep(1000000); //10
						sleep(1); //10
						//motion("3");
						motion("4");
					} else if (mode == 5 || mode == 6) {
						usleep(700000);
						motion("5");
					}
					tendang = true;
				} else {
					ballPositioning(-pPanOper, pTiltOper, ballPositioningSpeed); //0.15
				}
			} else { //kanan
				if (ballPos) { //printf("ball pos right true\n");
					motion("0");
					if (mode == 1 || mode == 2) {
						usleep(700000); //7
						motion("2");
					} else if (mode == 3 || mode == 4) {
						//usleep(1000000); //10
						sleep(1); //10
						//motion("4");
						motion("3");
					} else if (mode == 5 || mode == 6) {
						usleep(700000); //7
						motion("6");
					}
					tendang = true;
				} else {
					ballPositioning(pPanOper, pTiltOper, ballPositioningSpeed); //0.15
				}
			}
		} else {
			if (headTilt >= (cAktif + 0.2) && headPan >= -0.4 && headPan <= 0.4) {
				Imu(sudut, cSekarang - 0.20);
			} else {
				robotDirection = false;
				followBall(0);
			}
		}
	} else {
		if	(posTilt >= SetPointTilt) { posTilt = SetPointTilt; }
		else if	(posTilt < -2.0) { posTilt = -2.0; }

		errorfPan  = posPan - SetPointPan;
		errorfTilt = posTilt - SetPointTilt;

		if (posTilt >= SetPointTilt && posPan < 0.4 && posPan > -0.4 && receler.Ball_X != -1 && receler.Ball_Y != -1) { //Stop(bola sudah dekat)
			PxMove = 0.0; //jalan ditempat
			PyMove = errorfPan * 0.040; //0.045
			PaMove = errorfPan * 0.20; //0.30; //0.045
		} else { //Kejar Bola(bola masih jauh)
			PxMove = kejarMax; //0.06
			PyMove = errorfPan * 0.045; //0.045
			PaMove = errorfPan * 0.25; //0.35; //0.045
		}

		if (errorfPan > -0.4 && errorfPan < 0.4) { 	//printf("AAAAAAAA\n");
			Walk(PxMove, 0.0, PaMove);
		} else { 					//printf("BBBBBBBB\n");
			Walk(0.0, 0.0, PaMove);
		}
		//followBall(0);
	}
}

void rotateKickOff(double timeRotate, int mode) {
//	trackBall();

	if (headTilt >= -1.0) {
	//if (headTilt >= -0.8 && headPan >= -0.4 && headPan <= 0.4) {
		if (reset > 5) { //printf("set......................!!!\n");
			cekWaktu(abs(timeRotate));
			if (timer) { //printf("true......................!!!\n");
				if (posPan >= 0) { //kiri
					if (ballPos) { //printf("ball pos left true\n");
						motion("0");
						if (mode == 1 || mode == 2) {
							usleep(700000); //6
							motion("1");
						} else if (mode == 3 || mode == 4) {
							//usleep(1000000); //10
							sleep(1); //10
							//motion("3");
							motion("4");
						} else if (mode == 5 || mode == 6) {
							usleep(700000);
							motion("5");
						}
						tendang = true;
					} else {
						ballPositioning(-pPanOper, pTiltOper, ballPositioningSpeed); //0.15
					}
				} else { //kanan
					if (ballPos) { //printf("ball pos right true\n");
						motion("0");
						if (mode == 1 || mode == 2) {
							usleep(700000); //7
							motion("2");
						} else if (mode == 3 || mode == 4) {
							//usleep(1000000); //10
							sleep(1); //10
							//motion("4");
							motion("3");
						} else if (mode == 5 || mode == 6) {
							usleep(700000); //7
							motion("6");
						}
						tendang = true;
					} else {
						ballPositioning(pPanOper, pTiltOper, ballPositioningSpeed); //0.15
					}
				}
			} else {
				motion("9");
				//rotateParabolic(timeRotate, cSekarang);
				if (timeRotate < 0) { //kanan
					Walk(rotateGoal_x, rotateGoal_y, -rotateGoal_a);
				} else { //kiri
					Walk(rotateGoal_x, -rotateGoal_y, rotateGoal_a);
				}
			}
		} else { //printf("cek......................!!!\n");
			motion("9");
			Walk(0.0, 0.0, 0.0);
			setWaktu();
			reset++;
		}
	} else {
		reset = 0;
		followBall(0);
	}
}

// Localization ==============================================================================
int	kecepatanRobot = 25, //cm
	finalGoalValue = 300; //jarak berhenti robot
int	jarakTiangKanan = 0,
	jarakTiangKiri = 0,
	goalDistanceValue = 0;
double	Timing = 0;
int kalkulasiJarakGawang(int tiangKiri, int tiangKanan) {
	jarakTiangKiri = tiangKiri;
	jarakTiangKanan = tiangKanan;

	if (jarakTiangKiri > jarakTiangKanan) {
		if (jarakTiangKiri > 900) {
			goalDistanceValue = jarakTiangKanan;
		} else {
			goalDistanceValue = jarakTiangKiri;
		}
	} else {
		if (jarakTiangKanan > 900) {
			goalDistanceValue = jarakTiangKiri;
		} else {
			goalDistanceValue = jarakTiangKanan;
		}
	}

	if (jarakTiangKiri == -1 && jarakTiangKanan == -1) {
		Timing = 0;
	} else {
		Timing = (goalDistanceValue - finalGoalValue) / kecepatanRobot;
	}

	return Timing;
}

//////////////////////////////////////////////////////////////////////////////////////////////
double 	targetCoorY, sekarangCoorY, Ytar,
	targetCoorX, sekarangCoorX, Xtar,
	targetCoorR, sekarangCoorR, setPointRotateR1, setPointRotateR2,
	setfPointY,
	setRotatePointR,
	errorfX,
	errorfY,
	errorAngle,
	LyMove = 0,	//Y
	LxMove = 0,	//X
	LaMove = 0;	//A
int	cL,	//count lock for atribut target
	cD,	//count peralihan dari rotate ke jalan
	cR,	//count peralihan dari break ke rotate
	cB,	//count untuk break ketika sudah didalam setpoint target
	cZ,
	cA,
	cV,cY,
	cN;
bool	donePosition = false,		//var untuk menandakan fungsi selesai
	kembaliMasuk = false,		//var untuk menandakan target diantara >90 && <-90
	awalMasuk = false,		//var untuk menandakan target diantara <90 && >-90
	cX = false,			//var untuk menandakan koordinat x sudah dalam range
	lockMidBack = false,		//var untuk mengunci posisi balik yang dituju -> tengah (main strategi)
	lockLeftBack = false,		//var untuk mengunci posisi balik yang dituju -> kiri (support strategi)
	lockRightBack = false,		//var untuk mengunci posisi balik yang dituju -> kanan (support strategi)
	balikTengah = false,		//var untuk lock posisi balik use koordinasi
	balikSampingKanan = false,	//var untuk lock posisi balik use koordinasi
	balikSampingKiri = false;	//var untuk lock posisi balik use koordinasi
//////////////////////////////////////////////////////////////////////////////////////////////

int	tama = 0;
double	Waktu = 0;
void updateCoordinatFromVision() {
	if (receler.RobotCoor_X >=-450 && receler.RobotCoor_X <=450 && receler.RobotCoor_Y >= -300 && receler.RobotCoor_Y<=300) {
		if (receler.RobotCoor_X!=-1 && receler.RobotCoor_Y!=-1) {
			initialPos_X = receler.RobotCoor_X;
			initialPos_Y = receler.RobotCoor_Y;
			odometry.deltaPos_X = odometry.deltaPos_Y = 0;
		}
	}
}

bool backIn = false;
int status[3], statusBackIn[3];
int totalRobotStatus = 0, totalRobotStatusBackIn = 0;
void koordinasiRobotBalik() { // robotStatus, jika 0 = kill jika 1 = run	//hari ini
	if (backIn== true) {
		if (robotID == 1) {		//3, 4, 5
			if (koordinasi.robot3Status == 1 && koordinasi.robot3BackIn == 0) {
				status[0] = 1;
				statusBackIn[0] = 0;
			} else if (koordinasi.robot3Status == 0 && koordinasi.robot3BackIn == 1) {
				status[0] = 0;
				statusBackIn[0] = 1;
			} else { status[0] = 0; statusBackIn[0] = 0; }

			if (koordinasi.robot4Status == 1 && koordinasi.robot4BackIn == 0) {
				status[1] = 1;
				statusBackIn[1] = 0;
			} else if (koordinasi.robot4Status == 0 && koordinasi.robot4BackIn == 1) {
				status[1] = 0;
				statusBackIn[1] = 1;
			} else { status[1] = 0; statusBackIn[1] = 0; }

			if (koordinasi.robot5Status == 1 && koordinasi.robot5BackIn == 0) {
				status[2] = 1;
				statusBackIn[2] = 0;
			} else if (koordinasi.robot5Status == 0 && koordinasi.robot5BackIn == 1) {
				status[2] = 0;
				statusBackIn[2] = 1;
			} else { status[2] = 0; statusBackIn[2] = 0; }
		} else if (robotID == 3) {	//1, 4, 5
			if (koordinasi.robot1Status == 1 && koordinasi.robot1BackIn == 0) {
				status[0] = 1;
				statusBackIn[0] = 0;
			} else if (koordinasi.robot1Status == 0 && koordinasi.robot1BackIn == 1) {
				status[0] = 0;
				statusBackIn[0] = 1;
			} else { status[0] = 0; statusBackIn[0] = 0; }

			if (koordinasi.robot4Status == 1 && koordinasi.robot4BackIn == 0) {
				status[1] = 1;
				statusBackIn[1] = 0;
			} else if (koordinasi.robot4Status == 0 && koordinasi.robot4BackIn == 1) {
				status[1] = 0;
				statusBackIn[1] = 1;
			} else { status[1] = 0; statusBackIn[1] = 0; }

			if (koordinasi.robot5Status == 1 && koordinasi.robot5BackIn == 0) {
				status[2] = 1;
				statusBackIn[2] = 0;
			} else if (koordinasi.robot5Status == 0 && koordinasi.robot5BackIn == 1) {
				status[2] = 0;
				statusBackIn[2] = 1;
			} else { status[2] = 0; statusBackIn[2] = 0; }
		} else if (robotID == 4) {	//1, 3, 5
			if (koordinasi.robot1Status == 1 && koordinasi.robot1BackIn == 0) {
				status[0] = 1;
				statusBackIn[0] = 0;
			} else if (koordinasi.robot1Status == 0 && koordinasi.robot1BackIn == 1) {
				status[0] = 0;
				statusBackIn[0] = 1;
			} else { status[0] = 0; statusBackIn[0] = 0; }

			if (koordinasi.robot3Status == 1 && koordinasi.robot3BackIn == 0) {
				status[1] = 1;
				statusBackIn[1] = 0;
			} else if (koordinasi.robot3Status == 0 && koordinasi.robot3BackIn == 1) {
				status[1] = 0;
				statusBackIn[1] = 1;
			} else { status[1] = 0; statusBackIn[1] = 0; }

			if (koordinasi.robot5Status == 1 && koordinasi.robot5BackIn == 0) {
				status[2] = 1;
				statusBackIn[2] = 0;
			} else if (koordinasi.robot5Status == 0 && koordinasi.robot5BackIn == 1) {
				status[2] = 0;
				statusBackIn[2] = 1;
			} else { status[2] = 0; statusBackIn[2] = 0; }
		} else if (robotID == 5) {	//1, 3, 4
			if (koordinasi.robot1Status == 1 && koordinasi.robot1BackIn == 0) {
				status[0] = 1;
				statusBackIn[0] = 0;
			} else if (koordinasi.robot1Status == 0 && koordinasi.robot1BackIn == 1) {
				status[0] = 0;
				statusBackIn[0] = 1;
			} else { status[0] = 0; statusBackIn[0] = 0; }

			if (koordinasi.robot3Status == 1 && koordinasi.robot3BackIn == 0) {
				status[1] = 1;
				statusBackIn[1] = 0;
			} else if (koordinasi.robot3Status == 0 && koordinasi.robot3BackIn == 1) {
				status[1] = 0;
				statusBackIn[1] = 1;
			} else { status[1] = 0; statusBackIn[1] = 0; }

			if (koordinasi.robot4Status == 1 && koordinasi.robot4BackIn == 0) {
				status[2] = 1;
				statusBackIn[2] = 0;
			} else if (koordinasi.robot4Status == 0 && koordinasi.robot4BackIn == 1) {
				status[2] = 0;
				statusBackIn[2] = 1;
			} else { status[2] = 0; statusBackIn[2] = 0; }
		}

		totalRobotStatus = status[0] + status[1] + status[2];
		totalRobotStatusBackIn = statusBackIn[0] + statusBackIn[1] + statusBackIn[2];


		if (totalRobotStatus == 0) {
			if (totalRobotStatusBackIn == 0) {		//masuk tengah karna sendiri
				balikSampingKiri = false;
				balikSampingKanan = false;
				balikTengah = true;
			} else if (totalRobotStatusBackIn != 0 && !balikTengah) {
				if (totalRobotStatusBackIn < 3) {		//izin masuk diterima
					//motion("9");
					if (masukKiri) {
						balikSampingKiri = true;
						balikSampingKanan = false;
						balikTengah	= false;
					} else if (masukKanan) {
						balikSampingKiri = false;
						balikSampingKanan = true;
						balikTengah	= false;
					}
				} else if (totalRobotStatusBackIn >=3) {	//izin masuk ditolak
					motion("0");
				}
			}
		} else {
			if (totalRobotStatus < 3) {		//izin masuk diterima
					//motion("9");
					if (masukKiri) {
						balikSampingKiri = true;
						balikSampingKanan = false;
						balikTengah	= false;
					} else if (masukKanan) {
						balikSampingKiri = false;
						balikSampingKanan = true;
						balikTengah	= false;
					}
			} else if (totalRobotStatus >= 3) {	//izin masuk ditolak
				motion("0");
			}
		}
	} else if (backIn == false) {
		if (robotID == 1) {
			//printf("   this is Robot 1..........................\n");
			if (robotStatus == 1) {	//run
				balikTengah = true;	//eksekusi
				balikSampingKanan = false;
				balikSampingKiri = false;
				//printf("   Robot 1 Run...........................\n");
			} else {		//kill
				balikTengah = false;
				balikSampingKanan = false;
				balikSampingKiri = false;
				//printf("   Robot 1 Kill..........................\n");
			}
		} else if (robotID == 3) {
			//printf("   this is Robot 3..........................\n");
			if (robotStatus == 1) {	//run
				if (koordinasi.robot1Status == 0) {		//ketika robot 1 tidak aktif
					balikTengah = true;
					balikSampingKanan = false;
					balikSampingKiri = false;
					//printf("   Robot 3 Run Balik Tengah..........................\n");
				} else if (koordinasi.robot1Status == 1) {	//ketika robot 1 aktif
					balikTengah = false;
					balikSampingKanan = true;
					balikSampingKiri = false;
					//printf("   Robot 3 Run Balik Samping Kanan..........................\n");
				}
			} else {		//kill
				balikTengah = false;
				balikSampingKanan = false;
				balikSampingKiri = false;
				//printf("   Robot 3 Kill..........................\n");
			}
		} else if (robotID == 4) {
			//printf("   this is Robot 4..........................\n");
			if (robotStatus ==1 ) {	//run
				if (koordinasi.robot1Status == 0) {	//ketika robot 1 tidak aktif
					if (koordinasi.robot3Status == 0) {	//ketika robot 3 tidak aktif
						balikTengah = true;
						balikSampingKanan = false;
						balikSampingKiri = false;
						//printf("   Robot 4 Run Balik Tengah..........................\n");
					} else if (koordinasi.robot3Status == 1) {	//ketika robot 3 aktif
						balikTengah = false;
						balikSampingKanan = true;
						balikSampingKiri = false;
						//printf("   Robot 4 Run Balik Samping Kanan..........................\n");
					}
				} else if (koordinasi.robot1Status == 1) {	//ketika robot 1 aktif
					if (koordinasi.robot3Status == 0) {	//ketika robot 3 tidak aktif
						balikTengah = false;
						balikSampingKanan = true;
						balikSampingKiri = false;
						//printf("   Robot 4 Run Balik Samping Kanan..........................\n");
					} else if (koordinasi.robot3Status == 1) {	//ketika robot 3 aktif
						balikTengah = false;
						balikSampingKanan = false;
						balikSampingKiri = true;
						//printf("   Robot 4 Run Balik Samping Kiri..........................\n");
					}
				}
			} else {		//kill
				balikTengah = false;
				balikSampingKanan = false;
				balikSampingKiri = false;
				//printf("   Robot 4 Kill..........................\n");
			}
		} else if (robotID == 5) {
			//printf("   this is Robot 5..........................\n");
			if (robotStatus == 1) {	//Run
				if (koordinasi.robot1Status == 0) {	//ketika robot 1 tidak aktif
					if (koordinasi.robot3Status == 0 || koordinasi.robot4Status == 0) {	//ketika robot 3 tidak aktif atau robot 4 tidak aktif
						balikTengah = true;
						balikSampingKanan = false;
						balikSampingKiri = false;
						//printf("   Robot 5 Run Balik Samping Tengah..........................\n");
					} else if (koordinasi.robot3Status == 1 || koordinasi.robot4Status == 1) {	//ketiak robot 3 aktif atau robot 4 akti
						balikTengah = false;
						balikSampingKanan = true;
						balikSampingKiri = false;
						//printf("   Robot 5 Run Balik Samping Kanan..........................\n");
					}
				} else if (koordinasi.robot1Status == 1) {	//ketika robot 1 aktif
					if (koordinasi.robot3Status == 0 || koordinasi.robot4Status == 0) {	//ketika robot 3 tidak aktif atau robot 4 tidak aktif
						balikTengah = false;
						balikSampingKanan = true;
						balikSampingKiri = false;
						//printf("   Robot 5 Run Balik Samping Kanan..........................\n");
					} else if (koordinasi.robot3Status == 1 || koordinasi.robot4Status == 1) {	//ketika robot 3 aktif atau robot 4 aktif
						balikTengah = false;
						balikSampingKanan = false;
						balikSampingKiri = true;
						//printf("   Robot 5 Run Balik Samping Kiri..........................\n");
					}
				}
			} else {
				balikTengah = false;
				balikSampingKanan = false;
				balikSampingKiri = false;
				//printf("   Robot 5 Kill..........................\n");
			}
		}
	}
}

void moveLokalisasi( double mode, double Xtarget, double Ytarget) {
	/*Robot berjalan berdasarkan koordinat yang di imput*/
	Xtar = Xtarget; Ytar = Ytarget;
	targetCoorX = robotPos_X - Xtarget;//-240--240	= 0
	targetCoorY = robotPos_Y - Ytarget;//-320--0	= -320
	targetCoorR = sqrt((targetCoorX*targetCoorX)+(targetCoorY*targetCoorY));
	if (robotPos_X > Xtarget) {		//Target ada dibelakang
		if (robotPos_Y > Ytarget) {	//saat ini sebelah kanan
			setRotatePointR = -180 + asin(targetCoorY/targetCoorR)*(180/PI);
		} else if (robotPos_Y < Ytarget) {	//saat ini sebelah kiri
			setRotatePointR = 180 + asin(targetCoorY/targetCoorR)*(180/PI);
		}
	} else if (robotPos_X < Xtarget) {	//Target ada didepan
		if (robotPos_Y < Ytarget) {	//saat ini sebelah kiri
			setRotatePointR = 0 - asin(targetCoorY/targetCoorR)*(180/PI);
		} else if (robotPos_Y > Ytarget) {	//saat ini sebalah kanan
			setRotatePointR = 0 - asin(targetCoorY/targetCoorR)*(180/PI);
		}
	}

	if(robotPos_X < (Xtarget+15) && robotPos_X > (Xtarget-15)){	//Koreksi koordinatX
		cX = true;
	} else {
		cB = 0;
	}
	if(cX){
		if (mode == 0) { //Koreksi koordinat Y Jika koordinat X sudah masuk range
			if (robotPos_Y < (Ytarget+15) && robotPos_Y > (Ytarget-15)) {
					cB ++;
			} else {
				if (robotPos_Y > (Ytarget+15)) {
					if (cY>10) {
						if (posRotateNew && cX) {
							jalanDirection(kejarMax,0.0,-90);
						} else {
							rotateBodyImuNew(-90);
						}
					} else {
						Walk(0.0,0.0,0.0);
						cY++;
					}
				} else if (robotPos_Y < (Ytarget-15)) {
					if (cY>10) {
						if (posRotateNew && cX) {
							jalanDirection(kejarMax,0.0,90);
						} else {
							rotateBodyImuNew(90);
						}
					} else {
						Walk(0.0,0.0,0.0);
						cY++;
					}
				}
			}
		} else if (mode == 1) {	//tanpa koreksi koordinat Y
			cB ++;
		}
	}

	if(cB >= 1){
		donePosition = true;
		posRotateNew = false;
		LxMove = 0.0;
	} else {
		if(!cX){
			LxMove = kejarMax;
			jalanDirection(LxMove,0.0,setRotatePointR);	//Jika bisa baca gawang
		}
	}
}

void moveBackLokalisasi( double mode, double Xtarget, double Ytarget) {
	/*Robot berjalan berdasarkan koordinat yang di imput*/
	Xtar = Xtarget; Ytar = Ytarget;
	targetCoorX = robotPos_X - Xtarget;//-240--240	= 0
	targetCoorY = robotPos_Y - Ytarget;//-320--0	= -320
	targetCoorR = sqrt((targetCoorX*targetCoorX)+(targetCoorY*targetCoorY));
	if (robotPos_X > Xtarget) {		//Target ada dibelakang
		if (robotPos_Y > Ytarget) {	//saat ini sebelah kanan
			setRotatePointR = -180 + asin(targetCoorY/targetCoorR)*(180/PI);
		} else if (robotPos_Y < Ytarget) {	//saat ini sebelah kiri
			setRotatePointR = 180 + asin(targetCoorY/targetCoorR)*(180/PI);
		}
	} else if (robotPos_X < Xtarget) {	//Target ada didepan
		if (robotPos_Y < Ytarget) {	//saat ini sebelah kiri
			setRotatePointR = 0 - asin(targetCoorY/targetCoorR)*(180/PI);
		} else if (robotPos_Y > Ytarget) {	//saat ini sebalah kanan
			setRotatePointR = 0 - asin(targetCoorY/targetCoorR)*(180/PI);
		}
	}

	if(robotPos_X < (Xtarget+15) && robotPos_X > (Xtarget-15)){	//Koreksi koordinatX
		cX = true;
	} else {
		cB = 0;
	}
	if(cX){
		if (mode == 0) { //Koreksi koordinat Y Jika koordinat X sudah masuk range
			if (robotPos_Y < (Ytarget+15) && robotPos_Y > (Ytarget-15)) {
					cB ++;
			} else {
				if (robotPos_Y > (Ytarget+15)) {
					if (cY>10) {
						if (posRotateNew && cX) {
							jalanDirection(kejarMax,0.0,-90);
						} else {
							rotateBodyImuNew(-90);
						}
					} else {
						Walk(0.0,0.0,0.0);
						cY++;
					}
				} else if (robotPos_Y < (Ytarget-15)) {
					if (cY>10) {
						if (posRotateNew && cX) {
							jalanDirection(kejarMax,0.0,90);
						} else {
							rotateBodyImuNew(90);
						}
					} else {
						Walk(0.0,0.0,0.0);
						cY++;
					}
				}
			}
		} else if (mode == 1) {	//tanpa koreksi koordinat Y
			cB ++;
		}
	}

	if(cB >= 1){
		donePosition = true;
		posRotateNew = false;
		LxMove = 0.0;
	} else {
		if(!cX){
			LxMove = kejarMax;
			//jalanDirection(LxMove,0.0,setRotatePointR);	//Jika bisa baca gawang
			jalanDirection(LxMove, 0.0, 180);		//Jika tidak bisa baca gawang
		}
	}
}

//New Ability 2020 use Grid in Field
void gridLocalization() {	//Konvert koordinat posisi robot (x,y) jadi Grid posisi robot
        int 	tempGridX, tempGridY, GridX, GridY;
	/*
	if (robotPos_X >= -450 && robotPos_X <= 450 && robotPos_Y >= -300 && robotPos_Y <= 300) {
		tempGridX = ((robotPos_X+450)/100)+1;
		tempGridY = ((robotPos_Y+300)/100)+1;

		if (tempGridX >= 9) {tempGridX = 9;}
		else if (tempGridX <= 0) {tempGridX = 0;}
		if (tempGridY >= 6) {tempGridY = 6;}
		else if (tempGridY <= 0) {tempGridY = 0;}

		if (tempGridX > 0 && tempGridX <=9 && tempGridY > 0 && tempGridY <=6){
			Grid = tempGridY + (6*(tempGridX-1));
			//printf("%d\n",Grid);
			if (Grid <= 1) { Grid = 1;}
			else if (Grid >= 54) { Grid = 54;}
		} else {
			Grid  = 88;
		}
	} else {
		Grid = 88;
	}
	*/
	GridX = robotPos_X;
	GridY = robotPos_Y;

	if (GridX >= 450) { GridX = 450; }
	else if (GridX <= -450) { GridX = -450; }
	if (GridY >= 300) { GridY = 300; }
	else if (GridY <= -300) { GridY = -300; }

	tempGridX = ((GridX+450) / 100) + 1;
	tempGridY = ((GridY+300) / 100) + 1;

	if (tempGridX >= 9) {tempGridX = 9;}
	else if (tempGridX <= 1) {tempGridX = 1;}
	if (tempGridY >= 6) {tempGridY = 6;}
	else if (tempGridY <= 1) {tempGridY = 1;}

	Grid = tempGridY + (6*(tempGridX-1));
	if (Grid <= 1) { Grid = 1; }
	else if (Grid >= 54) { Grid = 54; }
}

void gridBall() {	//Konvert koordinat posisi bola (x,y) jadi Grid posisi bola
	int tempX, tempY;
	//if (receler.Ball_X != 0 && receler.Ball_Y != 0) {
	if (receler.Ball_D > 0 && receler.Ball_D <= 900) {
		if (receler.BallCoor_X != 0 || receler.BallCoor_Y != 0){
			tempX = ((receler.BallCoor_X+450)/100)+1;
			tempY = ((receler.BallCoor_Y+300)/100)+1;

			if (tempX >= 9) {tempX = 9;}
			else if (tempX <= 0) {tempX = 0;}
			if (tempY >= 6) {tempY = 6;}
			else if (tempY <= 0) {tempY = 0;}

			if (tempX > 0 && tempX <=9 && tempY > 0 && tempY <=6){
				GridBall = tempY + (6*(tempX-1));
				//printf("%d\n",GridBall);
				if (GridBall <= 1) { GridBall = 1;}
				else if (GridBall >= 54) { GridBall = 54;}
			} else {
				GridBall  = 88;
			}
		} else {
			GridBall = 88;
		}
	} else {
		GridBall = 88;
	}
}

int lastGrid;
void saveGrid() {	//Save nilai grid
	lastGrid = Grid;
}

int convertGridX(int valueGrid, int valueOffSetX){	//Konvert grid posisi robot jadi nilai koordinat x
	int tempCoorX, tempGridtoX;

	if (valueGrid%6 == 0){
		//printf("POPO\n");
		tempGridtoX = valueGrid / 6;
	} else {
		//printf("PIPI\n");
		tempGridtoX =  (valueGrid / 6) + 1;
	}

	if(tempGridtoX <= 1) {tempGridtoX = 1;}
	else if (tempGridtoX >=9) {tempGridtoX = 9;}

	tempCoorX = (((tempGridtoX*100)-500)+ valueOffSetX);

	return tempCoorX;
}

int convertGridY(int valueGrid, int valueOffSetY){	//Konvert grid posisi robot jadi nilai koordinat y
	int tempCoorY, tempGridtoY;
	if (valueGrid%6 == 0){
		//printf("POPO\n");
		tempGridtoY = 6;
	} else {
		//printf("PIPI\n");
		tempGridtoY = (valueGrid - ((valueGrid/6)*6));
	}

	if(tempGridtoY <= 1) {tempGridtoY = 1;}
	else if(tempGridtoY >=6) {tempGridtoY = 6;}

	tempCoorY = (((tempGridtoY*100)-350)+ valueOffSetY);

	return tempCoorY;
}

bool	doneHeaded = false;
void headGrid(int valueGrid, int valueOffSetX, int valueOffSetY){	//menghadap posisi Grid yang tentukan
	double x, y, r, rotate;
	/*Robot berjalan berdasarkan koordinat yang di imput*/
	x = robotPos_X - convertGridX(valueGrid, valueOffSetX);//-240--240	= 0
	y = robotPos_Y - convertGridY(valueGrid, valueOffSetY);//-320--0	= -320
	r = sqrt((x*x)+(y*y));
	//printf("ADA APA\n");
	if (robotPos_X >= convertGridX(valueGrid, valueOffSetX)) {		//Target ada dibelakang
		//printf("MASUK HAHA\n");
		if (robotPos_Y >= convertGridY(valueGrid, valueOffSetY)) {	//saat ini sebelah kanan
			rotate = -180 + asin(y/r)*(180/PI);
		} else if (robotPos_Y < convertGridY(valueGrid, valueOffSetY)) {	//saat ini sebelah kiri
			rotate = 180 + asin(y/r)*(180/PI);
		}
	} else if (robotPos_X < convertGridX(valueGrid, valueOffSetX)) {	//Target ada didepan
		//printf("MASUK HIHI\n");
		if (robotPos_Y < convertGridY(valueGrid, valueOffSetY)) {	//saat ini sebelah kiri
			//printf("MASUK HUHU\n");
			rotate = 0 - asin(y/r)*(180/PI);
			//printf("%.2lf\n",valueRotateBody);
		} else if (robotPos_Y >= convertGridY(valueGrid, valueOffSetY)) {	//saat ini sebalah kanan
			//printf("MASUK HOHO\n");
			rotate = 0 - asin(y/r)*(180/PI);
		}
	}
	if(posRotateNew){
		doneHeaded = true;
		posRotateNew = false;
	} else {
		rotateBodyImuNew(rotate);
	}
}

bool	doneMoved 	= false,
	setGrid1	= false,
	setGrid2	= false;

int	countMoveGrid1	= 0,	//count walk ditempat
	countMoveGrid2	= 0,	//count rotate
	countMoveGrid3	= 0;	//count walk x,y

double rotateMoveGrid = 0;

void moveGrid(int valueGrid, int valueOffSetX, int valueOffSetY) {	//Bergerak menuju grid yang ditentukan
	double	c,s,sn,x, y, r, rotate, speedX, speedY, speedrX, speedrY, nilaiSudut;

	c = cos(angle);
	s = sin(angle);
	sn = sin(angle)*-1;

	if (angle < 0) {
		nilaiSudut = angle + 360;
	} else {
		nilaiSudut = angle;
	}
	//printf("Nilai Sudut = %.2lf\n", nilaiSudut);
	x = robotPos_X - convertGridX(valueGrid, valueOffSetX);//-240--240	= 0
	y = robotPos_Y - convertGridY(valueGrid, valueOffSetY);//-320--0	= -320
	r = sqrt((x*x)+(y*y));
	//printf("NILAI R = %.2lf\n",r);
	//printf("ADA APA\n");
	if (robotPos_X >= convertGridX(valueGrid, valueOffSetX)) {		//Target ada dibelakang
		//printf("MASUK HAHA\n");
		if (robotPos_Y >= convertGridY(valueGrid, valueOffSetY)) {	//saat ini sebelah kanan
			rotate = -180 + asin(y/r)*(180/PI);
		} else if (robotPos_Y < convertGridY(valueGrid, valueOffSetY)) {	//saat ini sebelah kiri
			rotate = 180 + asin(y/r)*(180/PI);
		}
	} else if (robotPos_X < convertGridX(valueGrid, valueOffSetX)) {	//Target ada didepan
		//printf("MASUK HIHI\n");
		if (robotPos_Y < convertGridY(valueGrid, valueOffSetY)) {	//saat ini sebelah kiri
			//printf("MASUK HUHU\n");
			rotate = 0 - asin(y/r)*(180/PI);
			//printf("%.2lf\n",valueRotateBody);
		} else if (robotPos_Y >= convertGridY(valueGrid, valueOffSetY)) {	//saat ini sebalah kanan
			//printf("MASUK HOHO\n");
			rotate = 0 - asin(y/r)*(180/PI);
		}
	}
	rotateMoveGrid = rotate;
	if (robotPos_X >= (convertGridX(valueGrid, valueOffSetX) - 15) && robotPos_X < (convertGridX(valueGrid, valueOffSetX) + 15) &&
		robotPos_Y >= (convertGridY(valueGrid, valueOffSetY) - 15) && robotPos_Y < (convertGridY(valueGrid, valueOffSetY) + 15)) {
			countMoveGrid1 =
			countMoveGrid2 =
			countMoveGrid3 = 0;

			posRotateNew =
			setGrid1 =
			setGrid2 = false;
			doneMoved = true;
	} else {
		if (countMoveGrid1 >= 5) {
			//RY < TY => RY = -30 TY = -25 DY = -30
			if (r < 30){
				//printf("TIME TO SHOWWWWWW>>>>>> \n");
				if (countMoveGrid2 >= 5) {
					if (nilaiSudut > 270 || nilaiSudut <= 90){
						//printf("MASUK 0 0 0 0\n");
						if (posRotateNew) {
							//printf("SELESAI ROTATE 0\n");
							if (countMoveGrid3 >= 5) {
								if (!setGrid1 && !setGrid2){
									if (robotPos_X >= convertGridX(valueGrid,valueOffSetX)){
										setGrid1 = true; //RX = -300 TX = -400 DX = -300 -(-400) = 100
												//RX = 30 TX = -400 DX = 30 -(-400) = 430
									} else if (robotPos_X < convertGridX(valueGrid,valueOffSetX)) {
										setGrid2 = true; //RX = 300 TX = 400 DX = 300 - 400 = -100
												//RX = -30 TX = -25 DX = -30 -(-25) = -5
									}
								} else {
									if (setGrid1) {
										speedX = x * 0.02;
										//printf("SET GRID 1\n");
									} else if (setGrid2) {
										speedX = x * -0.02;
										//printf("SET GRID 2\n");
									}
								}

								speedY = y * 0.02;

								if (speedX >= 0.08) {speedX = 0.08;}
								else if (speedX <= -0.03) {speedX = -0.03;}
								if (speedY >= 0.04) {speedY = 0.04;}
								else if (speedY <= -0.04) {speedY = -0.04;}

								Walk (speedX,speedY, 0.0);
							} else {
								Walk(0.0,0.0,0.0);
								countMoveGrid3++;
							}
						} else {
							rotateBodyImuNew(0);
						}
					} else if (nilaiSudut > 90 || nilaiSudut <= 270) {
						//printf("MASUK 180 180 180 180\n");
						if (posRotateNew) {
							//printf("SELESAI ROTATE 0\n");
							if (countMoveGrid3 >= 5) {
								if (!setGrid1 && !setGrid2){
									if (robotPos_X >= convertGridX(valueGrid,valueOffSetX)){
										setGrid1 = true; //RX = -300 TX = -400 DX = -300 -(-400) = 100
												//RX = 30 TX = -400 DX = 30 -(-400) = 430
									} else if (robotPos_X < convertGridX(valueGrid,valueOffSetX)) {
										setGrid2 = true; //RX = 300 TX = 400 DX = 300 - 400 = -100
												//RX = -30 TX = -25 DX = -30 -(-25) = -5
									}
								} else {
									if (setGrid1) {
										speedX = x * 0.02;
										//printf("SET GRID 1\n");
									} else if (setGrid2) {
										speedX = x * -0.02;
										//printf("SET GRID 2\n");
									}
								}
								speedY = y * 0.02;

								if (speedX >= 0.08) {speedX = 0.08;}
								else if (speedX <= -0.03) {speedX = -0.03;}
								if (speedY >= 0.04) {speedY = 0.04;}
								else if (speedY <= -0.04) {speedY = -0.04;}

								Walk (speedX,-speedY, 0.0);
							} else {
								Walk(0.0,0.0,0.0);
								countMoveGrid3++;
							}
						} else {
							rotateBodyImuNew(180);
						}
					}
				} else {
				        posRotateNew = false;
					Walk(0.0, 0.0, 0.0);
					countMoveGrid2++;
				}
			} else {
				setGrid1 = false;
				setGrid2 = false;
				posRotateNew = false;
				countMoveGrid2 = 0;
				countMoveGrid3 = 0;
				jalanDirection (kejarMax, 0.0, rotate);
			}
		} else {
			Walk(0.0, 0.0, 0.0);
			countMoveGrid1++;
		}
	}
}

bool	inversValue = false;
void localization() {
	if (reset > 5) {
		//lock initial pos
		//if (lastSecRemaining == 0) { //jika data game controller tidak terbaca
		if (gameController.Remaining == 0) { //jika data game controller tidak terbaca
			motion("0");
			resetAllVariable();
			predictGoal(angle, posTiltGoal);
			backPosition = false;
		//} else if (lastSecRemaining == 600) { //masuk lapangan pertama kali mulai permainan
		} else if ((gameController.Remaining == 600 && gameController.SecondaryState == 0) || (gameController.Remaining == 300 && gameController.SecondaryState == 2)) { //masuk lapangan pertama kali mulai permainan
			if(awalMasuk){	//ketika sudah sampai posisi awal masuk
				//refreshMoveLokalisasi();
				motion("0");
				refreshMoveGrid();
				if (goalLost(20)) {
					panSearchBall(posTiltLocal);
				} else {
					trackGoal();
					if (useLocalization && useUpdateCoordinate) {
						updateCoordinatFromVision();
					}
				}
			} else {		//ketika belum sampai posisi masuk
				if (doneMoved) {
					predictGoal(0.0, posTiltGoal);
					if (cR >= 10){
						if (strategy.strategyNumber < 3) {
							if (posRotateNew) {
								awalMasuk = true;
							} else {
								rotateBodyImuNew(0);
							}
						} else if (strategy.strategyNumber == 3) {
							if (posRotateNew) {
								awalMasuk = true;
							} else {
								rotateBodyImuNew(odometry.ArchSinTeng);
							}
						}
					} else {
						posRotateNew = false;
						Walk(0.0, 0.0, 0.0);
						cR++;
					}
				} else {
					motion ("9");
					if (odometry.walkTot >= 6) {
						searchBallRectang(-1.6, -1.6, -0.8, 1.6);
						moveGrid(initGrid, offsetX, offsetY);
					} else {		//sini dulu, sampai counting jalan terpenuhi
						predictGoal(angle, -1.6);
						if (count >5) {		//counting biasa terpenuhi
							Walk(kejarMax, 0.0, 0.0);
						} else {		///sini, counting dulu
							count ++;
							Walk(0.0, 0.0, 0.0);
						}
					}
				}
			} backPosition = false;
		//} else if (lastSecRemaining != 600) { //kembali ke posisi setelah ada goal/dropball
		} else if ((gameController.Remaining != 600 && gameController.SecondaryState == 0) || (gameController.Remaining != 300 && gameController.SecondaryState == 2)) { //kembali ke posisi setelah ada goal/dropball
			//printf("\n  >>>>>>>> balik ke belakang, goal/dropball <<<<<<<< \n");
			if(kembaliMasuk){
				motion("0");
				//refreshMoveLokalisasi();
				count = 0;
				refreshMoveGrid();
				if (goalLost(20)) {
					panSearchBall(posTiltLocal);
				} else {
					trackGoal();
					if (useLocalization && useUpdateCoordinate) {
						updateCoordinatFromVision();
					}
				}
			} else {
				if (useCoordination){		//kondisi balik ketika menggunakan useCoordination
					if (doneMoved) {
						inversValue = false;
						predictGoal(0, posTiltGoal);
						if (cR >= 10) {
							if (balikTengah) {
								if (posRotateNew) {
									kembaliMasuk = true;
									//refreshMoveGrid();
								} else {
									rotateBodyImuNew(0);
								}
							} else if (balikSampingKanan || balikSampingKiri) {
								if (posRotateNew) {
									kembaliMasuk = true;
									//refreshMoveGrid();
								} else {
									rotateBodyImuNew(odometry.ArchSinTeng);
								}
							}
						} else {
							posRotateNew = false;
							Walk(0.0, 0.0, 0.0);
							cR ++;
						}
					} else {
						koordinasiRobotBalik();
						if (count>5) {
							inversValue = true;
							if (backIn) {
								searchBallRectang(-1.6, -1.6, -0.8, 1.6);
							} else {
								/*
								if (rotateMoveGrid >= -90 || rotateMoveGrid <= 90) {	//Arah pandang ke gawang lawan
									predictGoal(odometry.ArchSinEnemy, posTiltGoal);
								} else {						//Arah pandang ke gawang team
									predictGoalTeam(odometry.ArchSinTeam, posTiltGoal);
								}*/
								if (goalLost(20)) {
									panSearchBall(posTiltLocal);
								} else {
									trackGoal();
									if (useLocalization && useUpdateCoordinate) {
										updateCoordinatFromVision();
									}
								}
							}

							if (gameController.KickOff == BARELANG_COLOR || gameController.KickOff == DROPBALL) { //Attack
								printf("  Our robot kick off !!!\n");
								if (balikTengah) {
									motion("9");
									moveGrid(21, 0, 50);
									balikSampingKiri = false;
									balikSampingKanan = false;
								} else if (balikSampingKanan) {
									motion("9");
									moveGrid (23, -50, 0);
									balikSampingKiri = false;
									balikTengah = false;
								} else if (balikSampingKiri) {
									motion("9");
									moveGrid (20, -50, 0);
									balikSampingKanan = false;
									balikTengah = false;
								} else {
									motion("0");
									predictGoal(angle, posTiltGoal);
								}
							} else {	//Defense
								printf("   Our robot defend !!!\n");
								if (balikTengah) {
									motion("9");
									moveGrid(21, -50, 50);
									balikSampingKiri = false;
									balikSampingKanan = false;
								} else if (balikSampingKanan) {
									motion("9");
									moveGrid (17, -50, 0);
									balikSampingKiri = false;
									balikTengah = false;
								} else if (balikSampingKiri) {
									motion("9");
									moveGrid (14, -50, 0);
									balikSampingKanan = false;
									balikTengah = false;
								} else {
									motion("0");
									predictGoal(angle, posTiltGoal);
								}
							}
						} else {
							Walk(0.0, 0.0, 0.0);
							count ++;
						}
					}
				} else {	//tanpa koordinasi
					if (doneMoved) {
						inversValue = false;
						predictGoal(0, posTiltGoal);
						if (cR >= 10) {
							if (strategy.strategyNumber < 3) {
								if (posRotateNew) {
									kembaliMasuk = true;
								} else {
									rotateBodyImuNew(0);
								}
							} else if (strategy.strategyNumber == 3) {
								if (posRotateNew) {
									kembaliMasuk = true;
								} else {
									rotateBodyImuNew(odometry.ArchSinTeng);
								}
							}
						} else {
							posRotateNew = false;
							Walk(0.0, 0.0, 0.0);
							cR ++;
						}
					} else {
						if (count >= 5) {
							inversValue = true;
							motion("9");

							if (rotateMoveGrid >= -90 || rotateMoveGrid <= 90) {	//Arah pandang ke gawang lawan
								predictGoal(odometry.ArchSinEnemy, posTiltGoal);
							} else {						//Arah pandang ke gawang team
								predictGoalTeam(odometry.ArchSinTeam, posTiltGoal);
							}

							if (useLocalization && useUpdateCoordinate) {
								updateCoordinatFromVision();
							}

							if (gameController.KickOff == BARELANG_COLOR || gameController.KickOff == DROPBALL) { //Attack
								if (strategy.strategyNumber < 3) {	//balik depan kondisi kick off
									moveGrid(21, 0, 50);
								} else if (strategy.strategyNumber == 3) { //balik samping kondisi kick off
									if (!lockLeftBack && !lockRightBack) {	//penentuan kiri atau kanan
										if (robotPos_Y >= 0) {
											lockRightBack = true;
										} else if (robotPos_Y < 0) {
											lockLeftBack = true;
										}
									} else {
										if (lockLeftBack) {	//balik kiri
											moveGrid (20, -50, 0);
										} else if (lockRightBack) {	//balik kanan
											moveGrid (23, -50, 0);
										}
									}
								}
							} else {	//Defense
								if (strategy.strategyNumber < 3) {	//balik depan kondisi kick off
									moveGrid(21, -50, 50);
								} else if (strategy.strategyNumber == 3) { //balik samping kondisi kick off
									if (!lockLeftBack && !lockRightBack) {	//penentuan kiri atau kanan
										if (robotPos_Y >= 0) {
											lockRightBack = true;
										} else if (robotPos_Y < 0) {
											lockLeftBack = true;
										}
									} else {
										if (lockLeftBack) {	//balik kiri
											moveGrid (14, -50, 0);
										} else if (lockRightBack) {	//balik kanan
											moveGrid (17, -50, 0);
										}
									}
								}
							}
						} else {
							inversValue = false;
							Walk(0.0, 0.0, 0.0);
							count ++;
						}
					}
				}
			} backPosition = true;
		}
	} else {
		cR = 0;
		refreshMoveLokalisasi();
		refreshMoveGrid();
		posRotate = false;
		lockMidBack = false;
		lockLeftBack = false;
		lockRightBack = false;
		kembaliMasuk = false;
		awalMasuk = false;
		reset++;
	}
}

int	totalDetectLandmarks = 0,
	detectLandmarks[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0},
	gL = 0, gR = 0, lcL = 0, lcR = 0, pen = 0, xL = 0,
	xR = 0, tL = 0, tR = 0, ballDis = 0;
double	 panAngle = 0;

/*
//tidak menggunakan kamera zed
void sendLokalisasi(int goal, int Ld, int Rd, int Gc, int imu, int Bd, double sudutPan, double Robot_Posx, double Robot_Posy, double odomImu) {
	if ( receler.Goal_X >= 260 && receler.Goal_X <= 380 && receler.Goal_Y >= 210 && receler.Goal_Y <= 270 ) {
		if (Ld >= 0 && Ld <= 900 && Rd >= 0 && Rd <= 900){
			if (abs(Ld-Rd) <= 260 && Gc == 0){
				sprintf(dataLokalisasi, "%d,%d,%d,%d,%d,%.f,%.f,%.f,%.f", goal, Ld, Rd, imu, Bd, sudutPan, Robot_Posx, Robot_Posy,odomImu);
				sendto(socklokalisasi, dataLokalisasi, strlen(dataLokalisasi), 0,(struct sockaddr *) &addrlokalisasi, sizeof(addrlokalisasi));
			} else {
				sprintf(dataLokalisasi, "%d,%d,%d,%d,%d,%.f,%.f,%.f,%.f", goal, -1, -1, imu, Bd, sudutPan, Robot_Posx, Robot_Posy,odomImu);
				sendto(socklokalisasi, dataLokalisasi, strlen(dataLokalisasi), 0,(struct sockaddr *) &addrlokalisasi, sizeof(addrlokalisasi));
			}
		} else {
			sprintf(dataLokalisasi, "%d,%d,%d,%d,%d,%.f,%.f,%.f,%.f", goal, -1, -1, imu, Bd, sudutPan, Robot_Posx, Robot_Posy,odomImu);
			sendto(socklokalisasi, dataLokalisasi, strlen(dataLokalisasi), 0,(struct sockaddr *) &addrlokalisasi, sizeof(addrlokalisasi));
		}
	} else {
		sprintf(dataLokalisasi, "%d,%d,%d,%d,%d,%.f,%.f,%.f,%.f", goal, -1, -1, imu, Bd, sudutPan, Robot_Posx, Robot_Posy,odomImu);
		sendto(socklokalisasi, dataLokalisasi, strlen(dataLokalisasi), 0,(struct sockaddr *) &addrlokalisasi, sizeof(addrlokalisasi));
	}
       //printf("  dataLokalisasi = %s\n", dataLokalisasi);
}


void * sendData(void * argument) { //send socket lokalisasi
        while (1) {
			//sleep(1);
			usleep(500000);
			sendLokalisasi(goalSide, receler.Goal_LD, receler.Goal_RD, receler.Goal_C, 360-sendAngle, receler.Ball_D, 57.29*headPan, robotPos_X, robotPos_Y, odometry.ArchSin);
        }
}
*/

// Use Zed camera
void sendLokalisasi(int goal, int gLd, int gRd, int lLd, int lRd, int xLd, int xRd, int tLd, int tRd, int imu, int Bd, double sudutPan, double Robot_Posx, double Robot_Posy) {
	if (gLd > 0 && gLd <= 900 || gRd > 0 && gRd <= 900 || lLd > 0 && lLd <= 900 || lRd > 0 && lRd <= 900) {	//saat nilai gawang kiri || gawang kanan || Lcross kiri || Lcross kanan

                if (gLd > 0 && gLd <= 900) {	//jarak gawang kiri
                         detectLandmarks[0] = 1;
	                 gL = gLd;
                } else {
	                detectLandmarks[0] = 0;
	                gL = -1;
                }

                if (gRd > 0 && gRd <= 900) {	//jarak gawang kanan
	                detectLandmarks[1] = 1;
       		        gR = gRd;
                } else {
        	        detectLandmarks[1] = 0;
        	        gR = -1;
                }

		if (inversValue == false) {	//invers true ketika robot balik

		        if (lLd > 0 && lLd <= 900) {	//jarak Lcross kiri
			        detectLandmarks[2] = 1;
			        lcL = lLd;
		        } else {
			        detectLandmarks[2] = 0;
			        lcL = -1;
		        }

		        if (lRd > 0 && lRd <= 900) {	//jarak Lcross kanan
			        detectLandmarks[3] = 1;
			        lcR = lRd;
		        } else {
			        detectLandmarks[3] = 0;
			        lcR = -1;
		        }

		        if (xLd > 0 && xLd <= 500) {	//Jarak Xcross kiri
			        detectLandmarks[5] = 1;
			        xL = xLd;
		        } else {
			        detectLandmarks[5] = 0;
			        xL = -1;
		        }

		        if (xRd > 0 && xRd <= 500) {	//Jarak Xcross kanan
			        detectLandmarks[6] = 1;
			        xR = xRd;
		        } else {
			        detectLandmarks[6] = 0;
			        xR = -1;
		        }

		        if (tLd > 0 && tLd <= 500) {	//Jarak Tcross kiri
			        detectLandmarks[7] = 1;
			        tL = tLd;
		        } else {
			        detectLandmarks[7] = 0;
			        tL = -1;
		        }

		        if (tRd > 0 && tRd <= 500) {	//Jarak Tcross kanan
			        detectLandmarks[8] = 1;
			        tR = tRd;
		        } else {
			        detectLandmarks[8] = 0;
			        tR = -1;
		        }
		} else {
			lcL = lcR = xL = xR = tL = tR = 0;
		        detectLandmarks[2] = detectLandmarks[3] =  detectLandmarks[4] = detectLandmarks[5] = detectLandmarks[6] = detectLandmarks[7] = detectLandmarks[8] = 0;
		}
        } else {
		gL = gR = lcL = lcR = xL = xR = tL = tR = 0;
                detectLandmarks[0] = detectLandmarks[1] = detectLandmarks[2] = detectLandmarks[3] =  detectLandmarks[4] =
                detectLandmarks[5] = detectLandmarks[6] = detectLandmarks[7] = detectLandmarks[8] = 0;
        }
	if (Bd > 0 && Bd <= 900) {
		ballDis = Bd;
	} else {
		ballDis = -1;
	}

	panAngle = imu + sudutPan;
	totalDetectLandmarks = detectLandmarks[0] + detectLandmarks[1] + detectLandmarks[2] + detectLandmarks[3] + detectLandmarks[4] + detectLandmarks[5] + detectLandmarks[6] + detectLandmarks[7] + detectLandmarks[8];
	sprintf(dataLokalisasi, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.f,%.f,%.f", totalDetectLandmarks, goal, gL, gR, lcL, lcR, xL, xR, tL, tR, ballDis, panAngle, Robot_Posx, Robot_Posy);	//fulldata
	sendto(socklokalisasi, dataLokalisasi, strlen(dataLokalisasi), 0,(struct sockaddr *) &addrlokalisasi, sizeof(addrlokalisasi));
       	//printf("  dataLokalisasi = %s\n", dataLokalisasi);

}

void * sendData(void * argument) { //send socket lokalisasi
        while (1) {
		usleep(500000);
		sendLokalisasi(goalSide, receler.Goal_LD, receler.Goal_RD, receler.Lcross_LD, receler.Lcross_RD, receler.Xcross_LD, receler.Xcross_RD, receler.Tcross_LD, receler.Tcross_RD, 360-sendAngle, receler.Ball_D, 57.29*headPan, robotPos_X, robotPos_Y);

        }
}

//main
// Coordinations ===============================================================================
int	delayTrackGoal = 0;
bool	goalFound = false,
	breaked = false;
void myTurn() {
	motion("9");
	printf("  MYTURN MYTURN MYTURN MYTURN !!!!\n");
	//if (useFollowSearchGoal && useSearchGoal) {
	if (useFollowSearchGoal && followSearchAktif) {
		if (angle >= -80 && angle <= 80) { //angle dan jarak terpenuhi
			if (chotto > 20) {
				FollowGoal = true;
			} else {
				chotto++;
			}
		}

		if (FollowGoal) {
			Activated = false;
			if (useCoordination) { semeh = 1; koordinasi.sendRobotCoordinationData(robotID, robotStatus, 232, Grid, 1, semeh, GridBall, backIn); }

			if (!searchGoalFinish) { //pertama
				if (!ballposTracked) { //pertama
					if (ballLost(20)) {
						if (waiting > 40) {
							resetCase1();
							stateCondition = 1;
						} else {
							tiltSearchBall(0.0);
						} waiting++;
						Walk(0.0, 0.0, 0.0);
					} else {
						trackBall();
						if (headTilt > -1.4 && !useNearFollowSearchGoal) { //kelewatan
							delay = 0;
							tunggu = 0;

							prediksiGoalPan = 0;
							saveAngle = 0;

							goalFound = false;
							goalSearch = true;

							Walk(kejar, 0.0, 0.0);

							ballposTracked = true;
							searchGoalFinish = true;
						} else if (headTilt >= -1.7 && headPan >= -0.1 && headPan <= 0.1) {
							delay = 0;

							Walk(kejar, 0.0, 0.0);
							//followBall(0);

							if (delayTrackBall > 20) {
								ballposTracked = true;
							} else { delayTrackBall++; }
						} else { //masih jauh
							delayTrackBall = 0;

							prediksiGoalPan = 0;
							saveAngle = 0;

							goalFound = false;
							goalSearch = false;

							followBall(0);
						}
					}
				} else { //kedua
					if (!goalSearch) { //pertama
						tunggu = 0;
						if (delay > 5) {
							cekWaktu(6);
							if (ballTilt < -1.6) { //jarak jauh
								Walk(0.06, 0.0, 0.0);
							} else if (ballTilt < -1.5 && ballTilt >= -1.6) { //jarak sedang
								if (second < 4.5) { //sambil kejar
									Walk(0.05, 0.0, 0.0);
								} else { //space sebelum sampai ke bola (biar gk nabrak bola)
									Walk(0.0, 0.0, 0.0);
								}
							} else { //jarak dekat
								if (second < 3) { //sambil kejar //3.5
									Walk(0.05, 0.0, 0.0);
								} else { //space sebelum sampai ke bola (biar gk nabrak bola)
									Walk(0.0, 0.0, 0.0);
								}
							}

							if (goalLost(20)) { //pertama
								delayTrackGoal = 0;
								if (!timer) { //pertama
									if (second <= 2.0) { //search goal sambil kejar
										predictGoal(angle, -2.0); //tengah
									} else if (second > 4.0) {
										predictGoal(angle+30, -2.0); //kanan
									} else {
										predictGoal(angle-30, -2.0); //kiri
									}
								} else { //kedua
									prediksiGoalPan = 0;
									saveAngle = 0;

									if (ballTilt < -1.6) {
										loadBallLocation(0.2); //0.15
									} else if (ballTilt >= -1.5 && ballTilt >= -1.6) {
										loadBallLocation(0.4); //0.35
									} else {
										loadBallLocation(0.6);
									}

									goalFound = false;
									goalSearch = true;
								}
							} else { //kedua
								if (timer || delayTrackGoal > 50) { //kedua
									if (((saveAngle - prediksiGoalPan) > 90) || ((saveAngle - prediksiGoalPan) < -90)) {
										prediksiGoalPan = 0;
										saveAngle = 0;
									}

									if (ballTilt < -1.6) {
										loadBallLocation(0.2); //0.15
									} else if (ballTilt >= -1.5 && ballTilt >= -1.6) {
										loadBallLocation(0.4); //0.35
									} else {
										loadBallLocation(0.6);
									}

									goalFound = true;
									goalSearch = true;
								} else { //pertama
									trackGoal();
									prediksiArahGoal();
									saveSudutImu();
									if(useUpdateCoordinate) { updateCoordinatFromVision(); }
									delayTrackGoal++;
								}
							}
						} else {
							trackBall();
							Walk(kejar, 0.0, 0.0);
							setWaktu();
							delayTrackGoal = 0;
							delay++;
						}
					} else { //kedua
						if (tunggu < 5) { //20 //pertama
							trackBall();
							delay = 0;
							waiting = 0;
							robotDirection = false;
							setWaktu();
							tunggu++;
						} else { //kedua
							if (ballLost(20)) {
								if (waiting > 40) {
									resetCase1();
									stateCondition = 1;
								} else {
									tiltSearchBall(0.0);
								} waiting++;
								Walk(0.0, 0.0, 0.0);
							} else {
								trackBall();
								waiting = 0;

								if (goalFound == true) {
									if (robotDirection && headPan >= -0.4 && headPan <= 0.4) { //kedua
										//if (headTilt >= (cSekarang - 0.2)) {
											Walk(0.0, 0.0, 0.0);
											searchGoalFinish = true;
										//} else {
										//	if (headTilt >= -1.0) {
										//		ballPositioning(0.0, cSekarang, 0.12);
										//	} else {
										//		followBall(0);
										//	}
										//}
									} else { //pertama
										if (headTilt >= cAktif && headPan >= -0.4 && headPan <= 0.4) { //kedua
											if (delay > 5) {
												cekWaktu(20);
												if (timer) {
													robotDirection = true;
												} else {
													if (((saveAngle - prediksiGoalPan) > -30) && ((saveAngle - prediksiGoalPan) < 30)) {
														if (prediksiGoalPan > 0) { errorGoal = -abs((saveAngle-prediksiGoalPan) * 0.4); } //0.3 //0.33 //0.4
														else { errorGoal = abs((saveAngle-prediksiGoalPan) * 0.4); } //0.3 //0.33 //0.4
														if (errorGoal >= 20) { errorGoal = 20; }
													} else if (((saveAngle - prediksiGoalPan) >= 30) && ((saveAngle - prediksiGoalPan) < 60) || ((saveAngle - prediksiGoalPan) > -60) && ((saveAngle - prediksiGoalPan) <= -30)) {
														if (prediksiGoalPan > 0) { errorGoal = -abs((saveAngle-prediksiGoalPan) * 0.3); } //0.3 //0.33 //0.4
														else { errorGoal = abs((saveAngle-prediksiGoalPan) * 0.3); } //0.3 //0.33 //0.4
														if (errorGoal >= 20) { errorGoal = 20; }
													} else {
														errorGoal = 0;
														//if (prediksiGoalPan > 0) { errorGoal = -abs((saveAngle-prediksiGoalPan) * 0.33); } //- //0.3 //0.33 //0.4
														//else { errorGoal = abs((saveAngle-prediksiGoalPan) * 0.33); } //- //0.3 //0.33 //0.4
														//if (errorGoal >= 20) { errorGoal = 20; }
													}

													if (useSideKick) {
														if (prediksiGoalPan >= 45) { //kiri
															modeKick = 4; //tendangSamping
															Imu(90 + (saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
														} else if (prediksiGoalPan <= -45) { //kanan
															modeKick = 3; //tendangSamping
															Imu(-90 + (saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
														} else {
															modeKick = tendangJauh;
															Imu((saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
															lastDirection = angle;
														}
													} else {
														modeKick = tendangJauh;
														Imu((saveAngle - prediksiGoalPan) + errorGoal, cSekarang);
														lastDirection = angle;
													}
												}
											} else {
												setWaktu();
												robotDirection = false;
												delay++;
											}
										} else { //pertama
											delay = 0;
											followBall(0);
										}
									}
								} else {
									searchGoalFinish = true;
								}
							}
						}
					}
				}
			} else { //kedua (next case)
				if (ballLost(20)) {
					if (tunggu > 40) {
						resetCase1();
						stateCondition = 1;
					} else {
						tiltSearchBall(0.0);
					} tunggu++;
					Walk(0.0, 0.0, 0.0);
				} else {
					trackBall();
					tunggu = 0;

					if (headTilt >= cAktif) {
						if (goalFound) {
							resetCase5();
							stateCondition = 5;
						} else {
							resetCase7();
							stateCondition = 7;
						}
					} else {
						followBall(0);
					}
				}
			}
		} else { //jika gawang dibelakang (next case)
			if (ballLost(20)) {
				if (useCoordination) { semeh = 232; koordinasi.sendRobotCoordinationData(robotID, robotStatus, stateCondition, Grid, 0, semeh, GridBall, backIn); }
				if (tunggu > 40) {
					resetCase1();
					stateCondition = 1;
				} else {
					tiltSearchBall(0.0);
				} tunggu++;
				Walk(0.0, 0.0, 0.0);
			} else {
				trackBall();
				if (useCoordination) { semeh = 1; koordinasi.sendRobotCoordinationData(robotID, robotStatus, 232, Grid, 1, semeh, GridBall, backIn); }
				tunggu = 0;

				if (headTilt >= cAktif) {
					resetCase7();
					stateCondition = 7;
				} else {
					if (useLocalization && useUpdateCoordinate){
						updateCoordinatFromVision();
					}
					followBall(0);
				}
			}
		}
	} else { //without follow search goal (next case)
		if (ballLost(20)) {
			if (useCoordination) { semeh = 232; koordinasi.sendRobotCoordinationData(robotID, robotStatus, stateCondition, Grid, 0, semeh, GridBall, backIn); }
			if (tunggu > 40) {
				resetCase1();
				stateCondition = 1;
			} else {
				tiltSearchBall(0.0);
			} tunggu++;
			Walk(0.0, 0.0, 0.0);
		} else {
			trackBall();
			if (useCoordination) { semeh = 1; koordinasi.sendRobotCoordinationData(robotID, robotStatus, 232, Grid, 1, semeh, GridBall, backIn); }
			tunggu = 0;

			if (headTilt >= cAktif) {
				resetCase7();
				stateCondition = 7;
			} else {
				if (useLocalization && useUpdateCoordinate){
					updateCoordinatFromVision();
				}
				followBall(0);
			}
		}
	}
}

int 	delayWalkWaiting = 0,
	robot1pos_X = 0,
	robot1pos_Y = 0,
	robot3pos_X = 0,
	robot3pos_Y = 0,
	robot1Column = 0,
	robot3Column = 0,
	gridTarget = 0;

void waitingTurn() {
	if (useLocalization) {
		printf("  WAITINGTURN WAITINGTURN WAITINGTURN WAITINGTURN !!!!\n");
		//cek kembali status si eksekutor
		if (robotID == 1) {
			if (koordinasi.robot2State == 1) {
				robot2exeCutor = false;
			}
			if (koordinasi.robot3Status == 1) {
				robot3exeCutor = false;
			}
			if (koordinasi.robot4State == 1) {
				robot4exeCutor = false;
			}
			if (koordinasi.robot5State == 1) {
				robot5exeCutor = false;
			}
		} else if (robotID == 3) {
			if (koordinasi.robot1State == 1) {
				robot1exeCutor = false;
			}
			if (koordinasi.robot2State == 1) {
				robot2exeCutor = false;
			}
			if (koordinasi.robot4State == 1) {
				robot4exeCutor = false;
			}
			if (koordinasi.robot5State == 1) {
				robot5exeCutor = false;
			}
		}

		//pergerakkan kepala & state
		if(ballLost(20)) {
			if (useCoordination) { semeh = 232; koordinasi.sendRobotCoordinationData(robotID, robotStatus, stateCondition, Grid, 0, semeh, GridBall, backIn); }
			if (tunggu > 40) {
				//threeSearchBall();
				if (robotID == 1) {
					if (!robot2exeCutor && !robot3exeCutor && !robot4exeCutor && !robot5exeCutor) {
						resetCase1();
						stateCondition = 1;
					} else {
						threeSearchBall();
					}
				} else if (robotID == 3) {
					if (!robot1exeCutor && !robot2exeCutor && !robot4exeCutor && !robot5exeCutor) {
						resetCase1();
						stateCondition = 1;
					} else {
						threeSearchBall();
					}
				}
				//resetCase1();
				//stateCondition = 1;
			} else {
				searchKe = 0;
				tiltSearchBall(0.0);
			} tunggu ++;
		} else {
			trackBall();
			if (useCoordination) { koordinasi.sendRobotCoordinationData(robotID, robotStatus, stateCondition, Grid, 1, semeh, GridBall, backIn); }
			delay = 0;
			tunggu = 0;
		}

		//kalkulasi posisi robot
		if (robot1exeCutor || robot3exeCutor) {
			if (robot1exeCutor) {
				//Baca koordinat robot eksekusi berdasarkan Grid data
				robot1pos_X = convertGridX(koordinasi.robot1GridPosition, 0);
				robot1pos_Y = convertGridY(koordinasi.robot1GridPosition, 0);

				//Ubah grid pos jadi column
				if (koordinasi.robot1GridPosition % 6 == 0) {
					robot1Column = koordinasi.robot1GridPosition / 6;
				} else {
					robot1Column = (koordinasi.robot1GridPosition / 6) + 1;
				}

				if (robot1Column >= 9) {
					if (robot1pos_Y < 0 ) {
						gridTarget = ((robot1Column-1) * 6) - 2;
					} else if (robot3pos_Y >= 0) {
						gridTarget = ((robot1Column-1) * 6) - 3;
					}
				} else {
					if (robot1pos_Y < 0 ) {
						gridTarget = ((robot1Column) * 6) + 4;
					} else if (robot1pos_Y >= 0) {
						gridTarget = ((robot1Column) * 6) + 3;
					}
				}
			} else if (robot3exeCutor) {
				//Baca koordinat robot eksekusi berdasarkan Grid data
				robot3pos_X = convertGridX(koordinasi.robot3GridPosition, 0);
				robot3pos_Y = convertGridY(koordinasi.robot3GridPosition, 0);

				//Ubah grid pos jadi column
				if (koordinasi.robot3GridPosition % 6 == 0) {
					robot3Column = koordinasi.robot3GridPosition / 6;
				} else {
					robot3Column = (koordinasi.robot3GridPosition / 6) + 1;
				}

				if (robot3Column >= 9) {
					if (robot3pos_Y < 0 ) {
						gridTarget = ((robot3Column-1) * 6) - 2;
					} else if (robot3pos_Y >= 0) {
						gridTarget = ((robot3Column-1) * 6) - 3;
					}
				} else {
					if (robot3pos_Y < 0 ) {
						gridTarget = ((robot3Column) * 6) + 4;
					} else if (robot3pos_Y >= 0) {
						gridTarget = ((robot3Column) * 6) + 3;
					}
				}
			}

			if(doneMoved) {
			delayWaitBall = 0;
			motion("0");

			//Cek kembali posisi robot eksekusi, jika diluar range refresh kemudian jalan lagi
			//if (robot1exeCutor) {
			//	if (Grid > ((robot1Column-1)*6) && (Grid < ((robot1Column+1)*6))) {
			//		motion("0");
			//	} else {
			//		refreshMoveGrid();
			//	}
			//} else if (robot3exeCutor) {
			//	if (Grid > ((robot1Column-1)*6) && (Grid < ((robot1Column+1)*6))) {
				//		motion("0");
			//	} else {
			//		refreshMoveGrid();
			//	}
			//}
			} else {
				motion("9");
				if (delayWaitBall > 10) {
					moveGrid(gridTarget, 0, 0);
				} else {
					Walk(0.0, 0.0, 0.0);
				}
			}
		} else {
			motion("0");
			robot1pos_X = 0,
			robot1pos_Y = 0,
			robot3pos_X = 0,
			robot3pos_Y = 0,
			robot1Column = 0,
			robot3Column = 0,
			gridTarget = 0;
		}
	} else {
		if (ballLost(20)) {
			if (useCoordination) { semeh = 232; koordinasi.sendRobotCoordinationData(robotID, robotStatus, stateCondition, Grid, 0, semeh, GridBall, backIn); }
			if (tunggu > 40) {
				resetCase1();
				stateCondition = 1;
			} else {
				searchKe = 0;
				tiltSearchBall(0.0);
				Walk(0.0, 0.0, 0.0);
			} tunggu++;
		} else {
			trackBall();
			if (useCoordination) { koordinasi.sendRobotCoordinationData(robotID, robotStatus, stateCondition, Grid, 1, semeh, GridBall, backIn); }

			setWaktu();
			delay = 0;
			tunggu = 0;
			robotDirection = false;

			//if (headTilt > -1.3 && headPan >= -0.4 && headPan <= 0.4) {
			//	Walk(-0.03, 0.0, 0.0);
			//} else if (headTilt >= -1.5 && headTilt <= -1.3 && headPan >= -0.4 && headPan <= 0.4) {
			if (headTilt > -1.5 && headPan >= -0.4 && headPan <= 0.4) {
				//if (angle >= 0) { Imu(90, -1.4); }
				//else { Imu(-90, -1.4); }
				if (angle >= 0) { //Imu 90
					if (angle > 80 && angle < 100) {
						if (headTilt > -1.3) {
							bodyXImu = -0.03; alfaImu = 0.0;
						} else {
							bodyXImu = alfaImu = 0.0;
						}
						bodyYImu = errorCPosPan * 0.06;
					} else if ((angle <= 90) && (angle >= -90)) {
						rotateDirec(-1, -1.4); //printf("  rotate ke kanan\n\n");
					} else {
						rotateDirec(1, -1.4); //printf("  rotate ke kiri\n\n");
					} Walk(bodyXImu, bodyYImu, alfaImu);
				} else { //Imu -90
					if (angle < -80 && angle > -100) {
						if (headTilt > -1.3) {
							bodyXImu = -0.03; alfaImu = 0.0;
						} else {
							bodyXImu = alfaImu = 0.0;
						}
						bodyYImu = errorCPosPan * 0.06;
					} else if ((angle <= 90) && (angle >= -90)) {
						rotateDirec(1, -1.4); //printf("  rotate ke kiri\n\n");
					} else {
						rotateDirec(-1, -1.4); //printf("  rotate ke kanan\n\n");
					} Walk(bodyXImu, bodyYImu, alfaImu);
				}
			} else {
				if (useLocalization && useUpdateCoordinate){
					updateCoordinatFromVision();
				}
				followBall(0);
			}
		}
	}
}

//main
void yourTurn() {
	if (robotID == 1) {
		if ( //jika robot saya lebih dekat dengan bola dari robot lain
			(semeh < koordinasi.robot2DBall) &&
			(semeh < koordinasi.robot3DBall) &&
			(semeh < koordinasi.robot4DBall) &&
			(semeh < koordinasi.robot5DBall)
		   ) {
			myTurn();
		} else { //jalan Imu, jika terlalu dekat maka mundur, jika terlalu jauh maka kejar
			waitingTurn();
		}
	} else if (robotID == 2) {
		if ( //jika robot saya lebih dekat dengan bola dari robot lain
			(semeh < koordinasi.robot1DBall) &&
			(semeh < koordinasi.robot3DBall) &&
			(semeh < koordinasi.robot4DBall) &&
			(semeh < koordinasi.robot5DBall)
		   ) {
			myTurn();
		} else { //jalan Imu, jika terlalu dekat maka mundur, jika terlalu jauh maka kejar
			waitingTurn();
		}
	} else if (robotID == 3) {
		if ( //jika robot saya lebih dekat dengan bola dari robot lain
			(semeh < koordinasi.robot1DBall) &&
			(semeh < koordinasi.robot2DBall) &&
			(semeh < koordinasi.robot4DBall) &&
			(semeh < koordinasi.robot5DBall)
		   ) {
			myTurn();
		} else { //jalan Imu, jika terlalu dekat maka mundur, jika terlalu jauh maka kejar
			waitingTurn();
		}
	} else if (robotID == 4) {
		if ( //jika robot saya lebih dekat dengan bola dari robot lain
			(semeh < koordinasi.robot1DBall) &&
			(semeh < koordinasi.robot2DBall) &&
			(semeh < koordinasi.robot3DBall) &&
			(semeh < koordinasi.robot5DBall)
		   ) {
			myTurn();
		} else { //jalan Imu, jika terlalu dekat maka mundur, jika terlalu jauh maka kejar
			waitingTurn();
		}
	} else if (robotID == 5) {
		if ( //jika robot saya lebih dekat dengan bola dari robot lain
			(semeh < koordinasi.robot1DBall) &&
			(semeh < koordinasi.robot2DBall) &&
			(semeh < koordinasi.robot3DBall) &&
			(semeh < koordinasi.robot4DBall)
		   ) {
			myTurn();
		} else { //jalan Imu, jika terlalu dekat maka mundur, jika terlalu jauh maka kejar
			waitingTurn();
		}
	}
}

int	gommen = 0, maxRef = 800,
	countR1 = 0, countR2 = 0, countR3 = 0, countR4 = 0, countR5 = 0,
	lastR1StateBall = 0, lastR2StateBall = 0, lastR3StateBall = 0, lastR4StateBall = 0, lastR5StateBall = 0;
void refreshComm() {
	if (countR1 >= maxRef || countR2 >= maxRef || countR3 >= maxRef || countR4 >= maxRef || countR5 >= maxRef) {
		//printf("\n.....................................................................refresh\n");
		koordinasi.refresh = true;
		if (robotID != 1) { koordinasi.robot1Id = koordinasi.robot1FBall = koordinasi.robot1State = koordinasi.robot1Status = 0; koordinasi.robot1DBall = 232; }
		if (robotID != 2) { koordinasi.robot2Id = koordinasi.robot2FBall = koordinasi.robot2State = koordinasi.robot2Status = 0; koordinasi.robot2DBall = 232; }
		if (robotID != 3) { koordinasi.robot3Id = koordinasi.robot3FBall = koordinasi.robot3State = koordinasi.robot3Status = 0; koordinasi.robot3DBall = 232; }
		if (robotID != 4) { koordinasi.robot4Id = koordinasi.robot4FBall = koordinasi.robot4State = koordinasi.robot4Status = 0; koordinasi.robot4DBall = 232; }
		if (robotID != 5) { koordinasi.robot5Id = koordinasi.robot5FBall = koordinasi.robot5State = koordinasi.robot5Status = 0; koordinasi.robot5DBall = 232; }
		countR1 = countR2 = countR3 = countR4 = countR5 = 0;
	} else {
		koordinasi.refresh = false;
		if (robotID != 1) {
			if (koordinasi.robot1State != 1 && koordinasi.robot1State != 150 && koordinasi.robot1State != 50 && koordinasi.robot1State != 90 && koordinasi.robot1State != 100 && lastR1StateBall == koordinasi.robot1State) { countR1++; }
			else { countR1 = 0; lastR1StateBall = koordinasi.robot1State; }
		}
		if (robotID != 2) {
			if (koordinasi.robot2State != 1 && koordinasi.robot2State != 150 && koordinasi.robot2State != 50 && koordinasi.robot2State != 90 && koordinasi.robot2State != 100 && lastR2StateBall == koordinasi.robot2State) { countR2++; }
			else { countR2 = 0; lastR2StateBall = koordinasi.robot2State; }
		}
		if (robotID != 3) {
			if (koordinasi.robot3State != 1 && koordinasi.robot3State != 150 && koordinasi.robot3State != 50 && koordinasi.robot3State != 90 && koordinasi.robot3State != 100 && lastR3StateBall == koordinasi.robot3State) { countR3++; }
			else { countR3 = 0; lastR3StateBall = koordinasi.robot3State; }
		}
		if (robotID != 4) {
			if (koordinasi.robot4State != 1 && koordinasi.robot4State != 150 && koordinasi.robot4State != 50 && koordinasi.robot4State != 90 && koordinasi.robot4State != 100 && lastR4StateBall == koordinasi.robot4State) { countR4++; }
			else { countR4 = 0; lastR4StateBall = koordinasi.robot4State; }
		}
		if (robotID != 5) {
			if (koordinasi.robot5State != 1 && koordinasi.robot5State != 150 && koordinasi.robot5State != 50 && koordinasi.robot5State != 90 && koordinasi.robot5State != 100 && lastR5StateBall == koordinasi.robot5State) { countR5++; }
			else { countR5 = 0; lastR5StateBall = koordinasi.robot5State; }
		}
	} //printf("\n  %d,%d,%d,%d,%d\n", countR1, countR2, countR3, countR4, countR5);
}

void backToCoordinations() {
	if (robotID == 1) {
		if ( //jika ada robot lain yang sudah masuk case eksekusi
			(koordinasi.robot2State == 7 || koordinasi.robot2State == 3 || koordinasi.robot2State == 8 || koordinasi.robot2State == 4 || koordinasi.robot2State == 5 || koordinasi.robot2State == -10 || koordinasi.robot2State == 10 || koordinasi.robot2State == 20 || koordinasi.robot2State == 30) ||
			(koordinasi.robot3State == 7 || koordinasi.robot3State == 3 || koordinasi.robot3State == 8 || koordinasi.robot3State == 4 || koordinasi.robot3State == 5 || koordinasi.robot3State == -10 || koordinasi.robot3State == 10 || koordinasi.robot3State == 20 || koordinasi.robot3State == 30) ||
			(koordinasi.robot4State == 7 || koordinasi.robot4State == 3 || koordinasi.robot4State == 8 || koordinasi.robot4State == 4 || koordinasi.robot4State == 5 || koordinasi.robot4State == -10 || koordinasi.robot4State == 10 || koordinasi.robot4State == 20 || koordinasi.robot4State == 30) ||
			(koordinasi.robot5State == 7 || koordinasi.robot5State == 3 || koordinasi.robot5State == 8 || koordinasi.robot5State == 4 || koordinasi.robot5State == 5 || koordinasi.robot5State == -10 || koordinasi.robot5State == 10 || koordinasi.robot5State == 20 || koordinasi.robot5State == 30)
		   ) {
			resetCase2();
			stateCondition = 2;
		}
	} else if (robotID == 2) {
		if ( //jika ada robot lain yang sudah masuk case eksekusi
			(koordinasi.robot1State == 7 || koordinasi.robot1State == 3 || koordinasi.robot1State == 8 || koordinasi.robot1State == 4 || koordinasi.robot1State == 5 || koordinasi.robot1State == -10 || koordinasi.robot1State == 10 || koordinasi.robot1State == 20 || koordinasi.robot1State == 30) ||
			(koordinasi.robot3State == 7 || koordinasi.robot3State == 3 || koordinasi.robot3State == 8 || koordinasi.robot3State == 4 || koordinasi.robot3State == 5 || koordinasi.robot3State == -10 || koordinasi.robot3State == 10 || koordinasi.robot3State == 20 || koordinasi.robot3State == 30) ||
			(koordinasi.robot4State == 7 || koordinasi.robot4State == 3 || koordinasi.robot4State == 8 || koordinasi.robot4State == 4 || koordinasi.robot4State == 5 || koordinasi.robot4State == -10 || koordinasi.robot4State == 10 || koordinasi.robot4State == 20 || koordinasi.robot4State == 30) ||
			(koordinasi.robot5State == 7 || koordinasi.robot5State == 3 || koordinasi.robot5State == 8 || koordinasi.robot5State == 4 || koordinasi.robot5State == 5 || koordinasi.robot5State == -10 || koordinasi.robot5State == 10 || koordinasi.robot5State == 20 || koordinasi.robot5State == 30)
		   ) {
			resetCase2();
			stateCondition = 2;
		}
	} else if (robotID == 3) {
		if ( //jika ada robot lain yang sudah masuk case eksekusi
			(koordinasi.robot1State == 7 || koordinasi.robot1State == 3 || koordinasi.robot1State == 8 || koordinasi.robot1State == 4 || koordinasi.robot1State == 5 || koordinasi.robot1State == -10 || koordinasi.robot1State == 10 || koordinasi.robot1State == 20 || koordinasi.robot1State == 30) ||
			(koordinasi.robot2State == 7 || koordinasi.robot2State == 3 || koordinasi.robot2State == 8 || koordinasi.robot2State == 4 || koordinasi.robot2State == 5 || koordinasi.robot2State == -10 || koordinasi.robot2State == 10 || koordinasi.robot2State == 20 || koordinasi.robot2State == 30) ||
			(koordinasi.robot4State == 7 || koordinasi.robot4State == 3 || koordinasi.robot4State == 8 || koordinasi.robot4State == 4 || koordinasi.robot4State == 5 || koordinasi.robot4State == -10 || koordinasi.robot4State == 10 || koordinasi.robot4State == 20 || koordinasi.robot4State == 30) ||
			(koordinasi.robot5State == 7 || koordinasi.robot5State == 3 || koordinasi.robot5State == 8 || koordinasi.robot5State == 4 || koordinasi.robot5State == 5 || koordinasi.robot5State == -10 || koordinasi.robot5State == 10 || koordinasi.robot5State == 20 || koordinasi.robot5State == 30)
		   ) {
			resetCase2();
			stateCondition = 2;
		}
	} else if (robotID == 4) {
		if ( //jika ada robot lain yang sudah masuk case eksekusi
			(koordinasi.robot1State == 7 || koordinasi.robot1State == 3 || koordinasi.robot1State == 8 || koordinasi.robot1State == 4 || koordinasi.robot1State == 5 || koordinasi.robot1State == -10 || koordinasi.robot1State == 10 || koordinasi.robot1State == 20 || koordinasi.robot1State == 30) ||
			(koordinasi.robot2State == 7 || koordinasi.robot2State == 3 || koordinasi.robot2State == 8 || koordinasi.robot2State == 4 || koordinasi.robot2State == 5 || koordinasi.robot2State == -10 || koordinasi.robot2State == 10 || koordinasi.robot2State == 20 || koordinasi.robot2State == 30) ||
			(koordinasi.robot3State == 7 || koordinasi.robot3State == 3 || koordinasi.robot3State == 8 || koordinasi.robot3State == 4 || koordinasi.robot3State == 5 || koordinasi.robot3State == -10 || koordinasi.robot3State == 10 || koordinasi.robot3State == 20 || koordinasi.robot3State == 30) ||
			(koordinasi.robot5State == 7 || koordinasi.robot5State == 3 || koordinasi.robot5State == 8 || koordinasi.robot5State == 4 || koordinasi.robot5State == 5 || koordinasi.robot5State == -10 || koordinasi.robot5State == 10 || koordinasi.robot5State == 20 || koordinasi.robot5State == 30)
		   ) {
			resetCase2();
			stateCondition = 2;
		}
	} else if (robotID == 5) {
		if ( //jika ada robot lain yang sudah masuk case eksekusi
			(koordinasi.robot1State == 7 || koordinasi.robot1State == 3 || koordinasi.robot1State == 8 || koordinasi.robot1State == 4 || koordinasi.robot1State == 5 || koordinasi.robot1State == -10 || koordinasi.robot1State == 10 || koordinasi.robot1State == 20 || koordinasi.robot1State == 30) ||
			(koordinasi.robot2State == 7 || koordinasi.robot2State == 3 || koordinasi.robot2State == 8 || koordinasi.robot2State == 4 || koordinasi.robot2State == 5 || koordinasi.robot2State == -10 || koordinasi.robot2State == 10 || koordinasi.robot2State == 20 || koordinasi.robot2State == 30) ||
			(koordinasi.robot3State == 7 || koordinasi.robot3State == 3 || koordinasi.robot3State == 8 || koordinasi.robot3State == 4 || koordinasi.robot3State == 5 || koordinasi.robot3State == -10 || koordinasi.robot3State == 10 || koordinasi.robot3State == 20 || koordinasi.robot3State == 30) ||
			(koordinasi.robot4State == 7 || koordinasi.robot4State == 3 || koordinasi.robot4State == 8 || koordinasi.robot4State == 4 || koordinasi.robot4State == 5 || koordinasi.robot4State == -10 || koordinasi.robot4State == 10 || koordinasi.robot4State == 20 || koordinasi.robot4State == 30)
		   ) {
			resetCase2();
			stateCondition = 2;
		}
	}
}

// normal search Ball ========================================================================
bool	posRotasi = false;
int	rotasi = 0;
void rotateSearchBall(int rotate) {
	if (rotate >= 0) {
		rotasi = rotate - 180;
		sudut();

		setPoint1 =  20 + rotasi; //20
		setPoint2 = -20 + rotasi; //20

		if (angle > setPoint2 && angle < setPoint1 ) {
			posRotasi = true;
		} else {
			//jalanDirection(0.0, 0.0, rotasi);
			Walk(0.0, 0.0, 0.27);
		}
	} else {
		rotasi = rotate + 180;
		sudut();

		setPoint1 =  20 + rotasi; //20
		setPoint2 = -20 + rotasi; //20

		if (angle > setPoint2 && angle < setPoint1 ) {
			posRotasi = true;
		} else {
			//jalanDirection(0.0, 0.0, rotasi);
			Walk(0.0, 0.0, 0.27);
		}
	}
}

bool	firstRotate  = false,
	secondRotate = false,
	thirdRotate = false,
	fourthRotate = false,

	firstWalk = false,
	secondWalk = false;
void normalSearchBall() {
	//mode1
	/*if (!firstRotate) { //ini step yang pertama kali dilakukan saat masuk case 0
		if (searchKe >= 5) {
			Walk(0.0, 0.0, 0.0); //X, Y, W
			firstRotate  = true;
			searchKe = 0;
		} else if (searchKe >= 2 && searchKe < 5) { //printf("  2.........\n\n");
			SearchBall(2);
			//tiltSearchBall(0.0);
			Walk(0.0, 0.0, 0.15);
		} else { //printf("  1.........\n\n");
			SearchBall(2);
			Walk(0.0, 0.0, 0.0); //X, Y, W
		}
	} else { //setelah rotate pertama tidak dapat, maka cari sambil jalan(dengan imu)
		if (!secondRotate) { //yang pertama kali dilakukan
			if (searchKe >= 3) { //printf("  4.........\n\n");
				SearchBall(2);
				//tiltSearchBall(0.0);
				Walk(0.0, 0.0, 0.12);

				if (searchKe >= 5) {
					searchKe = 0;
					secondRotate  = true;
				}
			} else { //printf("  3.........\n\n");
				SearchBall(2);
				Walk(kejar, 0.0, 0.0);
			}
		} else { //jalan dengan arah sebaliknya
			if (searchKe >= 6) { //printf("  6.........\n\n");
				SearchBall(2);
				if (searchKe >= 8) {
					if (searchKe > 12) {
						jalanDirection(kejar, 0.0, 0);
					} else {
						//searchKe = 0;
						//firstRotate = secondRotate = false;
						Walk(kejar, 0.0, 0.0);
					}
				} else {
					//tiltSearchBall(0.0);
					Walk(0.0, 0.0, 0.12);
				}
			} else { //printf("  5.........\n\n");
				SearchBall(2);
				Walk(kejar, 0.0, 0.0);
			}
		}
	}*/

	//mode 2
	if (!firstRotate) {
		if (matte > 5) {
			if (searchKe == 1) {
				if (posRotasi) {
				        motion("0");
					//Walk(0.0, 0.0, 0.0);
					//jalanDirection(0.0, 0.0, rotasi);     //before
					matte = 0;
					firstRotate = true;
				} else {
				        motion("9");
					//headMove(0.0, -1.6);
					threeSearchBall();
					rotateSearchBall(saveAngle);
				}
			} else {
        			motion("0");
				saveSudutImu();
				threeSearchBall();
				//Walk(0.0, 0.0, 0.0);  //before
			}
		} else {
			posRotasi = false;
			sabar = 0;
			searchKe = 0;
			matte++;
		}
	} else {
		if (!secondRotate) {
			if (matte > 5) {
				if (searchKe == 1) {
					if (posRotasi) {
					        motion("0");
						//Walk(0.0, 0.0, 0.0);
						//jalanDirection(0.0, 0.0, rotasi);     //before
						matte = 0;
						secondRotate = true;
					} else {
					        motion("9");
						//headMove(0.0, -1.6);
						threeSearchBall();
						rotateSearchBall(saveAngle);
					}
				} else {
				        motion("0");
					saveSudutImu();
					threeSearchBall();
					//Walk(0.0, 0.0, 0.0);
					//jalanDirection(0.0, 0.0, saveAngle);  //before
				}
			} else {
				posRotasi = false;
				sabar = 0;
				searchKe = 0;
				matte++;
			}
		} else {
			if (!firstWalk) {
				if (matte > 5) {
					if (searchKe == 2) {
					        motion("0");
						//posRotasi = false;
						saveSudutImu();
						//Walk(0.0, 0.0, 0.0);
						//jalanDirection(0.0, 0.0, saveAngle);  //before
						matte = 0;
						firstWalk = true;
					} else {
					        motion("9");
						threeSearchBall();
						//Walk(kejar, 0.0, 0.0);
						jalanDirection(kejar, 0.0, rotasi);
					}
				} else {
					posRotasi = false;
					sabar = 0;
					searchKe = 0;
					matte++;
				}
			} else {
				if (!thirdRotate) {
					if (matte > 5) {
						if (posRotasi) {
						        motion("0");
							//Walk(0.0, 0.0, 0.0);
							//jalanDirection(0.0, 0.0, rotasi);
							//sabar = 0;
							//searchKe = 0;
							matte = 0;
							thirdRotate = true;
						} else {
						        motion("9");
							threeSearchBall();
							rotateSearchBall(saveAngle);
						}
					} else {
						posRotasi = false;
						sabar = 0;
						searchKe = 0;
						matte++;
					}
				} else {
					if (!secondWalk) {
						if (matte > 5) {
							if (searchKe == 4) {
							        motion("0");
								//posRotasi = false;
								saveSudutImu();
								//Walk(0.0, 0.0, 0.0);
								//jalanDirection(0.0, 0.0, rotasi);
								matte = 0;
								secondWalk = true;
							} else {
							        motion("9");
								threeSearchBall();
								//Walk(kejar, 0.0, 0.0);
								jalanDirection(kejar, 0.0, rotasi);
							}
						} else {
							posRotasi = false;
							sabar = 0;
							searchKe = 0;
							matte++;
						}
					} else {
						if (!fourthRotate) {
							if (matte > 5) {
								if (posRotasi) {
								        motion("0");
									//Walk(0.0, 0.0, 0.0);
									//jalanDirection(0.0, 0.0, rotasi);
									//sabar = 0;
									//searchKe = 0;
									matte = 0;
									fourthRotate = true;
								} else {
								        motion("9");
									threeSearchBall();
									rotateSearchBall(saveAngle);
								}
							} else {
								posRotasi = false;
								sabar = 0;
								searchKe = 0;
								matte++;
							}
						} else {
							if (matte > 5) {
							        motion("9");
								threeSearchBall();
								//Walk(kejar, 0.0, 0.0);
								jalanDirection(kejar, 0.0, rotasi);
							} else {
								posRotasi = false;
								sabar = 0;
								searchKe = 0;
								matte++;
							}
						}
					}
				}
			}
		}
	}
}

bool	rotatePertama	= false,
	rotateKedua	= false,
	rotateKetiga	= false,
	finishSearch	= false,
	searchOne	= false,
	jalanPertama	= false,
	jalanKedua	= false;
int	waitSearchBall = 0;
void normalSearchBallGrid(int valueGrid1, int valueOffSetX1, int valueOffSetY1, int valueGrid2, int valueOffSetX2, int valueOffSetY2) {
        if (!rotatePertama) {
		if (useLocalization && useUpdateCoordinate){
			updateCoordinatFromVision();
		}
                if (waitSearchBall > 5) {
                        if (searchKe >= 1) {
                                if (posRotasi){
                                        motion("0");
                                        waitSearchBall = 0;
                                        rotatePertama = true;
                                } else {
                                        motion("9");
                                        threeSearchBall();
                                        rotateSearchBall(saveAngle);
                                }
                        } else {
                                //motion("9");
                                //Walk(0, 0, 0);
				motion("0");
                                threeSearchBall();
                                saveSudutImu();
                        }
                } else {
                        sabar = 0;
                        posRotasi = false;
                        searchKe = 0;
                        waitSearchBall++;
                }
        } else {
                if (!rotateKedua) {
			if (useLocalization && useUpdateCoordinate){
				updateCoordinatFromVision();
			}
                        if (waitSearchBall > 5) {
                                if (searchKe >= 1) {
                                        if (posRotasi) {
                                                motion("0");
                                                waitSearchBall = 0;
                                                rotateKedua = true;
                                        } else {
                                                motion("9");
                                                threeSearchBall();
                                                rotateSearchBall(saveAngle);
                                        }
                                } else {
                                        motion("0");
                                        threeSearchBall();
                                        saveSudutImu();
                                }
                        } else {
                                sabar = 0;
                                posRotasi = false;
                                searchKe = 0;
                                waitSearchBall++;
                        }
                } else {
                        if (!jalanPertama){
                                if (waitSearchBall > 5) {
                                        if (searchKe >= 1) {
                                                if (doneMoved){
                                                        motion("0");
                                                        waitSearchBall = 0;
                                                        jalanPertama = true;
                                                } else {
                                                        motion("9");
                                                        threeSearchBall();
                                                        moveGrid(valueGrid1,valueOffSetX1,valueOffSetY1);
                                                }
                                        } else {
                                                motion("0");
                                                threeSearchBall();
                                        }
                                } else {
                                        refreshMoveGrid();
                                        sabar = 0;
                                        searchKe = 0;
                                        waitSearchBall++;
                                }
                        } else {
                                if (!rotateKetiga) {
					if (useLocalization && useUpdateCoordinate){
						updateCoordinatFromVision();
					}
                                        if (waitSearchBall > 5) {
                                                if (searchKe >= 1) {
                                                        if (posRotasi) {
                                                                motion("0");
                                                                waitSearchBall = 0;
                                                                rotateKetiga = true;
                                                        } else {
                                                                motion("9");
                                                                threeSearchBall();
                                                                rotateSearchBall(saveAngle);
                                                        }
                                                } else {
                                                        motion("0");
                                                        saveSudutImu();
                                                        threeSearchBall();
                                                }
                                        } else {
                                                sabar = 0;
                                                searchKe = 0;
                                                posRotasi = false;
                                                waitSearchBall++;
                                        }
                                } else {
                                        if (!jalanKedua) {
                                                if (waitSearchBall > 5) {
                                                        if (searchKe >= 1) {
                                                                if (doneMoved) {
                                                                        motion("0");
                                                                        waitSearchBall = 0;
                                                                        jalanKedua = true;
                                                                } else {
                                                                        motion("9");
                                                                        threeSearchBall();
                                                                        moveGrid(valueGrid2,valueOffSetX2,valueOffSetY2);
                                                                }
                                                        } else {
                                                                motion("0");
                                                                threeSearchBall();
                                                        }
                                                } else {
                                                        refreshMoveGrid();
                                                        sabar = 0;
                                                        searchKe = 0;
                                                        waitSearchBall++;
                                                }
                                        } else {
                                                if (!finishSearch) {	//rotate
                                                        if (waitSearchBall > 5) {
                                                                if (searchKe >= 1) {
                                                                        if (posRotasi){
                                                                                motion("0");
                                                                                waitSearchBall = 0;
                                                                                finishSearch = true;
                                                                        } else {
                                                                                motion("9");
                                                                                threeSearchBall();
                                                                                rotateSearchBall(saveAngle);
                                                                        }
                                                                } else {
                                                                        motion("0");
                                                                        saveSudutImu();
                                                                        threeSearchBall();
                                                                }
                                                        } else {
                                                                sabar = 0;
                                                                searchKe = 0;
                                                                posRotasi = false;
                                                                waitSearchBall++;
                                                        }
                                                } else {
							if (useLocalization && useUpdateCoordinate){
								updateCoordinatFromVision();
							}
                                                        if (!searchOne) {
                                                                //printf("SEARCH ONE FALSEEEEEE\n");
                                                                if (waitSearchBall > 5) {
                                                                        if (searchKe >=2) {
                                                                                if (posRotasi) {
                                                                                        motion("0");
                                                                                        waitSearchBall = 0;
                                                                                        searchOne = true;
                                                                                } else {
                                                                                        motion("9");
                                                                                        threeSearchBall();
                                                                                        rotateSearchBall(saveAngle);
                                                                                }
                                                                        } else {
                                                                                motion("0");
                                                                                threeSearchBall();
                                                                                saveSudutImu();
                                                                        }
                                                                } else {
                                                                        sabar = 0;
                                                                        searchKe = 0;
                                                                        posRotasi = false;
                                                                        waitSearchBall++;
                                                                }
                                                        } else if (searchOne) {
                                                                printf("SEARCH ONE TRUEEEEE\n");
                                                                if (waitSearchBall > 5) {
                                                                        if (searchKe >=2) {
                                                                                if (posRotasi) {
                                                                                        motion("0");
                                                                                        waitSearchBall = 0;
                                                                                        searchOne = false;
                                                                                } else {
                                                                                        motion("9");
                                                                                        threeSearchBall();
                                                                                        rotateSearchBall(saveAngle);
                                                                                }
                                                                        } else {
                                                                                motion("0");
                                                                                threeSearchBall();
                                                                                saveSudutImu();
                                                                        }
                                                                } else {
                                                                        sabar = 0;
                                                                        searchKe = 0;
                                                                        posRotasi = false;
                                                                        waitSearchBall++;
                                                                }
                                                        }
                                                }
                                        }
                                }
                        }
		}
	}
}
// Reset Variabel ============================================================================
void resetCaseAwal() { //strategi awal
	kondisiBola		=
	elapsedTime		=
	second			=
	reset			=
	cntOke1			=
	cntOke2			=
	cntUlang		=
	delay 			=
	count 			=
	tunggu 			=
	Waktu			=
	Timing			= 0;

	modeKick		= tendangJauh;

	ulang			=
	followSearchAktif	= true;

	timer			=
	oke			=
	tracked			=
	posRotate		=
	robotDirection		=
	tendang			= false;
}

void resetCase1() { //search ball
	delayWaitBall	=
	reset		=
	tracked 	= false;
	i		= 1;
	if (useLocalization) {
		if (stateCondition != 1 || finishSearch) {
			//tipe data int
			saveAngle	=
			tiltPos		=
			searchKe	=
			waitSearchBall	=
			rotasi		=
			sabar		= 0;
			//tiltPos		= 0;

			//tipe data boolean
			rotatePertama	=
			rotateKedua	=
			rotateKetiga	=
			jalanPertama	=
			jalanKedua	=
			searchOne	=
			finishSearch 	= false;
		}
	} else {
		saveAngle	=
		searchKe	=	//counting berapa kali search
		rotasi		=
		sabar		=
		matte		=
		tiltPos		= 0;

		posRotasi	=
		firstRotate	=
		secondRotate	=
		thirdRotate	=
		fourthRotate	=
		firstWalk	=
		secondWalk	= false;
	}
}

void resetCase2() { //cek koordinasi, follow, n cari gawang
	searchKe		=
	tunggu			=
	delay			=
	delayTrackBall		=
	delayWalkWaiting	=
	//elapsedTime		=
	//second			=
	reset			=
	waiting			=
	errorGoal		=
	goalPan			=
	saveAngle		=
	prediksiGoalPan		=
	chotto			= 0;

	modeKick		= tendangJauh;

	Activated		= true;

	exeCutor		=
	FollowGoal		=
	searchGoalFinish	=
	//timer			=
	ballPos			=
	goalSearch		=
	ballposTracked		=
	goalFound		=
	breaked			=
	robotDirection		= false;
	refreshMoveGrid();
}

void resetCase7() { //imus
	elapsedTime	=
	second		=
	reset		=
	delay		=
	delayTrackGoal	=
	tunggu		= 0;

	modeKick	= tendangJauh;

	timer		=
	robotDirection	= false;
}

void resetCase3() { //dribble
	searchKe	=
	tunggu		=
	countDribble	= 0;

	robotDirection	= false;
}

void resetCase8() { //robot must to check direction again after dribble
	searchKe	=
	tunggu		=
	delay		= 0;

	robotDirection	= false;
}

void resetCase4() { //search goal
	searchKe		=
	tunggu			=
	Rotate			=
	delay			=
	waiting			=
	elapsedTime		=
	second			=
	reset			=
	count			=
	bodyP_ControllerG	=
	bodyTrueG		=
	delayTrueG		=
	errorGoal		=
	goalPan			=
	saveAngle		=
	prediksiGoalPan		= 0;

	modeKick		= tendangJauh;

	searchGoalFinish	=
	timer			=
	ballPos			=
	goalSearch		=
	rotateGoal		=
	ballposTracked		=
	errorGoal		=
	robotDirection		= false;
}

void resetCase5() { //kick
	searchKe	=
	delay		=
	tunggu		= 0;

	kanan		=
	kiri		=
	ballPos 	=
	tendang 	= false;
}

void resetCase6() { //search after kick
	searchKe	=
	tunda		=
	tunggu		=
	elapsedTime	=
	second		=
	delay		=
	confirmsBall	=
	sumTilt		=
	waitTracking	=
	countTilt	= 0;

	neckX		= true;

	searchRectangle =
	timer		= false;
}

void resetOdometry(){
	robotPos_X		=
	robotPos_Y		=
	odometry.ArchSinEnemy	=
	odometry.csmKiri	=
	odometry.csmKanan	=
	odometry.walkTot	=
	odometry.deltaPos_X	=
	varCount		=
	odometry.deltaPos_Y	=
	odometry.walkTotMun	= 0;
}

void refreshMoveLokalisasi(){
	cL = 	//count lock for atribut target
	cD =	//count peralihan dari rotate ke jalan
	cR =	//count peralihan dari break ke rotate
	cZ = cV = cN = cY =
	cB = 0;	//count untuk break ketika sudah didalam setpoint target
	cX = false,
	donePosition = false;
}

void refreshMoveGrid() {
	posRotateNew = false;
	doneMoved = false;
	countMoveGrid1 =
	countMoveGrid2 =
	countMoveGrid3 = 0;
}

void resetKoordinasiRobotBalik(){
	balikTengah = false;
	balikSampingKanan = false;
	balikSampingKiri = false;
}

void refreshRecelerLokalisasi(){
	receler.RobotCoor_X	=
	receler.RobotCoor_Y	=
	receler.BallCoor_X	=
	receler.BallCoor_Y	= 0;
}

void resetAllLokalisasiVariable() {
	lockLeftBack	=
	lockRightBack	=
	kembaliMasuk	=
	lockMidBack 	=
	awalMasuk	= false;

	//resetOdometry();
	//refreshRecelerLokalisasi();
	refreshMoveLokalisasi();
}

void resetAllVariable() { //setelah tendang
	resetCaseAwal();
	resetCase1();
	resetCase2();
	resetCase3();
	resetCase4();
	resetCase5();
	resetCase6();
	resetCase7();
	resetCase8();

	kembaliMasuk = false,
	awalMasuk = false,
	lockLeftBack = false,
	lockMidBack 	= false,
	lockRightBack = false;
	resetKoordinasiRobotBalik();
	resetAllLokalisasiVariable();
}

// Load Parameter configure.ini ==============================================================
double value = 0;
void loadConfig(){
	//geti -> tipe data INT
	//getd -> tipe data double
	//getf -> tipe data float
	//getl -> tipe data long

	if((value = ini -> geti("config", "robotNumber", INVALID_VALUE)) != INVALID_VALUE)  	robotNumber = value;
	if((value = ini -> geti("config", "frame_X", INVALID_VALUE)) != INVALID_VALUE)  	frame_X = value;
	if((value = ini -> geti("config", "frame_Y", INVALID_VALUE)) != INVALID_VALUE)  	frame_Y = value;
	if((value = ini -> geti("config", "tendangJauh", INVALID_VALUE)) != INVALID_VALUE)   	tendangJauh = value;
	if((value = ini -> geti("config", "tendangDekat", INVALID_VALUE)) != INVALID_VALUE)   	tendangDekat = value;
	if((value = ini -> geti("config", "tendangSamping", INVALID_VALUE)) != INVALID_VALUE)   tendangSamping = value;
	if((value = ini -> geti("config", "sudutTengah", INVALID_VALUE)) != INVALID_VALUE)   	sudutTengah = value;
	if((value = ini -> geti("config", "sudutKanan", INVALID_VALUE)) != INVALID_VALUE)   	sudutKanan = value;
	if((value = ini -> geti("config", "sudutKiri", INVALID_VALUE)) != INVALID_VALUE)   	sudutKiri = value;

	if((value = ini -> getd("config", "ball_panKP", INVALID_VALUE)) != INVALID_VALUE)	ball_panKP = value;
	if((value = ini -> getd("config", "ball_panKD", INVALID_VALUE)) != INVALID_VALUE)   	ball_panKD = value;
	if((value = ini -> getd("config", "ball_tiltKP", INVALID_VALUE)) != INVALID_VALUE)	ball_tiltKP = value;
	if((value = ini -> getd("config", "ball_tiltKD", INVALID_VALUE)) != INVALID_VALUE)   	ball_tiltKD = value;

	if((value = ini -> getd("config", "goal_panKP", INVALID_VALUE)) != INVALID_VALUE)	goal_panKP = value;
	if((value = ini -> getd("config", "goal_panKD", INVALID_VALUE)) != INVALID_VALUE)   	goal_panKD = value;
	if((value = ini -> getd("config", "goal_tiltKP", INVALID_VALUE)) != INVALID_VALUE)	goal_tiltKP = value;
	if((value = ini -> getd("config", "goal_tiltKD", INVALID_VALUE)) != INVALID_VALUE)   	goal_tiltKD = value;

	if((value = ini -> getd("config", "ballPositioningSpeed", INVALID_VALUE)) != INVALID_VALUE) ballPositioningSpeed = value;
	if((value = ini -> getd("config", "pPanTendang", INVALID_VALUE)) != INVALID_VALUE)   	pPanTendang = value;
	if((value = ini -> getd("config", "pTiltTendang", INVALID_VALUE)) != INVALID_VALUE)   	pTiltTendang = value;
	if((value = ini -> getd("config", "pPanOper", INVALID_VALUE)) != INVALID_VALUE)   	pPanOper = value;
	if((value = ini -> getd("config", "pTiltOper", INVALID_VALUE)) != INVALID_VALUE)   	pTiltOper = value;
	if((value = ini -> getd("config", "cSekarang", INVALID_VALUE)) != INVALID_VALUE)   	cSekarang = value;
	if((value = ini -> getd("config", "cAktif", INVALID_VALUE)) != INVALID_VALUE)   	cAktif = value;
	if((value = ini -> getd("config", "posTiltLocal", INVALID_VALUE)) != INVALID_VALUE)   	posTiltLocal = value;
	if((value = ini -> getd("config", "posTiltGoal", INVALID_VALUE)) != INVALID_VALUE)   	posTiltGoal = value;
	if((value = ini -> getd("config", "erorrXwalk", INVALID_VALUE)) != INVALID_VALUE)   	erorrXwalk = value;
	if((value = ini -> getd("config", "erorrYwalk", INVALID_VALUE)) != INVALID_VALUE)   	erorrYwalk = value;
	if((value = ini -> getd("config", "erorrAwalk", INVALID_VALUE)) != INVALID_VALUE)   	erorrAwalk = value;
	if((value = ini -> getd("config", "jalan", INVALID_VALUE)) != INVALID_VALUE)   		jalan = value;
	if((value = ini -> getd("config", "lari", INVALID_VALUE)) != INVALID_VALUE)   		lari = value;
	if((value = ini -> getd("config", "kejar", INVALID_VALUE)) != INVALID_VALUE)   		kejar = value;
	if((value = ini -> getd("config", "kejarMid", INVALID_VALUE)) != INVALID_VALUE)   	kejarMid = value;
	if((value = ini -> getd("config", "kejarMax", INVALID_VALUE)) != INVALID_VALUE)   	kejarMax = value;
	if((value = ini -> getd("config", "rotateGoal_x", INVALID_VALUE)) != INVALID_VALUE)   	rotateGoal_x = value;
	if((value = ini -> getd("config", "rotateGoal_y", INVALID_VALUE)) != INVALID_VALUE)   	rotateGoal_y = value;
	if((value = ini -> getd("config", "rotateGoal_a", INVALID_VALUE)) != INVALID_VALUE)   	rotateGoal_a = value;
	if((value = ini -> getd("config", "myAccrX", INVALID_VALUE)) != INVALID_VALUE)   	myAccrX = value;
	if((value = ini -> getd("config", "myAccrY", INVALID_VALUE)) != INVALID_VALUE)   	myAccrY = value;
	if((value = ini -> getd("config", "tinggiRobot", INVALID_VALUE)) != INVALID_VALUE)   	tinggiRobot = value;
	if((value = ini -> getd("config", "outputSudutY1", INVALID_VALUE)) != INVALID_VALUE)   	outputSudutY1 = value;
	if((value = ini -> getd("config", "inputSudutY1", INVALID_VALUE)) != INVALID_VALUE)   	inputSudutY1 = value;
	if((value = ini -> getd("config", "outputSudutY2", INVALID_VALUE)) != INVALID_VALUE)   	outputSudutY2 = value;
	if((value = ini -> getd("config", "inputSudutY2", INVALID_VALUE)) != INVALID_VALUE)   	inputSudutY2 = value;
	if((value = ini -> getd("config", "outputSudutX1", INVALID_VALUE)) != INVALID_VALUE)   	outputSudutX1 = value;
	if((value = ini -> getd("config", "inputSudutX1", INVALID_VALUE)) != INVALID_VALUE)   	inputSudutX1 = value;
	if((value = ini -> getd("config", "outputSudutX2", INVALID_VALUE)) != INVALID_VALUE)   	outputSudutX2 = value;
	if((value = ini -> getd("config", "inputSudutX2", INVALID_VALUE)) != INVALID_VALUE)  	inputSudutX2 = value;
}

void display() {
	printf ("\n  signIn = %d,", signIn);
	//printf ("  TIMER1 = %d\n", timer);
	//printf ("  detik1 = %.2lf\n", second);
	//printf ("  KondisiBola = %d\n",kondisiBola);
	//printf ("\n  robotDirection = %d,", robotDirection);
	//printf ("  useImu = %d,\n", useImu);
	printf ("\n  RobotStatus = %d,\n", robotStatus);
	printf ("\n  zeroState = %d,\n", zeroState);
	//printf ("\n  pickUp = %d,\n", pickUp);
	//---------------------------------------------------------
	printf("\n  Ball_X = %d,  Ball_Y = %d, Ball_D = %d, \n", receler.Ball_X, receler.Ball_Y, receler.Ball_D);
	printf("  Goal_X = %d,  Goal_Y = %d, Goal_LD = %d, Goal_RD = %d, \n", receler.Goal_X, receler.Goal_Y, receler.Goal_LD, receler.Goal_RD);
	printf("  Pinalty_D - %d, \n", receler.Pinalty_D);
	printf("  Lcross_LD - %d,  Lcross_RD = %d, \n", receler.Lcross_LD, receler.Lcross_RD);
	printf("  Xcross_LD - %d,  Xcross_RD = %d, \n", receler.Xcross_LD, receler.Xcross_RD);
	printf("  Tcross_LD - %d,  Tcross_RD = %d, \n", receler.Tcross_LD, receler.Tcross_RD);
	//printf("  initialPos_X = %2.lf, \n", initialPos_X);
	//printf("  initialPos_Y = %2.lf, \n", initialPos_Y);
	//---------------------------------------------------------
	printf("\n  stateGameController = %d,", stateGameController);
	//printf("  lastStateGameController = %d,", lastStateGameController);
	printf("  stateCondition = %d,", stateCondition);
	//printf("  stateChange = %d,\n", stateChange);
	printf("  Strategi = %d, \n", Strategi);
	//---------------------------------------------------------
	printf("\n  kickOff = %d,", kickOff);
	printf("  lastKickOff = %d,\n", lastKickOff);
	//---------------------------------------------------------
	//printf("\n  secsRemaining = %d,", secRemaining);
	//printf("  lastSecsRemaining = %d,\n", lastSecRemaining);
	//---------------------------------------------------------
	//getSensor();
	//printf("  accr(%.1lf, %.1lf, %.1lf),", accrX, accrY, accrZ);
	//printf("  gyro(%.lf, %.lf, %.lf),", gyroX, gyroY, gyroZ);
	//printf("  angle(%.lf, %.lf, %.lf),", angleX, angleY, angleZ);
	//---------------------------------------------------------
	//sudut();
	//printf("\n  Bearing = %d,", strategy.bearingValue);
	printf("  Sudut = %.f, \n", angle);
	printf("  Sudut Offset = %.f, \n", strategy.offset);
	printf("  Sudut Kalkulasi = %.f, \n", odometry.ArchSinEnemy);
	//printf("  Sudut OutGrid = %.f, \n", outGrid);
	//printf("  Odometry Robot Koordinat X = %.2lf\n", robotPos_X);
	//printf("  Odometry Robot Koordinat Y = %.2lf\n", robotPos_Y);
	//---------------------------------------------------------
	printf("\n  posPan = %.2f,  posTilt = %.2f,", posPan, posTilt);
	//printf("  lastPan = %.2f,  lastTilt = %.2f,", ballPan, ballTilt);
	//printf("  searchKe = %.f,", searchKe);
	//printf("  Matte = %d,", matte);
	//printf("  semeh = %d,", semeh);
	//---------------------------------------------------------
	printf("\n  BarelangFC%d : Found = %d, \t Dis = %d, \t State = %d, \t Play = %d, \n", koordinasi.robot1Id, koordinasi.robot1FBall, koordinasi.robot1DBall, koordinasi.robot1State, koordinasi.robot1Status);
	printf("  BarelangFC%d : Found = %d, \t Dis = %d, \t State = %d, \t Play = %d, \n", koordinasi.robot2Id, koordinasi.robot2FBall, koordinasi.robot2DBall, koordinasi.robot2State, koordinasi.robot2Status);
	printf("  BarelangFC%d : Found = %d, \t Dis = %d, \t State = %d, \t Play = %d, \n", koordinasi.robot3Id, koordinasi.robot3FBall, koordinasi.robot3DBall, koordinasi.robot3State, koordinasi.robot3Status);
	printf("  BarelangFC%d : Found = %d, \t Dis = %d, \t State = %d, \t Play = %d, \n", koordinasi.robot4Id, koordinasi.robot4FBall, koordinasi.robot4DBall, koordinasi.robot4State, koordinasi.robot4Status);
	printf("  BarelangFC%d : Found = %d, \t Dis = %d, \t State = %d, \t Play = %d, \n", koordinasi.robot5Id, koordinasi.robot5FBall, koordinasi.robot5DBall, koordinasi.robot5State, koordinasi.robot5Status);
	if (useCoordination){
		printf("  BarelangFC%d : Tengah = %d, \t Kanan = %d, \t Kiri = %d, \n", robotID, balikTengah, balikSampingKanan, balikSampingKiri);
	} else {
		printf("  BarelangFC%d : Tengah = %d, \t Kanan = %d, \t Kiri = %d, \n", robotID, lockMidBack, lockRightBack, lockLeftBack);
	}
	if (robotID == 1) {
		printf("\n  Robot1exeCutor = %d\tRobot2exeCutor = %d\tRobot3exeCutor = %d\n  Robot4exeCutor = %d\tRobot5exeCutor = %d\n", exeCutor, robot2exeCutor, robot3exeCutor, robot4exeCutor, robot5exeCutor);
	} else if (robotID == 3) {
		printf("\n  Robot1exeCutor = %d\tRobot2exeCutor = %d\tRobot3exeCutor = %d\n  Robot4exeCutor = %d\tRobot5exeCutor = %d\n", robot1exeCutor, robot2exeCutor, exeCutor, robot4exeCutor, robot5exeCutor);
	}
	//---------------------------------------------------------
	//getServPos();
	//printf("\n  head(%.lf, %.lf),", head1, head2);
	//printf("  Larm(%.lf, %.lf, %.lf),", Larm1, Larm2, Larm3);
	//printf("  Rarm(%.lf, %.lf, %.lf),", Rarm1, Rarm2, Rarm3);
	//printf("  Lleg(%.lf, %.lf, %.lf, %.lf, %.lf, %.lf),", Lleg1, Lleg2, Lleg3, Lleg4, Lleg5, Lleg6);
	//printf("  Rleg(%.lf, %.lf, %.lf, %.lf, %.lf, %.lf),\n", Rleg1, Rleg2, Rleg3, Rleg4, Rleg5, Rleg6);
	//---------------------------------------------------------
	//kalkulasiJarakBola();
	//printf("\n  jarakBola = %.f,", jarakBola);
	//printf("  jarakBola_Y = %.f,", jarakBola_Y);
	//printf("  jarakBola_X = %.f,\n", jarakBola_X);
	//hitungGerakBola();
	//printf("  deltaY = %.f,", deltaY);
	//printf("  deltaX = %.f,", deltaX);
	//printf("  gradient = %.f,", gradient);
	//---------------------------------------------------------
	//printf("  countDribble = %d,", countDribble);
	//---------------------------------------------------------
	//printf("  data motion = %s,", dataMotion);
	//printf("  data head = %s,", dataHead);
	//---------------------------------------------------------
	//Robocup GameController Data
	//printf("\n\n  Game State = %d,", gameController.State);
	//printf("\n  Version = %d,", gameController.Version);
	//printf("\n  packetNumber = %d,", gameController.PacketNumber);
	//printf("\n  playersPerTeam = %d,", gameController.PlayerTeam);
	//printf("\n  gameType = %d,", gameController.GameTipe);
	//printf("\n  firstHalf = %d,", gameController.FirstHalf);
	//printf("\n  kickOffTeam = %d,", gameController.KickOff);
	//printf("\n  secondaryState = %d,", gameController.SecondaryState);
	//printf("\n  dropInTeam = %d,", gameController.DropTeam);
	//printf("\n  dropInTime = %d,", gameController.DropTime);
	//printf("\n  secsRemaining = %d,", gameController.Remaining);
	//printf("\n  secondaryTime = %d,", gameController.SecondaryTime);

	//Team Info left
	//printf("\n\n  TeamNumber = %d,", gameController.timNumber1);
	//printf("\n  teamColour = %d,", gameController.timColour1);
	//printf("\n  score = %d,", gameController.Score1);
	//printf("\n  penaltyShot = %d,", gameController.Penaltyshoot1);
	//printf("\n  singleShots = %d,", gameController.Singleshoot1);
	//printf("\n  coachSequence = %d,", gameController.Coachsequence1);

	//Team Info Right
	//printf("\n\n  TeamNumber = %d,", gameController.timNumber2);
	//printf("\n  teamColour = %d,", gameController.timColour2);
	//printf("\n  score = %d,", gameController.Score2);
	//printf("\n  penaltyShot = %d,", gameController.Penaltyshoot2);
	//printf("\n  singleShots = %d,", gameController.Singleshoot2);
	//printf("\n  coachSequence = %d,", gameController.Coachsequence2);

	//Robot Info Left
	//printf("\n\n  penalty = %d,", gameController.Penalty1);
	//printf("\n  secsTillUnpenalised = %d,", gameController.TimeUnpenalis1);
	//printf("\n  yellowCardCount = %d,", gameController.YellowCard1);
	//printf("\n  redCardCount = %d,", gameController.RedCard1);

	//Robot Info Right
	//printf("\n\n  penalty = %d,", gameController.Penalty2);
	//printf("\n  secsTillUnpenalised = %d,", gameController.TimeUnpenalis2);
	//printf("\n  yellowCardCount = %d,", gameController.YellowCard2);
	//printf("\n  redCardCount = %d,", gameController.RedCard2);

	//printf("\n ---------------PARAMETER LOKALISASI---------------\n");
	printf("\n  Robot Koordinat X = %.2lf\tRobot Koordinat Y = %.2lf\n  GridPosition Robot = %d\n\n", robotPos_X, robotPos_Y, Grid);
	printf("  Ball Koordinat X = %d\t\tBall Koordinat Y = %d\n  GridPosition Ball = %d\n", ballPos_X, ballPos_Y, GridBall);
	printf("\n  GridTarget = %d\n", gridTarget);
	//printf("  Odometry Rotate  = %.2lf\n", odometry.ArchSinEnemy);
	printf("\n");
}

void displayMoveLokalisasi(){
//	printf(" Arah Pandang = %.f , Goal Side = %d\n", arahPandang, goalSide);
	printf ("\n signIn = %d,", signIn);
	printf("\n stateGameController = %d,", stateGameController);
//	printf(" Robot Bergerak = %d\n", odometry.robotBergerak);
//	printf(" Walk X = %.2lf, Y = %.2lf, A = %.2lf\n", odometry.RobotWalk_X, odometry.RobotWalk_Y, odometry.RobotWalk_A);
	printf(" Sudut = %.f\n", angle);
	printf(" CountInitPos = %d\n", countInitPos);
	printf(" stateCondition = %d\tRobotStatus = %d\n", stateCondition, robotStatus);
	printf(" Balik Tengah = %d\t Balik Kanan = %d\t Balik Kiri = %d\n", balikTengah, balikSampingKanan, balikSampingKiri);
	printf(" pickUp = %d\t BackIn = %d\n", pickUp, backIn);
	printf(" totalRobotStatus = %d\ttotalRobotStatusBackIn = %d\n", totalRobotStatus, totalRobotStatusBackIn);
	printf(" TIMER1 = %.2lf\n", second);
	printf(" FinishSearch = %d\n", finishSearch);
//	printf(" InitialGrid = %d\tOffsetX = %d\tOffsetY = %d\n", initGrid, offsetX, offsetY);
	/*
	if(walkX < 0){
		printf(" Jumlah Mundur = %d\n", odometry.walkTotMun);
	} else if (walkX > 0){
		printf(" Jumlah Maju = %d\n", odometry.walkTot);
	}
	if(walkY > 0) {
		printf(" Jumlah Kiri = %d\n", odometry.csmKiri);
	} else if(walkY < 0) {
		printf(" Jumlah Kanan = %d\n", odometry.csmKanan);
	}
	*/
//	printf(" Jarak gawang = %d\n",receler.Goal_LD);
	//printf(" Initial Pos X = %.2lf, Y = %.2lf\n ", initialPos_X, initialPos_Y);
//	printf(" Odometry  X = %.2lf, Y = %.2lf\n ", odometry.deltaPos_X, odometry.deltaPos_Y);
	//printf(" POSISI SEKARANG X = %.2lf , Y = %.2lf\n", robotPos_X, robotPos_Y);
//	printf(" POSISI TARGET   X = %.2lf , Y = %.2lf\n", Xtar, Ytar);
//	printf(" Angle = %.f , AngleTar = %.f\n", angle, setRotatePointR);
//	printf(" SP1 = %.f , SP2 = %.f\n", setPoint1, setPoint2);
//	printf(" Done Position  = %d\n", donePosition);
//	printf(" CountLock	= %d\n", cL);
//	printf(" CountBreak	= %d\n",cB);
//	printf(" CountRotate	= %d\n",cR);
//	printf(" CountDiam	= %d\n",cD);
//	printf(" CZ = %d , CX = %d , CV = %d , CN = %d\n", cZ, cX, cV, cN);
	printf("\n");
}

void setting() {
//========================================================================
	modePlay		= 1;
	usePenaltyStrategy	= false; //untuk pinalti
	useVision		= true;
	useSocket		= true;
	useImu			= true;
	useGameController	= true;
	useCoordination		= true;
	useLocalization		= true;
	useUpdateCoordinate	= false;
	useFollowSearchGoal	= false;
	useSearchGoal		= false;
	useDribble		= false;
	dribbleOnly		= false;
	useSideKick		= true;
	useLastDirection	= true;
	useNearFollowSearchGoal	= false;
	firstStateCondition	= 0; //-2; //0;
//========================================================================
}

/////////////////////////////////////////////////////////
/////////////////// Main_Program ////////////////////////
/////////////////////////////////////////////////////////
int main(void) {
	loadConfig(); //robot parameters
	setting();

	//open socket for head&motion
	if (useSocket) {
		initSendDataMotion();
		initSendDataHead();
	}

	//open socket for lokalisasi
	initSendDataLokalisasi();

	//open socket for monitoring
	monitor.initSendMonitor();


	runLuaProgram();
	motion("8"); //robot berdiri

	pthread_create(&threadMonitor,NULL, sendMonitoring, NULL);
	pthread_create(&threadStrategy,NULL, updateStrategy, NULL);
	//pthread_create(&threadVision, NULL, camera, NULL);
	//pthread_create(&threadPID, NULL, camera, NULL);
	pthread_create(&threadReceiverVision, NULL, receiverVision, NULL);
	pthread_create(&threadReceiverLocalization, NULL, receiverLocalization, NULL);
	pthread_create(&threadGC_Read, NULL, gcReadData, NULL);
	pthread_create(&threadGC_Return, NULL, gcReturnData, NULL);
	pthread_create(&Lokalisasi, NULL, sendData, NULL);

	//GameController
	gameController.Player	= robotID;
	gameController.Team	= TEAM;
	gameController.Server	= SERVER;

	//Coordination
	koordinasi.RobotID	= robotID;

	//communications
	if (useCoordination) {
		koordinasi.initCommunicationIn();
		koordinasi.initCommunicationOut();
	}

	/////////////////////////////////
	////////// first doing //////////
	/////////////////////////////////
	motion("0"); //robot diam
	headMove(0.0, -1.2);

	while(1) {
		usleep(40000);
		monitor.sendMonitor(robotID, robotPos_X, robotPos_Y, strategy.bearingValue, outGrid, stateCondition, ballPos_X, ballPos_Y);
		getImuSensor();
		getSensor();
		sudut();
		cekArah();
		display();
//		displayMoveLokalisasi();

		if (useLocalization){
			if (strategy.strategyNumber != 4) {
				getServPos();
				gridLocalization();
				gridBall();
				gridCoor();
				odometry.bacaKondisi(odometry.RobotWalk_X, odometry.RobotWalk_Y);

				robotPos_X = odometry.deltaPos_X + initialPos_X;
				robotPos_Y = odometry.deltaPos_Y + initialPos_Y;

				odometry.RobotPos_X = robotPos_X;
				odometry.RobotPos_Y = robotPos_Y;
				odometry.trigonoMetri();
			}
		} else { followSearchAktif = true; }

		if (useLastDirection == true && lastDirection != 0) {
			if ((angle < lastDirection+15) && (angle > lastDirection-15)) {
				useImu = false;
				//robotDirection = true;
			} else {
				lastDirection = 0;
				useImu = true;
				//robotDirection = true;
			}
		}

		//INITIAL POSISI
		if (modePlay == 0) { //INTIAL POSITION NASIONAL
			if(useGameController) {
				if(!play && gameController.State == 0 && !usePenaltyStrategy) {
					if ((!pickUp && gameController.Remaining == 600 &&  gameController.SecondaryState == 0) || (!pickUp && gameController.Remaining == 300 &&  gameController.SecondaryState == 2)) {
						if (strategy.strategyNumber <=4 || strategy.strategyNumber == 13) {	//Initial nilai odometry masuk tengah
							if (countInitPos == 0) {
								initialPos_X = -358;
								initialPos_Y = 0;
								if (robotStatus == 1) {
									countInitPos = 1;
								} else {
									countInitPos = 0;
								}
							}
						} else {								//Initial nilai odometry masuk samping
							if ( angle >= 0 && countInitPos == 0) {				//Support Kanan
								initialPos_X = -358;
								initialPos_Y = 130;
								if (robotStatus == 1) {
									countInitPos = 1;
								} else {
									countInitPos = 0;
								}
							} else if (angle < 0 && countInitPos == 0) {			//Support Kiri
								initialPos_X = -358;
								initialPos_Y = -130;
								if (robotStatus == 1) {
									countInitPos = 1;
								} else {
									countInitPos = 0;
								}
							}

						}
					}
				} else if (play && !usePenaltyStrategy) {
					if ((pickUp && gameController.Remaining != 600 && gameController.SecondaryState == 0) || (pickUp && gameController.Remaining != 300 && gameController.SecondaryState == 2)) {
						if (angle >= 0 && countInitPos == 0) {
							initialPos_X = -75;
							initialPos_Y = -305;
						} else if (angle < 0 && countInitPos == 0) {
							initialPos_X = -75;
							initialPos_Y = 305;
						}
					}
				} else if (!play && gameController.State == 0 && usePenaltyStrategy) {
					initialPos_X = 150;
					initialPos_Y = 0;
				}
			} else {
				if (!play && !usePenaltyStrategy){
					initialPos_X = -350;
					initialPos_Y = 0;
				} else if (!play && usePenaltyStrategy) {
					initialPos_X = 150;
					initialPos_Y = 0;
				}
			}
		} else if (modePlay == 1) {	//INTIAL POSITION INTERNASIONAL
			if(useGameController) {
				if(!play && gameController.State == 0 && !usePenaltyStrategy) {
					if ((!pickUp && gameController.Remaining == 600 &&  gameController.SecondaryState == 0) || (!pickUp && gameController.Remaining == 300 &&  gameController.SecondaryState == 2)) {
						if (countInitPos == 0) {
							if (gameController.KickOff == BARELANG_COLOR || gameController.KickOff == DROPBALL) { //Attack
								if (strategy.strategyNumber < 3) {	//masuk depan
									if (angle >= 0) {
										initialPos_X = -130; initialPos_Y = -300;
										initGrid = 21; 	offsetX = 50; offsetY = 50;
									} else if (angle < 0) {
										initialPos_X = -130; initialPos_Y = 300;
										initGrid = 22; offsetX = 50; offsetY = -50;
									}
								} else if (strategy.strategyNumber == 3) {	//masuk samping depan
									if (angle >= 0) {
										initialPos_X = -300; initialPos_Y = -305;
										initGrid = 20; 	offsetX = 0; offsetY = 0;
									} else if (angle < 0) {
										initialPos_X = -300; initialPos_Y = 305;
										initGrid = 23; offsetX = 0; offsetY = 0;
									}
								}
							} else {	//Defense
								if (strategy.strategyNumber < 3) {	//masuk depan
									if (angle >= 0) {
										initialPos_X = -130; initialPos_Y = -300;
										initGrid = 21; 	offsetX = -50; offsetY = 50;
									} else if (angle < 0) {
										initialPos_X = -130; initialPos_Y = 300;
										initGrid = 22; offsetX = -50; offsetY = -50;
									}
								} else if (strategy.strategyNumber == 3) {	//masuk samping belakang
									if (angle >= 0) {	//Kiri
										initialPos_X = -300; initialPos_Y = -305;
										initGrid = 14; 	offsetX = 0; offsetY = 0;
									} else if (angle < 0) {	//Kanan
										initialPos_X = -300; initialPos_Y = 305;
										initGrid = 17; offsetX = 0; offsetY = 0;
									}
								}
							}
							if (robotStatus == 1) {
								countInitPos = 1;
							} else {
								countInitPos = 0;
							}
						}
					}
				} else if (play && !usePenaltyStrategy || switched) {
					if ((pickUp && gameController.Remaining != 600 && gameController.SecondaryState == 0) || (pickUp && gameController.Remaining != 300 && gameController.SecondaryState == 2)) {
						if (angle >= 0 && countInitPos == 0) {
							//printf(" masuk setingan pickUp balikKiri\n");
							initialPos_X = -300;
							initialPos_Y = -305;
							masukKiri = true;
							masukKanan = false;
							countInitPos = 1;
						} else if (angle < 0 && countInitPos == 0) {
							//printf(" masuk setingan pickUp balikKanan\n");
							initialPos_X = -300;
							initialPos_Y = 305;
							masukKanan = true;
							masukKiri = false;
							countInitPos = 1;
						}
					}
				} else if (!play && gameController.State == 0 && usePenaltyStrategy) {
					initialPos_X = 240;
					initialPos_Y = 0;
				}
			} else {
				if (!play && !usePenaltyStrategy){
					initialPos_X = -350;
					initialPos_Y = 0;
				} else if (!play && usePenaltyStrategy) {
					initialPos_X = 240;
					initialPos_Y = 0;
				}
			}
		} else {/*nothing*/}





		if (useCoordination) {
			refreshComm(); //refresh data communications


			if (zeroState == true || play == false || stateCondition == 1 || stateCondition == 150 || stateCondition == 200) {
				countHilang = 0;
				semeh = 232; koordinasi.sendRobotCoordinationData(robotID, robotStatus, stateCondition, Grid, 0, semeh, GridBall, backIn);
			} else if (stateCondition == 7 || stateCondition == 3 || stateCondition == 8 || stateCondition == 4 || stateCondition == 5 || stateCondition == -10 || stateCondition == 10 || stateCondition == 20 || stateCondition == 30) {
				countHilang = 0;
				semeh = 1; koordinasi.sendRobotCoordinationData(robotID, robotStatus, 232, Grid, 1, semeh, GridBall, backIn);
			} else if (stateCondition == 2 || stateCondition == 6) {
				//langsung kirim didalam case
			} else { //sisa stateCondition yang tidak dikondisikan
				if (receler.Ball_X == -1 && receler.Ball_Y == -1) { //bola hilang
					if (countHilang > 20) {
						semeh = 232;
						koordinasi.sendRobotCoordinationData(robotID, robotStatus, stateCondition, Grid, 0, semeh, GridBall, backIn);
					} else {
						countHilang++;
					}
				} else { //dapat bola
					countHilang = 0;
					trackBall();
					koordinasi.sendRobotCoordinationData(robotID, robotStatus, stateCondition, Grid, 1, semeh, GridBall, backIn);
				}
			} koordinasi.readRobotCoordinationData();
		}

		if (strategy.strategyNumber == 4) { //printf("  check receler\n");
			motion("0");
			resetAllVariable();

			getSensor();

			gameController.State = 0;
			stateGameController = 0;
			lastStateGameController = 0;

			play	= false;

			backIn = pickUp	= false;
			//firstTimes = true;
			manual = true;
			countInitPos = 0;

			robotStatus = 0;

			if (ballLost(20)) {
				tiltSearchBall(0.0);
			} else {
				trackBall();
			} stateCondition = 150;
		} else {
			if (state == 0) {
				if (backIn) {
					robotStatus = 0;
				} else { robotStatus = 1; }
			}
		}

		if ((gameController.Penalty1 == 34 && gameController.timNumber1 == BARELANG_COLOR) || (gameController.Penalty2 == 34 && gameController.timNumber2 == BARELANG_COLOR)) {
			zeroState = true;
			signIn = false;
		} else {
			signIn = true;
		}

		if (stateGameController == 3) {
			if (robotFall == true) {
		                resetCase1();
				jatuh = true;
			} else if (robotFall == false && jatuh){
			        motion("0");
				jatuh = false;
				stateCondition = 1;
			}
		}

		// Untuk Cek Imu, Jika Selalu 0 maka ke case 200
//		if (strategy.bearingValue == 0) {
//			countBearing += 1;
//			if (countBearing >= 200) {
//				stateCondition = 200;
//			}
//		} else {
//			countBearing = 0;
//		}


		////////////////////////////////////////////////////////////////////////
		////////////////.............Game Controller.............///////////////
		////////////////////////////////////////////////////////////////////////
		if (useGameController) {
			stateGameController = gameController.State;
//			if (lastStateGameController != stateGameController) {
//				if (stateGameController > lastStateGameController) { //printf("maju\n");
//					if (stateChange > 30) { //40
//						lastStateGameController = stateGameController;
//					} else { stateChange++; }
//				} else if (stateGameController < lastStateGameController) { //printf("mundur\n");
//					if (
//						(lastStateGameController == 4 && stateGameController == 0) ||
//						(lastStateGameController == 3 && stateGameController == 1)
//					   ) {
//						if (stateChange > 30) { //40
//							lastStateGameController = stateGameController;
//						} else { stateChange++; }
//					}
//				}
//			} else { stateChange = 0; }


			/*kickOff = gameController.KickOff;
			if (lastKickOff != kickOff) {
				if (kickOffChange > 40) { //30
					lastKickOff = kickOff;
				} else { kickOffChange++; }
			} else { kickOffChange = 0; }


			secRemaining = gameController.Remaining;
			if (lastSecRemaining != secRemaining) {
				if (secRemainingChange > 40) { //30
					lastSecRemaining = secRemaining;
				} else { secRemainingChange++; }
			} else { secRemainingChange = 0; }*/


			//switch (lastStateGameController) {
			switch (stateGameController) {
				case 0 : //printf("  Initial\n\n");
					zeroState = false;
					play = false;

					if (strategy.strategyNumber != 4) {
						motion("0"); //Walk(0.0, 0.0, 0.0);
						//headMove(0.0, -1.1);
						//sudut();
						predictGoal(angle, -1.6);

						backPosition = false;
						resetAllVariable();

						stateCondition = 150;

						//posRotate = false;
						//setWaktu();
					}
				break;

				case 1 : //printf("  Ready\n\n\n");
					zeroState = false;
					play = false;

					if (strategy.strategyNumber != 4) {
						//if (backIn) {
						//	motion("0");
						//	robotStatus = 0;
						//	resetAllVariable();
						//	predictGoal(angle, -1.6);
						//} else {
						localization();
						//}
						stateCondition = 150;
					}
				break;

				case 2 : //printf("  \n\n\nSet\n\n\n\n");
					zeroState = false;
					play = false;
					kurama = 0;
					resetKoordinasiRobotBalik();
					if (strategy.strategyNumber != 4) {
						motion("0");

						firstTimes	= true;
						pickUp		= false;
						switched	= false;
						kondisiBola	= 0;
						stateCondition	= 0;
						reset		= 0;
						countDef	= 0;
						if (backIn) {
							robotStatus = 1;
							backIn = false;
						}
						resetCaseAwal();
						//resetAllVariable();
						stateCondition = 150;

						if (ballLost(20)) {
							SearchBall(2);
						} else {
							trackBall();
						}
					}
				break;

				case 3 : //printf("play\n");
					kurama = 0;
					if (strategy.strategyNumber != 4) {
						if (firstTimes) {
							searchKe = 0;
							timer = false;
							resetCaseAwal();
							//resetAllVariable();

							stateCondition = firstStateCondition;
							firstTimes = false;
							play = true;
						}
					}
				break;

				case 4 : //printf("finish\n");
					zeroState = false;
					kurama = 0;
					motion("0");
					motion("7");
					predictGoal(angle, -1.6);

					play		= false;
					manual		= false;
					backPosition	= false;
					resetAllVariable();
					stateCondition = 150;
				break;

				default: break;
			}
		} else {
			if (firstTimes) {
				backPosition = false;
				resetAllVariable();
				stateCondition = firstStateCondition;
				firstTimes = false;
			}
			play = true;
			backIn = false;
		}

		////////////////////////////////////////////////////////////////////////
		////////////////.............Strategy Button.............///////////////
		////////////////////////////////////////////////////////////////////////
		//state = strategy.killnRun;
		if (strategy.killnRun == 1) { //printf("killall\n")
			if (wait < 20) { wait++; }
			else {
				state = 1;
				if (stateGameController == 3 || lastStateGameController == 3) { zeroState = true; }
			}
		} else { //printf("state run lua\n");
			wait = 0;
			state = 0;
		}

		//printf("wait = %d, state = %d\n",wait,state);

		if (lastState != state) {
			if (state == 1) { //printf("killall\n");
				system("killall screen;");
				resetAllVariable();
				resetOdometry();
				refreshRecelerLokalisasi();

				stateCondition = 150;
				countInitPos = 0;
				robotStatus = 0;

				backIn = pickUp = false;
				gameController.Penalise = false;
			} else if (state == 0) { //printf("state run lua\n");
				runLuaProgram();
				motion("8");

				setting();
				zeroState = false; //didnt find ball for coor
				backPosition = false;
				kurama = 0;
				manual = false;
				reset = 0;

				if (stateGameController == 1) {
					robotStatus = 0;
					backIn = true;
					switched = true;
				} else {
					robotStatus = 1;
					backIn = false;
					switched = false;
				}

				gameController.State = 3;
				stateGameController = 3;
				lastStateGameController = 3;

				gameController.Penalise = true;

				play = true;

				pickUp = true;
				stateCondition = firstStateCondition;
			} lastState = state;
		}


		///////////////////////////////////////////////////////////////////////
		//////////////.............Role of execution............///////////////
		///////////////////////////////////////////////////////////////////////
		if (play) { //printf("BarelangFC Rock n Roll\n");
			switch(stateCondition) {
				case 0: // First Strategy
					if (!backPosition) {
						Strategi = strategy.strategyNumber;
					} else {
						Strategi = 0;
					}

					switch (Strategi) {
						case 0 : // Serang lurus - Defense depan di tengah
							if (gameController.KickOff == BARELANG_COLOR || gameController.KickOff == DROPBALL) { //Attack
								if (receler.Ball_X != -1 && receler.Ball_Y != -1) {
									trackBall();
								}

								if (pickUp) {
									if (signIn) {
										motion("9");
										setWaktu();
										stateCondition = 50;
									} else {
										motion("0");
										headMove(0.0, -1.4);
									}
								} else {
									motion("9");
									if (usePenaltyStrategy) {
										useFollowSearchGoal = false;
										useSearchGoal	= false;
									}
									stateCondition	= -10; //-10; //-20;
								}
							} else { //Defense
								if (receler.Ball_X != -1 && receler.Ball_Y != -1) {
									trackBall();
								}

								if (pickUp) {
									if (signIn) {
										motion("9");
										setWaktu();
										stateCondition = 50;
									} else {
										motion("0");
										headMove(0.0, -1.4);
									}
								} else {
									motion("0");
									setWaktu();
									stateCondition	= 130;
								}
							}
						break;

						case 1 : // serang ke kanan - Defense belakang di kiri/kanan
							if (gameController.KickOff == BARELANG_COLOR || gameController.KickOff == DROPBALL) { //Attack
								if (receler.Ball_X != -1 && receler.Ball_Y != -1) {
									trackBall();
								}

								if (pickUp) {
									if (signIn) {
										motion("9");
										setWaktu();
										stateCondition = 50;
									} else {
										motion("0");
										headMove(0.0, -1.4);
									}
								} else {
									motion("9");
									if (usePenaltyStrategy) {
										useFollowSearchGoal = false;
										useSearchGoal	= false;
									}
									stateCondition	= 10;
								}
							} else { //Defense
								if (receler.Ball_X != -1 && receler.Ball_Y != -1) {
									trackBall();
								}

								if (pickUp) {
									if (signIn) {
										motion("9");
										setWaktu();
										stateCondition = 50;
									} else {
										motion("0");
										headMove(0.0, -1.4);
									}
								} else {
									motion("0");
									setWaktu();
									stateCondition	= 140;
								}
							}
						break;

						case 2 : // serang ke kiri - Defense belakang di kiri/kanan
							if (gameController.KickOff == BARELANG_COLOR || gameController.KickOff == DROPBALL) { //Attack
								if (receler.Ball_X != -1 && receler.Ball_Y != -1) {
									trackBall();
								}

								if (pickUp) {
									if (signIn) {
										motion("9");
										setWaktu();
										stateCondition = 50;
									} else {
										motion("0");
										headMove(0.0, -1.4);
									}
								} else {
									motion("9");
									if (usePenaltyStrategy) {
										useFollowSearchGoal = false;
										useSearchGoal	= false;
									}
									stateCondition	= 20;
								}
							} else { //Defense
								if (receler.Ball_X != -1 && receler.Ball_Y != -1) {
									trackBall();
								}

								if (pickUp) {
									if (signIn) {
										motion("9");
										setWaktu();
										stateCondition = 50;
									} else {
										motion("0");
										headMove(0.0, -1.4);
									}
								} else {
									motion("0");
									setWaktu();
									stateCondition	= 140;
								}
							}
						break;

						case 3 : // Backup robot di kiri/kanan - Defense belakang di kiri/kanan
							if (gameController.KickOff == BARELANG_COLOR || gameController.KickOff == DROPBALL) { //Attack
								if (receler.Ball_X != -1 && receler.Ball_Y != -1) {
									trackBall();
								}

								if (pickUp) {
									if (signIn) {
										motion("9");
										setWaktu();
										stateCondition = 50;
									} else {
										motion("0");
										headMove(0.0, -1.4);
									}
								} else {
									motion("9");
									if (usePenaltyStrategy) {
										useFollowSearchGoal = false;
										useSearchGoal	= false;
									}
									stateCondition	= 30;
								}
							} else { //Defense
								if (receler.Ball_X != -1 && receler.Ball_Y != -1) {
									trackBall();
								}

								if (pickUp) {
									if (signIn) {
										motion("9");
										setWaktu();
										stateCondition = 50;
									} else {
										motion("0");
										headMove(0.0, -1.4);
									}
								} else {
									motion("0");
									setWaktu();
									stateCondition	= 140;
								}
							}
						break;

						case 4 : // cek Vision
							stateCondition = 150;
						break;

						default:
						break;
					}
				break;

				case 1: // searching ball
					if (receler.Ball_X == -1 && receler.Ball_Y == -1 && !tracked) {
						delayWaitBall = 0;
						if (useLocalization) {
							if (robotID == 1) {
								normalSearchBallGrid(39,0,-25, 21,0,-25);
							} else if (robotID == 3) {
								normalSearchBallGrid(40,0,-25, 22,0,-25);
							} else if (robotID == 4) {
								normalSearchBallGrid(39,0,25, 21,0,25);
							} else if (robotID == 5) {
								normalSearchBallGrid(40,0,25, 22,0,25);
							}
							//normalSearchBallGrid(39,0,50, 15,0,50);
							//normalSearchBall();
						} else {
							//motion("9");
							normalSearchBall();
						}
					} else {
						tracked = true;
					}

					if (tracked) {
						if (ballLost(20)) {
							tracked = false;
						} else {
							trackBall();
							if (delayWaitBall > 30) { //30
							        motion("9");
								if (headTilt >= -1.8 && headPan >= -0.1 && headPan <= 0.1) {
									resetCase2();
									stateCondition = 2;
								} else {
									followBall(0);
								}
							} else {
							        //motion("0");    //tes
								//Walk(0.0, 0.0, 0.0);
								delayWaitBall++;
							} //printf("  delayWaitBall = %d,", delayWaitBall);
						}
					}
				break;

				case 2: // follow, search goal dan coor
					//motion("9");

					if (useCoordination) {
						if (robotID == 1) {
							if ( //jika ada robot lain yang sudah masuk case eksekusi
								(koordinasi.robot2State == 232 || koordinasi.robot2State == 7 || koordinasi.robot2State == 3 || koordinasi.robot2State == 8 || koordinasi.robot2State == 4 || koordinasi.robot2State == 5 || koordinasi.robot2State == -10 || koordinasi.robot2State == 10 || koordinasi.robot2State == 20 || koordinasi.robot2State == 30) ||
								(koordinasi.robot3State == 232 || koordinasi.robot3State == 7 || koordinasi.robot3State == 3 || koordinasi.robot3State == 8 || koordinasi.robot3State == 4 || koordinasi.robot3State == 5 || koordinasi.robot3State == -10 || koordinasi.robot3State == 10 || koordinasi.robot3State == 20 || koordinasi.robot3State == 30) ||
								(koordinasi.robot4State == 232 || koordinasi.robot4State == 7 || koordinasi.robot4State == 3 || koordinasi.robot4State == 8 || koordinasi.robot4State == 4 || koordinasi.robot4State == 5 || koordinasi.robot4State == -10 || koordinasi.robot4State == 10 || koordinasi.robot4State == 20 || koordinasi.robot4State == 30) ||
								(koordinasi.robot5State == 232 || koordinasi.robot5State == 7 || koordinasi.robot5State == 3 || koordinasi.robot5State == 8 || koordinasi.robot5State == 4 || koordinasi.robot5State == 5 || koordinasi.robot5State == -10 || koordinasi.robot5State == 10 || koordinasi.robot5State == 20 || koordinasi.robot5State == 30)
							   ) {
								if (koordinasi.robot2State == 232 || koordinasi.robot2State == 7 || koordinasi.robot2State == 3 || koordinasi.robot2State == 8 || koordinasi.robot2State == 4 || koordinasi.robot2State == 5 || koordinasi.robot2State == -10 || koordinasi.robot2State == 10 || koordinasi.robot2State == 20 || koordinasi.robot2State == 30) {
									robot2exeCutor = true;
									robot3exeCutor = false;
									robot4exeCutor = false;
									robot5exeCutor = false;
								} else if (koordinasi.robot3State == 232 || koordinasi.robot3State == 7 || koordinasi.robot3State == 3 || koordinasi.robot3State == 8 || koordinasi.robot3State == 4 || koordinasi.robot3State == 5 || koordinasi.robot3State == -10 || koordinasi.robot3State == 10 || koordinasi.robot3State == 20 || koordinasi.robot3State == 30) {
									robot2exeCutor = false;
									robot3exeCutor = true;
									robot4exeCutor = false;
									robot5exeCutor = false;
								} else if (koordinasi.robot4State == 232 || koordinasi.robot4State == 7 || koordinasi.robot4State == 3 || koordinasi.robot4State == 8 || koordinasi.robot4State == 4 || koordinasi.robot4State == 5 || koordinasi.robot4State == -10 || koordinasi.robot4State == 10 || koordinasi.robot4State == 20 || koordinasi.robot4State == 30) {
									robot2exeCutor = false;
									robot3exeCutor = false;
									robot4exeCutor = true;
									robot5exeCutor = false;
								} else if (koordinasi.robot5State == 232 || koordinasi.robot5State == 7 || koordinasi.robot5State == 3 || koordinasi.robot5State == 8 || koordinasi.robot5State == 4 || koordinasi.robot5State == 5 || koordinasi.robot5State == -10 || koordinasi.robot5State == 10 || koordinasi.robot5State == 20 || koordinasi.robot5State == 30) {
									robot2exeCutor = false;
									robot3exeCutor = false;
									robot4exeCutor = false;
									robot5exeCutor = true;
								}
								exeCutor = false;
							} else if ( //jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
								((headTilt*-100) < koordinasi.robot2DBall) &&
								((headTilt*-100) < koordinasi.robot3DBall) &&
								((headTilt*-100) < koordinasi.robot4DBall) &&
								((headTilt*-100) < koordinasi.robot5DBall)
							   ) {
								//selain nomer robotID, false semua ...
								robot2exeCutor = false;
								robot3exeCutor = false;
								robot3pos_X = robot3pos_Y = 0;
								robot4exeCutor = false;
								robot5exeCutor = false;
								exeCutor = true;
							}

							if (exeCutor) { //printf("\n  MyTurn......................\n");
								refreshMoveGrid();
								myTurn();
							} else { //printf("\n  YourTurn......................\n");
								waitingTurn();
							}
						} else if (robotID == 2) {
							if ( //jika ada robot lain yang sudah masuk case eksekusi
								(koordinasi.robot1State == 232 || koordinasi.robot1State == 7 || koordinasi.robot1State == 3 || koordinasi.robot1State == 8 || koordinasi.robot1State == 4 || koordinasi.robot1State == 5 || koordinasi.robot1State == -10 || koordinasi.robot1State == 10 || koordinasi.robot1State == 20 || koordinasi.robot1State == 30) ||
								(koordinasi.robot3State == 232 || koordinasi.robot3State == 7 || koordinasi.robot3State == 3 || koordinasi.robot3State == 8 || koordinasi.robot3State == 4 || koordinasi.robot3State == 5 || koordinasi.robot3State == -10 || koordinasi.robot3State == 10 || koordinasi.robot3State == 20 || koordinasi.robot3State == 30) ||
								(koordinasi.robot4State == 232 || koordinasi.robot4State == 7 || koordinasi.robot4State == 3 || koordinasi.robot4State == 8 || koordinasi.robot4State == 4 || koordinasi.robot4State == 5 || koordinasi.robot4State == -10 || koordinasi.robot4State == 10 || koordinasi.robot4State == 20 || koordinasi.robot4State == 30) ||
								(koordinasi.robot5State == 232 || koordinasi.robot5State == 7 || koordinasi.robot5State == 3 || koordinasi.robot5State == 8 || koordinasi.robot5State == 4 || koordinasi.robot5State == 5 || koordinasi.robot5State == -10 || koordinasi.robot5State == 10 || koordinasi.robot5State == 20 || koordinasi.robot5State == 30)
							   ) {
								if (koordinasi.robot1State == 232 || koordinasi.robot1State == 7 || koordinasi.robot1State == 3 || koordinasi.robot1State == 8 || koordinasi.robot1State == 4 || koordinasi.robot1State == 5 || koordinasi.robot1State == -10 || koordinasi.robot1State == 10 || koordinasi.robot1State == 20 || koordinasi.robot1State == 30) {
									robot1exeCutor = true;
									robot3exeCutor = false;
									robot4exeCutor = false;
									robot5exeCutor = false;
								} else if (koordinasi.robot3State == 232 || koordinasi.robot3State == 7 || koordinasi.robot3State == 3 || koordinasi.robot3State == 8 || koordinasi.robot3State == 4 || koordinasi.robot3State == 5 || koordinasi.robot3State == -10 || koordinasi.robot3State == 10 || koordinasi.robot3State == 20 || koordinasi.robot3State == 30) {
									robot1exeCutor = false;
									robot3exeCutor = true;
									robot4exeCutor = false;
									robot5exeCutor = false;
								} else if (koordinasi.robot4State == 232 || koordinasi.robot4State == 7 || koordinasi.robot4State == 3 || koordinasi.robot4State == 8 || koordinasi.robot4State == 4 || koordinasi.robot4State == 5 || koordinasi.robot4State == -10 || koordinasi.robot4State == 10 || koordinasi.robot4State == 20 || koordinasi.robot4State == 30) {
									robot1exeCutor = false;
									robot3exeCutor = false;
									robot4exeCutor = true;
									robot5exeCutor = false;
								} else if (koordinasi.robot5State == 232 || koordinasi.robot5State == 7 || koordinasi.robot5State == 3 || koordinasi.robot5State == 8 || koordinasi.robot5State == 4 || koordinasi.robot5State == 5 || koordinasi.robot5State == -10 || koordinasi.robot5State == 10 || koordinasi.robot5State == 20 || koordinasi.robot5State == 30) {
									robot1exeCutor = false;
									robot3exeCutor = false;
									robot4exeCutor = false;
									robot5exeCutor = true;
								}
								exeCutor = false;
							} else if ( //jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
								((headTilt*-100) < koordinasi.robot1DBall) &&
								((headTilt*-100) < koordinasi.robot3DBall) &&
								((headTilt*-100) < koordinasi.robot4DBall) &&
								((headTilt*-100) < koordinasi.robot5DBall)
							   ) {
								exeCutor = true;
								//selain nomer robotID, false semua ...
								robot1exeCutor = false;
								robot3exeCutor = false;
								robot4exeCutor = false;
								robot5exeCutor = false;
							}

							if (exeCutor) { //printf("\n  MyTurn......................\n");
								myTurn();
							} else { //printf("\n  YourTurn......................\n");
								waitingTurn();
							}
						} else if (robotID == 3) {
							if ( //jika ada robot lain yang sudah masuk case eksekusi
								(koordinasi.robot1State == 232 || koordinasi.robot1State == 7 || koordinasi.robot1State == 3 || koordinasi.robot1State == 8 || koordinasi.robot1State == 4 || koordinasi.robot1State == 5 || koordinasi.robot1State == -10 || koordinasi.robot1State == 10 || koordinasi.robot1State == 20 || koordinasi.robot1State == 30) ||
								(koordinasi.robot2State == 232 || koordinasi.robot2State == 7 || koordinasi.robot2State == 3 || koordinasi.robot2State == 8 || koordinasi.robot2State == 4 || koordinasi.robot2State == 5 || koordinasi.robot2State == -10 || koordinasi.robot2State == 10 || koordinasi.robot2State == 20 || koordinasi.robot2State == 30) ||
								(koordinasi.robot4State == 232 || koordinasi.robot4State == 7 || koordinasi.robot4State == 3 || koordinasi.robot4State == 8 || koordinasi.robot4State == 4 || koordinasi.robot4State == 5 || koordinasi.robot4State == -10 || koordinasi.robot4State == 10 || koordinasi.robot4State == 20 || koordinasi.robot4State == 30) ||
								(koordinasi.robot5State == 232 || koordinasi.robot5State == 7 || koordinasi.robot5State == 3 || koordinasi.robot5State == 8 || koordinasi.robot5State == 4 || koordinasi.robot5State == 5 || koordinasi.robot5State == -10 || koordinasi.robot5State == 10 || koordinasi.robot5State == 20 || koordinasi.robot5State == 30)
							   ) {
								if (koordinasi.robot1State == 232 || koordinasi.robot1State == 7 || koordinasi.robot1State == 3 || koordinasi.robot1State == 8 || koordinasi.robot1State == 4 || koordinasi.robot1State == 5 || koordinasi.robot1State == -10 || koordinasi.robot1State == 10 || koordinasi.robot1State == 20 || koordinasi.robot1State == 30) {
									robot1exeCutor = true;
									robot2exeCutor = false;
									robot4exeCutor = false;
									robot5exeCutor = false;
								} else if (koordinasi.robot2State == 232 || koordinasi.robot2State == 7 || koordinasi.robot2State == 3 || koordinasi.robot2State == 8 || koordinasi.robot2State == 4 || koordinasi.robot2State == 5 || koordinasi.robot2State == -10 || koordinasi.robot2State == 10 || koordinasi.robot2State == 20 || koordinasi.robot2State == 30) {
									robot1exeCutor = false;
									robot2exeCutor = true;
									robot4exeCutor = false;
									robot5exeCutor = false;
								} else if (koordinasi.robot4State == 232 || koordinasi.robot4State == 7 || koordinasi.robot4State == 3 || koordinasi.robot4State == 8 || koordinasi.robot4State == 4 || koordinasi.robot4State == 5 || koordinasi.robot4State == -10 || koordinasi.robot4State == 10 || koordinasi.robot4State == 20 || koordinasi.robot4State == 30) {
									robot1exeCutor = false;
									robot2exeCutor = false;
									robot4exeCutor = true;
									robot5exeCutor = false;
								} else if (koordinasi.robot5State == 232 || koordinasi.robot5State == 7 || koordinasi.robot5State == 3 || koordinasi.robot5State == 8 || koordinasi.robot5State == 4 || koordinasi.robot5State == 5 || koordinasi.robot5State == -10 || koordinasi.robot5State == 10 || koordinasi.robot5State == 20 || koordinasi.robot5State == 30) {
									robot1exeCutor = false;
									robot2exeCutor = false;
									robot4exeCutor = false;
									robot5exeCutor = true;
								}
								exeCutor = false;
							} else if ( //jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
								((headTilt*-100) < koordinasi.robot1DBall) &&
								((headTilt*-100) < koordinasi.robot2DBall) &&
								((headTilt*-100) < koordinasi.robot4DBall) &&
								((headTilt*-100) < koordinasi.robot5DBall)
							   ) {
								//selain nomer robotID, false semua ...
								robot1exeCutor = false;
								robot1pos_X = robot1pos_Y = 0;
								robot2exeCutor = false;
								robot4exeCutor = false;
								robot5exeCutor = false;
								exeCutor = true;
							}

							if (exeCutor) { //printf("\n  MyTurn......................\n");
								refreshMoveGrid();
								myTurn();
							} else { //printf("\n  YourTurn......................\n");
								waitingTurn();
							}
						} else if (robotID == 4) {
							if ( //jika ada robot lain yang sudah masuk case eksekusi
								(koordinasi.robot1State == 232 || koordinasi.robot1State == 7 || koordinasi.robot1State == 3 || koordinasi.robot1State == 8 || koordinasi.robot1State == 4 || koordinasi.robot1State == 5 || koordinasi.robot1State == -10 || koordinasi.robot1State == 10 || koordinasi.robot1State == 20 || koordinasi.robot1State == 30) ||
								(koordinasi.robot2State == 232 || koordinasi.robot2State == 7 || koordinasi.robot2State == 3 || koordinasi.robot2State == 8 || koordinasi.robot2State == 4 || koordinasi.robot2State == 5 || koordinasi.robot2State == -10 || koordinasi.robot2State == 10 || koordinasi.robot2State == 20 || koordinasi.robot2State == 30) ||
								(koordinasi.robot3State == 232 || koordinasi.robot3State == 7 || koordinasi.robot3State == 3 || koordinasi.robot3State == 8 || koordinasi.robot3State == 4 || koordinasi.robot3State == 5 || koordinasi.robot3State == -10 || koordinasi.robot3State == 10 || koordinasi.robot3State == 20 || koordinasi.robot3State == 30) ||
								(koordinasi.robot5State == 232 || koordinasi.robot5State == 7 || koordinasi.robot5State == 3 || koordinasi.robot5State == 8 || koordinasi.robot5State == 4 || koordinasi.robot5State == 5 || koordinasi.robot5State == -10 || koordinasi.robot5State == 10 || koordinasi.robot5State == 20 || koordinasi.robot5State == 30)
							   ) {
								if (koordinasi.robot1State == 232 || koordinasi.robot1State == 7 || koordinasi.robot1State == 3 || koordinasi.robot1State == 8 || koordinasi.robot1State == 4 || koordinasi.robot1State == 5 || koordinasi.robot1State == -10 || koordinasi.robot1State == 10 || koordinasi.robot1State == 20 || koordinasi.robot1State == 30) {
									robot1exeCutor = true;
									robot2exeCutor = false;
									robot3exeCutor = false;
									robot5exeCutor = false;
								} else if (koordinasi.robot2State == 232 || koordinasi.robot2State == 7 || koordinasi.robot2State == 3 || koordinasi.robot2State == 8 || koordinasi.robot2State == 4 || koordinasi.robot2State == 5 || koordinasi.robot2State == -10 || koordinasi.robot2State == 10 || koordinasi.robot2State == 20 || koordinasi.robot2State == 30) {
									robot1exeCutor = false;
									robot2exeCutor = true;
									robot3exeCutor = false;
									robot5exeCutor = false;
								} else if (koordinasi.robot3State == 232 || koordinasi.robot3State == 7 || koordinasi.robot3State == 3 || koordinasi.robot3State == 8 || koordinasi.robot3State == 4 || koordinasi.robot3State == 5 || koordinasi.robot3State == -10 || koordinasi.robot3State == 10 || koordinasi.robot3State == 20 || koordinasi.robot3State == 30) {
									robot1exeCutor = false;
									robot2exeCutor = false;
									robot3exeCutor = true;
									robot5exeCutor = false;
								} else if (koordinasi.robot5State == 232 || koordinasi.robot5State == 7 || koordinasi.robot5State == 3 || koordinasi.robot5State == 8 || koordinasi.robot5State == 4 || koordinasi.robot5State == 5 || koordinasi.robot5State == -10 || koordinasi.robot5State == 10 || koordinasi.robot5State == 20 || koordinasi.robot5State == 30) {
									robot1exeCutor = false;
									robot2exeCutor = false;
									robot3exeCutor = false;
									robot5exeCutor = true;
								}
								exeCutor = false;
							} else if ( //jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
								((headTilt*-100) < koordinasi.robot1DBall) &&
								((headTilt*-100) < koordinasi.robot2DBall) &&
								((headTilt*-100) < koordinasi.robot3DBall) &&
								((headTilt*-100) < koordinasi.robot5DBall)
							   ) {
								exeCutor = true;
								//selain nomer robotID, false semua ...
								robot1exeCutor = false;
								robot2exeCutor = false;
								robot3exeCutor = false;
								robot5exeCutor = false;
							}

							if (exeCutor) { //printf("\n  MyTurn......................\n");
								myTurn();
							} else { //printf("\n  YourTurn......................\n");
								waitingTurn();
							}
						} else if (robotID == 5) {
							if ( //jika ada robot lain yang sudah masuk case eksekusi
								(koordinasi.robot1State == 232 || koordinasi.robot1State == 7 || koordinasi.robot1State == 3 || koordinasi.robot1State == 8 || koordinasi.robot1State == 4 || koordinasi.robot1State == 5 || koordinasi.robot1State == -10 || koordinasi.robot1State == 10 || koordinasi.robot1State == 20 || koordinasi.robot1State == 30) ||
								(koordinasi.robot2State == 232 || koordinasi.robot2State == 7 || koordinasi.robot2State == 3 || koordinasi.robot2State == 8 || koordinasi.robot2State == 4 || koordinasi.robot2State == 5 || koordinasi.robot2State == -10 || koordinasi.robot2State == 10 || koordinasi.robot2State == 20 || koordinasi.robot2State == 30) ||
								(koordinasi.robot3State == 232 || koordinasi.robot3State == 7 || koordinasi.robot3State == 3 || koordinasi.robot3State == 8 || koordinasi.robot3State == 4 || koordinasi.robot3State == 5 || koordinasi.robot3State == -10 || koordinasi.robot3State == 10 || koordinasi.robot3State == 20 || koordinasi.robot3State == 30) ||
								(koordinasi.robot4State == 232 || koordinasi.robot4State == 7 || koordinasi.robot4State == 3 || koordinasi.robot4State == 8 || koordinasi.robot4State == 4 || koordinasi.robot4State == 5 || koordinasi.robot4State == -10 || koordinasi.robot4State == 10 || koordinasi.robot4State == 20 || koordinasi.robot4State == 30)
							   ) {
								if (koordinasi.robot1State == 232 || koordinasi.robot1State == 7 || koordinasi.robot1State == 3 || koordinasi.robot1State == 8 || koordinasi.robot1State == 4 || koordinasi.robot1State == 5 || koordinasi.robot1State == -10 || koordinasi.robot1State == 10 || koordinasi.robot1State == 20 || koordinasi.robot1State == 30) {
									robot1exeCutor = true;
									robot2exeCutor = false;
									robot3exeCutor = false;
									robot4exeCutor = false;
								} else if (koordinasi.robot2State == 232 || koordinasi.robot2State == 7 || koordinasi.robot2State == 3 || koordinasi.robot2State == 8 || koordinasi.robot2State == 4 || koordinasi.robot2State == 5 || koordinasi.robot2State == -10 || koordinasi.robot2State == 10 || koordinasi.robot2State == 20 || koordinasi.robot2State == 30) {
									robot1exeCutor = false;
									robot2exeCutor = true;
									robot3exeCutor = false;
									robot4exeCutor = false;
								} else if (koordinasi.robot3State == 232 || koordinasi.robot3State == 7 || koordinasi.robot3State == 3 || koordinasi.robot3State == 8 || koordinasi.robot3State == 4 || koordinasi.robot3State == 5 || koordinasi.robot3State == -10 || koordinasi.robot3State == 10 || koordinasi.robot3State == 20 || koordinasi.robot3State == 30) {
									robot1exeCutor = false;
									robot2exeCutor = false;
									robot3exeCutor = true;
									robot4exeCutor = false;
								} else if (koordinasi.robot4State == 232 || koordinasi.robot4State == 7 || koordinasi.robot4State == 3 || koordinasi.robot4State == 8 || koordinasi.robot4State == 4 || koordinasi.robot4State == 5 || koordinasi.robot4State == -10 || koordinasi.robot4State == 10 || koordinasi.robot4State == 20 || koordinasi.robot4State == 30) {
									robot1exeCutor = false;
									robot2exeCutor = false;
									robot3exeCutor = false;
									robot4exeCutor = true;
								}
								exeCutor = false;
							} else if ( //jika jarak saya paling dekat dengan bola / saya dapat bola lebih dulu
								((headTilt*-100) < koordinasi.robot1DBall) &&
								((headTilt*-100) < koordinasi.robot2DBall) &&
								((headTilt*-100) < koordinasi.robot3DBall) &&
								((headTilt*-100) < koordinasi.robot4DBall)
							   ) {
								exeCutor = true;
								//selain nomer robotID, false semua ...
								robot1exeCutor = false;
								robot2exeCutor = false;
								robot3exeCutor = false;
								robot4exeCutor = false;
							}

							if (exeCutor) { //printf("\n  MyTurn......................\n");
								myTurn();
							} else { //printf("\n  YourTurn......................\n");
								waitingTurn();
							}
						}
					} else {
						myTurn();
					}
				break;

				case 7: // imu
					//motion("9");
					printf("  MYTURN MYTURN MYTURN MYTURN !!!!\n");
					motion("0");
					if (ballLost(20)) {
						if (tunggu > 40) {
							resetCase1();
							stateCondition = 1;
						} else {
							tiltSearchBall(0.0);
						} tunggu++;
						Walk(0.0, 0.0, 0.0);
					} else {
						trackBall();
						tunggu = 0;
						/*
						if (useImu) {
							if (robotDirection && headPan >= -0.4 && headPan <= 0.4) {
								Walk(0.0, 0.0, 0.0);

								if (useDribble == true || useSearchGoal == true) {
									resetCase3();
									stateCondition = 3;
								} else {
									resetCase5();
									stateCondition = 5;
								}
							} else {
								if (headTilt >= cAktif && headPan >= -0.4 && headPan <= 0.4) {
									if (delay > 5) {
										cekWaktu(20);
										if (timer) {
											robotDirection = true;
										} else {
											if (useSideKick) {
												if (useLocalization) {
													if (Grid >= 1 && Grid <= 42) {
														if (angle >= 45) { //45
															modeKick = 4; //tendangSamping
															Imu(90 + outGrid, cSekarang); //90
														} else if (angle <= -45) { //-45
															modeKick = 3; //tendangSamping
															Imu(-90 + outGrid, cSekarang); //90
														} else {
															modeKick = tendangJauh;
															Imu(outGrid, cSekarang);
														}
													} else if (Grid >= 43 && Grid <= 54) {
														if (robotPos_Y > 0) { // posisi Y dari 0 - 300
															if (angle >= 20 && angle <= 180) { //45
																modeKick = 4; //tendangSamping
																Imu(90 + outGrid, cSekarang); //90
															} else {
																modeKick = tendangJauh;
																Imu(outGrid, cSekarang);
															}
														} else { // posisi Y dari -300 - 0
															if (angle <= -20 && angle >= -180) { //-45
																modeKick = 3; //tendangSamping
																Imu(-90 + outGrid, cSekarang); //90
															} else {
																modeKick = tendangJauh;
																Imu(outGrid, cSekarang);
															}
														}
													}
												} else {
													if (angle >= 50) { //45
														modeKick = 4; //tendangSamping
														Imu(75, cSekarang); //90
													} else if (angle <= -50) { //-45
														modeKick = 3; //tendangSamping
														Imu(-75, cSekarang); //-90
													} else {
														modeKick = tendangJauh;
														Imu(0, cSekarang);
													}
												}
											} else {
												modeKick = tendangJauh;
												if (useLocalization){
													Imu(outGrid, cSekarang);
												} else {
													Imu(0, cSekarang);
												}
											}
										}
									} else {
										setWaktu();
										robotDirection = false;
										delay++;
									}
								} else {
									delay = 0;
									followBall(0);
								}
							}
						} else {
							if (headTilt >= (cSekarang - 0.2)) {
								Walk(0.0, 0.0, 0.0);

								if (useDribble == true || useSearchGoal == true) {
									resetCase3();
									stateCondition = 3;
								} else {
									resetCase5();
									stateCondition = 5;
								}
							} else {
								if (useLocalization && useUpdateCoordinate){
									updateCoordinatFromVision();
								}
								followBall(0);
							}
						}
						*/
					}
				break;

				case 3: // dribble
					motion("9");

					if(ballLost(20)) {
						if (tunggu > 40) {
							resetCase1();
							stateCondition = 1;
						} else {
							tiltSearchBall(0.0);
						} tunggu++;
						Walk(0.0, 0.0, 0.0);
					} else {
						trackBall();
						tunggu = 0;

						if (useDribble) {
							if (countDribble > 300) { //300 //printf("ke state SearchGoal....................................................................\n\n\n");
								resetCase8();
								stateCondition = 8;
							} else {
								//dribble(sudutTengah, 0.15); //0.14 //0.05 //0.08 //arah imu, speed; speed = speed * 0.3
								dribble(outGrid, 0.15); //0.14 //0.05 //0.08 //arah imu, speed; speed = speed * 0.3
								countDribble++;
							}
						} else {
							Walk(0.0, 0.0, 0.0);

							if (useSearchGoal == true) {
								resetCase4();
								stateCondition = 4;
							} else {
								resetCase5();
								stateCondition = 5;
							}
						}
					}
				break;

				case 8: //Imu lagi setelah dribble
					motion("9");

					if(ballLost(20)) {
						if (tunggu > 40) {
							resetCase1();
							stateCondition = 1;
						} else {
							tiltSearchBall(0.0);
							Walk(0.0, 0.0, 0.0);
						} tunggu++;
					} else {
						trackBall();
						tunggu = 0;

						if (useImu) {
							if (robotDirection && headPan >= -0.4 && headPan <= 0.4) {
								Walk(0.0, 0.0, 0.0);
								resetCase4();
								stateCondition = 4;
							} else {
								if (headTilt >= cAktif && headPan >= -0.4 && headPan <= 0.4) {
									modeKick = tendangJauh;
									Imu(outGrid, cSekarang);
								} else {
									robotDirection = false;
									followBall(0);
								}
							}
						} else {
							if (headTilt >= (cSekarang - 0.2) && headPan >= -0.4 && headPan <= 0.4) {
								Walk(0.0, 0.0, 0.0);
								resetCase4();
								stateCondition = 4;
							} else {
								followBall(0);
							}
						}
					}
				break;

				case 4: // Search Goal
					motion("9");

					if (useSearchGoal) { //printf("cariGawang\n\n");
						if (rotateGoal == true) {
							Walk(0.0, 0.0, 0.0);
							if (dribbleOnly) {
								resetCase3();
								stateCondition = 3; //dribble
							} else {
								resetCase5();
								stateCondition = 5; //kick
							}
						} else {
							rotateToGoal(1); //1
						}
					} else { //printf("tidakCariGawang\n\n");
						Walk(0.0, 0.0, 0.0);
						if (dribbleOnly) {
							resetCase3();
							stateCondition = 3; //dribble
						} else {
							resetCase5();
							stateCondition = 5; //kick
						}
					}
				break;

				case 5: // Kick
					if (ballLost(30)) {
						if (tunggu > 40) {
							resetCase1();
							stateCondition = 1;
						} else {
							tiltSearchBall(0.0);
							Walk(0.0, 0.0, 0.0);
						} tunggu++;
					} else {
						trackBall();

						if (tendang) {
							resetCase6();
							stateCondition = 6;
						} else {
							kick(modeKick);
						}
					}
				break;

				case 6: // Search After Kick
					motion("9");
					if (useLocalization && useUpdateCoordinate){
						updateCoordinatFromVision();
					}
					if (tunggu > 5) { //kedua
						cekWaktu(3); //4

						if (timer) {
							if (ballLost(20)) {
								confirmsBall = 0;
								waitTracking = 0;

								if (searchRectangle) {
									if (useCoordination) { semeh = 232; koordinasi.sendRobotCoordinationData(robotID, robotStatus, stateCondition, Grid, 0, semeh, GridBall, backIn); }

									if (sumTilt > 300) {
										resetCase1();
										stateCondition = 1;
									} else {
										searchBallRectang(-1.5, -1.6, -0.8, 1.6);
										sumTilt++;
									}

									if (modeKick == 3 || modeKick == 4) { //sideKick
										jalanDirection(kejar, 0.0, lastDirection); //saveAngle
									} else {
										Walk(kejar, 0.0, 0.0);
									}
								} else {
									if (useCoordination) { semeh = 1; koordinasi.sendRobotCoordinationData(robotID, robotStatus, 232, Grid, 1, semeh, GridBall, backIn); }
									Walk(0.0, 0.0, 0.0);

									if (tunda > 10) {
										SearchBall(3); //first Search
										if (posPan > 1.5 && countTilt == 1) {
											tiltRate = 0.05;
											panRate = 0.05;
											searchRectangle = true;
										}
									} else {
										posPan  = 1.45;
										posTilt = -0.8;
										tiltRate = -0.05;
										panRate = -0.05;
										headMove(posPan, posTilt);
										tunda++;
									}
								}
							} else {
								trackBall();

								if (waitTracking > 30) {
									searchRectangle = true;
								} else {
									tiltRate = 0.05;
									panRate = 0.05;
									waitTracking++;
								}

								if (useCoordination) {
									koordinasi.sendRobotCoordinationData(robotID, robotStatus, stateCondition, Grid, 1, semeh, GridBall, backIn);
									backToCoordinations();
								}

								if (confirmsBall > 30) {
									if (headTilt >= -0.7 && searchRectangle == false) { //langsung Tendang lagi
										Walk(0.0, 0.0, 0.0);
										resetCase5();
										stateCondition = 5;
									} else {
										//if (headTilt >= -1.5 && headPan >= -0.2 && headPan <= 0.2) {
										//if (headTilt >= -1.8 && headPan >= -0.2 && headPan <= 0.2) {
										if (headTilt >= -1.6 && headPan >= -0.2 && headPan <= 0.2) {
											resetCase2();
											stateCondition = 2;
										} else {
											followBall(0);
										}
									}
								} else {
									Walk(0.0, 0.0, 0.0);
									confirmsBall++;
								}
							}
						} else {
							if (useCoordination) { semeh = 1; koordinasi.sendRobotCoordinationData(robotID, robotStatus, 232, Grid, 1, semeh, GridBall, backIn); }
							//Walk(0.0, 0.0, 0.0);

							if (modeKick == 3) { //tendangSamping
								posPan	= -1.6;
								posTilt	= -1.6;
								headMove(posPan, posTilt);
							} else if (modeKick == 4) { //tendangSamping
								posPan	= 1.6;
								posTilt	= -1.6;
								headMove(posPan, posTilt);
							} else { //tendangDepan
								posPan	= 0.0;
								posTilt	= -1.6;
								headMove(posPan, posTilt);
							}
							countTilt = 0;
							sumTilt = 0;
							tunda = 0;
							confirmsBall = 0;
							waitTracking = 0;
						}
					} else { //pertama
						setWaktu();
						tunggu++;
					}
				break;

				case -20: // serang lurus -> dribble
					if (ballLost(20)) {
						if (tunggu > 40) {
							resetCase1();
							stateCondition = 1;
						} else {
							tiltSearchBall(0.0);
							Walk(0.0, 0.0, 0.0);
						}
						tunggu++;
						delay = 0;
                                        } else {
						trackBall();
						tunggu = 0;

						if (robotDirection && headPan >= -0.4 && headPan <= 0.4) {
							//if (headTilt >= (cSekarang - 0.2)) {
								if (delay > 5) {
									cekWaktu(10);
									if(timer) {
										resetCase4();
										stateCondition = 4;
									} else {
										dribble(sudutTengah, 0.15);
										//followBall(0);
									}
								} else {
									setWaktu();
									delay++;
								}
							//} else {
							//	if (headTilt >= -1.0) {
							//		ballPositioning(0.0, cSekarang, 0.12);
							//	} else {
							//		followBall(0);
							//	}
							//}
						} else {
							if (headTilt >= cAktif && headPan >= -0.8 && headPan <= 0.8) {
								Imu(sudutTengah, cSekarang);
							} else {
								robotDirection = false;
								followBall(0);
							}
						}
					}
				break;

				case -10: // serang lurus -> eksekusi tendang
					if (ballLost(20)) {
						if (tunggu > 40) {
							resetCase1();
							stateCondition = 1;
						} else {
							tiltSearchBall(0.0);
							Walk(0.0, 0.0, 0.0);
						} tunggu++;
					} else {
						trackBall();
						tunggu = 0;

						if (tendang) {
							Walk(0.0, 0.0, 0.0);
							resetCase6();
							stateCondition = 6;
						} else {
							rotateKickOff(0.0, tendangJauh);
							//rotateKickOffImu(0, tendangDekat);
							//kick(tendangOper);
						}
					}
				break;

				case 10: // serang ke kanan
					if (ballLost(20)) {
						if (tunggu > 40) {
							resetCase1();
							stateCondition = 1;
						} else {
							tiltSearchBall(0.0);
							Walk(0.0, 0.0, 0.0);
						} tunggu++;
					} else {
						trackBall();
						tunggu = 0;

						if (tendang) {
							Walk(0.0, 0.0, 0.0);
							resetCase6();
							stateCondition = 6;
						} else {
							if (!usePenaltyStrategy) {
								rotateKickOff(-2, tendangDekat); //-2.5
								//rotateKickOffImu(30, tendangDekat);
							} else {
								rotateKickOff(-2, tendangDekat); //-2.5
								//rotateKickOffImu(25, tendangJauh);
							}
						}
					}
				break;

				case 20: // serang ke kiri
					if (ballLost(20)) {
						if (tunggu > 40) {
							resetCase1();
							stateCondition = 1;
						} else {
							tiltSearchBall(0.0);
							Walk(0.0, 0.0, 0.0);
						} tunggu++;
					} else {
						trackBall();
						tunggu = 0;

						if (tendang) {
							Walk(0.0, 0.0, 0.0);
							resetCase6();
							stateCondition = 6;
						} else {
							if (!usePenaltyStrategy) {
								rotateKickOff(2, tendangDekat); //2.5
								//rotateKickOffImu(-30, tendangDekat);
							} else {
								rotateKickOff(2, tendangDekat); //2.5
								//rotateKickOffImu(-25, tendangJauh);
							}
						}
					}
				break;

				case 30: // Backup kiri-kanan
					if (ballLost(20)) {
						delay = 0;
						if (tunggu > 40) {
							resetCase1();
							stateCondition = 1;
						} else {
							tiltSearchBall(0.0);
							Walk(0.0, 0.0, 0.0);
						} tunggu++;
					} else {
						trackBall();
						tunggu = 0;

						if (useCoordination) { backToCoordinations(); }

						if (delay > 30) {
							//if (headTilt >= -1.5 && headPan >= -0.2 && headPan <= 0.2) {
							if (headTilt >= -1.8 && headPan >= -0.2 && headPan <= 0.2) {
							//if (headTilt >= -1.6 && headPan >= -0.2 && headPan <= 0.2) {
								resetCase2();
								stateCondition = 2;
							} else {
								followBall(0);
							}
						} else {
							Walk(0.0, 0.0, 0.0);
							delay++;
						}
					}
				break;

				case 50: // pickup
					if (backIn) {
						countInitPos = 0;
					} else {
						countInitPos = 1;
					}//lock initial pos
					cekWaktu(30);
					if (second > 5) { //5
						backIn = false;
						if (receler.Ball_X == -1 && receler.Ball_Y == -1 && !tracked) {
							delay = 0;

							if (timer) {
								Walk(0.0, 0.0, 0.0);
								resetCase1();
								stateCondition = 1;
							} else {
								threeSearchBall();
								if (second <= 15) {
									Walk(kejar, 0.0, 0.0);
								} else {
									jalanDirection(kejar, 0.0, 0); //XYA
								}
							}
						} else {
							tracked = true;
						}

						if (tracked) {
							if (ballLost(20)) {
								tracked = false;
							} else {
								trackBall();

								if (useCoordination) { backToCoordinations(); }

								if (delay > 30) {
									//if (headTilt >= -1.5 && headPan >= -0.2 && headPan <= 0.2) {
									if (headTilt >= -1.8 && headPan >= -0.2 && headPan <= 0.2) {
									//if (headTilt >= -1.6 && headPan >= -0.2 && headPan <= 0.2) {
										resetCase2();
										stateCondition = 2;
									} else {
										followBall(0);
									}
								} else {
									Walk(0.0, 0.0, 0.0);
									delay++;
								}
							}
						}
					} else {
						if (second > 2) { tiltSearchBall(0.0); }
						else {
							predictGoal(angle, -1.4);
						}
						Walk(kejar, 0.0, 0.0);
					}
				break;

				case 130: // defense didepan
					cekWaktu(10);
					if (second < 3) {
						ball_panKP	= 0.05;
						ball_panKD	= 0.0000755;
						ball_tiltKP	= 0.05;
						ball_tiltKD	= 0.0000755;
					}

					if (timer) {
						motion("9");

						loadConfig(); //robot parameters
						resetCase2();
						stateCondition = 2;
					} else {
						if (receler.Ball_X == -1 && receler.Ball_Y == -1 && !tracked) {
							if ( countDef > 5 ) {
								normalSearchBall();
							} else {
								resetCase1();
								countDef++;
							}
						} else {
							tracked = true;
						}

						if (tracked) {
							if (ballLost(30)) {
								tracked = false;
							} else {
								trackBall();

								if (tunggu >= 100) {
									hitungGerakBola();
									if (kondisiBola == 0) {
										motion("0");
									} else {
										motion("9");

										loadConfig(); //robot parameters
										resetCase2();
										stateCondition = 2;
									}
								} else {
									motion("0");
									tunggu++;
								}
							}
						}
					}
				break;

				case 140: // defense disamping
					cekWaktu(10);
					if (second < 3) {
						ball_panKP	= 0.05;
						ball_panKD	= 0.0000755;
						ball_tiltKP	= 0.05;
						ball_tiltKD	= 0.0000755;
					}

					if (timer) {
						motion("9");

						loadConfig(); //robot parameters
						resetCase2();
						stateCondition = 2;
					} else {
						if (receler.Ball_X == -1 && receler.Ball_Y == -1 && !tracked) {
							if ( countDef > 5 ) {
								normalSearchBall();
							} else {
								resetCase1();
								countDef++;
							}
						} else {
							tracked = true;
						}

						if (tracked) {
							if (ballLost(30)) {
								tracked = false;
							} else {
								trackBall();

								if (tunggu >= 100) {
									hitungGerakBola();
									if (kondisiBola == 0) {
										motion("0");
									} else {
										motion("9");

										loadConfig(); //robot parameters
										resetCase2();
										stateCondition = 2;
									}
								} else {
									motion("0");
									tunggu++;
								}
							}
						}
					}
				break;

				case 150: // setting receler
				break;

				case 200: // heandle error Imu
					if (strategy.bearingValue == 0) {
						zeroState = true;

						motion("0");
						headMove(batasKanan, batasAtas);
					} else {
						zeroState = false;

						resetCase1();
						stateCondition = 1;
					}
				break;

				case 1000:
					//normalSearchBall();
					motion("0");
					//predictGoalTeam(180, posTiltGoal);
					//Walk(0.0, 0.0, 0.0);
					if (goalLost(20)) {
						//threeSearchBall();
						//searchBallRectang(-1.5, -1.6, -0.8, 1.6);
						//tiltSearchBall(0.0);
						panSearchBall(-1.6);
						//headMove(0.0, -1.8);
					} else {
						trackGoal();
					//	updateCoordinatFromVision();
					}
					//kalkulasiJarakGawang(receler.Goal_LD, receler.Goal_RD); //jarak menjadi waktu
				break;

				case 1001:	//untuk ambil data odometry per langkahnya
					//headMove(0.0,-1.5);
					getServPos();

					if (odometry.csmKiri >= 10) {
						motion("0");
						//if ( varCount > 5 ) {
						//	cekWaktu(3);
						//	if (timer) {
						//		motion("7");
						//	}
						//} else {
						//	setWaktu();
						//	varCount++;
						//}
					} else {
						motion("9");
						jalanDirection(0.0,0.03,0);
					}

					/*
					odometry.bacaKondisi(odometry.RobotWalk_X, odometry.RobotWalk_Y);
					if (varCount > 200) {
						motion("0");
					} else {
						Walk(0.0,-0.03,0.0);
						motion("9");
						printf("  Lleg(%.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf),", Lleg1, Lleg2, Lleg3, Lleg4, Lleg5, Lleg6);
						printf("  Rleg(%.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf),\n", Rleg1, Rleg2, Rleg3, Rleg4, Rleg5, Rleg6);
						varCount++;
					}
					*/
				break;

				case 1002 :

				break;

				default:
				break;
			}
		}
	}
	system("killall -9 screen;");
	return 0;
}
