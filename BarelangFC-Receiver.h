#ifndef BARELANGFC_RECEIVER_H_
#define BARELANGFC_RECEIVER_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <unistd.h>

#define PORT_VISION 2000
#define PORT_LOCALIZATION 5006

#define BUFLEN_LOCALIZATION 512 //Max length of buffer
#define BUFLEN_VISION 4096 //Max length of buffer

namespace BarelangFC {
        class BarelangReceiver {
                private:
			char * parseLocalization;
			char * parseVision;
			char recvLocalization[BUFLEN_LOCALIZATION];
			char recvVision[BUFLEN_VISION];
			int index_localization;
			int index_vision;

			struct sockaddr_in si_meV, si_meL, si_otherV, si_otherL, addrTerima, addrKirim;
			socklen_t slenV, slenL, slen = sizeof(addrKirim);
			int v, l, recv_lenV, recv_lenL, socTrim;

                public:
			int Ball_X, Ball_Y, Ball_W, Ball_H, Ball_D;
			int Goal_X, Goal_Y, Goal_W, Goal_H, Goal_LH, Goal_RH, Goal_C, Goal_LD, Goal_RD;
			int Pinalty_D, Lcross_LD, Lcross_RD, Xcross_LD, Xcross_RD, Tcross_LD, Tcross_RD;
			int GoalLeft_X, GoalLeft_Y, GoalRight_X, GoalRight_Y;
			int RobotCoor_X, RobotCoor_Y, BallCoor_X, BallCoor_Y, HeadingCoor;

			void initialize_vision();
			void initialize_localization();
			void processing_vision();
			void processing_localization();
			void die(char *s);
        };
}
#endif
