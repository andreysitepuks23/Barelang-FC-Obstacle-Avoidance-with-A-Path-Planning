#include "BarelangFC-Receiver.h"

bool UDP_LAN = false;

using namespace BarelangFC;

void BarelangReceiver::die(char *s) {
    perror(s);
    exit(1);
}

int abs(int data) {
	if (data < 0) data = data * -1;
	else data = data;
	return data;
}

void BarelangReceiver::initialize_vision() {
	if(UDP_LAN) {
		socTrim = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
		memset((char *) &addrTerima, 0, sizeof(addrTerima));
		addrTerima.sin_family = AF_INET;
		addrTerima.sin_port = htons(PORT_VISION);
		addrTerima.sin_addr.s_addr = htonl(INADDR_ANY);
		bind(socTrim, (struct sockaddr *) &addrTerima, sizeof(addrTerima));
	} else {
		if ((v=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
			die("socket");
		}
		memset((char *) &si_meV, 0, sizeof(si_meV));
		si_meV.sin_family = AF_INET;
		si_meV.sin_port = htons(PORT_VISION);
		si_meV.sin_addr.s_addr = htonl(INADDR_ANY);
		if (bind(v, (struct sockaddr*)&si_meV, sizeof(si_meV)) == -1) {
			die("bind");
		}
	}
}

void BarelangReceiver::initialize_localization() {
	if ((l=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
		die("socket");
	}
	memset((char *) &si_meL, 0, sizeof(si_meL));
	si_meL.sin_family = AF_INET;
	si_meL.sin_port = htons(PORT_LOCALIZATION);
	si_meL.sin_addr.s_addr = htonl(INADDR_ANY);
	if (bind(l, (struct sockaddr*)&si_meL, sizeof(si_meL)) == -1) {
		die("bind");
	}
}

void BarelangReceiver::processing_vision() {
	initialize_vision();

	while(1) {
		fflush(stdout);
		bzero(recvVision,sizeof(recvVision));
		if(UDP_LAN) {
			recvfrom(socTrim, recvVision, BUFLEN_VISION, 0, (struct sockaddr *) &addrKirim, &slen);
		} else {
			if ((recv_lenV = recvfrom(v, recvVision, BUFLEN_VISION, 0, (struct sockaddr *) &si_otherV, &slenV)) == -1) {
				die("recvfrom()");
			}
		}
		//printf("Data Vision : %s\n",recvVision);
		index_vision = 0;
		parseVision = strtok (recvVision,",");

		while (parseVision != NULL) {
			index_vision++;
			if (index_vision == 1)      { sscanf(parseVision,"%d", &Ball_X); }
			else if (index_vision == 2) { sscanf(parseVision,"%d", &Ball_Y); }
			else if (index_vision == 3) { sscanf(parseVision,"%d", &Ball_D); }
			else if (index_vision == 4) { sscanf(parseVision,"%d", &Goal_X); }
			else if (index_vision == 5) { sscanf(parseVision,"%d", &Goal_Y); }
			else if (index_vision == 6) { sscanf(parseVision,"%d", &Goal_LD); }
			else if (index_vision == 7) { sscanf(parseVision,"%d", &Goal_RD); }
			else if (index_vision == 8) { sscanf(parseVision,"%d", &Pinalty_D); }
			else if (index_vision == 9) { sscanf(parseVision,"%d", &Lcross_LD); }
			else if (index_vision == 10) { sscanf(parseVision,"%d", &Lcross_RD); }
			else if (index_vision == 11) { sscanf(parseVision,"%d", &Xcross_LD); }
			else if (index_vision == 12) { sscanf(parseVision,"%d", &Xcross_RD); }
			else if (index_vision == 13) { sscanf(parseVision,"%d", &Tcross_LD); }
			else if (index_vision == 14) { sscanf(parseVision,"%d", &Tcross_RD); }
			parseVision = strtok (NULL,",");
		}

		if (Ball_D == -2147483648 || Ball_D == 2147483647) {
			Ball_D = 0;
		}
		if (Goal_LD == -2147483648 || Goal_LD == 2147483647) {
			Goal_LD = 0;
		}
		if (Goal_RD == -2147483648 || Goal_RD == 2147483647) {
			Goal_RD = 0;
		}
		if (Pinalty_D == -2147483648 || Pinalty_D == 2147483647) {
			Pinalty_D = 0;
		}
		if (Lcross_LD == -2147483648 || Lcross_LD == 2147483647) {
			Lcross_LD = 0;
		}
		if (Lcross_RD == -2147483648 || Lcross_RD == 2147483647) {
			Lcross_RD = 0;
		}
		if (Xcross_LD == -2147483648 || Xcross_LD == 2147483647) {
			Xcross_LD = 0;
		}
		if (Xcross_RD == -2147483648 || Xcross_RD == 2147483647) {
			Xcross_RD = 0;
		}
		if (Tcross_LD == -2147483648 || Tcross_LD == 2147483647) {
			Tcross_LD = 0;
		}
		if (Tcross_RD == -2147483648 || Tcross_RD == 2147483647) {
			Tcross_RD = 0;
		}
	}
}

void BarelangReceiver::processing_localization() {
	initialize_localization();
	while(1) {
		fflush(stdout);
		bzero(recvLocalization,sizeof(recvLocalization));
		if ((recv_lenL = recvfrom(l, recvLocalization, BUFLEN_LOCALIZATION, 0, (struct sockaddr *) &si_otherL, &slenL)) == -1) {
			die("recvfrom()");
		}

		index_localization = 0;
		parseLocalization = strtok (recvLocalization,",");

		while (parseLocalization != NULL) {
			index_localization++;
			if (index_localization == 1)      { sscanf(parseLocalization,"%d", &RobotCoor_X); }
			else if (index_localization == 2) { sscanf(parseLocalization,"%d", &RobotCoor_Y); }
			else if (index_localization == 3) { sscanf(parseLocalization,"%d", &HeadingCoor); }
			else if (index_localization == 4) { sscanf(parseLocalization,"%d", &BallCoor_X); }
			else if (index_localization == 5) { sscanf(parseLocalization,"%d", &BallCoor_Y); }
			parseLocalization = strtok (NULL,",");
		}
	}
}
