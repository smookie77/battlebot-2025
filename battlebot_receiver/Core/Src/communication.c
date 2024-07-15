/*
 * communication.c
 *
 *  Created on: Jul 3, 2024
 *      Author: smookie
 */
#include <communication.h>
bool adjustFromMSG(char MSG[32]){
	bool MSGok = true;
	if(MSG[0] != '0'){
		MSGok = false;
		return MSGok;
	}
	if(MSG[1] != 'b' && MSG[1] != 'n' ){
		MSGok = false;
		return MSGok;
		}
	if(MSG[4] != 'L' ){
		MSGok = false;
		return MSGok;
		}
	if(MSG[9] != 'R'){
		MSGok = false;
		return MSGok;
		}
	if(MSG[14] != 'M'){
		MSGok = false;
		return MSGok;
		}


		char batNums[] = {MSG[2], MSG[3], '\0'};
		contBat = atoi(batNums);


		char leftMNums[] = {MSG[5], MSG[6], MSG[7], MSG[8], '\0'};
		leftSpeed = atoi(leftMNums);


		char rightMNums[] = {MSG[10], MSG[11], MSG[12], MSG[13], '\0'};
		rightSpeed = atoi(rightMNums);


		char midMNums[] = {MSG[5], MSG[6], MSG[7], MSG[8], '\0'};
		midSpeed = atoi(midMNums);


	adjustCTRL(); //Adjust the PWM or DAC to the new values

	return MSGok;
}




