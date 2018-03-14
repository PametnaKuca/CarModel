#include "package.h"

short stringLength(char *str)
{
	char *tempStr = str;
	short len = 0;
	while(*(tempStr++) != '\0')
		len++;
	return len;
}

char* stringXOR(char *message)
{
		static char xorString[2];
		char *tempMsg = message + 1;
		xorString[0] = *message;
		while(*tempMsg!='\0')
		xorString[0] = xorString[0] ^ *(tempMsg++);
		xorString[1] = '\0';
		return xorString;		
}

char* createPackage(uint8_t ID, uint8_t subID, uint8_t ctrl, char *data)
{
			
	static char packageTemp[MAX_PACK_LEN + 1];
	char *xorStr;
	short size = stringLength(data) + 4; // Message length + ID byte + SUB_ID byte + CTRL byte + XOR byte
	char protocolData[] ={(char)ID , (char)subID, (char)ctrl, '\0'}; //zadnji bajt mora biti \0 zbog sprintf-fje
	
	sprintf(packageTemp, "%.3d", size); //
	strcat(packageTemp,protocolData);
	strcat(packageTemp,data); 
	xorStr = stringXOR(packageTemp);
	strcat(packageTemp, xorStr);
	return packageTemp;
}

int checkIfValid(char* str, int size){
	char xorString = *str;
	char *nextChar = str + 1;
	int tempSize = 1;
	while(*(nextChar + 1) != '\0'){
		xorString = xorString ^ *(nextChar++);
		tempSize++;
	}
	if(size == 0)
		return 0;
	//printf("Got: %c, Required: %c\r\n", xorString, *nextChar);
	tempSize = tempSize + XOR_STR_LEN - SIZE_STR_LEN;
	if((*nextChar == xorString) && (tempSize == size))
		return 1;
	//printf("Received and sent string are different!\r\n");
	return 0;
}

int getSize(char *str){
	char size[SIZE_STR_LEN];
	int i;
	for (i = 0; i < SIZE_STR_LEN; i++){
		size[i] = *(str++);
	}
	return (short)atoi(size);
}

char getID(char *str){
	char ID;
	int i;
	for(i = 0; i < SIZE_STR_LEN; i++)
		str++;
	ID = *str;
	return ID;
}

char getSubID(char *str){
	char subID;
	int i;
	for(i = 0; i < (SIZE_STR_LEN + ID_STR_LEN); i++)
		str++;
	subID = *str;
	return subID;
}

char getConf(char *str){
	char conf;
	int i;
	for(i = 0; i < (SIZE_STR_LEN + ID_STR_LEN + SUB_ID_STR_LEN); i++)
		str++;
	conf = *str;
	return conf;
}

char *getData(char *str, int size){
	char *tempMessage = (char *) malloc(sizeof (char) * (size - EXTRA_STR_LEN + 1));
	int i;
	char c[2] = " ";
	
	*tempMessage = '\0';
	/* Skip size, ID, subID and configuration bytes */
	for(i = 0;i < (SIZE_STR_LEN + ID_STR_LEN + SUB_ID_STR_LEN + CONF_STR_LEN); i++)
		str++;
	for(i = 0; i < (size - EXTRA_STR_LEN); i++){
		c[0] = *(str++);
		strcat(tempMessage, c);
	}
	return tempMessage;
}
