/**
*	@file package.h
*	@brief This file contains all function declartions used for creating protocol package.
*	
*/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stdint.h"

#define MAX_PACK_LEN 999
#define SIZE_STR_LEN 3
#define ID_STR_LEN 1
#define SUB_ID_STR_LEN 1
#define CONF_STR_LEN 1
#define XOR_STR_LEN 1
#define EXTRA_STR_LEN (ID_STR_LEN + SUB_ID_STR_LEN + CONF_STR_LEN + XOR_STR_LEN)

/**
*	The function measures and returns string length.
* @param string which we want to measure
*/
//short stringLength(char *str);
/**
*	The function packs all the data into data package and returns this package
* as a string. It adds XOR byte at the end for error detection.
* @param sensor type ID, sensor sub ID, control byte, data string to send
*/
char* createPackage(uint8_t ID, uint8_t subID, uint8_t ctrl, char *data);

/*  Function checks if the received string is valid by xor-ing 
 *  input string (ignoring the last byte) and compares it with 
 *  the last byte which is before calculated xor value
 *  Additionaly, function counts takes required size of the message and while
 *  calculating xor value, calculates received string size.
 *	@param str -> string to check
 *  @param size -> size of the string (without first 3 bytes)
 * 	return -> 0 if different string was received than sent, 1 if strings are same
 */
int checkIfValid(char* str, int size);

/*  Function which parses first 3 bytes from the input string
 * 	@param str -> string to parse
 *	return -> first 3 bytes of the string casted into short
 */
int getSize(char *str);

/*  Function parses out ID (4th byte) from the input string 
 *  @param str -> string to parse
 *  return -> 4th byte (ID)
 */
char getID(char *str);

/*  Function parses out subID (5th byte) from the input string 
 *  @param str -> string to parse
 *  return -> 5th byte (subID)
 */
char getSubID(char *str);

/*  Function parses out configuration (6th byte) from the input string 
 *  @param str -> string to parse
 *  return -> 6th byte (configuration)
 */
char getConf(char *str);

/*	Function parses message(sensor data) of the received string.  
 *	@param str -> input string
 *	@param size -> size of the string excluding first 3 bytes 
 *	return -> pointer to the first character of the message
 */
char* getData(char *str, int size);

