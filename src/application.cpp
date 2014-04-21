/**
 ******************************************************************************
 * @file    application.cpp
 * @authors  Satish Nair, Zachary Crockett and Mohit Bhoite
 * @version V1.0.0
 * @date    05-November-2013
 * @brief   Tinker application
 ******************************************************************************
  Copyright (c) 2013 Spark Labs, Inc.  All rights reserved.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation, either
  version 3 of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this program; if not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************
 */

/*****
 * ENABLE JTAG DEBUGGING */
#define USE_SWD_JTAG 1


/* Includes ------------------------------------------------------------------*/  
#include "application.h"
#include "chibi.h"

/* Function prototypes -------------------------------------------------------*/

void cmdGetShortAddr(int arg_cnt, char **args);
void cmdSetShortAddr(int arg_cnt, char **args);
void cmdSend(int arg_cnt, char **args);
int strCat(char *buf, unsigned char index, char arg_cnt, char **args);

int tinkerDigitalRead(String pin);
int tinkerDigitalWrite(String command);
int tinkerAnalogRead(String pin);
int tinkerAnalogWrite(String command);

/* This function is called once at start up ----------------------------------*/
void setup()
{
  //Setup the Tinker application here

  //Register all the Tinker functions
  Spark.function("digitalread", tinkerDigitalRead);
  Spark.function("digitalwrite", tinkerDigitalWrite);

  Spark.function("analogread", tinkerAnalogRead);
  Spark.function("analogwrite", tinkerAnalogWrite);

  // Initialize the chibi command line and set the speed to 57600 bps
  chibiCmdInit(57600);

  // TODO: Remove this. It's just here becasue my test board /RESET pin was stuck in A0. 
  //       It should be simply tied to the Sparkcore's main /RESET line.
  pinMode(A0, OUTPUT); // Chibi Radio /RESET
  digitalWrite(A0, LOW);
  delay(5);
  digitalWrite(A0, HIGH);
  delay(5);

  // Initialize the chibi wireless stack
  chibiInit();

  // This is where you declare the commands for the command line.
  // The first argument is the alias you type in the command line. The second
  // argument is the name of the function that the command will jump to.

  chibiCmdAdd("getsaddr", cmdGetShortAddr);  // set the short address of the node
  chibiCmdAdd("setsaddr", cmdSetShortAddr);  // get the short address of the node
  chibiCmdAdd("send", cmdSend);   // send the string typed into the command line

}

/* This function loops forever --------------------------------------------*/
void loop()
{
  static uint8_t count = 0;
  uint8_t data[] = "PING-000";

  // This function checks the command line to see if anything new was typed.
  chibiCmdPoll();

  // Check if any data was received from the radio. If so, then handle it.
  if (chibiDataRcvd() == true)
  { 
    int rssi, src_addr;
    byte buf[100];  // this is where we store the received data

    // retrieve the data and the signal strength
    chibiGetData(buf);
    rssi = chibiGetRSSI();
    src_addr = chibiGetSrcAddr();

    // Print out the message and the signal strength
    Serial.print("Message received from node 0x");
    Serial.print(src_addr, HEX);
    Serial.print(": "); 
    Serial.print((char *)buf); 
    Serial.print(", RSSI = 0x"); Serial.println(rssi, HEX);
  }

  // Continuous PING
  sprintf((char *)data, "PING-%03d", count);
  chibiTx((uint16_t)0xFFFFF, data, 9);
  count++;
  delay(500);
}

/**************************************************************************/
// CHIBI USER FUNCTIONS
/**************************************************************************/

/**************************************************************************/
/*!
  Get short address of device from EEPROM
Usage: getsaddr
 */
/**************************************************************************/
void cmdGetShortAddr(int arg_cnt, char **args)
{
  int val;

  val = chibiGetShortAddr();
  Serial.print("Short Address: "); Serial.println(val, HEX);
}

/**************************************************************************/
/*!
  Write short address of device to EEPROM
Usage: setsaddr <addr>
 */
/**************************************************************************/
void cmdSetShortAddr(int arg_cnt, char **args)
{
  int val;

  val = chibiCmdStr2Num(args[1], 16);
  chibiSetShortAddr(val);
}

/**************************************************************************/
/*!
  Transmit data to another node wirelessly using Chibi stack. Currently
  only handles ASCII string payload
Usage: send <addr> <string...>
 */
/**************************************************************************/
void cmdSend(int arg_cnt, char **args)
{
  byte data[100];
  int addr, len;

  // convert cmd line string to integer with specified base
  addr = chibiCmdStr2Num(args[1], 16);

  // concatenate strings typed into the command line and send it to
  // the specified address
  len = strCat((char *)data, 2, arg_cnt, args);    
  chibiTx(addr, data,len);
}

/**************************************************************************/
/*!
  Concatenate multiple strings from the command line starting from the
  given index into one long string separated by spaces.
 */
/**************************************************************************/
int strCat(char *buf, unsigned char index, char arg_cnt, char **args)
{
  uint8_t i, len;
  char *data_ptr;

  data_ptr = buf;
  for (i=0; i<arg_cnt - index; i++)
  {
    len = strlen(args[i+index]);
    strcpy((char *)data_ptr, (char *)args[i+index]);
    data_ptr += len;
    *data_ptr++ = ' ';
  }
  *data_ptr++ = '\0';

  return data_ptr - buf;
}

/*** CHIBI FUNCTIONS END ***/


/*******************************************************************************
 * Function Name  : tinkerDigitalRead
 * Description    : Reads the digital value of a given pin
 * Input          : Pin 
 * Output         : None.
 * Return         : Value of the pin (0 or 1) in INT type
                    Returns a negative number on failure
 *******************************************************************************/
int tinkerDigitalRead(String pin)
{
	//convert ascii to integer
	int pinNumber = pin.charAt(1) - '0';
	//Sanity check to see if the pin numbers are within limits
	if (pinNumber< 0 || pinNumber >7) return -1;

	if(pin.startsWith("D"))
	{
		pinMode(pinNumber, INPUT_PULLDOWN);
		return digitalRead(pinNumber);
	}
	else if (pin.startsWith("A"))
	{
		pinMode(pinNumber+10, INPUT_PULLDOWN);
		return digitalRead(pinNumber+10);
	}
	return -2;
}

/*******************************************************************************
 * Function Name  : tinkerDigitalWrite
 * Description    : Sets the specified pin HIGH or LOW
 * Input          : Pin and value
 * Output         : None.
 * Return         : 1 on success and a negative number on failure
 *******************************************************************************/
int tinkerDigitalWrite(String command)
{
	bool value = 0;
	//convert ascii to integer
	int pinNumber = command.charAt(1) - '0';
	//Sanity check to see if the pin numbers are within limits
	if (pinNumber< 0 || pinNumber >7) return -1;

	if(command.substring(3,7) == "HIGH") value = 1;
	else if(command.substring(3,6) == "LOW") value = 0;
	else return -2;

	if(command.startsWith("D"))
	{
		pinMode(pinNumber, OUTPUT);
		digitalWrite(pinNumber, value);
		return 1;
	}
	else if(command.startsWith("A"))
	{
		pinMode(pinNumber+10, OUTPUT);
		digitalWrite(pinNumber+10, value);
		return 1;
	}
	else return -3;
}

/*******************************************************************************
 * Function Name  : tinkerAnalogRead
 * Description    : Reads the analog value of a pin
 * Input          : Pin 
 * Output         : None.
 * Return         : Returns the analog value in INT type (0 to 4095)
                    Returns a negative number on failure
 *******************************************************************************/
int tinkerAnalogRead(String pin)
{
	//convert ascii to integer
	int pinNumber = pin.charAt(1) - '0';
	//Sanity check to see if the pin numbers are within limits
	if (pinNumber< 0 || pinNumber >7) return -1;

	if(pin.startsWith("D"))
	{
		pinMode(pinNumber, INPUT);
		return analogRead(pinNumber);
	}
	else if (pin.startsWith("A"))
	{
		pinMode(pinNumber+10, INPUT);
		return analogRead(pinNumber+10);
	}
	return -2;
}

/*******************************************************************************
 * Function Name  : tinkerAnalogWrite
 * Description    : Writes an analog value (PWM) to the specified pin
 * Input          : Pin and Value (0 to 255)
 * Output         : None.
 * Return         : 1 on success and a negative number on failure
 *******************************************************************************/
int tinkerAnalogWrite(String command)
{
	//convert ascii to integer
	int pinNumber = command.charAt(1) - '0';
	//Sanity check to see if the pin numbers are within limits
	if (pinNumber< 0 || pinNumber >7) return -1;

	String value = command.substring(3);

	if(command.startsWith("D"))
	{
		pinMode(pinNumber, OUTPUT);
		analogWrite(pinNumber, value.toInt());
		return 1;
	}
	else if(command.startsWith("A"))
	{
		pinMode(pinNumber+10, OUTPUT);
		analogWrite(pinNumber+10, value.toInt());
		return 1;
	}
	else return -2;
}
