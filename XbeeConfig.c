/*
 * XbeeConfig.c
 *
 * provides simple character I/O through MSP432 Launchpad backchannel UART
 * RWB 3/17/2017 Original
 * IJL 5/24/2017 Modified for what I needed
 */

#include "driverlib.h"
/* Standard Includes */
#include <stdio.h>
#include <string.h>

extern int testIdx;
extern char *XbeeString;
extern Delay1ms();

void Xbee_puts(char *outString)
{
  unsigned int i, len;

  len = strlen(outString);

  for(i=0 ; i<len ; i++)
  {
      while((UCA1IFG & UCTXIFG) != UCTXIFG);  // wait until flag is set to indicate a new byte can be sent
      UCA1TXBUF = (uint8_t) outString[i];;  // load register with byte to send
  }
}
