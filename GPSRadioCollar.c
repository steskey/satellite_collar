/* The blood sweat and tears of 2 less than fortunate EE's that programmed this. */

/* DriverLib Includes */
#include "driverlib.h"
#include "msp432.h"
#include "rom_map.h"

#include "GPS.h"

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

//Declaration of functions that are used within this main code that are called in other C files.
//Mainly just here to help keep this file smaller.
extern void IOSetup(void);
extern void Xbee_puts(char *outString);
extern void initPCUART(void);
extern void initXbeeUART(void);
extern void initGPSUART(void);
extern void disablePCUART(void);
extern void disableXbeeUART(void);
extern void disableGPSUART(void);

//Where Flash Bank 1 begins
#define START_LOC 0x00020000

//Flash Globals
volatile _Bool NoFixFlag = 0; //In the event that the GPS did not get a fix within the alloted time, this flag is set to a 1.
static volatile char CurrentFixSaveString[33]; //This is the string that is constructed that has the time, date, lat and long
//in it that came from the GPS when a fix was to be obtained. When there isn't a fix, it's populated properly before getting
//written to flash.
static volatile char FixRead[33]; //Stores the 32 bytes from flash that has a fix in it, it's 33 length for the end character
static volatile char SectorRead[4097]; //Same, but this one can store an entire sector.
volatile _Bool MemoryFull = 0; //Once the end of flash has been reached, this gets set to trigger the collar to be in longevity mode
//which means that it doesn't store anymore fixes into flash.
static volatile uint8_t FixMemoryLocator[2]; //FixMemoryLocator[0] stores position in the current sector, range 0-124. //??
//FixMemoryLocator[1] stores the current sector, range 0-31.
static volatile uint8_t MemPlaceholder[2]; //MemPlaceholder[0] stores position in the readout sector, range 0-128. //??
//MemPlaceholder[1] stores the readout sector, range 0-31

//Status Globals
//Req is when a request for a device is on, enable is what actually turns on a pin or off. Enables are set by the StateMachine
//which handles the requests and the priorites set out
volatile _Bool XbeeReq = 0;
volatile _Bool XbeeEn = 0;
volatile _Bool GPSReq = 0;
volatile _Bool GPSEn = 0;
volatile _Bool VHFReq = 0;
volatile _Bool VHFEn = 0;

//Hour or Minute RTC interrupt flags
volatile _Bool HourInt = 1; //Basically when we have nothing that is time critical to the minute, we want to wake up less
//when Xbee is on, we are in the minute interrupt and want to turn it on and off accordingly. It is too much work to wake up
//with the SysTick every second or so and create another counter.  This uses the low frequency external oscillator.
volatile _Bool MinInt = 0;

//Scheduling Globals
volatile uint8_t XbeeMinCount = 0; //Used within the RTC handler which counts from 1 to 4 and gets reset, this is how the Xbee
//is turned on and off accordingly, one minute out of every 5.
volatile uint8_t XbeeConnectionTimeWindowCount = 0; //Used to keep track of the number of hours the Xbee has been on for in terms
//of scheduling.  This made it so the Xbee can handle being on over midnight without creating more convoluted code.

//RTC Globals
static volatile RTC_C_Calendar SystemTime; //This is for within the system, it gets updated every time the RTC interrupt happens
static volatile RTC_C_Calendar SetTime; //Had to have a separate structure which got populated for updating the RTC time.  It was weird
//in that it wouldn't work with using the same structure, likely because the RTC handler was messing with it.

//UART ISR Globals
volatile uint8_t RXData = 0; //These are where the characters obtained on the UART buffer for each channel are stored.
volatile uint8_t RX0Data = 0;
volatile uint8_t RX1Data = 0;

//PC Hardwire Globals
volatile char PCdataString[99]; //Puts the characters from the buffer into the string and increments so we can get more than one character
//at a time and get a useful message
volatile char PCString[99]; //Puts the raw string from the buffer into a new location for parsing. Gets updated when the PCdataString has a complete string.
volatile int PCindex = 0; //Increments within the string and puts the characters in and increments to the next position
volatile int PCflag = 0; //Used for when there has been the start character detected and the characters on the buffer are to be put into PCdataString
volatile int PCStringExtractGo = 0; //When a complete string has been formed and needs to be parsed, this flag gets set.
volatile _Bool USBPresentFlag = 0; //When the USB is connected, this flag is set and used within the code.
volatile _Bool USBPresentOns = 1; //One shot to initialize the PC UART channel
volatile _Bool MagnetRemovedFlag = 0; //Used for when the magnet has been removed and the "start up" procedure can take place.

//Power Calculation Globals
//All of these uint8_t's are for power calc's. The second counts are incremented in the SysTick ISR, and corresponding minutes and seconds thereafter.
volatile uint8_t XbeeSecCount;
volatile uint8_t XbeeSecOnCount;
volatile uint8_t XbeeMinOnCount;
volatile uint8_t XbeeHourOnCount;
volatile uint8_t GPSAttemptOnTime = 0; //Used for determining if the GPS has timed out and not gotten a fix
volatile uint8_t GPSSecOnCount = 0;
volatile uint8_t GPSMinOnCount = 0;
volatile uint8_t GPSHourOnCount = 0;
volatile _Bool LongevityMode = 0; //When the battery is on the last legs this flag gets set and doesn't allow the GPS to get data points anymore
volatile _Bool CurrentLocationRequestInitiated = 0; //On the Xbee if the researcher requests for the current location of the collar, this is set
//and allows the location to be determined.
volatile _Bool NowGetFix = 0; //Another bit for getting the current location of the collar
volatile char BatteryString[4]; //The value of the calculated battery percentage is stored in here and sent over the PC and Xbee connection

volatile uint8_t VHFStartUpCount = 0; //On startup, this gets incremented and is a "beacon" so they know the collar is on.

volatile _Bool gpsoneshot = 1; //These one shots are because we do PLC programming and that's how our brains work. But they're also here
//so we didn't reinitialize UART channels over and over and over and over and over and over again.  It seemed to cause problems. Hence
//one shot.
volatile _Bool xbeeoneshot = 0;
volatile _Bool gpsnegons = 0;
volatile _Bool xbeenegons = 0;
volatile _Bool vhfoneshot = 0;

//Cool structure I made to store all of the parameters obtained through the UART connection
typedef struct _Configparameters
{
    uint8_t COM; //Command program or read (program or read)
    uint8_t GPS; //GPS sample interval
    uint8_t WTM; //Wireless transmission mode (confirmed or spew)
    uint8_t WTD; //Wireless transmission day
    uint8_t WCT; //Wireless connection start time
    uint8_t WCW; //Wireless connection window
    uint8_t VST; //VHF broadcast start time
    uint8_t VET; //VHF broadcast end time
    uint8_t DOP; //PDOP Threshold
    uint8_t GTO; //GPS timeout
} Configparameters;
static volatile Configparameters Config;

//GPS Globals
volatile char dataString[300]; //Raw characters from the buffer are put in here
volatile char GPSString[300]; //When something needed to be parsed then it's put in this string
volatile int index = 0; //Same thing as PC string
volatile int flag = 0; // Look above for the comments on these, the naming stays the same
volatile int GPSStringClassifyGo = 0;
volatile _Bool FixAttemptFailed = 0;

static volatile GPSinfostructure GPSinfo;

//Xbee Globals
volatile char XbeedataString[20]; //Same for the data, uart, index, and flag as normal. Also stringextract
volatile char XbeeUARTString[20];
volatile int Xbeeindex = 0;
volatile int Xbeeflag = 0;
volatile int XbeeStringExtractGo = 0;
volatile _Bool XbeeTransmissionComplete = 0; //This is to not turn the Xbee on again for the rest of the connection
//window if it has already completed the task for the week.
volatile _Bool LongevityXbeeBypass = 0; //This is a bypass for when the current location has been requested
volatile char XbeePutsString[20]; //Store what you want to push out over the Xbee into a string before shipping it off.
volatile _Bool XbeeTransmissionIP[8]; //These are there so the Xbee doesn't immediately shut off while it still has stuff
//in the buffer ready to ship out.
volatile _Bool AllSent = 0; //This is if the entire data set was requested and sent over the Xbee, in this case for the
//check sum it will set the last known confirmed connection pointer to be at the end so it doesn't have to be re-sent if
//you don't desire.
volatile _Bool WaitingForTAFT = 0; //This is a flag which blocks multiple transmissions of data if the request doesn't shut off
//fast enough.

volatile char PDOPString[6];

//Another structure for the Xbee parameters that are obtained.
typedef struct _Xbeeparameters
{
    uint8_t Command; //Command read all or since last confirmed connection
    //1 = all data within flash, 2 = since last confirmed connection
    int NumRec; //Number of points LabView received consecutively correctly
    //during wireless download. The collar compares this with the number sent
    //from flash.  If they match the location for the next time of confirmed
    //connection is moved up.
    uint8_t CurLocReq; //Command while the device has low battery life to request
//for the current GPS location of the device. 1 = get location, 0 = no action

} Xbeeparameters;

static volatile Xbeeparameters XbeeComms;

//How a second delay is created without having to use ACLK or another source besides what I know. Delay1ms is included in this
void parrotdelay(unsigned long ulCount)
{
    __asm ( "pdloop:  subs    r0, #1\n"
            "    bne    pdloop\n");
}

void Delay1ms(uint32_t n) //just burns CPU as it decrements this counter. It was scaled to 3MHz from what we had in another project.
{
    while (n)
    {
        parrotdelay(369);    // 1 msec, tuned at 48 MHz is 5901, 3MHz set to 369
        n--;
    }
}

//This is a psuedo parser, not like the GPS one that is pretty involved.  This just checks the string that was received and
//sets the things within the structures as needed.
void XbeeCommExtract(void)
{
    int clearindex;
    int CheckSumNumber;
    uint8_t MaxPlaceholder[2];
    int MaxNumber;

    if (strncmp(&XbeeUARTString[1], "L6E9=", 5) == 0) //see if it starts with the correct proceeding information
    {
        if (strncmp(&XbeeUARTString[6], "1", 1) == 0) //Check if it's a 1 or 2, for read all or just read latest
        {
            XbeeComms.Command = 1;
            if (WaitingForTAFT == 0) //Not in longevity mode or it hasn't been sent once yet,
            //send the data
            {
                XbeeTransmissionIP[2] = 1; //These were described above in the comment about the global.

                if (LongevityMode == 0)
                {
                    Xbee_puts("@GPSDATA^@"); //Send the data
                    readout_memory_all();
                    Xbee_puts("^");
                }

                Xbee_puts("@BATTERY^@"); //Send the battery estimate
                Xbee_puts(BatteryString);
                Xbee_puts("^");

                AllSent = 1; //This is so the check sum for some validation can be incremented
                WaitingForTAFT = 1; //Sets so it only sent the data once, like a oneshot
                XbeeTransmissionIP[2] = 0;

                Delay1ms(500);
            }
            XbeeComms.Command = 0; //Resets the command within the structure
            XbeeUARTString[6] = '0'; // Clears the 1 or 2 spot so it doesn't go again.
        }

        if (strncmp(&XbeeUARTString[6], "2", 1) == 0) //If you want the latest data, it does this one.
        //This is the same idea and method like when we want to get all of the data.
        {
            XbeeComms.Command = 2;
            if (WaitingForTAFT == 0)
            {
                XbeeTransmissionIP[3] = 1;

                if (LongevityMode == 0)
                {
                    Xbee_puts("@GPSDATA^@");
                    readout_memory_new();
                    Xbee_puts("^");
                }

                Xbee_puts("@BATTERY^@");
                Xbee_puts(BatteryString);
                Xbee_puts("^");

                WaitingForTAFT = 1;
                XbeeTransmissionIP[3] = 0;
                Delay1ms(500);
            }
            XbeeComms.Command = 0;
            XbeeUARTString[6] = '0';
        }
    }

    if (strncmp(&XbeeUARTString[8], "NDNB=", 5) == 0) //Interprets if current GPS location
    //is requested in the event the device is in low power mode or they just want it
    {
        if (XbeeUARTString[13] == '1')
        {
            XbeeComms.CurLocReq = 1;
        }
    }

    if (strncmp(&XbeeUARTString[1], "TAFT=", 5) == 0) //Gets back the number of data points successfully
    //received within LabView from the start of the wireless transmission
    {
        WaitingForTAFT = 0; //Resets this so the data can be sent again if so desired.

        //Gets the numeric value from the string
        XbeeComms.NumRec = ((XbeeUARTString[6] - '0') * 1000)
                + ((XbeeUARTString[7] - '0') * 100)
                + ((XbeeUARTString[8] - '0') * 10) + (XbeeUARTString[9] - '0');

        //Pulls from flash the values within them
        MemPlaceholder[0] = *(uint8_t*) (0x0003E000);
        MemPlaceholder[1] = *(uint8_t*) (0x0003E001);

        MaxPlaceholder[0] = *(uint8_t*) (0x0003F000);
        MaxPlaceholder[1] = *(uint8_t*) (0x0003F001);

        //Reconstructs from a sector and place in sector value to a decimal value for easier comparison
        CheckSumNumber = MemPlaceholder[1] * 128 + MemPlaceholder[0];
        MaxNumber = MaxPlaceholder[1] * 128 + MaxPlaceholder[0];

        CheckSumNumber += XbeeComms.NumRec;

        //Checks to see that the number sent is equal to the number received. Covers both
        //just the latest points and also all points and increments the last confirmed connection locator
        if ((MaxNumber == CheckSumNumber) || MaxNumber == XbeeComms.NumRec)
        {
            if (AllSent == 1) //If all data was sent, we don't want the locator to be past where the most
            //recent fix was, so it gets set to the latest location in memory as it should be.
            {
                CheckSumNumber = MaxNumber;
                AllSent = 0;
            }
            //Takes the decimal number and breaks it down again into sector and place in sector before storing
            //it to flash
            MemPlaceholder[0] = CheckSumNumber % 128;
            MemPlaceholder[1] = CheckSumNumber / 128;

            transmission_placeholder_store(); //Storing those 2 values into flash.
        }

        if (XbeeComms.CurLocReq == 1) //If the current location is requested, this gets set.
        {
            NowGetFix = 1;
        }
        XbeeTransmissionComplete = 1;
    }

    for (clearindex = 0; clearindex < sizeof XbeeUARTString; clearindex++)
    {
        XbeeUARTString[clearindex] = "";
    }

    XbeeStringExtractGo = 0;

}

// Takes the string sent from LabView and populates the structure which houses the variables
// used to write to flash to keep the programmable parameters onboard
void ExtractParameters(void)
{
    char *TempString; //Where the tokenized portion gets stored each time the input string is tokenized
    const char PCtokenization[2] = "&"; //The token which is used to separate the input string into TempString sections
    int ParameterCounter = 1; //Each time the string is tokenized, this increments so we know which parameter we're at and
    //which variable in the structure it needs to go to
    _Bool TempStoreResult = 0;
    int clearindex;
    char PCTimeString[13];

    TempString = strtok(PCString, PCtokenization); //Takes the input string, searches for the token ('&' in this case) and takes
    //from the start of the string to the first instance of the '&' and stores that result into TempString

    while (TempString != NULL) //Goes through the whole input string until it reaches the end
    {
        switch (ParameterCounter)
        //Looks at what parameter number we're on. Switch statement makes it look nicer than "if's"
        {
        case 1: //Checks if it's a program or read command and sets the variable of COM within the structure to be a 1 or 2.
            if (TempString[4] == '1')
            {
                Config.COM = 1;
            }
            if (TempString[4] == '2')
            {
                PC_puts("@GPSDATA^@");
                readout_memory_all();
                PC_puts("^");

                XbeeSecOnCount = 255;
                XbeeMinOnCount = 255;
                store_battery_counters();
                readout_battery_counters();

                Config.COM = 0;
            }
            if (TempString[4] == '3')
            {
                PC_puts("@TIME^@");
                sprintf(PCTimeString, "%02d%02d%02d%02d%02d%02d",
                        SystemTime.hours, SystemTime.minutes,
                        SystemTime.seconds, SystemTime.dayOfmonth,
                        SystemTime.month, SystemTime.year);
                PC_puts(PCTimeString);
                PC_puts("^");
            }
            break;

        case 2: //Number of GPS fixes a day
            Config.GPS = (TempString[4] - '0') * 10 + (TempString[5] - '0'); //simple technique to get the numeric value from the ASCII character
            break;

        case 3: //Confirmed (1) or Spew (2) mode for the Xbee communication
            if (TempString[4] == '1')
            {
                Config.WTM = 1;
            }
            if (TempString[4] == '2')
            {
                Config.WTM = 2;
            }
            break;

        case 4: //Day of the week for the Wireless Transmission
            Config.WTD = TempString[4] - '0'; //Getting the decimal value from the ASCII
            break;

        case 5:
            Config.WCT = (TempString[4] - '0') * 10 + (TempString[5] - '0'); //Getting the decimal value from the ASCII
            //again.  There are two characters to signify the selection, so that's why the one in the 4th spot
            //gets multiplied by 10
            break;

        case 6:
            Config.WCW = TempString[4] - '0'; //Wireless Connection Window
            break;

        case 7:
            Config.VST = (TempString[4] - '0') * 10 + (TempString[5] - '0'); //VHF start time
            break;

        case 8:
            Config.VET = (TempString[4] - '0') * 10 + (TempString[5] - '0'); //VHF end time
            break;

        case 9:
            strncpy(GPSinfo.GPStime, &TempString[4], 6); //copies what was in dataString into GPSString
            //so it doesn't get reset and GPSString can be used and parsed
            //GPSinfo.GPStime[6] = '\0'; //puts a NULL at the end of the string again because strncpy doesn't
            break;

        case 10:
            strncpy(GPSinfo.GPSdate, &TempString[4], 6); //copies what was in dataString into GPSString
            //so it doesn't get reset and GPSString can be used and parsed
            //GPSinfo.GPStime[6] = '\0'; //puts a NULL at the end of the string again because strncpy doesn't
            break;
        case 11:
            Config.DOP = (TempString[4] - '0') * 10 + (TempString[5] - '0'); //PDOP threshold
            break;
        case 12:
            Config.GTO = (TempString[4] - '0') * 100
                    + (TempString[5] - '0') * 10 + (TempString[6] - '0'); //GPS Timeout
            break;
        default:

        }

        TempString = strtok(NULL, PCtokenization); //Getting the next time and putting it in TempString
        ParameterCounter++; //Incrementing which parameter we're going to store stuff in next time
    }

    if (Config.COM == 1) //If they did a program command, it clears all the flash data, and puts the new
    //parameters into flash. It also sets the time and date from the string.
    {
		//TODO ST 5-7-2018 Get rid of these things that erase memory
        flash_mass_erase();
        reset_memory_locator();
        transmission_placeholder_reset();
        XbeeSecOnCount = 255;
        XbeeMinOnCount = 255;
        store_battery_counters();
        readout_battery_counters();

        TempStoreResult = store_config_params();

        setTimeDate();

        if (TempStoreResult == 1) //Lets you know if the data was written to flash successfully or not
        {
            PC_puts("@OK^");
        }
        else
        {
            PC_puts("@FAIL^");
        }

        Config.COM = 0; //Resets the command that was set within the structure.
    }

    for (clearindex = 0; clearindex < sizeof PCString; clearindex++) //Clears the string that it just parsed.
    {
        PCString[clearindex] = "";
    }

    PCStringExtractGo = 0; //After tokenization is complete, reset the flag which stores stuff into the structure so this
    //function isn't called again until it's needed or a new command comes across from LabView
}

void ClassifyString(void)
{
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //FOR GGA STRING

	// TODO EK 5-7-2018 Turn into struct and/or array of ints
    int lengthofstring = 0;
    int stringIdxstart = 0;
    int stringIdxstop = 0;
    int stringCpyLength = 0;
    int Idx = 0;
    int commacount = 0;
    int TimeStartCaptured = 0;
    int TimeEndCaptured = 0;
    int LongStartCaptured = 0;
    int LongEndCaptured = 0;
    int LatStartCaptured = 0;
    int LatEndCaptured = 0;
    int FixQualityStartCaptured = 0;
    int FixQualityEndCaptured = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //FOR GSV STRING

    int getGSV = 0; //put this to a 0 if the information from the GSV flag isn't of interest
    //the main things from this are the SNR, and the number of satellites in view of the GPS.
    //This will in theory speed up this function's execution speed.
    int NumSentStartCaptured = 0;
    int NumSentEndCaptured = 0;
    int SentenceOne = 0;
    int NumSatStartCaptured = 0;
    int NumSatEndCaptured = 0;
    int SNRStartCaptured = 0;
    int SNREndCaptured = 0;
    int DateStartCaptured = 0;
    int DateEndCaptured = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //FOR GSA STRING

    int PDOPStartCaptured = 0;
    int PDOPEndCaptured = 0;

    MAP_WDT_A_clearTimer();

    lengthofstring = strlen(GPSString); //find length of the incoming string

    if (strncmp(&GPSString[2], "GGA,", 4) == 0) //see if it starts with GPGGA, if so, come in here
    //this extracts the time to set the RTC from (comes in as UTC in form HHMMSS, adjusted to be EST),
    //latitude XXYY.YYY,D where XX is the degrees, and YY.YYY is the minutes along with direction (N/S),
    //longitude XXXYY.YYY,D where XXX is degrees, and YY.YYY is the minutes along with the direction (E/W),
    //and the fix quality (looking for a 1=GPS Fix).
    {
        stringIdxstart = 5; //starts looking right after the GPGGA of the string for ','. This is the delimiter
        //used in the NMEA string
        commacount = 0; //resets the comma counter to be at the beginning

        for (Idx = stringIdxstart; Idx < lengthofstring; Idx++) //scans through the entire string that starts with
        //GPGGA to find the ','s within
        {
            if (strncmp(&GPSString[Idx], ",", 1) == 0) //every time a comma is found, the counter increases to
            //keep track
            {
                MAP_WDT_A_clearTimer();
                commacount++;
            }

            if (commacount == 1 && TimeStartCaptured == 0)
            {
                stringIdxstart = Idx; //notes where the string to copy begins (makes it dynamic, mostly for security)
                TimeStartCaptured = 1; //sets a flag so the stringIdxstart doesn't keep increasing until the next ',' is found
            }
            if (commacount == 2 && TimeEndCaptured == 0)
            {
                stringIdxstop = Idx; //notes where the string to copy ends (makes it dynamic, mostly for security)
                stringCpyLength = stringIdxstop - stringIdxstart - 1; //size of string to copy is computed and put into this variable
                strncpy(GPSinfo.GPStime, &GPSString[stringIdxstart + 1],
                        stringCpyLength); //copies the string between the ','s
                GPSinfo.GPStime[stringCpyLength] = '\0'; //puts the NULL character at the end, not done with the strncpy function :(
                TimeEndCaptured = 1; //
            }

            if (commacount == 2 && LatStartCaptured == 0)
            {
                stringIdxstart = Idx; //notes where the string to copy begins (makes it dynamic, mostly for security)
                LatStartCaptured = 1; //sets a flag so the stringIdxstart doesn't keep increasing until the next ',' is found
            }
            if (commacount == 4 && LatEndCaptured == 0)
            {
                stringIdxstop = Idx; //notes where the string to copy ends (makes it dynamic, mostly for security)
                stringCpyLength = stringIdxstop - stringIdxstart - 1; //size of string to copy is computed and put into this variable
                strncpy(GPSinfo.GPSlat, &GPSString[stringIdxstart + 1],
                        stringCpyLength); //copies the string between the ','s
                GPSinfo.GPSlat[stringCpyLength] = '\0'; //puts the NULL character at the end, not done with the strncpy function :(
                LatEndCaptured = 1; //
            }

            if (commacount == 4 && LongStartCaptured == 0)
            {
                stringIdxstart = Idx; //notes where the string to copy begins (makes it dynamic, mostly for security)
                LongStartCaptured = 1; //sets a flag so the stringIdxstart doesn't keep increasing until the next ',' is found
            }
            if (commacount == 6 && LongEndCaptured == 0)
            {
                stringIdxstop = Idx; //notes where the string to copy ends (makes it dynamic, mostly for security)
                stringCpyLength = stringIdxstop - stringIdxstart - 1; //size of string to copy is computed and put into this variable
                strncpy(GPSinfo.GPSlong, &GPSString[stringIdxstart + 1],
                        stringCpyLength); //copies the string between the ','s
                GPSinfo.GPSlong[stringCpyLength] = '\0'; //puts the NULL character at the end, not done with the strncpy function :(
                LongEndCaptured = 1; //
            }

            if (commacount == 6 && FixQualityStartCaptured == 0)
            {
                stringIdxstart = Idx; //notes where the string to copy begins (makes it dynamic, mostly for security)
                FixQualityStartCaptured = 1; //sets a flag so the stringIdxstart doesn't keep increasing until the next ',' is found
            }
            if (commacount == 7 && FixQualityEndCaptured == 0)
            {
                stringIdxstop = Idx; //notes where the string to copy ends (makes it dynamic, mostly for security)
                stringCpyLength = stringIdxstop - stringIdxstart - 1; //size of string to copy is computed and put into this variable

                if (stringCpyLength == 1)
                {
                    GPSinfo.GPSfixquality = GPSString[Idx - 1] - '0';
                }
                else
                {
                    GPSinfo.GPSfixquality = 0;
                }
                FixQualityEndCaptured = 1; //
            }

        }

    }

    if ((strncmp(&GPSString[2], "GSV,", 4) == 0) && getGSV == 1) //see if it starts with GPGSV, the goal is to see if it's the first
    //sentence, and from that extract the number of satellites in view and the SNR, this might be used, if it's not,
    //it can always be disabled with the "getGSV" flag to speed up the function
    {
        stringIdxstart = 5; //starts looking right after the GPGSV of the string for ','. This is the delimiter
        //used in the NMEA string
        commacount = 0; //resets the comma counter to be at the beginning

        for (Idx = stringIdxstart; Idx < lengthofstring; Idx++) //scans through the entire string that starts with
        //GPGSV to find the ','s within
        {
            if (strncmp(&GPSString[Idx], ",", 1) == 0) //every time a comma is found, the counter increases to
            //keep track
            {
                MAP_WDT_A_clearTimer();
                commacount++;
            }

            if (commacount == 2 && NumSentStartCaptured == 0)
            {
                stringIdxstart = Idx; //notes where the string to copy begins (makes it dynamic, mostly for security)
                NumSentStartCaptured = 1; //sets a flag so the stringIdxstart doesn't keep increasing until the next ',' is found
            }
            if (commacount == 3 && NumSentEndCaptured == 0)
            {
                stringIdxstop = Idx; //notes where the string to copy ends (makes it dynamic, mostly for security)
                stringCpyLength = stringIdxstop - stringIdxstart - 1; //size of string to copy is computed and put into this variable
                if ((stringCpyLength == 1) && ((GPSString[Idx - 1] - '0') == 1))
                {
                    SentenceOne = 1;
                }
                NumSentEndCaptured = 1; //
            }
            if (SentenceOne == 1)
            {
                if (commacount == 3 && NumSatStartCaptured == 0)
                {
                    stringIdxstart = Idx; //notes where the string to copy begins (makes it dynamic, mostly for security)
                    NumSatStartCaptured = 1; //sets a flag so the stringIdxstart doesn't keep increasing until the next ',' is found
                }
                if (commacount == 4 && NumSatEndCaptured == 0)
                {
                    stringIdxstop = Idx; //notes where the string to copy ends (makes it dynamic, mostly for security)
                    stringCpyLength = stringIdxstop - stringIdxstart - 1; //size of string to copy is computed and put into this variable
                    if (stringCpyLength == 2)
                    {
                        GPSinfo.GPSnumsat = ((GPSString[Idx - 2] - '0') * 10)
                                + (GPSString[Idx - 1] - '0');
                    }
                    else
                    {
                        GPSinfo.GPSnumsat = 0;
                    }
                    NumSatEndCaptured = 1; //
                }

                if (commacount == 7 && SNRStartCaptured == 0)
                {
                    stringIdxstart = Idx; //notes where the string to copy begins (makes it dynamic, mostly for security)
                    SNRStartCaptured = 1; //sets a flag so the stringIdxstart doesn't keep increasing until the next ',' is found
                }
                if (commacount == 8 && SNREndCaptured == 0)
                {
                    stringIdxstop = Idx; //notes where the string to copy ends (makes it dynamic, mostly for security)
                    stringCpyLength = stringIdxstop - stringIdxstart - 1; //size of string to copy is computed and put into this variable

                    switch (stringCpyLength)
                    {
                    case (2):
                    {
                        //printf("\nCase = 2");
                        GPSinfo.GPSSNR = ((GPSString[Idx - 2] - '0') * 10)
                                + (GPSString[Idx - 1] - '0');
                    }
                    case (1):
                    {
                        //printf("\nCase = 1");
                        GPSinfo.GPSSNR = GPSString[Idx - 1] - '0';
                    }
                    default:
                    {
                        //printf("\nDefault");
                        GPSinfo.GPSSNR = 0;
                    }
                    }
                    SNREndCaptured = 1; //
                }
            }
        }
    }

    if (strncmp(&GPSString[2], "RMC,", 4) == 0) //see if it starts with GPRMC, the goal
    //is to get the date from it
    {
        stringIdxstart = 5; //starts looking right after the GPRMC of the string for ','. This is the delimiter
        //used in the NMEA string
        commacount = 0; //resets the comma counter to be at the beginning

        for (Idx = stringIdxstart; Idx < lengthofstring; Idx++) //scans through the entire string that starts with
        //GPGGA to find the ','s within
        {
            if (strncmp(&GPSString[Idx], ",", 1) == 0) //every time a comma is found, the counter increases to
            //keep track
            {
                MAP_WDT_A_clearTimer();
                commacount++;
            }

            if (commacount == 9 && DateStartCaptured == 0)
            {
                stringIdxstart = Idx; //notes where the string to copy begins (makes it dynamic, mostly for security)
                DateStartCaptured = 1; //sets a flag so the stringIdxstart doesn't keep increasing until the next ',' is found
            }
            if (commacount == 10 && DateEndCaptured == 0)
            {
                stringIdxstop = Idx; //notes where the string to copy ends (makes it dynamic, mostly for security)
                stringCpyLength = stringIdxstop - stringIdxstart - 1; //size of string to copy is computed and put into this variable
                strncpy(GPSinfo.GPSdate, &GPSString[stringIdxstart + 1],
                        stringCpyLength); //copies the string between the ','s
                GPSinfo.GPSdate[stringCpyLength] = '\0'; //puts the NULL character at the end, not done with the strncpy function :(
                DateEndCaptured = 1; //
            }
        }
    }

    if (strncmp(&GPSString[2], "GSA,", 4) == 0) //see if it starts with GPGSA, the goal
    //is to get the horizontal and veritcal dilution of precision from these
    {
        stringIdxstart = 5; //starts looking right after the GPGSA of the string for ','. This is the delimiter
        //used in the NMEA string
        commacount = 0; //resets the comma counter to be at the beginning

        for (Idx = stringIdxstart; Idx < lengthofstring; Idx++) //scans through the entire string that starts with
        //GPGGA to find the ','s within
        {
            if (strncmp(&GPSString[Idx], ",", 1) == 0) //every time a comma is found, the counter increases to
            //keep track
            {
                MAP_WDT_A_clearTimer();
                commacount++;
            }

            if (commacount == 15 && PDOPStartCaptured == 0)
            {
                stringIdxstart = Idx; //notes where the string to copy begins (makes it dynamic, mostly for security)
                PDOPStartCaptured = 1; //sets a flag so the stringIdxstart doesn't keep increasing until the next ',' is found
            }
            if (commacount == 16 && PDOPEndCaptured == 0)
            {
                stringIdxstop = Idx; //notes where the string to copy ends (makes it dynamic, mostly for security)
                stringCpyLength = stringIdxstop - stringIdxstart - 1; //size of string to copy is computed and put into this variable
                strncpy(PDOPString, &GPSString[stringIdxstart + 1],
                        stringCpyLength); //copies the string between the ','s
                PDOPString[stringCpyLength] = '\0'; //puts the NULL character at the end, not done with the strncpy function :(
                PDOPEndCaptured = 1; //

                if (PDOPString[1] != '.')
                {
                    GPSinfo.GPSPDOP = ((PDOPString[0] - '0') * 10)
                            + (PDOPString[1] - '0');
                }
                else
                {
                    GPSinfo.GPSPDOP = (PDOPString[0] - '0');
                }

            }
        }
    }

    MAP_WDT_A_clearTimer();
    GPSStringClassifyGo = 0;
}

//Saves the current fix into memory and increments the location tracking
void save_current_fix(void)
{
    uint8_t ReadFixCount[2]; //This stores the current sector position and current sector read out from flash

    //Get current memory location
    ReadFixCount[0] = *(uint8_t*) (0x0003F000);
    ReadFixCount[1] = *(uint8_t*) (0x0003F001);

    //Compute the offset for the save address
    unsigned offset = (32 * ReadFixCount[0]) + (4096 * ReadFixCount[1]);

    //Compute the sector address
    unsigned CurSector = pow(2, ReadFixCount[1]);

    //Save fix if memory available
    if (MemoryFull == 0)
    {
        FlashCtl_unprotectSector(FLASH_MAIN_MEMORY_SPACE_BANK1, CurSector); //unprotect sector
        FlashCtl_enableWordProgramming(FLASH_IMMEDIATE_WRITE_MODE); // Allow for immediate writing
        FlashCtl_programMemory(CurrentFixSaveString,
                               (void*) 0x00020000 + offset, 32); //write the data
        FlashCtl_protectSector(FLASH_MAIN_MEMORY_SPACE_BANK1, CurSector); //protect sector
    }

    //Update the memory location tracking
	//@NOTE 5-7-2018 128 = num of fixes that can be stored in sector
    if (FixMemoryLocator[0] < 127)
    {
        FixMemoryLocator[0]++;
    }
    else
    {
        FixMemoryLocator[0] = 0;
        if (FixMemoryLocator[1] < 27) //Sectors 28, 29, 30, and 31 reserved, not for location data
        {
            FixMemoryLocator[1]++;
        }
        else
        {
            MemoryFull = 1;
            LongevityMode = 1;

            Config.WTM = 1;
            store_config_params();
        }
    }
    //Save the memory location tracking to flash
	// @NOTE 5-7-2018 Location of most recent fix in flash memory
    FlashCtl_unprotectSector(FLASH_MAIN_MEMORY_SPACE_BANK1, FLASH_SECTOR31); //unprotect sector
    FlashCtl_eraseSector(0x0003F000); //erase the sector
    FlashCtl_enableWordProgramming(FLASH_IMMEDIATE_WRITE_MODE); // Allow for immediate writing
    FlashCtl_programMemory(FixMemoryLocator, (void*) 0x0003F000, 2); //write the data
    FlashCtl_protectSector(FLASH_MAIN_MEMORY_SPACE_BANK1, FLASH_SECTOR31); //protect sector
}

//Reads out a single fix given a starting address, see flash address cheat sheet if needed
void readout_fix(unsigned startposition)
{
    int i = 0;
    for (i = 0; i < 32; i++)
    {
        FixRead[i] = *(uint8_t*) (i + startposition);
    }
    FixRead[32] = '\0';
}

//Reads out a complete sector given a starting address, see flash address cheat sheet if needed
void readout_sector(unsigned startposition)
{
    int i = 0;
    for (i = 0; i < 4096; i++)
    {
        SectorRead[i] = *(uint8_t*) (i + startposition);
    }
    SectorRead[4096] = '\0';
}

//Reads out all of the NEW memory locations with data, see flash address cheat sheet if needed
void readout_memory_new(void)
{
    int i = 0;
    uint8_t ReadFixCount[2]; //This stores the current sector position and current sector read out from flash
    uint8_t ReadPlaceholder[2]; //This stores the sector position and sector of the last point passed over wireless transmission succesfully

    //Get current memory location
    ReadFixCount[0] = *(uint8_t*) (0x0003F000);
    ReadFixCount[1] = *(uint8_t*) (0x0003F001);

    //Compute amount of points saved
    int max = (ReadFixCount[1] * 128) + ReadFixCount[0];

    //Get the address that the Xbee left off at last transmission
    ReadPlaceholder[0] = *(uint8_t*) (0x0003E000);
    ReadPlaceholder[1] = *(uint8_t*) (0x0003E001);

    //Compute the hexadecimal offset for the readout address
    unsigned offset = (32 * ReadPlaceholder[0]) + (4096 * ReadPlaceholder[1]);

    //Compute the integer offset for the readout address
    int offsetInt = (ReadPlaceholder[1] * 128) + ReadPlaceholder[0];

    //Grab fixes from flash and push them out through the Xbee
    for (i = 0; i < max - offsetInt; i++)
    {
        MAP_WDT_A_clearTimer();
        readout_fix(0x00020000 + offset + (i * 0x00000020));
        Xbee_puts(FixRead);
    }
}

//Reads out the last known GPS location from flash
void readout_last_known_location(void)
{
    MAP_WDT_A_clearTimer();
    int i = 0;
    uint8_t ReadFixCount[2]; //This stores the current sector position and current sector read out from flash

    //Get current memory location
    ReadFixCount[0] = *(uint8_t*) (0x0003F000);
    ReadFixCount[1] = *(uint8_t*) (0x0003F001);

    //Compute amount of points saved
    int max = (ReadFixCount[1] * 128) + ReadFixCount[0];

    //Grab fixes from flash and push them out through the Xbee
    for (i = max - 1; i > 0; i--)
    {
        readout_fix(0x00020000 + (i * 0x00000020));
        if (FixRead[17] != '*')
        {
            Xbee_puts(FixRead);
            i = 0; //fail safe :)
            break;
        }

    }
}

//Reads out all of the memory locations with data
void readout_memory_all(void)
{
    int i_1 = 0;
    uint8_t ReadFixCount[2]; //This stores the current sector position and current sector read out from flash

    //Get current memory location
    ReadFixCount[0] = *(uint8_t*) (0x0003F000);
    ReadFixCount[1] = *(uint8_t*) (0x0003F001);

    //Compute amount of points saved
    int max = (ReadFixCount[1] * 128) + ReadFixCount[0];

    //Grab fixes from flash and push them out through the Xbee or PC
    for (i_1 = 0; i_1 < max; i_1++)
    {
        MAP_WDT_A_clearTimer();
        readout_fix(0x00020000 + (i_1 * 0x00000020));

        if (USBPresentFlag == 1)
        {
            PC_puts(FixRead);
        }
        else
        {
            Xbee_puts(FixRead);
        }
    }
}

//Resets the memory location tracking
void reset_memory_locator(void)
{
    FixMemoryLocator[0] = 0;
    FixMemoryLocator[1] = 0;
    FlashCtl_unprotectSector(FLASH_MAIN_MEMORY_SPACE_BANK1, FLASH_SECTOR31); //unprotect sector
    FlashCtl_eraseSector(0x0003F000); //erase the sector
    FlashCtl_enableWordProgramming(FLASH_IMMEDIATE_WRITE_MODE); // Allow for immediate writing
    FlashCtl_programMemory(FixMemoryLocator, (void*) 0x0003F000, 2); //write the data
    FlashCtl_protectSector(FLASH_MAIN_MEMORY_SPACE_BANK1, FLASH_SECTOR31); //protect sector
}

//Executes a mass erase of the flash
void flash_mass_erase(void)
{
    int i = 0;
    int sector_array[32] = { FLASH_SECTOR0, FLASH_SECTOR1, FLASH_SECTOR2,
    FLASH_SECTOR3,
                             FLASH_SECTOR4, FLASH_SECTOR5,
                             FLASH_SECTOR6,
                             FLASH_SECTOR7, FLASH_SECTOR8, FLASH_SECTOR9,
                             FLASH_SECTOR10,
                             FLASH_SECTOR11,
                             FLASH_SECTOR12,
                             FLASH_SECTOR13, FLASH_SECTOR14, FLASH_SECTOR15,
                             FLASH_SECTOR16,
                             FLASH_SECTOR17,
                             FLASH_SECTOR18,
                             FLASH_SECTOR19, FLASH_SECTOR20, FLASH_SECTOR21,
                             FLASH_SECTOR22,
                             FLASH_SECTOR23,
                             FLASH_SECTOR24,
                             FLASH_SECTOR25, FLASH_SECTOR26, FLASH_SECTOR27,
                             FLASH_SECTOR28,
                             FLASH_SECTOR29,
                             FLASH_SECTOR30,
                             FLASH_SECTOR31 };

    for (i = 0; i < 32; i++)
    {
        WDT_A_clearTimer();
        FlashCtl_unprotectSector(FLASH_MAIN_MEMORY_SPACE_BANK1,
                                 sector_array[i]);
    }
    FlashCtl_initiateMassErase();
    for (i = 0; i < 32; i++)
    {
        WDT_A_clearTimer();
        FlashCtl_protectSector(FLASH_MAIN_MEMORY_SPACE_BANK1, sector_array[i]);
    }
}

//Stores the memory location of the next wireless transmission start point
void transmission_placeholder_store(void)
{
    //Save the last successful transmission memory location to flash
    FlashCtl_unprotectSector(FLASH_MAIN_MEMORY_SPACE_BANK1, FLASH_SECTOR30); //unprotect sector
    FlashCtl_eraseSector(0x0003E000); //erase the sector
    FlashCtl_enableWordProgramming(FLASH_IMMEDIATE_WRITE_MODE); // Allow for immediate writing
    FlashCtl_programMemory(MemPlaceholder, (void*) 0x0003E000, 2); //write the data
    FlashCtl_protectSector(FLASH_MAIN_MEMORY_SPACE_BANK1, FLASH_SECTOR30); //protect sector
}

//Resets the the placeholder for the next wireless transmission
void transmission_placeholder_reset(void)
{
    MemPlaceholder[0] = 0;
    MemPlaceholder[1] = 0;
    FlashCtl_unprotectSector(FLASH_MAIN_MEMORY_SPACE_BANK1, FLASH_SECTOR30); //unprotect sector
    FlashCtl_eraseSector(0x0003E000); //erase the sector
    FlashCtl_enableWordProgramming(FLASH_IMMEDIATE_WRITE_MODE); // Allow for immediate writing
    FlashCtl_programMemory(MemPlaceholder, (void*) 0x0003E000, 2); //write the data
    FlashCtl_protectSector(FLASH_MAIN_MEMORY_SPACE_BANK1, FLASH_SECTOR30); //protect sector
}

void RTC_setup(void)
{
    /* Specify an interrupt to assert every hour */
//    MAP_RTC_C_setCalendarEvent(RTC_C_CALENDAREVENT_MINUTECHANGE);
    MAP_RTC_C_setCalendarEvent(RTC_C_CALENDAREVENT_HOURCHANGE);

    /* Enable interrupt for RTC Ready Status, which asserts when the RTC
     * Calendar registers are ready to read.
     * Also, enable interrupts for the Calendar alarm and Calendar event. */

    //delete the read_ready_interrupt once this is ready to be buttoned up more
    MAP_RTC_C_clearInterruptFlag(
    RTC_C_TIME_EVENT_INTERRUPT | RTC_C_CLOCK_READ_READY_INTERRUPT);
    MAP_RTC_C_enableInterrupt(
    RTC_C_TIME_EVENT_INTERRUPT | RTC_C_CLOCK_READ_READY_INTERRUPT);

    /* Enable interrupts and go to sleep. */
    MAP_Interrupt_enableInterrupt(INT_RTC_C);
}

void setTimeDate(void) //This takes the data within the GPS time and date strings
//and updates the RTC to reflect that.
{
    //Declaration of variables that are used to set the RTC.
    int GPShours;
    int GPSminutes;
    int GPSseconds;
    int GPSmonth;
    int GPSday;
    int GPSyear;
    int GPSdayofweek;
    int DOW1;
    int DOW2;

    //Takes what's present in the GPS time and date strings and gets the decimal value
    //from those, and sets the RTC appropriately
    GPShours = ((GPSinfo.GPStime[0] - '0') * 10) + (GPSinfo.GPStime[1] - '0');
    GPSminutes = ((GPSinfo.GPStime[2] - '0') * 10) + (GPSinfo.GPStime[3] - '0');
    GPSseconds = ((GPSinfo.GPStime[4] - '0') * 10) + (GPSinfo.GPStime[5] - '0');
    GPSmonth = ((GPSinfo.GPSdate[2] - '0') * 10) + (GPSinfo.GPSdate[3] - '0');
    GPSday = ((GPSinfo.GPSdate[0] - '0') * 10) + (GPSinfo.GPSdate[1] - '0');
    GPSyear = ((GPSinfo.GPSdate[4] - '0') * 10) + (GPSinfo.GPSdate[5] - '0');
    SetTime.hours = GPShours;
    SetTime.minutes = GPSminutes;
    SetTime.seconds = GPSseconds;
    SetTime.month = GPSmonth;
    SetTime.dayOfmonth = GPSday;
    SetTime.year = GPSyear;

    //Calculating the day of the week based on the values obtained from the GPS
    DOW1 = ((GPSmonth + 9) % 12);
    DOW2 = (2000 + GPSyear - (DOW1 / 10));
    GPSdayofweek = ((365 * DOW2 + (DOW2 / 4) - (DOW2 / 100) + (DOW2 / 400)
            + ((DOW1 * 306 + 5) / 10) + GPSday + 2) % 7);

    SetTime.dayOfWeek = GPSdayofweek;

    //Actually sets the time and date and starts the RTC counting.
    MAP_RTC_C_holdClock();
    MAP_RTC_C_initCalendar(&SetTime, RTC_C_FORMAT_BINARY);
    MAP_RTC_C_startClock();
}

void CheckForUSB(void) //Looks for the USB being plugged into the micro or not.
//If the pin is low, there is no USB, and the MCU will go into LPM3 mode.
{
    if (GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN2) == GPIO_INPUT_PIN_LOW)
    {

        USBPresentFlag = 0; //Resets this flag which inhibits devices from turning on

        MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0); //Puts the enable pins for periph devices high so they're off
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);
        MAP_GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0);
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0);
        disableXbeeUART(); //Disables UART channels and associated pins, trying to stay the most power efficient.
        disableGPSUART();
        disablePCUART();
        USBPresentOns = 1;

        MAP_PCM_enableRudeMode();
        MAP_PCM_gotoLPM3(); //LPM3 wakes up from external interrupt and RTC, has RAM retention, LPM3.5/4.5 do not have
        //RAM retention, and LPM4 only can be woken from external interrupt, going to live in LPM3 mostly.
    }
}

void CheckForMagnet(void) //If the magnet is present, don't do anything and go to sleep.
{
    if (GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN3) == GPIO_INPUT_PIN_HIGH)
    {
        IOSetup(); //Initializes all of the pins in the most efficient way possible to keep battery life okay.
        MAP_PCM_enableRudeMode();
        MAP_PCM_gotoLPM4(); //this is for storing it on a shelf for an extended period of time. Uses the least power
        //for modes besides 4.5.
    }
}

void PowerCalculation(void) //Calculates the approximate battery percentage left based on counters for
//on times of each device.
{
    int XbeePowermA = 50; //average of time on being receiving and transmitting, receiving uses less current
    int GPSPowermA = 55; //power required as it's attempting to obtain a GPS fix
    double VHFPowermA = 3 * .00875; //RMS, peaks are approximately at 6
    int MCUPowermA = 0.075; //average power over the week with factoring in A.M. and the LPM's
    int BatteryCapacity = 2170; //in mAh, best case scenario
    int CritBattThreshold = BatteryCapacity * 0.15; //mAh left in battery before triggering to enter
    //longevity mode
    double XbeePowerConsumed = 0; //in mAh
    double GPSPowerConsumed = 0; //in mAh
    double VHFPowerConsumed = 0; //in mAh
    double TotalPowerConsumed = 0; //in mAh
    volatile double BatteryPercentage = 0;
    int NumberOfPointsObtained = FixMemoryLocator[1] * 128
            + FixMemoryLocator[0]; //Finds how many fixes are on the collar already.

    XbeePowerConsumed = XbeePowermA
            * (XbeeHourOnCount + ((double) XbeeMinOnCount / 60)
                    + ((double) XbeeSecOnCount / 3600)); //Calcs the mAh.
    GPSPowerConsumed = GPSPowermA
            * (GPSHourOnCount + ((double) GPSMinOnCount / 60)
                    + ((double) GPSSecOnCount / 3600));

    if (Config.VET > Config.VST)
    {
        VHFPowerConsumed = VHFPowermA
                * ((Config.VET - Config.VST)
                        * ((double) NumberOfPointsObtained / Config.GPS));
    }
    else if (Config.VET < Config.VST)
    {
        VHFPowerConsumed = VHFPowermA
                * (((24 - Config.VST) + Config.VET)
                        * ((double) NumberOfPointsObtained / Config.GPS));
    }
    else if (Config.VET == Config.VST)
    {
        VHFPowerConsumed = VHFPowermA
                * ((24) * ((double) NumberOfPointsObtained / Config.GPS));
    }

    //Calcs the mAh, uses the
    //number of fixes obtained on the GPS with the window time for the amount of days to compute
    //instead of the hour minute second counters like the others since it would require to be in
    //active mode to increment these counters and didn't seem worth it if it wasn't needed as the
    //VHF on window could be at a different time from the Xbee/GPS ones where it's the most
    //crucial that it isn't in Active Mode.

    TotalPowerConsumed = XbeePowerConsumed + GPSPowerConsumed
            + VHFPowerConsumed; //Sums up the mAh used by each device so it's not a
    //super long line of code.

    if (TotalPowerConsumed > (BatteryCapacity - CritBattThreshold)) //If more than the allowed
    //percentage of battery has been consumed already, kick the devie into longevity mode.
    {
        LongevityMode = 1;

        Config.WTM = 1;
        store_config_params();
    }

    BatteryPercentage = 100 - (TotalPowerConsumed / BatteryCapacity) * 100; //Calculating the battery
    //percentage to be sent to the user.

    if (BatteryPercentage > 99)
    {
        BatteryPercentage = 99;
    }

    if (BatteryPercentage < 10)
    {
        sprintf(BatteryString, "0%.0lf", BatteryPercentage);
    }
    else
    {
        sprintf(BatteryString, "%.0lf", BatteryPercentage);
    }

    BatteryString[2] = '%';
    BatteryString[3] = '\0';
}

void StateMachine(void)
{
    uint8_t IPBitIdx = 0;

    //Request Handling, priority: 1. Xbee 2. GPS 3. VHF
    if (XbeeReq == 1 && USBPresentFlag == 0)
    {
        XbeeEn = 1;
        GPSEn = 0;
        VHFEn = 0;
    }
    if ((GPSReq == 1 && XbeeReq == 0) && USBPresentFlag == 0)
    {
        XbeeEn = 0;
        GPSEn = 1;
        VHFEn = 0;
    }
    if ((VHFReq == 1 && XbeeReq == 0 && GPSReq == 0) && USBPresentFlag == 0)
    {
        XbeeEn = 0;
        GPSEn = 0;
        VHFEn = 1;
    }

    //Enable Pins
    if (XbeeEn == 1)
    {
        initClocks();
        MAP_WDT_A_startTimer();
        EnableSysTick();
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
        GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0);
        GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN7);
        GPSEn = 0;
        VHFEn = 0;

        //If in spew mode and longevity mode not on
        if (Config.WTM == 2 && LongevityMode == 0)
        {
            MAP_WDT_A_clearTimer();

            initXbeeUART();

            Delay1ms(16000);

            MAP_WDT_A_clearTimer();

            XbeeTransmissionIP[4] = 1;

            //Transmit all GPS data
            Xbee_puts("@GPSDATA^@");
            readout_memory_all();
            Xbee_puts("^");

            //Transmit battery estimate
            Xbee_puts("@BATTERY^@");
            Xbee_puts(BatteryString);
            Xbee_puts("^");

            Delay1ms(3000);

            XbeeTransmissionIP[4] = 0;

            XbeeTransmissionComplete = 1;
            XbeeReq = 0;
        }

        //Try to get a fix if requested
        if (CurrentLocationRequestInitiated == 1)
        {
            XbeeSecCount = 0;

            initXbeeUART();

            while (XbeeSecCount < 8)
                ;

            if (NoFixFlag == 0) //there was a fix obtained
            {
                XbeeTransmissionIP[5] = 1;

                strcpy(XbeePutsString, "@HTCF^@"); //Here's the current fix
                XbeePutsString[7] = '\0';
                Xbee_puts(XbeePutsString);
                Xbee_puts(CurrentFixSaveString);
                Xbee_puts("^");
                Xbee_puts(PDOPString);

                Delay1ms(500);

                XbeeTransmissionIP[5] = 0;
            }
            else
            {
                XbeeTransmissionIP[6] = 1;

                strcpy(XbeePutsString, "@DGAF^@"); //Didn't get a fix
                XbeePutsString[7] = '\0';
                Xbee_puts(XbeePutsString);
                readout_last_known_location(); //Pull the last known location from flash
                Xbee_puts("^");
                Xbee_puts(PDOPString);

                Delay1ms(500);

                XbeeTransmissionIP[6] = 0;
            }

            XbeeSecCount = 0;

            while (XbeeSecCount < 8)
                ;

            XbeeReq = 0;
            disableXbeeUART();
            CurrentLocationRequestInitiated = 0;
        }

        //If the current location is requested and in longevity mode or on demand request
        if (XbeeComms.CurLocReq == 1 && (NowGetFix == 1 || LongevityMode == 1))
        {
            XbeeTransmissionIP[7] = 1;

            strcpy(XbeePutsString, "@TTGF^");
            XbeePutsString[6] = '\0';
            Xbee_puts(XbeePutsString); //sending to LabView so researcher knows it's trying to get a GPS signal

            Delay1ms(500);

            XbeeTransmissionIP[7] = 0;

            Delay1ms(150); //this delay is magical and fixes the problems

            XbeeReq = 0;
            XbeeComms.CurLocReq = 0;
            GPSReq = 1;
            GPSAttemptOnTime = 0;
            FixAttemptFailed = 0;
            CurrentLocationRequestInitiated = 1;
        }
    }
    //Turn on the GPS, turn off everything else
    if (GPSEn == 1)
    {
        initClocks();
        MAP_WDT_A_startTimer();
        EnableSysTick();
        GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0);
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);
        GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN7);
        XbeeEn = 0;
        VHFEn = 0;
    }
    //Turn on VHF, turn off everything else
    if (VHFEn == 1)
    {
        if (MagnetRemovedFlag == 0)
        {
            DisableSysTick();
        }
        GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN7);
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);
        GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0);
        XbeeEn = 0;
        GPSEn = 0;
    }

    //Turn off Xbee when request shuts off
    if (XbeeReq == 0)
    {
        XbeeTransmissionIP[0] = 0;

        for (IPBitIdx = 1; IPBitIdx < 8; IPBitIdx++)
        {
            if (XbeeTransmissionIP[IPBitIdx] == 1)
            {
                XbeeTransmissionIP[0] = 1;
            }
        }

        if (XbeeTransmissionIP[0] == 0)
        {
            Delay1ms(2000);
            GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);
            XbeeEn = 0;
        }
    }
    //Turn off GPS
    if (GPSReq == 0)
    {
        GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0);
        GPSEn = 0;
    }
    //Turn off VHF
    if (VHFReq == 0)
    {
        GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN7);
        VHFEn = 0;
    }
}

void EnableSysTick(void)
{
    //SysTick configuration - trigger at 1500000 ticks at 3MHz=0.5s
    MAP_SysTick_enableModule();
    MAP_SysTick_setPeriod(2999000);
    MAP_SysTick_enableInterrupt();
}

void DisableSysTick(void)
{
    MAP_SysTick_disableModule();
    MAP_SysTick_disableInterrupt();
}

//Store the battery life counters to flash
void store_battery_counters(void)
{
    //Declare save array
    uint8_t BatSave[9];
    BatSave[0] = XbeeSecOnCount;
    BatSave[1] = XbeeMinOnCount;
    BatSave[2] = XbeeHourOnCount;
    BatSave[3] = GPSAttemptOnTime;
    BatSave[4] = GPSSecOnCount;
    BatSave[5] = GPSMinOnCount;
    BatSave[6] = GPSHourOnCount;
    BatSave[7] = LongevityMode;
    BatSave[8] = '\0';

    //Save the battery counters to flash
    FlashCtl_unprotectSector(FLASH_MAIN_MEMORY_SPACE_BANK1, FLASH_SECTOR29); //unprotect sector
    FlashCtl_eraseSector(0x0003D000); //erase the sector
    FlashCtl_enableWordProgramming(FLASH_IMMEDIATE_WRITE_MODE); // Allow for immediate writing
    FlashCtl_programMemory(BatSave, (void*) 0x0003D000, 8); //write the data
    FlashCtl_protectSector(FLASH_MAIN_MEMORY_SPACE_BANK1, FLASH_SECTOR29); //protect sector

    vhfoneshot = 0;
}

//Readout the battery life counters into the proper variables
void readout_battery_counters(void)
{
    uint8_t ClearThemAll = 0;

    uint8_t FlashCheck[8]; //Store what's in flash here

    //See if flash has been initialized
    FlashCheck[0] = *(uint8_t*) (0x0003D000);
    FlashCheck[1] = *(uint8_t*) (0x0003D001);
    FlashCheck[2] = *(uint8_t*) (0x0003D002);
    FlashCheck[3] = *(uint8_t*) (0x0003D003);
    FlashCheck[4] = *(uint8_t*) (0x0003D004);
    FlashCheck[5] = *(uint8_t*) (0x0003D005);
    FlashCheck[6] = *(uint8_t*) (0x0003D006);
    FlashCheck[7] = *(uint8_t*) (0x0003D007);

    //Fill structure from flash if initialized
    if (FlashCheck[0] != 255)
    {
        XbeeSecOnCount = *(uint8_t*) (0x0003D000);
        ClearThemAll++;
    }
    if (FlashCheck[1] != 255)
    {
        XbeeMinOnCount = *(uint8_t*) (0x0003D001);
        ClearThemAll++;
    }
    if (FlashCheck[2] != 255)
    {
        XbeeHourOnCount = *(uint8_t*) (0x0003D002);
        ClearThemAll++;
    }
    if (FlashCheck[3] != 255)
    {
        GPSAttemptOnTime = *(uint8_t*) (0x0003D003);
        ClearThemAll++;
    }
    if (FlashCheck[4] != 255)
    {
        GPSSecOnCount = *(uint8_t*) (0x0003D004);
        ClearThemAll++;
    }
    if (FlashCheck[5] != 255)
    {
        GPSMinOnCount = *(uint8_t*) (0x0003D005);
        ClearThemAll++;
    }
    if (FlashCheck[6] != 255)
    {
        GPSHourOnCount = *(uint8_t*) (0x0003D006);
        ClearThemAll++;
    }
    if (FlashCheck[7] != 255)
    {
        LongevityMode = *(uint8_t*) (0x0003D007);
        ClearThemAll++;
    }

    //Clear out the values if not initialized and store
    if (ClearThemAll < 8)
    {
        XbeeSecOnCount = 0;
        XbeeMinOnCount = 0;
        XbeeHourOnCount = 0;
        GPSAttemptOnTime = 0;
        GPSSecOnCount = 0;
        GPSMinOnCount = 0;
        GPSHourOnCount = 0;
        LongevityMode = 0;

        store_battery_counters();
    }

}

//Store the configuration parameters to flash
int store_config_params(void)
{
    int result = 0;

    //Declare save array
    uint8_t ConfigSave[10] = { 0 };
    ConfigSave[0] = Config.GPS;
    ConfigSave[1] = Config.WTM;
    ConfigSave[2] = Config.WTD;
    ConfigSave[3] = Config.WCT;
    ConfigSave[4] = Config.WCW;
    ConfigSave[5] = Config.VST;
    ConfigSave[6] = Config.VET;
    ConfigSave[7] = Config.DOP;
    ConfigSave[8] = Config.GTO;
    ConfigSave[9] = '\0';

    //Save the config params to flash
    FlashCtl_unprotectSector(FLASH_MAIN_MEMORY_SPACE_BANK1, FLASH_SECTOR28); //unprotect sector
    FlashCtl_eraseSector(0x0003C000); //erase the sector
    FlashCtl_enableWordProgramming(FLASH_IMMEDIATE_WRITE_MODE); // Allow for immediate writing
    result = FlashCtl_programMemory(ConfigSave, (void*) 0x0003C000, 10); //write the data
    FlashCtl_protectSector(FLASH_MAIN_MEMORY_SPACE_BANK1, FLASH_SECTOR28); //protect sector

    return result;
}

//Readout the configuration parameters from flash
void readout_config_params(void)
{
    uint8_t FlashCheck[9]; //Store what's in flash here

    //See if flash has been initialized
    FlashCheck[0] = *(uint8_t*) (0x0003C000);
    FlashCheck[1] = *(uint8_t*) (0x0003C001);
    FlashCheck[2] = *(uint8_t*) (0x0003C002);
    FlashCheck[3] = *(uint8_t*) (0x0003C003);
    FlashCheck[4] = *(uint8_t*) (0x0003C004);
    FlashCheck[5] = *(uint8_t*) (0x0003C005);
    FlashCheck[6] = *(uint8_t*) (0x0003C006);
    FlashCheck[7] = *(uint8_t*) (0x0003C007);
    FlashCheck[8] = *(uint8_t*) (0x0003C008);

    //Fill structure from flash if initialized
    if (FlashCheck[0] != 255)
    {
        Config.GPS = *(uint8_t*) (0x0003C000);
    }
    if (FlashCheck[1] != 255)
    {
        Config.WTM = *(uint8_t*) (0x0003C001);
    }
    if (FlashCheck[2] != 255)
    {
        Config.WTD = *(uint8_t*) (0x0003C002);
    }
    if (FlashCheck[3] != 255)
    {
        Config.WCT = *(uint8_t*) (0x0003C003);
    }
    if (FlashCheck[4] != 255)
    {
        Config.WCW = *(uint8_t*) (0x0003C004);
    }
    if (FlashCheck[5] != 255)
    {
        Config.VST = *(uint8_t*) (0x0003C005);
    }
    if (FlashCheck[6] != 255)
    {
        Config.VET = *(uint8_t*) (0x0003C006);
    }
    if (FlashCheck[7] != 255)
    {
        Config.DOP = *(uint8_t*) (0x0003C007);
    }
    if (FlashCheck[8] != 255)
    {
        Config.GTO = *(uint8_t*) (0x0003C008);
    }
}

//Prepares the GPS structure into a 32 byte save string
//Performs error checking and saves fix to flash
void GPS_handler(void)
{
    //Declare save arrays
    char SaveTime[5];
    char SaveDate[7];
    char SaveLat[10];
    char SaveLatDir[2];
    char SaveLong[11];
    char SaveLongDir[2];
    int i = 0;
    vhfoneshot = 1;

    //Prepare time
    strncpy(SaveTime, GPSinfo.GPStime, 4);
    SaveTime[4] = '\0';

    //Prepare date
    strcpy(SaveDate, GPSinfo.GPSdate);
    SaveDate[6] = '\0';

    //Prepare latitude
    strncpy(SaveLat, GPSinfo.GPSlat, 9);
    SaveLat[9] = '\0';
    strncpy(SaveLatDir, &GPSinfo.GPSlat[10], 1);
    SaveLatDir[1] = '\0';

    //Prepare longitude
    strncpy(SaveLong, GPSinfo.GPSlong, 10);
    SaveLong[10] = '\0';
    strncpy(SaveLongDir, &GPSinfo.GPSlong[11], 1);
    SaveLongDir[1] = '\0';

    //Error Checking and saving
    if (SaveLat[4] == '.' && (SaveLatDir[0] == 'N' || SaveLatDir[0] == 'S')
            && SaveLong[5] == '.'
            && (SaveLongDir[0] == 'W' || SaveLongDir[0] == 'E')
            && (GPSinfo.GPSPDOP <= Config.DOP) && (GPSinfo.GPSfixquality == 1))
    {
        //Create the string to save to flash
        snprintf(CurrentFixSaveString, 32, "%s%s%s%s%s%s", SaveTime, SaveDate,
                 SaveLat, SaveLatDir, SaveLong, SaveLongDir);
        CurrentFixSaveString[31] = ',';
        CurrentFixSaveString[32] = '\0';
        setTimeDate(); //sets the RTC time to what the GPS had in the time strings
        NoFixFlag = 0;
    }
    else
    {
        sprintf(CurrentFixSaveString, "%02d%02d%02d%02d%02dNOFIX",
                SystemTime.hours, SystemTime.minutes, SystemTime.dayOfmonth,
                SystemTime.month, SystemTime.year);
        for (i = 15; i < 31; i++)
        {
            CurrentFixSaveString[i] = '*';
        }
        CurrentFixSaveString[31] = ',';
        CurrentFixSaveString[32] = '\0';
        NoFixFlag = 1;
    }

    //Save
    save_current_fix();
    //Clear request and time counter
    GPSReq = 0;
    GPSAttemptOnTime = 0;
    FixAttemptFailed = 0;

    if (LongevityMode == 1 || NowGetFix == 1) //If the user had requested the current location
    //this turns the Xbee back on with the current fix on the collar or the last known location
    //that was stored in flash.
    {
        NowGetFix = 0;
        XbeeReq = 1;
        XbeeMinCount = 0;
        LongevityXbeeBypass = 1;
    }

    GPSinfo.GPSfixquality = 0;
    GPSinfo.GPSlat[11] = '0';
    GPSinfo.GPSlong[10] = '0';
    GPSinfo.GPSPDOP = 99;

}

//Initialize the memory location tracking
void memory_locator_init(void)
{
    uint8_t FlashCheck[2]; //Store what's in flash here

    //See if flash has been initialized
    FlashCheck[0] = *(uint8_t*) (0x0003F000);
    FlashCheck[1] = *(uint8_t*) (0x0003F001);

    //If flash hasn't been initialized, set to 0
    if (FlashCheck[0] == 255 && FlashCheck[1] == 255)
    {
        FixMemoryLocator[0] = 0;
        FixMemoryLocator[1] = 0;
        FlashCtl_unprotectSector(FLASH_MAIN_MEMORY_SPACE_BANK1, FLASH_SECTOR31); //unprotect sector
        FlashCtl_eraseSector(0x0003F000); //erase the sector
        FlashCtl_enableWordProgramming(FLASH_IMMEDIATE_WRITE_MODE); // Allow for immediate writing
        FlashCtl_programMemory(FixMemoryLocator, (void*) 0x0003F000, 2); //write the data
        FlashCtl_protectSector(FLASH_MAIN_MEMORY_SPACE_BANK1, FLASH_SECTOR31); //protect sector
    }
    //If initialized already, readout values
    else
    {
        FixMemoryLocator[0] = *(uint8_t*) (0x0003F000);
        FixMemoryLocator[1] = *(uint8_t*) (0x0003F001);
    }
}

//Initialize the placeholder for the next wireless transmission
void transmission_placeholder_init(void)
{
    uint8_t FlashCheck[2]; //Store what's in flash here

    //See if flash has been initialized
    FlashCheck[0] = *(uint8_t*) (0x0003E000);
    FlashCheck[1] = *(uint8_t*) (0x0003E001);

    //If flash hasn't been initialized, set to 0
    if (FlashCheck[0] == 255 && FlashCheck[1] == 255)
    {
        MemPlaceholder[0] = 0;
        MemPlaceholder[1] = 0;
        FlashCtl_unprotectSector(FLASH_MAIN_MEMORY_SPACE_BANK1, FLASH_SECTOR30); //unprotect sector
        FlashCtl_eraseSector(0x0003E000); //erase the sector
        FlashCtl_enableWordProgramming(FLASH_IMMEDIATE_WRITE_MODE); // Allow for immediate writing
        FlashCtl_programMemory(MemPlaceholder, (void*) 0x0003E000, 2); //write the data
        FlashCtl_protectSector(FLASH_MAIN_MEMORY_SPACE_BANK1, FLASH_SECTOR30); //protect sector
    }
    //If initialized already, readout values
    else
    {
        MemPlaceholder[0] = *(uint8_t*) (0x0003E000);
        MemPlaceholder[1] = *(uint8_t*) (0x0003E001);
    }
}

void initClocks(void)
{
    //Halting WDT
    MAP_WDT_A_holdTimer();

    CS_setExternalClockSourceFrequency(32768, 48000000);
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);
    CS_startHFXT(false);
    //Starting HFXT
    MAP_CS_initClockSignal(CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_16);
    //Starting LFXT
    MAP_CS_startLFXT(CS_LFXT_DRIVE3);
    MAP_CS_initClockSignal(CS_BCLK, CS_LFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);

    // Configuring WDT to timeout after 128M iterations of SMCLK, at 3MHz,
    // this will roughly equal 42 seconds
    MAP_SysCtl_setWDTTimeoutResetType(SYSCTL_SOFT_RESET);
    MAP_WDT_A_initWatchdogTimer(WDT_A_CLOCKSOURCE_SMCLK,
    WDT_A_CLOCKITERATIONS_128M);
}

int main(void)
{
    int GPSDateCheck = 0; //This protects us from satellites giving us incorrect dates from the past.

    //Halting WDT
    MAP_WDT_A_holdTimer();

    //Initializing IO on the MSP, leaving nothing floating
    IOSetup();

//    initPCUART(); //Turning on the PC UART so it can communicate.

    initClocks(); //Initializing all of the timing devices on the MSP

    memory_locator_init(); //Initialize the memory locator that tells us where to store GPS fixes in flash
    transmission_placeholder_init(); //Initialize the transmission placeholder that tells us where to start reading out new fixes from if the
    //researcher only wants new fixes

    GPSinfo.GPSPDOP = 99;    //reset PDOP to number above good threshold

    //Default Parameters
    Config.GPS = 24; //12 fixes - 12
    Config.WTM = 1; //Unconfirmed - 2
    Config.WTD = 4; //Thursday - 4
    Config.WCT = 23; //8am - 8
    Config.WCW = 1; //1 hour - 1
    Config.VST = 23; //8am - 8
    Config.VET = 23; //Noon - 12
    Config.DOP = 8;
    Config.GTO = 75;

    readout_config_params(); //Readout configurable parameters from flash
    readout_battery_counters(); //Readout battery counter variables from flash

    HourInt = 1;

    MAP_PSS_disableHighSide();

    // Enable all SRAM bank retentions prior to going to LPM3
    SYSCTL->SRAM_BANKRET |= SYSCTL_SRAM_BANKRET_BNK7_RET;

    RTC_setup(); //setup RTC

    setTimeDate(); //setting time and date on the RTC, default time is as specified above.

    int DoPowerCalc = 1; //Flag that tells the code when to do a power calculations

    //this time for nothing on, maybe vhf though
    strcpy(GPSinfo.GPStime, "225955"); //6am
    strcpy(GPSinfo.GPSdate, "250517"); //5-25-17

    setTimeDate();

    /* Enabling interrupts and starting the watchdog timer */
    MAP_WDT_A_startTimer();
    MAP_Interrupt_enableMaster();

    while (1)
    {
        MAP_WDT_A_clearTimer(); //Kick dog

        //VHF beacon on magnet removed condition, this lets researcher see the device actually turned on when he removed
        //the magnet
        if (MagnetRemovedFlag == 1)
        {
            if (VHFStartUpCount >= 60)
            {
                XbeeReq = 0;
                GPSReq = 0;
                VHFReq = 0;
                StateMachine();
                MagnetRemovedFlag = 0;
                DisableSysTick();
            }
            if (VHFStartUpCount < 60)
            {
                XbeeReq = 0;
                GPSReq = 0;
                VHFReq = 1;
                StateMachine();
            }
        }
        else
        {

            //If the USB is present, turn on the flag and shut off devices
            if (MAP_GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN2) == 1)
            {
                USBPresentFlag = 1;
                GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);
                GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0);
                GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN7);
                XbeeReq = 0;
                GPSReq = 0;
                VHFReq = 0;
            }
            else
            {
                USBPresentFlag = 0;
                USBPresentOns = 1;
            }

            if (USBPresentFlag == 1 && USBPresentOns == 1)
            {
                initPCUART();
                USBPresentOns = 0;
            }

            //Do a power calculation
            if (DoPowerCalc == 1)
            {
                PowerCalculation(); //Checks to see the status in terms of the battery life.
                DoPowerCalc = 0;
            }

            StateMachine(); //Handles the request flags for the various devices to see
            //what should be on given the priority levels established.  Turn on requests
            //are from the RTC scheduler
            MAP_WDT_A_clearTimer(); //Kick dog

            //If Xbee is enabled, set proper variables, initialize Xbee UART
            if (XbeeEn == 1)
            {
                if (xbeeoneshot == 1)
                {
                    //initPCUART(); //delete once done with debug
                    initXbeeUART();
                    xbeenegons = 1;
                    xbeeoneshot = 0;
                }

                if (XbeeStringExtractGo == 1)
                {
                    XbeeCommExtract(); //Extract info from Xbee serial communication
                }

            }

            //Turn off Xbee UART when Xbee turns off
            if (XbeeEn == 0)
            {
                if (xbeenegons == 1)
                {
                    //disablePCUART(); //delete once done with debug
                    disableXbeeUART();
                    xbeenegons = 0;
                }
            }

            //Reenable this once we go into full functionality testing of this do-dad
            if (PCStringExtractGo == 1)
            {
                ExtractParameters(); //Takes what was received from LabView and puts it into the elements of the structure
            }

            //If GPS is enabled, set proper variables, initialize GPS UART
            if (GPSEn == 1)
            {
                if (gpsoneshot == 1)
                {
                    //initPCUART(); //delete once done with debug
                    initGPSUART();
                    gpsoneshot = 0;
                    gpsnegons = 1;
                    GPSAttemptOnTime = 0;
                    EnableSysTick();
                }
                if (GPSStringClassifyGo == 1)
                {
                    ClassifyString(); //Takes what was received from the GPS and puts it into the elements of the structure
                }

                GPSDateCheck = (GPSinfo.GPSdate[4] - '0') * 10
                        + (GPSinfo.GPSdate[5] - '0'); //Compute satellite date to protect from false dates

                if (((GPSinfo.GPSfixquality == 1)
                        && (GPSinfo.GPSPDOP <= Config.DOP)
                        && (GPSDateCheck > 16)) || (FixAttemptFailed == 1))
                {
                    GPS_handler(); //writes to flash, fix or no fix, and only if the date is not from the past
                }
            }

            //Disable GPS UART when GPS turns off
            if (GPSEn == 0)
            {
                if (gpsnegons == 1)
                {
                    //disablePCUART(); //delete once done with debug
                    disableGPSUART();
                    gpsnegons = 0;
                }
            }

            MAP_WDT_A_clearTimer(); //Kick dog

            //Do a power calc and store the counters
            if ((GPSEn == 0) && (XbeeEn == 0)
                    && (CurrentLocationRequestInitiated == 0))
            {
                DoPowerCalc = 1;

                if (vhfoneshot == 1)
                {
                    store_battery_counters();
                }
                CheckForUSB(); //Call to go to LPM3 if USB isn't attached
                MAP_WDT_A_clearTimer(); //Kick dog
            }

        }

        StateMachine();

        MAP_WDT_A_clearTimer(); //Kick dog

        CheckForMagnet(); //open this up once 4.3 is connected to the debounce circuit
    }
}

// RTC ISR
void RTC_C_IRQHandler(void)
{
    uint32_t status;
    int n = 0;
    int HourArray[24] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
                          16, 17, 18, 19, 20, 21, 22, 23 }; //All of the hours of the day in an array

    status = MAP_RTC_C_getEnabledInterruptStatus();
    MAP_RTC_C_clearInterruptFlag(status);
    SystemTime = MAP_RTC_C_getCalendarTime(); //Update Time

    if (status & RTC_C_TIME_EVENT_INTERRUPT)
    {
        // Interrupts every hour or minute based on state of program

        //Every minute interrupt handling
        if (MinInt == 1)
        {
            if (LongevityXbeeBypass == 0)
            {
                XbeeMinCount++; //Increment Xbee minute count
            }

            //If the GPS programmed to get a fix at the same time as the Xbee is scheduled to turn on, keep the Xbee off for 5 minutes to let the GPS run
            if (GPSReq == 1 && SystemTime.minutes < 5)
            {
                XbeeReq = 0;
                XbeeMinCount = 0;
            }
            //In the Xbee window, Xbee on time 1 minute, Xbee off time 4 minutes
            if (XbeeReq == 1 && XbeeMinCount == 1)
            {
                XbeeReq = 0;
                WaitingForTAFT = 0;
                XbeeMinCount = 0;
            }
            if (XbeeReq == 0 && XbeeMinCount == 4
                    && XbeeTransmissionComplete == 0)
            {
                XbeeReq = 1;
                xbeeoneshot = 1;
                XbeeMinCount = 0;
            }
            //This lets us do things on the hour rollover in the minute interrupt mode
            if (SystemTime.minutes == 0)
            {
                if (XbeeTransmissionComplete == 0)
                {
                    XbeeReq = 1;
                    xbeeoneshot = 1;
                    XbeeMinCount = 0;
                }

                XbeeConnectionTimeWindowCount++; //Increment the counter that keeps track of how many hours the Xbee has been on

                if (LongevityMode == 0)
                {
                    //Check if the GPS should be on
                    for (n = 0; n < 24; n++)
                    {
                        if (SystemTime.hours == HourArray[n])
                        {
                            GPSReq = 1;
                            gpsoneshot = 1;
                            GPSAttemptOnTime = 0;
                            FixAttemptFailed = 0;
                        }
                        n = n + ((24 / Config.GPS) - 1);
                    }
                }
            }
            //Outside the window, turn Xbee off and switch to hour interrupt mode, reset flag which makes sure xbee doesn't
            //turn on again after it's been confirmed data was sent
            if (XbeeConnectionTimeWindowCount == Config.WCW)
            {
                WaitingForTAFT = 0;
                XbeeReq = 0;
                XbeeTransmissionComplete = 0;
                XbeeMinCount = 0;
                XbeeConnectionTimeWindowCount = 0;
                HourInt = 1;
                MinInt = 0;
                MAP_RTC_C_setCalendarEvent(RTC_C_CALENDAREVENT_HOURCHANGE);
            }
        }

        //Every hour interrupt handling
        if (HourInt == 1)
        {
            if (LongevityMode == 0)
            {
                //Check if the GPS should be on
                for (n = 0; n < 24; n++)
                {
                    if (SystemTime.hours == HourArray[n])
                    {
                        GPSReq = 1;
                        gpsoneshot = 1;
                        GPSAttemptOnTime = 0;
                        FixAttemptFailed = 0;
                    }
                    n = n + ((24 / Config.GPS) - 1);
                }
            }
            //Check if the Xbee should be on
            if ((SystemTime.dayOfWeek == Config.WTD)
                    && (SystemTime.hours == Config.WCT)
                    && (XbeeTransmissionComplete == 0))
            {
                XbeeReq = 1;
                xbeeoneshot = 1;
                MAP_RTC_C_setCalendarEvent(RTC_C_CALENDAREVENT_MINUTECHANGE);
                MinInt = 1;
                HourInt = 0;
            }
            //Check if the VHF transmitter should be on
            if ((SystemTime.hours == Config.VST) || (Config.VST == Config.VET))
            {
                VHFReq = 1;

            }
            if ((SystemTime.hours == Config.VET) && (Config.VST != Config.VET))
            {
                VHFReq = 0;
                vhfoneshot = 1;
            }
        }
    }
}

//This is used for a second interrupt to count how long the Xbee/GPS has been on.
//The seconds, minutes, and hours of each device are incremented and the values in
//each is stored into flash as needed.  These parameters are also pulled on startup.
void SysTick_IRQHandler(void)
{
    //If the magnet has been removed, increment the counter for the VHF beacon
    if (MagnetRemovedFlag == 1)
    {
        VHFStartUpCount++;
    }

    //Counter for the total Xbee on time
    if (XbeeEn == 1)
    {
        XbeeSecOnCount++;
        XbeeSecCount++;

        if (XbeeSecOnCount >= 60)
        {
            XbeeMinOnCount++;
            XbeeSecOnCount = 0;
        }

        if (XbeeMinOnCount >= 60)
        {
            XbeeHourOnCount++;
            XbeeMinOnCount = 0;
        }
    }

    //Counter for the total GPS on time
    if (GPSEn == 1)
    {
        GPSAttemptOnTime++;
        GPSSecOnCount++;

        if (GPSSecOnCount >= 60)
        {
            GPSMinOnCount++;
            GPSSecOnCount = 0;
        }

        if (GPSMinOnCount >= 60)
        {
            GPSHourOnCount++;
            GPSMinOnCount = 0;
        }

        if (GPSAttemptOnTime >= Config.GTO) //set to a minute and 15 seconds for an attempt maximum
        //easily changed
        {
            FixAttemptFailed = 1; //used to trigger writing "NO FIX" to flash
        }
    }
}

//PORT 4 ISR, this is for determining if the magnet is present or not on pin 3
//and if the USB is present or not on pin 2.
void PORT4_IRQHandler(void)
{
    //Get the status of Port 4
    uint32_t status;
    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P4);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4, status);

    initClocks(); //Initialize clocks because we are waking up from sleep
    MAP_WDT_A_startTimer(); //Start watchdog because we are waking up from sleep

    //If the magnet is removed
    if (status & GPIO_PIN3)
    {
        MAP_RTC_C_startClock(); //Start the RTC

        //Check for the UBS present
        if (MAP_GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN2) == 1)
        {
            initPCUART(); //Initialize the PC UART
            USBPresentFlag = 1; //USB present
            GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0);
            GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN7);
            XbeeReq = 0;
            GPSReq = 0;
            VHFReq = 0;
        }
        else
        {
            MagnetRemovedFlag = 1; //USB is not present, magnet is removed.
            VHFStartUpCount = 0; //Clear the timer for the VHF beacon that occurs on system start up.
            EnableSysTick();
        }
    }

    if (status & GPIO_PIN2)
    {
        initPCUART();
        MAP_RTC_C_startClock();
        USBPresentFlag = 1;
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);
        GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0);
        GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN7);
        XbeeReq = 0;
        GPSReq = 0;
        VHFReq = 0;
    }
}

// EUSCI A0 UART ISR - Captures what's input from the serial emulator and puts it in a buffer for the MCU to deal with.

void EUSCIA0_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);

    MAP_UART_clearInterruptFlag(EUSCI_A0_BASE, status);

    RX0Data = 0;

    if (status & EUSCI_A_UART_RECEIVE_INTERRUPT)
    {
        RX0Data = MAP_UART_receiveData(EUSCI_A0_BASE);
        //MAP_UART_transmitData(EUSCI_A1_BASE, RX0Data);

        int i;

        switch (RX0Data)
        {
        case '$': //searches for the start of the string, every NMEA string starts with a $
            /*for (i = 0; i < sizeof PCdataString; i++) //clears the old string that was stored there
             {
             PCdataString[i] = "";
             }*/
            PCindex = 0; //resets the index which increments to store each character received into
            //the buffer
            PCflag = 1; //resets the flag which is to start storing the characters into dataString

            break;

        case '#': //looks for a new line character which is the end of the NMEA string
            PCflag = 0;
            strncpy(PCString, PCdataString, PCindex); //copies what was in dataString into GPSString
            //so it doesn't get reset and GPSString can be used and parsed
            PCString[PCindex] = '\0'; //puts a NULL at the end of the string again because strncpy doesn't

            PCStringExtractGo = 1;
            break;

        default:
            if (PCflag) //if the data in the buffer is to be stored in dataString, it'll do it here
            {
                PCdataString[PCindex] = RX0Data; //puts what was in the buffer into an index in dataString
                PCindex++; //increments the index so the next character received in the buffer can be stored
                //into dataString
            }
            break;
        }

        if (PCindex == 99)
            PCindex = 0; //resets the index so it doesn't overflow
    }
}

/* EUSCI A1 UART ISR - Echoes data back to A0 (terminal emulator), allows you to see in terminal emulator what's
 * sent from another Xbee to this one.*/

void EUSCIA1_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A1_BASE);

    MAP_UART_clearInterruptFlag(EUSCI_A1_BASE, status);

    RX1Data = 0;

    if (status & EUSCI_A_UART_RECEIVE_INTERRUPT)
    {
        RX1Data = MAP_UART_receiveData(EUSCI_A1_BASE);
//        MAP_UART_transmitData(EUSCI_A0_BASE, RX1Data);

        int i;

        switch (RX1Data)
        {
        case '$': //searches for the start of the string, each broadcast begins with a '$'
            Xbeeindex = 0; //resets the index which increments to store each character received into
            //the buffer
            Xbeeflag = 1; //resets the flag which is to start storing the characters into dataString
            break;

        case '#': //looks for a new line character which is the end of the NMEA string
            Xbeeflag = 0;

            if (XbeeStringExtractGo == 0 && Config.WTM == 1)
            {
                strncpy(XbeeUARTString, XbeedataString, Xbeeindex); //copies what was in dataString into GPSString
                //so it doesn't get reset and GPSString can be used and parsed
                XbeeUARTString[Xbeeindex] = '\0'; //puts a NULL at the end of the string again because strncpy doesn't
                XbeeStringExtractGo = 1;
            }
            break;

        default:
            if (Xbeeflag) //if the data in the buffer is to be stored in dataString, it'll do it here
            {
                XbeedataString[Xbeeindex] = RX1Data; //puts what was in the buffer into an index in dataString
                Xbeeindex++; //increments the index so the next character received in the buffer can be stored
                //into dataString
            }
            break;
        }

        if (Xbeeindex == 19)
            Xbeeindex = 0; //resets the index so it doesn't overflow
    }
}

/* EUSCI A2 UART ISR - Echoes data back to PC host (from GPS to monitor on serial monitor) */
void EUSCIA2_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A2_BASE);

    MAP_UART_clearInterruptFlag(EUSCI_A2_BASE, status);

    RXData = 0;

    if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        RXData = MAP_UART_receiveData(EUSCI_A2_BASE);
//        MAP_UART_transmitData(EUSCI_A0_BASE, RXData);
        int i;
        switch (RXData)
        {
        case '$': //searches for the start of the string, every NMEA string starts with a $
            for (i = 0; i < sizeof dataString; i++) //clears the old string that was stored there
            {
                dataString[i] = "";
            }
            index = 0; //resets the index which increments to store each character received into
            //the buffer
            flag = 1; //resets the flag which is to start storing the characters into dataString

            break;

        case '\n': //looks for a new line character which is the end of the NMEA string
            flag = 0;
            if (GPSStringClassifyGo == 0)
            {
                strncpy(GPSString, dataString, index); //copies what was in dataString into GPSString
                //so it doesn't get reset and GPSString can be used and parsed
                GPSString[index] = '\0'; //puts a NULL at the end of the string again because strncpy doesn't
                GPSStringClassifyGo = 1;
            }
            break;

        default:
            if (flag) //if the data in the buffer is to be stored in dataString, it'll do it here
            {
                dataString[index] = RXData; //puts what was in the buffer into an index in dataString
                index++; //increments the index so the next character received in the buffer can be stored
                //into dataString
            }
            break;
        }

        if (index == 299)
            index = 0; //resets the index so it doesn't overflow
    }

}