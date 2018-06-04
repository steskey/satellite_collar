#ifndef GPS_H_
#define GPS_C_H_

typedef struct _GPSinfostructure
{
    char GPStime[11];
    char GPSlong[13];
    char GPSlat[12];
    uint8_t GPSfixquality;
    uint8_t GPSnumsat;
    uint8_t GPSSNR;
    char GPSdate[7];
    uint8_t GPSPDOP;
} GPSinfostructure;

#endif /* GPS_H */
