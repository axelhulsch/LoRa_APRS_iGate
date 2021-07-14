#include <logger.h>

#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#include "Task.h"
#include "TaskGPS.h"
#include "project_configuration.h"

static const int RXPin = 34, TXPin = 35;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

/* 
  From http://aprs.gids.nl/nmea/:
   
  $GPGSV
  
  GPS Satellites in view
  
  eg. $GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74
      $GPGSV,3,2,11,14,25,170,00,16,57,208,39,18,67,296,40,19,40,246,00*74
      $GPGSV,3,3,11,22,42,067,42,24,14,311,43,27,05,244,00,,,,*4D

  1    = Total number of messages of this type in this cycle
  2    = Message number
  3    = Total number of SVs in view
  4    = SV PRN number
  5    = Elevation in degrees, 90 maximum
  6    = Azimuth, degrees from true north, 000 to 359
  7    = SNR, 00-99 dB (null when not tracking)
  8-11 = Information about second SV, same as field 4-7
  12-15= Information about third SV, same as field 4-7
  16-19= Information about fourth SV, same as field 4-7
*/

static const int MAX_SATELLITES = 40;

TinyGPSCustom totalGPGSVMessages(gps, "GPGSV", 1); // $GPGSV sentence, first element
TinyGPSCustom messageNumber(gps, "GPGSV", 2);      // $GPGSV sentence, second element
TinyGPSCustom satsInView(gps, "GPGSV", 3);         // $GPGSV sentence, third element
TinyGPSCustom satNumber[4]; // to be initialized later
TinyGPSCustom elevation[4];
TinyGPSCustom azimuth[4];
TinyGPSCustom snr[4];

struct
{
  bool active;
  int elevation;
  int azimuth;
  int snr;
} sats[MAX_SATELLITES];


GPSTask::GPSTask() : Task(TASK_GPS, TaskGps), _beginCalled(false) {
  isValid=false;
}

GPSTask::~GPSTask() {
}

bool GPSTask::setup(System &system) {
  lat=system.getUserConfig()->beacon.positionLatitude;
  lng=system.getUserConfig()->beacon.positionLongitude;;

  ss.begin(GPSBaud);

  logPrintI("GPS Setup");
  
  // Initialize all the uninitialized TinyGPSCustom objects
  for (int i=0; i<4; ++i)
  {
    satNumber[i].begin(gps, "GPGSV", 4 + 4 * i); // offsets 4, 8, 12, 16
    elevation[i].begin(gps, "GPGSV", 5 + 4 * i); // offsets 5, 9, 13, 17
    azimuth[i].begin(  gps, "GPGSV", 6 + 4 * i); // offsets 6, 10, 14, 18
    snr[i].begin(      gps, "GPGSV", 7 + 4 * i); // offsets 7, 11, 15, 19
  }
  return true;
}

bool GPSTask::loop(System &system) {
  // Dispatch incoming characters
  if (ss.available() > 0)
  {
    gps.encode(ss.read());
    if (totalGPGSVMessages.isUpdated())
    {
      if (gps.location.isValid())
      {
        lat=gps.location.lat();
        lng=gps.location.lng();
        isValid=true;
        String sloc=String(lat)+","+String(lng);
        logPrintlnI(sloc);
        _stateInfo=sloc;
       
       }

      if (gps.date.isValid()) {
        logPrintI(String(gps.date.month()));
        logPrintI(F("/"));
        logPrintI(String(gps.date.day()));
        logPrintI(F("/"));
        logPrintlnI(String(gps.date.year()));
      }

      for (int i=0; i<4; ++i)
      {
        int no = atoi(satNumber[i].value());
//        logPrintI(F("SatNumber is ")); logPrintlnI(String(no));
        if (no >= 1 && no <= MAX_SATELLITES)
        {
          sats[no-1].elevation = atoi(elevation[i].value());
          sats[no-1].azimuth = atoi(azimuth[i].value());
          sats[no-1].snr = atoi(snr[i].value());
          sats[no-1].active = true;
        }
      }
      
      int totalMessages = atoi(totalGPGSVMessages.value());
      int currentMessage = atoi(messageNumber.value());
      if (totalMessages == currentMessage)
      {
        for (int i=0; i<MAX_SATELLITES; ++i)
          sats[i].active = false;
      }
    }
  }
  return true;
}
