#include <logger.h>

#include <TimeLib.h>

#include "Task.h"
#include "TaskRouter.h"
#include "project_configuration.h"
#include "TaskGPS.h"

#include <Wire.h>                                                       // required by BME280 library
#include <BME280_t.h> 

BME280<> BMESensor;                                                      // import BME280 template library
int TeleNr=0;
bool sendPos=false;
bool sendParamName=true;
bool sendParamUnit=true;
bool sendParamEqns=true;

String create_lat_aprs(double lat);
String create_long_aprs(double lng);

RouterTask::RouterTask(TaskQueue<std::shared_ptr<APRSMessage>> &fromModem, TaskQueue<std::shared_ptr<APRSMessage>> &toModem, TaskQueue<std::shared_ptr<APRSMessage>> &toAprsIs,GPSTask &GPS) : Task(TASK_ROUTER, TaskRouter), _fromModem(fromModem), _toModem(toModem), _toAprsIs(toAprsIs), _GPS(GPS) {
}

RouterTask::~RouterTask() {
}

bool RouterTask::setup(System &system) {

   Wire.begin(21,22);                                                      // initialize I2C that connects to sensor
   BMESensor.begin();  
   sendPos=true;
 // setup beacon
  _beacon_timer.setTimeout(system.getUserConfig()->beacon.timeout * 60 * 1000);

  _beaconMsg = std::shared_ptr<APRSMessage>(new APRSMessage());
  _beaconMsg->setSource(system.getUserConfig()->callsign);
  _beaconMsg->setDestination("APLG01");
  return true;
}

bool RouterTask::loop(System &system) {
  // do routing
  if (!_fromModem.empty()) {
    std::shared_ptr<APRSMessage> modemMsg = _fromModem.getElement();

    if (system.getUserConfig()->aprs_is.active && modemMsg->getSource() != system.getUserConfig()->callsign) {
      std::shared_ptr<APRSMessage> aprsIsMsg = std::make_shared<APRSMessage>(*modemMsg);
      String                       path      = aprsIsMsg->getPath();

      if (!(path.indexOf("RFONLY") != -1 || path.indexOf("NOGATE") != -1 || path.indexOf("TCPIP") != -1)) {
        if (!path.isEmpty()) {
          path += ",";
        }

        aprsIsMsg->setPath(path + "qAR," + system.getUserConfig()->callsign);

        logPrintD("APRS-IS: ");
        logPrintlnD(aprsIsMsg->toString());
        _toAprsIs.addElement(aprsIsMsg);
      } else {
        logPrintlnD("APRS-IS: no forward => RFonly");
      }
    } else {
      if (!system.getUserConfig()->aprs_is.active)
        logPrintlnD("APRS-IS: disabled");

      if (modemMsg->getSource() == system.getUserConfig()->callsign)
        logPrintlnD("APRS-IS: no forward => own packet received");
    }

    if (system.getUserConfig()->digi.active && modemMsg->getSource() != system.getUserConfig()->callsign) {
      std::shared_ptr<APRSMessage> digiMsg = std::make_shared<APRSMessage>(*modemMsg);
      String                       path    = digiMsg->getPath();

      // simple loop check
      if (path.indexOf("WIDE1-1") >= 0 || path.indexOf(system.getUserConfig()->callsign) == -1) {
        // fixme
        digiMsg->setPath(system.getUserConfig()->callsign + "*");

        logPrintD("DIGI: ");
        logPrintlnD(digiMsg->toString());

        _toModem.addElement(digiMsg);
      }
    }
  }

  // check for beacon
  if (_beacon_timer.check()) {
  
    BMESensor.refresh();                                                  // read current sensor data
    logPrintlnD(String(BMESensor.temperature));

    String lat,lng;
    if (!_GPS.isValid){
     lat = create_lat_aprs(system.getUserConfig()->beacon.positionLatitude);
     lng = create_long_aprs(system.getUserConfig()->beacon.positionLongitude);
    }
    else
    {
      lat=create_lat_aprs(_GPS.lat);
      lng=create_long_aprs(_GPS.lng);
    }

    if (sendPos)
    {
      _beaconMsg->getBody()->setData(String("=") + lat + "L" + lng  + "&" + system.getUserConfig()->beacon.message);
      if (Wire.available())
        sendPos=false;
    }
    else
    {
      char str[50];
      if (sendParamName)
      {
        sprintf(str, "PARM.Temperature,Humity");
        sendParamName=false;
      } else if (sendParamUnit)
      {
        sprintf(str, "UNIT.deg C,Perc");
        sendParamUnit=false;
      } else if (sendParamEqns)
      {
        sprintf(str, "EQNS.0,0.1,0,0,0.1,0");
        sendParamEqns=false;
      }
      else
      {
        sprintf(str, "T#%03d,%03d,%03d,%03d,%03d,%03d,00000000",TeleNr++,int(BMESensor.temperature*10),0,0,0,0);
      }
      _beaconMsg->getBody()->setData(str);
      sendPos=true;
    }

    logPrintD("[" + timeString() + "] ");
    logPrintlnD(_beaconMsg->encode());


    if (system.getUserConfig()->aprs_is.active)
      _toAprsIs.addElement(_beaconMsg);

    if (system.getUserConfig()->digi.beacon) {
      _toModem.addElement(_beaconMsg);
    }

    system.getDisplay().addFrame(std::shared_ptr<DisplayFrame>(new TextFrame("BEACON", _beaconMsg->toString())));
    _beacon_timer.start();
  }

  uint32_t diff = _beacon_timer.getTriggerTimeInSec();
  _stateInfo    = "beacon " + String(uint32_t(diff / 60)) + ":" + String(uint32_t(diff % 60));

  return true;
}
