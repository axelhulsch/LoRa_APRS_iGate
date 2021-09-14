#include <logger.h>

#include <TimeLib.h>


#include "Task.h"
#include "TaskSensor.h"
#include "project_configuration.h"

String create_lat_aprs(double lat);
String create_long_aprs(double lng);


SensorTask::SensorTask(TaskQueue<std::shared_ptr<APRSMessage>> &toAprsIs) : Task(TASK_SENSOR, TaskSensor), _toAprsIs(toAprsIs), ws980(false) {
}

SensorTask::~SensorTask() {
}

bool SensorTask::setup(System &system) {
  if (system.getUserConfig()->sensor.ws980ip.isEmpty())
    return false;
  ws980ip.fromString(system.getUserConfig()->sensor.ws980ip);
  _beaconMsg = std::shared_ptr<APRSMessage>(new APRSMessage());
  _beaconMsg->setSource(system.getUserConfig()->sensor.callsign);
  _beaconMsg->setDestination("APRS");
  
  logPrintlnI("Sensor Setup");
   _beacon_timer.setTimeout(system.getUserConfig()->sensor.timeout * 60 * 1000);
  ws980=true;
  return true;
}


bool SensorTask::loop(System &system) {
  if (!ws980)
    return false;
  if (_beacon_timer.check()) {

    logPrintlnE("Connect to ws980");
    if (ws980_client.connect(ws980ip, 45000,100)) {
      if (ws980_client.connected()) {
        logPrintlnE("ws980 connected");
        char buffer[]={0xff,0xff,0x0b,0x00,0x06,0x04,0x04,0x19};
        ws980_client.write(buffer,sizeof(buffer));
      }
      else return false;

    // wait for data to be available
      unsigned long timeout = millis();
      while (ws980_client.available() == 0) {
        if (millis() - timeout > 5000) {
          logPrintlnE(">>> Client Timeout !");
          ws980_client.stop();
          delay(60000);
          return false;
        }
      }

      // Read all the lines of the reply from server and print them to Serial
      logPrintlnE("receiving from remote server");
      // not testing 'client.connected()' since we do not need to send data here
      int i=0;
      uint8_t readbuffer[200];
      while (ws980_client.available()) {
        readbuffer[i++] = static_cast<uint8_t>(ws980_client.read());
        logPrintE(String(readbuffer[i-1],HEX));
      }

      // Close the connection
      logPrintlnE("closing connection");
      ws980_client.stop();


      logPrintlnE(" ");
      if (i<81) return false;

      byte sum=0;
      for(int s=5;s<=79;s++){
        sum+=readbuffer[s];
      }
      logPrintE("sum = ");
      logPrintlnE(String(sum,HEX));
      if (sum!=readbuffer[80]){
        logPrintlnE("WS980 checksum Error");
      }
      
      sum=0;
      for(int s=2;s<=80;s++){
        sum+=readbuffer[s];
      }
      if (sum!=readbuffer[81]){
        logPrintlnE("WS980 checksum Error");
      }
      logPrintE("sum = ");
      logPrintlnE(String(sum,HEX));


    

      String lat,lng;
      lat = create_lat_aprs(system.getUserConfig()->sensor.positionLatitude);
      lng = create_long_aprs(system.getUserConfig()->sensor.positionLongitude);


      String message=String("!") + lat + "/" + lng;

      message +="_";
      float wdir=static_cast<int16_t>(readbuffer[32]*0x100+readbuffer[33]); 
      if(wdir<100) { message += "0"; }
      if(wdir<10) { message += "0"; }
      String s=String(wdir,0);s.replace(" ","");
      message += s;

      message +="/";
      float wspd=static_cast<int16_t>(readbuffer[35]*0x100+readbuffer[36]); 
      wspd*=2.23694/10.0;
      if(wspd<100) { message += "0"; }
      if(wspd<10) { message += "0"; }
      s=String(wspd,0);s.replace(" ","");
      message += s;

      message +="g";
      float gspd=static_cast<int16_t>(readbuffer[38]*0x100+readbuffer[39]); 
      gspd*=2.23694/10.0;
      if(gspd<100) { message += "0"; }
      if(gspd<10) { message += "0"; }
      s=String(gspd,0);s.replace(" ","");
      message += s;
      //message +="_000/000g000";

      float Temperature=static_cast<int16_t>(readbuffer[10]*0x100+readbuffer[11]);
      float temp = Temperature*1.8/10;
      temp = temp + 32;
      message +="t"; 
      if(temp<100) { message += "0"; }
      if(temp<10) { message += "0"; }
      s = String(temp,0);s.replace(" ","");
      message += s;

      float Rain=static_cast<int32_t>(readbuffer[41]*0x1000000+readbuffer[42]*0x10000+readbuffer[43]*0x100+readbuffer[44]);
      float rain = Rain*0.3937;
      message +="r"; 
      if(rain<100) { message += "0"; }
      if(rain<10) { message += "0"; }
      s = String(rain,0);s.replace(" ","");
      message += s;
      
      Rain=static_cast<int32_t>(readbuffer[46]*0x1000000+readbuffer[47]*0x10000+readbuffer[48]*0x100+readbuffer[49]);
      rain = Rain*0.3937;
      message +="p"; 
      if(rain<100) { message += "0"; }
      if(rain<10) { message += "0"; }
      s = String(rain,0);s.replace(" ","");
      message += s;

      //message +="r000";
      //message +="p000";
      
 //     message += "h51b10218";
      float humi = readbuffer[24];
      message +="h" +String(humi,0);
    
      float press = static_cast<int16_t>(readbuffer[29]*0x100+readbuffer[30]);
      message += ("b");
      if(press<10000) { message += "0"; }
      message += String(press,0);

      message +=system.getUserConfig()->sensor.message;

      _beaconMsg->getBody()->setData(message);

      logPrintD("[" + timeString() + "] ");
      logPrintlnD(_beaconMsg->encode());


      if (system.getUserConfig()->aprs_is.active)
        _toAprsIs.addElement(_beaconMsg);

      system.getDisplay().addFrame(std::shared_ptr<DisplayFrame>(new TextFrame("BEACON", _beaconMsg->toString())));
      _beacon_timer.start();
    }
  }

  uint32_t diff = _beacon_timer.getTriggerTimeInSec();
  _stateInfo    = "beacon " + String(uint32_t(diff / 60)) + ":" + String(uint32_t(diff % 60));

  return true;
}

