#ifndef TASK_SENSOR_H_
#define TASK_SENSOR_H_

#include <Timer.h>
#include <TaskManager.h>
#include <WiFi.h>
#include "TaskAprsIs.h"


class SensorTask : public Task {
public:
  SensorTask(TaskQueue<std::shared_ptr<APRSMessage>> &toAprsIs);
  virtual ~SensorTask();

  virtual bool setup(System &system) override;
  virtual bool loop(System &system) override;
private:
 WiFiClient ws980_client;
 std::shared_ptr<APRSMessage> _beaconMsg;
 Timer   _beacon_timer;
 TaskQueue<std::shared_ptr<APRSMessage>> &_toAprsIs;
 IPAddress ws980ip;
 bool ws980;

};
#endif
