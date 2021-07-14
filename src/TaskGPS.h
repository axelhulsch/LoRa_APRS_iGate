#ifndef TASK_GPS_H_
#define TASK_GPS_H_

#include <BoardFinder.h>
#include <LoRa_APRS.h>
#include <Timer.h>
#include <TaskManager.h>

class GPSTask : public Task {
public:
  GPSTask();
  virtual ~GPSTask();

  virtual bool setup(System &system) override;
  virtual bool loop(System &system) override;
  double lat,lng;
  bool isValid;
private:
  bool      _beginCalled;
};
#endif
