#ifndef TASK_ROUTER_H_
#define TASK_ROUTER_H_

#include <APRSMessage.h>
#include <TaskGPS.h>
#include <TaskManager.h>

class RouterTask : public Task {
public:
  RouterTask(TaskQueue<std::shared_ptr<APRSMessage>> &fromModem, TaskQueue<std::shared_ptr<APRSMessage>> &toModem, TaskQueue<std::shared_ptr<APRSMessage>> &toAprsIs, GPSTask &GPS);
  virtual ~RouterTask();

  virtual bool setup(System &system) override;
  virtual bool loop(System &system) override;

private:
  TaskQueue<std::shared_ptr<APRSMessage>> &_fromModem;
  TaskQueue<std::shared_ptr<APRSMessage>> &_toModem;
  TaskQueue<std::shared_ptr<APRSMessage>> &_toAprsIs;

  std::shared_ptr<APRSMessage> _beaconMsg;
  Timer                        _beacon_timer;

  GPSTask &_GPS;
};

#endif
