#ifndef CHEETAH_SOFTWARE_MINICHEETAHSPI_CONTROLLER_H
#define CHEETAH_SOFTWARE_MINICHEETAHSPI_CONTROLLER_H

#include <RobotController.h>
#include "Spi_UserParameters.h"


class MiniCheetahSpi_Controller : public RobotController {
public:
  MiniCheetahSpi_Controller(){};
  virtual ~MiniCheetahSpi_Controller(){}

  virtual void initializeController(){}
  virtual void runController();
  virtual void updateVisualization(){}
  virtual ControlParameters* getUserControlParameters() {
    return &userParameters;
  }
protected:
Spi_UserParameters userParameters;

};

#endif //CHEETAH_SOFTWARE_MINICHEETAHSPI_CONTROLLER_H
