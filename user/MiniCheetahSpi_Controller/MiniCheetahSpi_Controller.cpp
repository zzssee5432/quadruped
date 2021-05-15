#include "MiniCheetahSpi_Controller.h"

void MiniCheetahSpi_Controller::runController() {
  static double  num=0;
  static double ln=_legController->datas[0].q(0);
  static int  badnum=0;
  if(num>1000)
  num=0;
  for(int i = 0; i < 4; i++) {
   _legController->commands[i].kpJoint(0,0)=num;
   _legController->commands[i].kpJoint(1,1)=num;
   _legController->commands[i].kpJoint(2,2)=num;
     _legController->commands[i].kdJoint(0,0)=num;
       _legController->commands[i].kdJoint(1,1)=num;
        _legController->commands[i].kdJoint(2,2)=num;
    _legController->commands[i].qDes(0)=num;
        _legController->commands[i].qDes(1)=num;
           _legController->commands[i].qDes(2)=num;
 _legController->commands[i].qdDes(0)=num;
        _legController->commands[i].qdDes(1)=num;
          _legController->commands[i].qdDes(2)=num;
  //}

 } 
   if(_legController->datas[0].q(0)-ln>=2||_legController->datas[0].qd(0)-ln>=2||_legController->datas[0].v(0)-ln>=2)
   {
      badnum++;
      printf("badnum:%d\r\n",badnum);
   }

     ln=_legController->datas[0].q(0);
   num+=1;
   
}
