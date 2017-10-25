/*
  FlowMeter based on
  Flow meter from http://www.seeedstudio.com/wiki/G1/2_Water_Flow_sensor +/- 3% acuracy
  Author: Peter
  Date: 2k13
*/

#ifndef FlowMeter_h
#define FlowMeter_h

//#include "WProgram.h"
//#include <Arduino.h>  // was WProgram.h, changed to Arduino.h in 1.0
#include <HardwareSerial.h>



class FlowMeter
{
  public:

    FlowMeter(int aPin, int aDeltaT); //deltaT in ms

	void  begin();
	void  end();


  	float getCurrentFlowRate();	// in L/sec
  	float getPreviousFlowRate();	// in L/sec
  	float getCummulativeVolume();	// in L/sec

  	void resetStatistics();
    void dayRollOver();

  	long	getMinFlowDuration();     //in seconds
  	long getMaxFlowDuration();     //in seconds
  	long getAverageFlowDuration(); //in seconds
  	long getTotalFlowDuration();
	void handleFlowDetection();
  void serialize( HardwareSerial *tracePort, bool includeCR );

protected:

    float computeFlowRate();

private:

//   init( int aPin );
   void updateCummulativeVolume();
   void updateFlowRunTime();
   float mCurrentFlowRate;
   float mPreviousFlowRate;
   float mCummulativeVolume;
   float mYDAYCumulativeVolume;
   long  mMaxFlowDuration;
   long	 mMinFlowDuration;
   long	 mAverageFlowDuration;
   int   mPin;
   int   mPulseCount;
   int	 mDeltaT;
   long	 mCurrentFlowDuration; //ms
   unsigned long mTotalFlowDuration;
   long mFlowCounts;

};

#endif