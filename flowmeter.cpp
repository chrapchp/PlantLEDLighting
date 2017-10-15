/*
  FlowMeter based on
  Flow meter from http://www.seeedstudio.com/wiki/G1/2_Water_Flow_sensor +/- 3% acuracy
  Author: Peter
  Date: 2k13
*/

#include <Streaming.h>
#include "FlowMeter.h"



FlowMeter::FlowMeter(int aPin, int aDeltaT )
{
 mPin = aPin;
 pinMode( mPin, INPUT);
 mCurrentFlowDuration = 0;
 mDeltaT = aDeltaT;
 resetStatistics();

}

void FlowMeter::begin()
{
	mPulseCount = 0;
}

void FlowMeter::end()
{
	computeFlowRate();
	updateCummulativeVolume();

	updateFlowRunTime();
}

void FlowMeter::updateCummulativeVolume()
{
	mCummulativeVolume += getCurrentFlowRate();
}

void FlowMeter::updateFlowRunTime()
{
	if( getCurrentFlowRate() == 0. )
	{
		if( getPreviousFlowRate() > 0. )  // water flow just stopped
		{
			//mCurrentFlowDuration += mDeltaT;
			//mTotalFlowDuration += mDeltaT;


			if( mCurrentFlowDuration > mMaxFlowDuration )
					mMaxFlowDuration =  mCurrentFlowDuration ;
			if( ( mMinFlowDuration == 0 ) || ( mCurrentFlowDuration < mMinFlowDuration ) )
					mMinFlowDuration =  mCurrentFlowDuration ;

			mAverageFlowDuration = (long) mTotalFlowDuration / mFlowCounts;
			mCurrentFlowDuration = 0;

		}
	}
	else
	{
		// water flow just started (rising edge)

		if( getPreviousFlowRate()  == 0 )
			mFlowCounts++;
		//else
	//	{
		// water still flowing after 1 epoch (deltaT)
			mCurrentFlowDuration += mDeltaT;
			mTotalFlowDuration += mDeltaT;
	//	}
	}




}

  // Pulse frequency (Hz) in Horizontal Test= 7.5Q, Q is flow rate in L/min
  // Pulse Count / 7.5 / 60 = Q L/sec
float FlowMeter::computeFlowRate()
{
	mPreviousFlowRate = mCurrentFlowRate;

	mCurrentFlowRate =   mPulseCount /60. / 7.5;

	return( mCurrentFlowRate);
}

void FlowMeter::handleFlowDetection()
{
	mPulseCount++;
}

float FlowMeter::getCurrentFlowRate()
{
	return( mCurrentFlowRate );
}

float FlowMeter::getPreviousFlowRate()
{
	return( mPreviousFlowRate );
}

float FlowMeter::getCummulativeVolume()
{
	return( mCummulativeVolume );
}



void FlowMeter::resetStatistics()
{
	mCummulativeVolume = 0.;
	mMaxFlowDuration = 0;
	mMinFlowDuration = 0;
	mAverageFlowDuration = 0.;
	mTotalFlowDuration = 0;
	mFlowCounts=0;

}

long	FlowMeter::getMaxFlowDuration()
{
	return( mMaxFlowDuration );
}

long FlowMeter::getMinFlowDuration()
{


	return( mMinFlowDuration );
}

long FlowMeter::getAverageFlowDuration()
{
	return ( mAverageFlowDuration );
}

long FlowMeter::getTotalFlowDuration()
{
	return( mTotalFlowDuration );
}

void FlowMeter::serialize(HardwareSerial *tracePort, bool includeCR)
{

	*tracePort << "{pin:" << mPin << " deltaT:" << mDeltaT << " curFlowRate:" << getCurrentFlowRate() ;
	*tracePort << " L/s minFlowDuration:" << getMinFlowDuration() << "s maxFlowDuration:" << getMaxFlowDuration()  ;
	*tracePort << " s cummulativeVolume:" << getCummulativeVolume() << "s totalFlowDuration:" << getTotalFlowDuration()   ;
	*tracePort << " s }" ; 
	if (includeCR)
		*tracePort << endl;
}