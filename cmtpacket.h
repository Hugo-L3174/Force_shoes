#ifndef XSENS_MONOLITHIC
/*! \file
	\brief	Contains the CMT Packet interface

	This level contains the packet interface of Cmt.
	All code should be OS-independent.

	\section FileCopyright Copyright Notice 
	Copyright (C) Xsens Technologies B.V., 2006.  All rights reserved.

	This source code is intended for use only by Xsens Technologies BV and
	those that have explicit written permission to use it from
	Xsens Technologies BV.

	THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
	KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
	IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
	PARTICULAR PURPOSE.

	\section FileChangelog	Changelog
	\par 2006-05-10, v0.0.1
	\li Job Mulder:	Created Cmtpacket.h
	\par 2006-07-21, v0.1.0
	\li Job Mulder:	Updated file for release 0.1.0
*/
#endif
#ifndef CMTPACKET_H
#define CMTPACKET_H

#ifndef XSENS_MONOLITHIC
#	include "cmtdef.h"
#	include "cmt2.h"
#endif


namespace xsens {


	//! Indicates that a data item is not available in the packet
#define CMT_DATA_ITEM_NOT_AVAILABLE		65535

	//! A structure containing MT data + timestamp and formatting information.
class Packet
{
	friend class Cmt4;
protected:
		//! Contains information about data in the packet and the format of that data
	mutable struct PacketInfo {
		uint16_t m_offset;
		uint16_t m_rawData;
		uint16_t m_rawAcc;
		uint16_t m_rawGyr;
		uint16_t m_rawMag;
		uint16_t m_rawTemp;
		uint16_t m_temp;
		uint16_t m_calData;
		uint16_t m_calAcc;
		uint16_t m_calGyr;
		uint16_t m_calMag;
		uint16_t m_oriQuat;
		uint16_t m_oriEul;
		uint16_t m_oriMat;
		uint16_t m_analogIn1;
		uint16_t m_analogIn2;
		uint16_t m_posLLA;
		uint16_t m_velNEDorNWU;
		uint16_t m_status;
		uint16_t m_sc;
		uint16_t m_utcTime;
		uint16_t m_utcNano;
		uint16_t m_utcYear;
		uint16_t m_utcMonth;
		uint16_t m_utcDay;
		uint16_t m_utcHour;
		uint16_t m_utcMinute;
		uint16_t m_utcSecond;
		uint16_t m_utcValid;
		uint16_t m_acc_g;
		uint16_t m_gpsPvtData;
		uint16_t m_gpsPvtPressure;
		uint16_t m_gpsPvtPressureAge;
		uint16_t m_gpsPvtGpsData;
		uint16_t m_gpsPvtItow;
		uint16_t m_gpsPvtLatitude;
		uint16_t m_gpsPvtLongitude;
		uint16_t m_gpsPvtHeight;
		uint16_t m_gpsPvtVeln;
		uint16_t m_gpsPvtVele;
		uint16_t m_gpsPvtVeld;
		uint16_t m_gpsPvtHacc;
		uint16_t m_gpsPvtVacc;
		uint16_t m_gpsPvtSacc;
		uint16_t m_gpsPvtGpsAge;
		uint16_t m_size;
		uint16_t m_doubleBoundary;
	}*	m_infoList;
	CmtDataFormat*	m_formatList;	//!< A list of the formats of the data items
	bool m_xm;						//!< Indicates that xbus-formatting is used

public:
	uint16_t		m_itemCount;	//!< The number of data items in the message
	Message			m_msg;			//!< The message
	TimeStamp		m_rtc;			//!< Sample time in ms, based on the sample counter
	TimeStamp		m_toa;			//!< Time of arrival

	Packet(uint16_t items, bool xbus);
	Packet(const Packet& pack);
	~Packet();

	void operator = (const Packet& pack);

	bool setDataFormat(const CmtDataFormat& format, const uint16_t index = 0);
	bool setDataFormat(const CmtOutputMode outputMode, const CmtOutputSettings outputSettings, const uint16_t index = 0);
	CmtDataFormat getDataFormat(const uint16_t index = 0) const;
	PacketInfo getInfoList(const uint16_t index = 0) const;
	void setXbus(bool xbus, bool convert = false);
	bool getXbus(void) const;


	/*! \brief Return the data size.

		\param index The index of the item of which the size should be returned.
	*/
	uint16_t getDataSize(const uint16_t index=0) const;

	/*! \brief Return the floating/fixed point value size

		\param index The index of the item whose fp size should be returned.
	*/
	uint16_t getFPValueSize(const uint16_t index) const;

	/*! \brief Return the Raw Accelerometer component of a data item.
	
		\param index The index of the item of which the data should be returned.
	*/
	CmtShortVector getRawAcc(const uint16_t index=0) const;
		//! Check if data item contains Raw Accelerometer data
	bool containsRawAcc(const uint16_t index=0) const;
		//! Add/update Raw Accelerometer data for the item
	bool updateRawAcc(const CmtShortVector& vec, const uint16_t index=0);
	/*! \brief Return the Raw Gyroscope component of a data item.
	
		\param index The index of the item of which the data should be returned.
	*/
	CmtShortVector getRawGyr(const uint16_t index=0) const;
		//! Check if data item contains Raw Gyroscope data
	bool containsRawGyr(const uint16_t index=0) const;
		//! Add/update Raw Gyroscope data for the item
	bool updateRawGyr(const CmtShortVector& vec, const uint16_t index=0);
	/*! \brief Return the Raw Magnetometer component of a data item.
	
		\param index The index of the item of which the data should be returned.
	*/
	CmtShortVector getRawMag(const uint16_t index=0) const;
		//! Check if data item contains Raw Magnetometer data
	bool containsRawMag(const uint16_t index=0) const;
		//! Add/update Raw Magnetometer data for the item
	bool updateRawMag(const CmtShortVector& vec, const uint16_t index=0);
	/*! \brief Return the Raw Temperature component of a data item.
	
		\param index The index of the item of which the data should be returned.
	*/
	uint16_t getRawTemp(const uint16_t index=0) const;
		//! Check if data item contains Raw Temperature data
	bool containsRawTemp(const uint16_t index=0) const;
		//! Add/update Raw Temperature data for the item
	bool updateRawTemp(uint16_t temp, const uint16_t index=0);
	/*! \brief Return the Raw Data component of a data item.
	
		\param index The index of the item of which the data should be returned.
	*/
	CmtRawData getRawData(const uint16_t index=0) const;
		//! Check if data item contains Raw Data
	bool containsRawData(const uint16_t index=0) const;
		//! Add/update Raw Data for the item
	bool updateRawData(const CmtRawData& data, const uint16_t index=0);
	/*! \brief Return the Gps PVT data component of a data item.
	\param index The index of the item of which the data should be returned.
	*/
	CmtGpsPvtData getGpsPvtData(const uint16_t index=0) const;
	//! Check if data item contains Gps PVT Data
	bool containsGpsPvtData(const uint16_t index=0) const;
	//! Add/update Gps PVT Data for the item
	bool updateGpsPvtData(const CmtGpsPvtData& data, const uint16_t index=0);

	//! This function is obsolete, use getGpsPvtData instead. This function will be removed in future versions of the MT SDK. \see getGpsPvtData
	#define	getRawGpsData getGpsPvtData
	//! This function is obsolete, use containsGpsPvtData instead. This function will be removed in future versions of the MT SDK. \see containsGpsPvtData
	#define	containsRawGpsData containsGpsPvtData
	//! This function is obsolete, use updateGpsPvtData instead. This function will be removed in future versions of the MT SDK. \see updateGpsPvtData
	#define	updateRawGpsData updateGpsPvtData

	/*! \brief Return the Raw Pressure Data component of a data item.
	\param index The index of the item of which the data should be returned.
	*/
	CmtRawPressureData getRawPressureData(const uint16_t index=0) const;
	//! Check if data item contains Raw Pressure Data
	bool containsRawPressureData(const uint16_t index=0) const;
	//! Add/update Raw Pressure Data for the item
	bool updateRawPressureData(const CmtRawPressureData& data, const uint16_t index=0);

	/*! \brief Return the Temperature component of a data item.

	\param index The index of the item of which the data should be returned.
	*/
	double getTemp(const uint16_t index=0) const;
	//! Check if data item contains Temperature data
	bool containsTemp(const uint16_t index=0) const;
	//! Add/update Calibrated Accelerometer data for the item
	bool updateTemp(const double& temp, const uint16_t index=0);
	/*! \brief Return the Calibrated Accelerometer component of a data item.
	
		\param index The index of the item of which the data should be returned.
	*/
	CmtVector getCalAcc(const uint16_t index=0) const;
		//! Check if data item contains Calibrated Accelerometer data
	bool containsCalAcc(const uint16_t index=0) const;
		//! Add/update Calibrated Accelerometer data for the item
	bool updateCalAcc(const CmtVector& vec, const uint16_t index=0);
	/*! \brief Return the Calibrated Gyroscope component of a data item.
	
		\param index The index of the item of which the data should be returned.
	*/
	CmtVector getCalGyr(const uint16_t index=0) const;
		//! Check if data item contains Calibrated Gyroscope data
	bool containsCalGyr(const uint16_t index=0) const;
		//! Add/update Calibrated Gyroscope data for the item
	bool updateCalGyr(const CmtVector& vec, const uint16_t index=0);
	/*! \brief Return the Calibrated Magnetometer component of a data item.
	
		\param index The index of the item of which the data should be returned.
	*/
	CmtVector getCalMag(const uint16_t index=0) const;
		//! Check if data item contains Calibrated Magnetometer data
	bool containsCalMag(const uint16_t index=0) const;
		//! Add/update Calibrated Magnetometer data for the item
	bool updateCalMag(const CmtVector& vec, const uint16_t index=0);
	/*! \brief Return the Calibrated Data component of a data item.
	
		\param index The index of the item of which the data should be returned.
	*/
	CmtCalData getCalData(const uint16_t index=0) const;
		//! Check if data item contains Calibrated Data
	bool containsCalData(const uint16_t index=0) const;
		//! Add/update Calibrated Data for the item
	bool updateCalData(const CmtCalData& data, const uint16_t index=0);

	/*! \brief Return the Orientation component of a data item as a Quaternion.
	
		\param index The index of the item of which the data should be returned.
	*/
	CmtQuat getOriQuat(const uint16_t index=0) const;
		//! Check if data item contains Quaternion Orientation data
	bool containsOriQuat(const uint16_t index=0) const;
		//! Add/update Quaternion Orientation data for the item
	bool updateOriQuat(const CmtQuat& data, const uint16_t index=0);
	/*! \brief Return the Orientation component of a data item as Euler angles.
	
		\param index The index of the item of which the data should be returned.
	*/
	CmtEuler getOriEuler(const uint16_t index=0) const;
		//! Check if data item contains Euler Orientation data
	bool containsOriEuler(const uint16_t index=0) const;
		//! Add/update Euler Orientation data for the item
	bool updateOriEuler(const CmtEuler& data, const uint16_t index=0);
	/*! \brief Return the Orientation component of a data item as an Orientation Matrix.
	
		\param index The index of the item of which the data should be returned.
	*/
	CmtMatrix getOriMatrix(const uint16_t index=0) const;
		//! Check if data item contains Matrix Orientation data
	bool containsOriMatrix(const uint16_t index=0) const;
		//! Add/update Matrix Orientation data for the item
	bool updateOriMatrix(const CmtMatrix& data, const uint16_t index=0);

		//! Check if data item contains Orientation Data of any kind
	bool containsOri(const uint16_t index=0) const;

	/*! \brief Return the AnalogIn 1 component of a data item.

	\param index The index of the item of which the data should be returned.
	*/
	CmtAnalogInData getAnalogIn1(const uint16_t index=0) const;
	//! Check if data item contains AnalogIn 1
	bool containsAnalogIn1(const uint16_t index=0) const;
	//! Add/update AnalogIn 1 for the item
	bool updateAnalogIn1(const CmtAnalogInData& data, const uint16_t index=0);

	/*! \brief Return the AnalogIn 2 component of a data item.

	\param index The index of the item of which the data should be returned.
	*/
	CmtAnalogInData getAnalogIn2(const uint16_t index=0) const;
	//! Check if data item contains AnalogIn 2
	bool containsAnalogIn2(const uint16_t index=0) const;
	//! Add/update AnalogIn 2 for the item
	bool updateAnalogIn2(const CmtAnalogInData& data, const uint16_t index=0);

	/*! \brief Return the Position Lat Lon Alt component of a data item.

	\param index The index of the item of which the data should be returned.
	*/
	CmtVector getPositionLLA(const uint16_t index=0) const;
	//! Check if data item contains Position Lat Lon Alt
	bool containsPositionLLA(const uint16_t index=0) const;
	//! Add/update Position Lat Lon Alt for the item
	bool updatePositionLLA(const CmtVector& data, const uint16_t index=0);

	/*! \brief Return the Velocity component of a data item.

	\param index The index of the item of which the data should be returned.
	*/
	CmtVector getVelocity(const uint16_t index=0) const;
	//! Check if data item contains Velocity
	bool containsVelocity(const uint16_t index=0) const;
	//! Add/update Velocity for the item
	bool updateVelocity(const CmtVector& data, const uint16_t index=0);

	/*! \brief Return the Status component of a data item.

	\param index The index of the item of which the data should be returned.
	*/
	uint8_t getStatus(const uint16_t index=0) const;
	//! Check if data item contains Status
	bool containsStatus(const uint16_t index=0) const;
	//! Add/update Status information for the item
	bool updateStatus(const uint8_t data, const uint16_t index=0);

	/*! \brief Return the Sample Counter component of the packet.
	
		\param index The index of the item of which the data should be returned. (ignored)
	*/
	uint16_t getSampleCounter(const uint16_t index=0) const;
		//! Check if data item contains Sample Counter
	bool containsSampleCounter(const uint16_t index=0) const;
		//! Add/update Sample Counter for all items
	bool updateSampleCounter(const uint16_t counter, const uint16_t index=0);

	/*! \brief Return the UTC Time component of the packet.
	
		\param index The index of the item of which the data should be returned. (ignored)
	*/
	CmtUtcTime getUtcTime(const uint16_t index=0) const;
		//! Check if data item contains UTC Time
	bool containsUtcTime(const uint16_t index=0) const;
		//! Add/update UTC Time for all items
	bool updateUtcTime(const CmtUtcTime& data, const uint16_t index=0);

	/*! \brief Return the RTC of the packet.
	
		\param index The index of the item of which the data should be returned. (ignored)
	*/
	CmtTimeStamp getRtc(const uint16_t index=0) const;


	/*! \brief Return the XKF-3 Acc-G component of the packet.
	
		\param index The index of the item of which the data should be returned.
	*/
	CmtVector getAccG(const uint16_t index=0) const;
		//! Check if data item contains XKF-3 Acc-G data
	bool containsAccG(const uint16_t index=0) const;
		//! Add/update XKF-3 Acc-G data for the item
	bool updateAccG(const CmtVector& g, const uint16_t index=0);

	#ifdef CMT_DLL_EXPORT
		//! Interpolate so resulting packet is (1-f)*pa + pb OR pb if pa and pb are non-matching types
	void interpolate(const Packet& pa, const Packet& pb, const double f, int32_t interpolationMode = 1);
	#endif
};

}	// end of xsens namespace

#endif	// CMTPACKET_H
