// -*- C++ -*-
/*!
 * @file  EEZYbotARMControlRTC.cpp
 * @brief Control module for EEZYbotARM by using RTno
 * @date $Date$
 *
 * $Id$
 */

#include "EEZYbotARMControlRTC.h"
#include <coil/Time.h> 
#include <cmath>
#include <math.h>       /* acos */

#define PI 3.14159265

// Module specification
// <rtc-template block="module_spec">
static const char* eezybotarmcontrolrtc_spec[] =
  {
    "implementation_id", "EEZYbotARMControlRTC",
    "type_name",         "EEZYbotARMControlRTC",
    "description",       "Control module for EEZYbotARM by using RTno",
    "version",           "1.0.0",
    "vendor",            "takahashi",
    "category",          "Manipulato",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
EEZYbotARMControlRTC::EEZYbotARMControlRTC(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_angleOut("angle", m_angle)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
EEZYbotARMControlRTC::~EEZYbotARMControlRTC()
{
}



RTC::ReturnCode_t EEZYbotARMControlRTC::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  addOutPort("angle", m_angleOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>

  m_angle.data.length(4);

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t EEZYbotARMControlRTC::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t EEZYbotARMControlRTC::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t EEZYbotARMControlRTC::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t EEZYbotARMControlRTC::onActivated(RTC::UniqueId ec_id)
{
	for (int i = 0; i < m_angle.data.length(); i++) {
		m_angle.data[i] = 0;
	}

    return RTC::RTC_OK;
}


RTC::ReturnCode_t EEZYbotARMControlRTC::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

void cart2polar(double x, double y, double *r, double *theta)
{
	// Determine magnitude of cartesian coordinates
	*r = std::hypot(x, y);

	// Don't try to calculate zero magnitude vectors angles
	if (r == 0) {
		return;
	}
	
	double c = x / *r;
	double s = y / *r;

	// Safety!
	if (s > 1) {
		s = 1;
	}
	if (c > 1) {
		c = 1;
	}
	if (s < -1) {
		s = -1;
	}
	if (c < -1) {
		c = -1;
	}

	// Calculate angle in 0..PI
	*theta = acos(c);

	// Convert to full range
	if (s < 0) {
		*theta = -(*theta);
	}
}

bool cosangle(double opp, double adj1, double adj2, double *theta)
{
	/* Cosine rule :
	 * C ^ 2 = A ^ 2 + B ^ 2 - 2 * A*B*cos(Angle_AB)
	 * cos(Angle_AB) = (A ^ 2 + B ^ 2 - C ^ 2) / (2 * A*B)
	 * C is opposite
	 * A, B are adjacent
	 */

	double den = 2 * adj1 * adj2;

	if (den == 0) {
		return false;
	}
	double c = (adj1 * adj1 + adj2 * adj2 - opp * opp) / den;
	
	if ((c > 1) || (c < -1)) {
		return false;
	}
	theta[0] = acos(c);
	
	return true;
}

#define L1 80
#define L2 80
#define L3 68

bool solve(double x, double y, double z, double *angles)
{
	double r, th0, R, ang_P;

	// Solve top - down view
	cart2polar(y, x, &r, &th0);
	r -= L3; // Account for the wrist length

	// In arm plane, convert to polar
	cart2polar(r, z, &R, &ang_P);

	double parmB[1] = { 0 };
	double parmC[1] = { 0 };

	// Solve arm inner angles as required
	if (!cosangle(L2, L1, R, parmB)) {
		return false;
	}
	if (!cosangle(R, L1, L2, parmC)) {
		return false;
	}

	double B = parmB[0];
	double C = parmC[0];

	// Solve for servo angles from horizontal
	double a0 = th0;
	double a1 = ang_P + B;
	double a2 = C + a1 - PI;

	angles[0] = a0;
	angles[1] = a1;
	angles[2] = a2;

	return true;
}

RTC::ReturnCode_t EEZYbotARMControlRTC::onExecute(RTC::UniqueId ec_id)
{
	double angles[3];
	double x, y, z;

	std::cout << "x=";
	std::cin >> x;
	std::cout << "y=";
	std::cin >> y;
	std::cout << "z=";
	std::cin >> z;

	if (solve(x, y, z, angles)) {
		m_angle.data[3] = angles[0] * 180 / PI;
		m_angle.data[1] = angles[1] * 180 / PI;
		m_angle.data[2] = angles[2] * 180 / PI;

		m_angleOut.write();

		std::cout << m_angle.data[0] * 180 / PI << std::endl;
		std::cout << m_angle.data[1] * 180 / PI<< std::endl;
		std::cout << m_angle.data[2] * 180 / PI << std::endl;
	}

    return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t EEZYbotARMControlRTC::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t EEZYbotARMControlRTC::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t EEZYbotARMControlRTC::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t EEZYbotARMControlRTC::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t EEZYbotARMControlRTC::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void EEZYbotARMControlRTCInit(RTC::Manager* manager)
  {
    coil::Properties profile(eezybotarmcontrolrtc_spec);
    manager->registerFactory(profile,
                             RTC::Create<EEZYbotARMControlRTC>,
                             RTC::Delete<EEZYbotARMControlRTC>);
  }
  
};


