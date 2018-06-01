/*
 * packets.h
 *
 *      Author: bigaw
 */

#ifndef INCLUDE_PACKETS_H_
#define INCLUDE_PACKETS_H_

#include "eigenaux.h"
#include <boost/date_time/gregorian/gregorian.hpp>

class MagnCalibrator;

/*!
 * @brief GPS timestamp structure.
 */
struct Timestamp
{
    uint16_t year;		//!< Year.
    uint8_t month;		//!< Month (1-12).
    uint8_t day;		//!< Day (1-31).
    uint8_t hour;		//!< Hour (0-23).
    uint8_t minute;		//!< Minute (0-59).
    uint8_t second;		//!< Second (0-59).
    uint8_t ssecond;	//!< Sentisecond (0-99).
};

/*!
 * @brief Navigation data structure.
 */
struct Nav
{
    Timestamp time;			//!< Timestamp.
    Vector3D geo;			//!< Geodetic coordinates.
    Vector3D pos;			//!< ECEF position.
    Vector3D vel;			//!< ECEF velocity.
    double msl_altitude;	//!< Altitude above mean sea level.
    bool fix;				//!< True if position fix os ok.
    bool isnew;				//!< True if data updated since last refresh.
};

/*!
 * @brief Filter input packet.
 */
struct FilterInput
{
	Vector3D w;    				//!< Angular rate vector in dps.
	Vector3D a;    				//!< Accelerometer readings vector in g.
	Vector3D m;    				//!< Normalized magnetometer readings vector.
	Vector3D geo;  				//!< Geodetic coordinates vector ( latitude and longitude in deg, altitude above ellipsoid in m).
	Vector3D pos;  				//!< Position vector in cartesian coordinates in m.
	Vector3D v;    				//!< Velocity vector in cartesian coordinates in m/s.
	boost::gregorian::date day; //!< Current date.
	double dt;      			//!< Elapsed time in s.
	bool gps_valid;				//!< GPS measurements are valid flag.
};

/*!
 * @brief Raw data packet.
 */
struct RawPacket
{
    uint32_t pkt_number;        //!< Packet number.
    Nav gps_data;               //!< GPS sensor data.
	Vector3D w;					//!< Angular rate vector.
	Vector3D a;					//!< Acceleration vector.
	Vector3D m;					//!< Magnetic field vector.
	double et;					//!< Elapsed time.
};

struct FilteredPacket
{
    uint32_t status;                            //!< Status word.
    float ecef_x;                             	//!< ECEF 'x' coordinate.
    float ecef_y;                             	//!< ECEF 'y' coordinate.
    float ecef_z;                             	//!< ECEF 'z' coordinate.
    float gps_lat;                      		//!< GPS latitude.
    float gps_lon;                      		//!< GPS longitude.
    float gps_alt;                      		//!< GPS altitude.
    int32_t gspeed;                             //!< GPS ground speed.
    int32_t heading;                      		//!< Heading angle.
    int32_t pitch;                              //!< Pitch angle.
    int32_t roll;                               //!< Roll angle.
    int32_t track;                              //!< Track angle.
    uint32_t pkt_number;                        //!< Packet number.
};

// TODO move to receiver

/*!
 * @brief Convert raw input data to filter input structure.
 * @param in raw input data instance.
 * @return filter input instance.
 */
FilterInput parse_input(const RawPacket & in, const MagnCalibrator & magn_calib);

#endif /* INCLUDE_PACKETS_H_ */
