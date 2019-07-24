/*
 * Copyright (C) 2009-2015,
 * Intelligent Robotics and Intelligent Systems (IRIS) Lab
 * CAMBADA robotic soccer team – http://robotica.ua.pt/CAMBADA/
 * University of Aveiro, Portugal
 *
 * This file is part of the CAMBADA UTILITIES
 *
 * CAMBADA UTILITIES is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CAMBADA UTILITIES is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef COMMON_H_
#define COMMON_H_

#define M_2PI	(M_PI * 2)

#define ENGAGED_THRESHOLD_HIGH	70
#define ENGAGED_THRESHOLD_LOW	35

/************* System Timer *************/
#define MOTION_TICK	20.0		/* Motion periodick tick ms */

/************* Holonomic Motion *************/
// Definition of the relation between max linear velocity and max angular velocity
#define MAX_VL			5.0							// Maximum linear velocity module for zero angular velocity
#define MAX_VA_RPM		90							// Maximum angular velocity module in RPM
#define MAX_VA			(MAX_VA_RPM * M_2PI / 60 )	// Maximum angular velocity module in rad/s
#define MIN_SLOPE		-(MAX_VL / MAX_VA)			// Module of the min slope of the ratio between lienar and angular velocities

namespace nubot{
}



#endif /* COMMON_H_ */
