/*
 * Copyright (C) 2009-2015,
 * Intelligent Robotics and Intelligent Systems (IRIS) Lab
 * CAMBADA robotic soccer team – http://robotica.ua.pt/CAMBADA/
 * University of Aveiro, Portugal
 *
 * This file is part of the CAMBADA RTDB
 *
 * CAMBADA RTDB is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CAMBADA RTDB is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _RTDB_INI_CREATOR_H_
#define _RTDB_USER_CREATOR_H_

#include "rtdb_structs.h"
#include "rtdb_configuration.h"

/* 
Function to write, compile and execute a rtdb_sizeof_tmp.c file
   that has included the headerfile where the datatype is defined
   and write a rtdb_size.tmp with the size of the datatype.
After that, the rtdb_size.tmp file is opened, the size of the
   datatype is read and returned 
*/
int getSizeof(char*, char*);

/*
Function to read the Global list of agents and the Global list
   of assignments and automatically generate a rtdb.ini file
*/
int printIniFile (rtdb_AgentList, rtdb_AssignmentList);

#endif

/* EOF: rtdb_ini_creator.h */
