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

#ifndef _RTDB_STRUCTS_H
#define _RTDB_STRUCTS_H

/* Structure to store an agent */
typedef struct
{
	unsigned num;	// Unique agent identification number (0-n)
	char* id;	// Agent name (alfanum)
} rtdb_Agent;

/* Structure to store a list of agents */
typedef struct
{
	rtdb_Agent* agents;	// Dynamic agent list 
	unsigned numAg; 	// Total number of agents in list
} rtdb_AgentList;

/* Structure to store an item */
typedef struct
{
	unsigned num;		// Unique item identification number (0-n)
	char* id;		// Item name (alfanum)
	char* datatype;		// C datatype identifier (alfanum)
	char* headerfile;	// C headerfile name where the datatype is declared (alfanum)
	unsigned period;	// Broadcasting period of DB item (1-4)
} rtdb_Item;

/* Structure to store an items list */
typedef struct
{
	rtdb_Item* items;	// Dynamic item list
	unsigned numIt;		// Total number os items in list
} rtdb_ItemList;

/* Structure to store a schema */
typedef struct
{
	char* id;			// Schema name (alfanum)
	rtdb_ItemList sharedItems;	// Shared items list in schema
	rtdb_ItemList localItems;	// Local items list in schema
} rtdb_Schema;

/* Structure to store a schemas list */
typedef struct
{
	rtdb_Schema* schemas;		// List of Schemas
	unsigned numSc;			// Total number of schemas in list
} rtdb_SchemaList;

/* Structure to store an assignment of items to agents */
typedef struct
{
	rtdb_Schema* schema;		// Schema to be assigned
	rtdb_AgentList agentList;	// Agents to use the schema
} rtdb_Assignment;

/* List of assignments of items to agents */
typedef struct 
{
	rtdb_Assignment* asList;	// List of assignments
	unsigned numAs;			// Total number os assignments in list
} rtdb_AssignmentList;


#endif

/* EOF: rtdb_structs.h */
