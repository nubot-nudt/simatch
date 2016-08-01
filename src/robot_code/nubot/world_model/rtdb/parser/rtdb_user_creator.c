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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
//#include <malloc.h>
#include "rtdb_user_creator.h"

/*
Function to write the rtdb_user.h file using the Global agents list
  and the Global Items list
*/
int printUserFile(rtdb_ItemList it, rtdb_AgentList ag)
{
	unsigned i;
	FILE *f;
	char* command;

	//Open the file rtdb_user.h for reading to check if it exists
	f= fopen(RTDB_USER_H, "r");
 	if (f != NULL)
	{
		//If it exists close it and ask the user permission to overwrite it
		fclose(f);
		char op = '\0';
		while ((op != 'y') && (op != 'n'))
		{
			printf("\nO ficheiro \e[33m%s\e[0m ja existe!\nDeseja substitui-lo? (y/n): ", RTDB_USER_H);;
			assert(scanf("%c", &op) == 1);
			//Clean stdin
			purge();
		}
		if (op == 'n')
		{
			return 1;
 		}
	}


	//Remove the rtdb_user.h file in case it exists, if we have permission from the user
//	command= strdup(RM_COMMAND);
	command= malloc ((2 + strlen(RM_COMMAND)+strlen(RTDB_USER_H)) * sizeof(char));
	sprintf(command, "%s %s", RM_COMMAND, RTDB_USER_H);
	assert(system(command) != -1);
	free(command);

	//If for some reason we can't create the file, abort
	if ((f= fopen(RTDB_USER_H, "w")) == NULL)
	{
		return 2;
	}

	//Write generic information to the file
	fprintf(f, "/* AUTOGEN FILE : rtdb_user.h */\n\n");
	fprintf(f, "#ifndef _NUBOT_RTDB_USER_\n#define _NUBOT_RTDB_USER_\n\n");
	fprintf(f, "/* agents section */\n\n");
 
	//Run through the agents list and write all the defines
	for(i= 0; i < ag.numAg; i++)
	{
		fprintf(f, "#define %s\t%d\n", ag.agents[i].id, ag.agents[i].num);
	}

	//Write the total number of agents
	fprintf(f, "\n#define N_AGENTS\t%d\n\n", ag.numAg);

	//Write a generic comment
	fprintf(f, "/* items section */\n\n");

	//Run through the item list and write all the defines
	for(i= 0; i < it.numIt; i++)
	{
		fprintf(f, "#define %s\t%d\n", it.items[i].id, it.items[i].num);
	}

	//Write the total number of items
	fprintf(f, "\n#define N_ITEMS\t%d\n\n", it.numIt);
	
	//Write generic information to the file
	fprintf(f, "#endif\n\n");
	fprintf(f, "/* EOF : rtdb_user.h */\n");

	//Close the rtdb_user.h file
	fclose(f);

	//Return 0, meaning all went smoothly
	return 0;	
}

/* EOF: rtdb_user_creator.c */
