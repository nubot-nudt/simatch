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
#include <malloc.h>
#include "rtdb_functions.h"
#include "rtdb_errors.h"



/**************  Agents manipulation Function  **************/

//Add an agent to the agents global list if it wasn't previouly declared
void agentCreate(char* newId)
{	
	unsigned i; 
	//Run through the agents list
	for(i = 0; i < agList.numAg; i++)
	{
		//If the agent is found, notify the user and abort
		if (strcmp(agList.agents[i].id, newId) == 0 ){
			char * err= malloc((1+strlen("O agente \e[33m")+strlen(newId)+strlen("\e[0m já foi declarado!")) * sizeof(char));
			sprintf(err, "O agente \e[33m%s\e[0m já foi declarado!", newId);
			abortOnError(err);
		}
	}
	//Add the agent to the list
	agList.agents[agList.numAg].num = agList.numAg;
	agList.agents[agList.numAg].id = strdup(newId);	
	//Increment the list counter
	agList.numAg++;
	//Reallocate memory to the list	
	agList.agents = realloc(agList.agents, (agList.numAg+1)*sizeof(rtdb_Agent));
	//If in Debug mode print a message with info about the agent added
	if (DEBUG)
	{
		printf("\nAgente \e[32m%s\e[0m adicionado com sucesso à lista de \e[32m%u\e[0m Agentes na posição \e[32m%u\e[0m!\n", agList.agents[agList.numAg-1].id, agList.numAg, agList.numAg-1); 
	}
}

/**************  Items manipulation Functions  **************/

//Add an item to the items global list if it wasn't previouly declared
rtdb_Item* itemCreate(char* newId)
{
	unsigned i; 
	//Run through the items list
	for(i = 0; i < itList.numIt; i++)
	{
		//If the item is found, notify the user and abort
		if (strcmp(itList.items[i].id, newId) == 0)
		{
			char * err= malloc((1+strlen("O item \e[33m")+ strlen(newId)+strlen("\e[0m já foi declarado!")) * sizeof(char));
			sprintf(err, "O item \e[33m%s\e[0m já foi declarado!", newId);
			abortOnError(err);
		}
	}
	//Add the item to the list
	itList.items[itList.numIt].num = itList.numIt;
	itList.items[itList.numIt].id = strdup(newId);
	itList.items[itList.numIt].datatype = strdup("\0");
	itList.items[itList.numIt].headerfile = strdup("\0");
	itList.items[itList.numIt].period = 0;
	//Increment the list counter
	itList.numIt++;
	
	//Reallocate memory to the list
	itList.items = realloc(itList.items, (itList.numIt+1)*sizeof(rtdb_Item));

	//If in Debug mode print info about the item just added
	if (DEBUG)
	{
		printf("\nItem \e[32m%s\e[0m adicionado com sucesso à lista de \e[32m%u\e[0m Items na posição \e[32m%u\e[0m!\n", itList.items[itList.numIt-1].id, itList.numIt, itList.numIt-1); 
	}
	
	//Return a pointer to the item
	return &itList.items[itList.numIt-1];
}

//Define a datatype in the item specified if it wasn't previouly defined
void itemAddDatatype(rtdb_Item* it, char* dt)
{
	//Check if the datatype wasn't already defined
	if (strcmp(it->datatype, "\0") == 0)
	{
		it->datatype = strdup(dt);
		//If in Debug mode tell the user that the datatype was correctly defined
		if (DEBUG)
		{
			printf("\nDatatype \e[32m%s\e[0m definido com sucesso no item \e[32m%s\e[0m\n", dt, it->id); 
		}
	}
	else
	{
		//If it was previsouly defined abort
		char * err= malloc((1+strlen("Tentativa de definição do datatype \e[33m")+strlen(dt)+strlen("\e[0m no item \e[33m")+strlen(it->id)+strlen("\e[0m já com o datatype \e[33m")+strlen(it->datatype)+strlen("\e[0m definido!")) * sizeof(char));
		sprintf(err, "Tentativa de definição do datatype \e[33m%s\e[0m no item \e[33m%s\e[0m já com o datatype \e[33m%s\e[0m definido!", dt, it->id, it->datatype);
		abortOnError(err);
	}
}

//Define a period in the item specified if it wasn't previouly defined
void itemAddPeriod(rtdb_Item* it, char* per)
{
	//Convert the string of the period to an unsigned
	unsigned p= ((unsigned)atoi(per));
	//Check if the period wasn't alrealy defined
	if (it->period == 0)
	{
		it->period = p;
		//If in Debug mode tell the user that the period was correctly defined
		if (DEBUG)
		{
			printf("\nPeriodo \e[32m%u\e[0m definido com sucesso no item \e[32m%s\e[0m\n", p, it->id); 
		}
	}
	else
	{
		//If it was already defined abort
		char strperiod[10];
		sprintf(strperiod, "%u", it->period);		
		char * err= malloc((1+strlen("Tentativa de definição do period \e[33m")+strlen(per)+strlen("\e[0m no item \e[33m")+strlen(it->id)+strlen("\e[0m já com o period \e[33m")+strlen(strperiod)+strlen("\e[0m definido!")) * sizeof(char));
		sprintf(err, "Tentativa de definição do period \e[33m%s\e[0m no item \e[33m%s\e[0m já com o period \e[33m%s\e[0m definido!", per, it->id, strperiod);
		abortOnError(err);
	}
}

//Define an headerfile in the item specified if it wasn't previouly defined
void itemAddHeaderfile(rtdb_Item* it, char* hdf)
{
	//Check if the headerfile wasn't alrealy defined
	if (strcmp(it->headerfile, "\0") == 0)
	{
		it->headerfile = strdup(hdf);
		//If in Debug mode tell the user that the headerfile was correctly defined
		if (DEBUG)
		{
			printf("\nHeaderfile \e[32m%s\e[0m definido com sucesso no item \e[32m%s\e[0m\n", hdf, it->id); 
		}
	}
	else
	{
		//If it was already defined abort
		char * err= malloc((1+strlen("Tentativa de definição do headerfile \e[33m")+ strlen(hdf)+strlen("\e[0m no item \e[33m")+strlen(it->id)+strlen("\e[0m já com o headerfile \e[33m")+strlen(it->headerfile)+strlen("\e[0m definido!")) * sizeof(char));
		sprintf(err, "Tentativa de definição do headerfile \e[33m%s\e[0m no item \e[33m%s\e[0m já com o headerfile \e[33m%s\e[0m definido!", hdf, it->id, it->headerfile);
		abortOnError(err);
	}
}

//Check if the item is well defined, all fields non specified, with default fields, are defined here
void itemVerify(rtdb_Item* it)
 {
	//Check that the datatype is defined
	if (strcmp(it->datatype, "\0") == 0)
	{	
		char * err= malloc((1+strlen("Não foi definido um \e[33mdatatype\e[0m no item \e[32m")+strlen(it->id)+strlen("\e[0m!")) * sizeof(char));
		sprintf(err, "Não foi definido um \e[33mdatatype\e[0m no item \e[32m%s\e[0m!", it->id);
		abortOnError(err);
	}
	//If the headerfile isn't defined, assign it the default value
	if (strcmp(it->headerfile, "\0") == 0)
	{		 
		it->headerfile = strdup(it->datatype);
		it->headerfile= realloc(it->headerfile, (strlen(it->headerfile)+2)*sizeof(char));	
		strcat(it->headerfile, ".h");
		//If in Debug mode tell the user that it wasn't defined an headerfile, and that the default value will be used
		if (DEBUG)
		{
			printf("\nNão foi definido headerfile no item \e[32m%s\e[0m! A atribuir default: \e[32m%s\e[0m\n", it->id, it->headerfile);
		}
	}
	//If the period isn't defined, assign it the default value
	if (it->period == 0)
	{
		it->period = 1;
		//If in Debug mode tell the user that it wasn't defined a period, and that the default value will be used
		if (DEBUG)
		{
			printf("\nNão foi definido period no item \e[32m%s\e[0m! A atribuir default: \e[32m%u\e[0m\n", it->id, it->period);
		}
	}
	//If in Debug mode tell the user that the item was correctly defined
	if (DEBUG)
	{
		printf("\nO item \e[32m%s\e[0m foi verificado com sucesso!\n", it->id);
	}
}

/**************  Schemas manipulation Functions  **************/

//Add a schema to the schemas global list (if it was previouly declared, return a pointer to it)
rtdb_Schema* schemaCreate(char* newId)
{
	unsigned i; 
	//Run through the schemas list
	for(i = 0; i < schemaList.numSc; i++)
	{
		//If the schema was previouly defined return a pointer to it
		if (strcmp(schemaList.schemas[i].id, newId) == 0)
		{
			//If in Debug mode tell the user about the schema redefinition
			if (DEBUG)
			{
				printf("\nJá foi definido o esquema \e[32m%s\e[0m na posição \e[32m%u\e[0m na lista de \e[32m%u\e[0m esquemas. A adicionar os items declarados ao esquema previamente declarado!\n", newId, i, schemaList.numSc); 
			}
			return &schemaList.schemas[i];
		}
	}
	//Add the schema to the schemas list
	schemaList.schemas[schemaList.numSc].id = strdup(newId);
	//Initialize the shared items list	
	schemaList.schemas[schemaList.numSc].sharedItems.numIt = 0;
	schemaList.schemas[schemaList.numSc].sharedItems.items = malloc(sizeof(rtdb_Item));
	//Initialize the local items list
	schemaList.schemas[schemaList.numSc].localItems.numIt = 0;
	schemaList.schemas[schemaList.numSc].localItems.items = malloc(sizeof(rtdb_Item));
	//Increment the schemas list counter
	schemaList.numSc++;
	//Reallocate memory for the schemas list
	schemaList.schemas = realloc(schemaList.schemas, (schemaList.numSc+1)*sizeof(rtdb_Schema) );
	
	//If in Debug mode tell the user the schema was added to the list
	if (DEBUG)
	{
		printf("\nEsquema \e[32m%s\e[0m adicionado com sucesso na posicao \e[32m%u\e[0m da lista de \e[32m%u\e[0m esquemas!\n", newId, schemaList.numSc-1, schemaList.numSc);
	}
	
	//Return a pointer to the schema
	return &schemaList.schemas[schemaList.numSc-1];
}

//Add an item to the shared items list in the schema specified if it wasn't previouly defined
void schemaAddSharedItem(rtdb_Schema* sc, char* itemId)
{
	unsigned i = 0;
	unsigned itemIndex = 0;
	unsigned itemExists = 0;

	//Check if the items list isn't empty
	if (itList.numIt == 0)
	{
		char * err= malloc((1+strlen("Tentativa de adicionar o item \e[32m")+strlen(itemId)+strlen("\e[0m inexistente \e[33m(A lista de items está vazia)\e[0m ao esquema \e[32m")+strlen(sc->id)+strlen("\e[0m!")) * sizeof(char));
		sprintf(err, "Tentativa de adicionar o item \e[32m%s\e[0m inexistente \e[33m(A lista de items está vazia)\e[0m ao esquema \e[32m%s\e[0m!", itemId, sc->id);
		abortOnError(err);
	}
	else
	{
		//Check if the items list isn't empty
		for(i = 0; i < itList.numIt; i++)
		{
			//If the items exists store it's position and define existance flag
			if (strcmp(itList.items[i].id, itemId) == 0)
			{
				itemIndex= i;
				itemExists= 1;
				break;
			}
			else
			{
				itemExists= 0;				
			}
		}
	}
	//If the item doesn't exist abort
	if (!itemExists) {
		char * err= malloc((1+strlen("\nTentativa de adicionar o item \e[32m")+strlen(itemId)+strlen("\e[0m \e[33minexistente na lista de items\e[0m ao esquema \e[32m")+strlen(sc->id)+strlen("\e[0m!")) * sizeof(char));
		sprintf(err, "\nTentativa de adicionar o item \e[32m%s\e[0m \e[33minexistente na lista de items\e[0m ao esquema \e[32m%s\e[0m!", itemId, sc->id);
		abortOnError(err);
	}
	//If the shared items list is empty initialize it with the item
	if (sc->sharedItems.numIt == 0)
	{
		//If in Debug mode inform the user about the list initialization
		if (DEBUG)
		{
			printf("\nA inicializar a lista de items partilhados do esquema \e[32m%s\e[0m com o item \e[32m%s\e[0m na posicao \e[32m%u\e[0m\n", sc->id, itList.items[itemIndex].id, sc->sharedItems.numIt);
		}
		sc->sharedItems.items[sc->sharedItems.numIt++] = itList.items[itemIndex];
	}
	else
	{
		//If the list isn't empty run the items list
		for(i = 0; i < sc->sharedItems.numIt; i++)
		{
			//If the item already exists in the list ignore the addition
			if ( strcmp(sc->sharedItems.items[i].id, itemId) == 0 )
			{
				//If in Debug mode inform the user about the item existance
				if (DEBUG)
				{
					printf("\nO item \e[32m%s\e[0m já existe na lista de items partilhados do esquema \e[32m%s\e[0m na posicao \e[32m%u\e[0m. A ignorar re-declaração!\n", itemId, sc->id, i);
				}
				return;
			}	//If the item doesn't already exist in the shared items list of the schema add it
			else if (i == sc->sharedItems.numIt-1) 
				{
				//If in Debug mode inform the user that the item was correctly added
				if (DEBUG)
				{
					printf("\nA adicionar item \e[32m%s\e[0m na posicao \e[32m%u\e[0m da lista de \e[32m%u\e[0m items partilhados do esquema \e[32m%s\e[0m\n", itList.items[itemIndex].id, sc->sharedItems.numIt, sc->sharedItems.numIt+1, sc->id);
				}
				sc->sharedItems.items[sc->sharedItems.numIt++] = itList.items[itemIndex];
				break;
				}
		}
	}
	//Reallocate memory for the shared items list
	sc->sharedItems.items = realloc(sc->sharedItems.items, (sc->sharedItems.numIt+1)*sizeof(rtdb_Item));
}

//Add an item to the local items list in the schema specified if it wasn't previouly defined
void schemaAddLocalItem(rtdb_Schema* sc, char* itemId)
{
	unsigned i = 0;
	unsigned itemIndex = 0;
	unsigned itemExists = 0;

	//Check if the items list isn't empty
	if (itList.numIt == 0)
	{
		char * err= malloc((1+strlen("Tentativa de adicionar o item \e[32m")+strlen(itemId)+strlen("\e[0m inexistente \e[33m(A lista de items está vazia)\e[0m ao esquema \e[32m")+strlen(sc->id)+strlen("\e[0m!")) * sizeof(char));
		sprintf(err, "Tentativa de adicionar o item \e[32m%s\e[0m inexistente \e[33m(A lista de items está vazia)\e[0m ao esquema \e[32m%s\e[0m!", itemId, sc->id);
		abortOnError(err);
	}
	else
	{
		//Check if the items list isn't empty
		for(i = 0; i < itList.numIt; i++)
		{
			//If the items exists store it's position and define existance flag
			if (strcmp(itList.items[i].id, itemId) == 0)
			{
				itemIndex= i;
				itemExists= 1;
				break;
			}
			else
			{
				itemExists= 0;				
			}
		}
	}
	//If the item doesn't exist abort
	if (!itemExists) {
		char * err= malloc((1+strlen("\nTentativa de adicionar o item \e[32m")+strlen(itemId)+strlen("\e[0m \e[33minexistente na lista de items\e[0m ao esquema \e[32m")+strlen(sc->id)+strlen("\e[0m!")) * sizeof(char));
		sprintf(err, "\nTentativa de adicionar o item \e[32m%s\e[0m \e[33minexistente na lista de items\e[0m ao esquema \e[32m%s\e[0m!", itemId, sc->id);
		abortOnError(err);
	}
	//If the local items list is empty initialize it with the item
	if (sc->localItems.numIt == 0)
	{
		//If in Debug mode inform the user about the list initialization
		if (DEBUG)
		{
			printf("\nA inicializar a lista de items locais do esquema \e[32m%s\e[0m com o item \e[32m%s\e[0m na posicao \e[32m%u\e[0m\n", sc->id, itList.items[itemIndex].id, sc->localItems.numIt);
		}
		sc->localItems.items[sc->localItems.numIt++] = itList.items[itemIndex];
	}
	else
	{
		//If the list isn't empty run the items list
		for(i = 0; i < sc->localItems.numIt; i++)
		{
			//If the item already exists in the list ignore the addition
			if ( strcmp(sc->localItems.items[i].id, itemId) == 0 )
			{
				//If in Debug mode inform the user about the item existance
				if (DEBUG)
				{
					printf("\nO item \e[32m%s\e[0m já existe na lista de items locais do esquema \e[32m%s\e[0m na posicao \e[32m%u\e[0m. A ignorar re-declaração!\n", itemId, sc->id, i);
				}
				return;
			}
			else if (i == sc->localItems.numIt-1) 
				{
				if (DEBUG)
				{
					printf("\nA adicionar item \e[32m%s\e[0m na posicao \e[32m%u\e[0m da lista de \e[32m%u\e[0m items locais do esquema \e[32m%s\e[0m\n", itList.items[itemIndex].id, sc->localItems.numIt, sc->localItems.numIt+1, sc->id);
				}
				sc->localItems.items[sc->localItems.numIt++] = itList.items[itemIndex];
				break;
				}
		}
	}
	//Reallocate memory for the local items list
	sc->localItems.items = realloc(sc->localItems.items, (sc->localItems.numIt+1)*sizeof(rtdb_Item));
}

//Check if the schema is well defined (if both items lists aren't empty)
void schemaVerify(rtdb_Schema* sc)
{
	//Check if both items lists aren't empty
	if ((sc->sharedItems.numIt == 0) && (sc->localItems.numIt == 0))
	{
		char * err= malloc((1+strlen("O esquema \e[32m")+strlen(sc->id)+strlen("\e[0m tem ambas as listas, de items partilhados e locais, vazias!\n")) * sizeof(char));
		sprintf(err, "O esquema \e[32m%s\e[0m tem ambas as listas, de items partilhados e locais, vazias!\n", sc->id);
		abortOnError(err);
	}
	//If in Debug mode tell the user that the schema was correctly defined
	if (DEBUG)
	{
		printf("\nO esquema \e[32m%s\e[0m foi verificado com sucesso!\n", sc->id);
	}
}


/**************  Assignments manipulation Functions  **************/

//Add an assignment to the assignments global list
rtdb_Assignment* assignmentCreate(void)
{
	//Add the assignment to the list
	assignList.asList[assignList.numAs].schema= NULL;
	assignList.asList[assignList.numAs].agentList.numAg= 0;
	assignList.asList[assignList.numAs].agentList.agents= malloc(sizeof(rtdb_Agent));
	assignList.numAs++;
	assignList.asList= realloc(assignList.asList, (assignList.numAs+1) * sizeof(rtdb_Assignment));

	//If in Debug mode inform the user that the assignment was created
	if (DEBUG)
	{
		printf("\nAssignment criado com sucesso na posiçao \e[32m%u\e[0m da lista de \e[32m%u\e[0m assignments\n", assignList.numAs-1, assignList.numAs);
	}

	//Return a pointer to the assignment
	return &assignList.asList[assignList.numAs-1];
}

//Define a schema in the assignment specified if it wasn't previouly defined
void assignmentAddSchema(rtdb_Assignment* as, char* schemaId)
{
	unsigned i = 0;
	unsigned schemaExists = 0;

	//Check if the schema wasn't previously defined
	if (as->schema != NULL)
	{
		//If it was abort
		char * err= malloc((1+strlen("Tentativa de definição do esquema \e[33m")+strlen(schemaId)+strlen("\e[0m num assignment com o esquema \e[33m")+strlen(as->schema->id)+strlen("\e[0m previamente definido!\n")) * sizeof(char));
		sprintf(err, "Tentativa de definição do esquema \e[33m%s\e[0m num assignment com o esquema \e[33m%s\e[0m previamente definido!\n", schemaId, as->schema->id);
		abortOnError(err);
	}
	else {
		//Check if the schemas list isn't empty
		if (schemaList.numSc == 0)
		{
			char * err= malloc((1+strlen("Tentativa de definir o esquema \e[32m")+strlen(schemaId)+strlen("\e[0m inexistente \e[33m(A lista de esquemas está vazia)\e[0m no assignment!")) * sizeof(char));
			sprintf(err, "Tentativa de definir o esquema \e[32m%s\e[0m inexistente \e[33m(A lista de esquemas está vazia)\e[0m no assignment!", schemaId);
			abortOnError(err);
		} else	{
				//Run through the schemas list
				for(i = 0; i < schemaList.numSc; i++)
				{
					//If the schema is found connect it to the assignment
					if (strcmp(schemaList.schemas[i].id, schemaId) == 0)
					{
						//If in Debug mode tell the user about the schema definition in the assignment and set existance flag to true
						if (DEBUG)
						{
							printf("\nA atribuir o esquema \e[32m%s\e[0m ao assignment.\n", schemaList.schemas[i].id);
						}
						as->schema= malloc(sizeof(rtdb_Schema));
						as->schema= &schemaList.schemas[i];
						schemaExists = 1;
						break;
					}
					else {
						schemaExists = 0;
					}
				}
			}
			//If the schema doesn't exist abort
			if (!schemaExists)
			{
				char * err= malloc((1+strlen("Tentativa de definir o esquema \e[32m")+strlen(schemaId)+strlen("\e[0m \e[33minexistente\e[0m no assignment!")) * sizeof(char));
				sprintf(err, "Tentativa de definir o esquema \e[32m%s\e[0m \e[33minexistente\e[0m no assignment!", schemaId);
				abortOnError(err);
			} 
	}
}

//Add an agent to the items list of the assignment specified if it wasn't previouly defined
void assignmentAddAgent(rtdb_Assignment* as, char* agentId)
{
	unsigned i = 0;
	unsigned j = 0;
	unsigned agentExists = 0;
	
	//Check if the agent list isn't empty
	if (agList.numAg == 0)
		{
			char * err= malloc((1+strlen("Tentativa de adicionar o agente \e[32m")+strlen(agentId)+strlen("\e[0m inexistente \e[33m(A lista de agentes está vazia)\e[0m ao assignment!\n")) * sizeof(char));
			sprintf(err, "Tentativa de adicionar o agente \e[32m%s\e[0m inexistente \e[33m(A lista de agentes está vazia)\e[0m ao assignment!\n", agentId);
			abortOnError(err);
		} else {
				//Run though all the agents
				for(i= 0; i < agList.numAg; i++)
				{
					//If the agent isn't found set existance flag to false
					if (strcmp(agList.agents[i].id, agentId) != 0)
					{
						agentExists= 0;
					}
					else 	{
							//If the agent exists run through the assignment agent list
							for (j= 0; j < as->agentList.numAg; j++)
							{
								//If it was already added ignore redifinition
								if (strcmp(as->agentList.agents[j].id, agentId) == 0)
								{
									//If in Debug mode tell the user about the agent readdition to the assignment
									if (DEBUG)
									{
										printf("\nO agente \e[32m%s\e[0m já existe na lista de agentes do assignment na posicao \e[32m%u\e[0m. A ignorar re-declaração!\n", agentId, j);
									}
									return;
								}
							}
							//If in Debug mode tell the user about the agent addition to the assignment
							if (DEBUG)
							{
								printf("\nA adicionar agente \e[32m%s\e[0m na posicao \e[32m%u\e[0m da lista de \e[32m%u\e[0m agentes do assignment\n", agentId, as->agentList.numAg, as->agentList.numAg+1);
							}
							//Add the agent to the assignment
							as->agentList.agents[as->agentList.numAg++]= agList.agents[i];
							agentExists= 1;
							break; 
						}
				}
			}
			//If the agent doesn't exist abort
			if (!agentExists)
			{
				char * err= malloc((1+strlen("Tentativa de adicionar o agente \e[32m")+strlen(agentId)+strlen("\e[0m \e[33minexistente\e[0m no assignment!")) * sizeof(char));
				sprintf(err, "Tentativa de adicionar o agente \e[32m%s\e[0m \e[33minexistente\e[0m no assignment!", agentId);
				abortOnError(err);
			}
	//Reallocate memory to the agents list of the assignment
	as->agentList.agents= realloc(as->agentList.agents, (as->agentList.numAg+1) * sizeof(rtdb_Agent));
}

//Check if the assignment is well defined (if the agents list isn't empty and if the schema is defined)
void assignmentVerify(rtdb_Assignment* as)
{
	//If the assignment hasn't a schema defined abort
	if (as->schema == NULL)
	{
		abortOnError("Não foi definido nenhum \e[33mesquema\e[0m no \e[33massignment\e[0m!\n");
	}
	//If the agent list of the assignment is empty abort
	if (as->agentList.numAg == 0)
	{
		abortOnError("A lista de \e[33magentes\e[0m do \e[33massignment\e[0m está vazia!\n");
	}
	//If in Debug mode tell the user that the assignment was correctly defined
	if (DEBUG)
	{
		printf("\n\e[32mAssignment\e[0m verificado com sucesso\n");
	}
}


/* EOF: rtdb_functions.c */
