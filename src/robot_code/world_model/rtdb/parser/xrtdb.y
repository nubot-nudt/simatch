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

%pure_parser
%start S
%token agentsDECL itemDECL schemaDECL assignmentDECL                                            /* Config file items keywords */
%token datatypeFIELD periodFIELD headerfileFIELD sharedFIELD localFIELD schemaFIELD agentsFIELD /* Config file fields keywords */ 
%token identifier headerfl integer                                                            /* Identifiers */
%token equal semicomma comma openbrace closebrace eol eof                                     /* Symbols */

%{
#include <stdio.h>
#include <stdlib.h>
#include "rtdb_configuration.h"
#include "rtdb_errors.h"
#include "rtdb_structs.h"
#include "rtdb_functions.h"
#include "rtdb_user_creator.h"
#include "rtdb_ini_creator.h"
#define YYSTYPE char*
#define YYERROR_VERBOSE 1

//Lexical analizer function prototype
int yylex(char **);
//Bison Error handling function prototype
void yyerror(char *);

//Global var to store the current input file line number for error handling
unsigned nline= 1;

//Aux global vars to create the global lists defined in rtdb_functions.h
rtdb_Item * pItem;
rtdb_Schema * pSchema;
rtdb_Assignment * pAssign;
%}

%%
S :   INITIAL;

INITIAL:    eol { nline++; } INITIAL
          | agentsDECL equal AGENTS eol { nline++; } INITIAL
          | agentsDECL equal AGENTS semicomma eol { nline++; } INITIAL
          | itemDECL identifier { pItem= itemCreate($2); } ITEMOPEN INITIAL
          | schemaDECL identifier { pSchema= schemaCreate($2); } SCHEMAOPEN INITIAL
          | assignmentDECL ASSIGNMENTOPEN INITIAL
          | eof { return 0; }
          | error { raiseError(nline, _ERR_INITIAL_); }
          ;

AGENTS:     identifier { agentCreate($1); }
          | AGENTS comma identifier { agentCreate($3); }
          | error { raiseError(nline, _ERR_AGENTS_); }
          ;

ITEMOPEN:   eol { nline++; } ITEMOPEN
          | openbrace ITEM
          | error { raiseError(nline, _ERR_ITEMOPEN_); }
          ;

ITEM:       eol { nline++; } ITEM
          | datatypeFIELD equal identifier { itemAddDatatype(pItem, $3); } ITEMAFTERFIELD
          | periodFIELD equal identifier { itemAddPeriod(pItem, $3); } ITEMAFTERFIELD
          | headerfileFIELD equal headerfl { itemAddHeaderfile(pItem, $3); } ITEMAFTERFIELD
          | closebrace { itemVerify(pItem); }
          | error { raiseError(nline, _ERR_ITEMFIELD_); }
          ;

ITEMAFTERFIELD:   eol { nline++; } ITEM
                | semicomma ITEM
                | closebrace { itemVerify(pItem); }
                | error { raiseError(nline, _ERR_ITEMAFTERFIELD_); }
                ;

SCHEMAOPEN:   eol { nline++; } SCHEMAOPEN
            | openbrace SCHEMA
            | error { raiseError(nline, _ERR_SCHEMAOPEN_); }
            ;

SCHEMA:     eol { nline++; } SCHEMA
          | sharedFIELD equal SHAREDITEMS semicomma SCHEMA
          | sharedFIELD equal SHAREDITEMS SCHEMA
          | localFIELD equal LOCALITEMS semicomma SCHEMA
          | localFIELD equal LOCALITEMS SCHEMA
          | closebrace { schemaVerify(pSchema); }
          | error { raiseError(nline, _ERR_SCHEMAFIELD_); }
          ;

SHAREDITEMS:      identifier { schemaAddSharedItem(pSchema, $1); }
                | SHAREDITEMS comma opt_eol identifier { schemaAddSharedItem(pSchema, $4); }
                | error { raiseError(nline, _ERR_ITEMSLIST_); }
                ;

opt_eol		:	/* lambda */
			|	eol { nline++; }
			;

LOCALITEMS:       identifier { schemaAddLocalItem(pSchema, $1); }
                | LOCALITEMS comma opt_eol identifier { schemaAddLocalItem(pSchema, $4); }
                | error { raiseError(nline, _ERR_ITEMSLIST_); }
                ;

ASSIGNMENTOPEN:    eol { nline++; } ASSIGNMENTOPEN
                 | openbrace { pAssign= assignmentCreate(); } ASSIGNMENT
                 | error { raiseError(nline, _ERR_ASSIGNMENTOPEN_); }
                 ;

ASSIGNMENT:   eol { nline++; } ASSIGNMENT
            | schemaFIELD equal identifier { assignmentAddSchema(pAssign, $3); } semicomma ASSIGNMENT
            | schemaFIELD equal identifier { assignmentAddSchema(pAssign, $3); } ASSIGNMENT
            | agentsFIELD equal ASSIGNMENTAGENTS semicomma ASSIGNMENT
            | agentsFIELD equal ASSIGNMENTAGENTS ASSIGNMENT
            | closebrace { assignmentVerify(pAssign); }
            | error { raiseError(nline, _ERR_ASSIGNMENT_); }
            ;

ASSIGNMENTAGENTS:     identifier { assignmentAddAgent(pAssign, $1); }
                    | ASSIGNMENTAGENTS comma identifier { assignmentAddAgent(pAssign, $3); }
                    | error { raiseError(nline, _ERR_AGENTSLIST_); }
                    ;

%%


/* Main function -> Entry point for the program */
int main(int argc, char *argv[])
{
        //If the program wasn't correctly called, abort and teach correct usage
        //Include the reference to the file pointer used by flex
        extern FILE *yyin;

        if (argc != 2)
        {
				printf("*** Using default configuration file ***\n");
                printf("To use other configuration file: $ %s fich_conf\n", *argv);
				//If the input file doesn't exist or there's no reading permissions, abort
				if (!( (yyin= fopen(RTDB_CONF, "r")) != NULL ))
				{
						printf("Erro na abertura do ficheiro de configuração!\nNão foi possível abrir \"%s\" para leitura\n", argv[1]);
						return 1;
				}

        }
		else
		{
				//If the input file doesn't exist or there's no reading permissions, abort
				if (!( (yyin= fopen(argv[1], "r")) != NULL ))
				{
						printf("Erro na abertura do ficheiro de configuração!\nNão foi possível abrir \"%s\" para leitura\n", argv[1]);
						return 1;
				}
		}

        // Initialize global lists to store the declared agents, items, schemas and assignments
        agList.numAg= 0;
        agList.agents= malloc(sizeof(rtdb_Agent));

        itList.numIt= 0;
        itList.items= malloc(sizeof(rtdb_Item));

        schemaList.numSc= 0;
        schemaList.schemas= malloc(sizeof(rtdb_Schema));

        assignList.numAs= 0;
        assignList.asList= malloc(sizeof(rtdb_Assignment));

        //Parse the rtdb.conf file
        if (yyparse() == 0)
        {
			if(argc == 2)		
                printf("\nFicheiro \e[33m%s\e[0m lido com sucesso!\n", argv[1]);
			else
                printf("\nFicheiro \e[33m%s\e[0m lido com sucesso!\n", RTDB_CONF);
        }
        else {
                printf("\n\e[33mERRO\e[0m a ler o ficheiro\e[32m%s\e[0m!\n", argv[1]);
                return 1;
             }

        //If in Debug mode, print detailed information about the data stored on memory after reading the input file
        if (DEBUG)
        {
                unsigned i, j;

                printf("\n\n\e[32mA imprimir informação de Debugging detalhada sobre os dados guardados\n   em memória após a leitura do ficheiro \e[33m%s\e[0m\e[32m.\e[0m\n\n", argv[1]);

                //Print Agents List
                printf("\n\e[33mA imprimir a lista de \e[32m%u\e[33m agentes:\e[0m", agList.numAg);
                for (i= 0; i < agList.numAg; i++)
                {
                        printf("\n\e[32mAgent:\e[0m %u: \e[32mID:\e[0m %s", agList.agents[i].num, agList.agents[i].id);
                }
                printf("\n\e[33mFim da lista de agentes.\e[0m\n");

                //Print Items list
                printf("\n\e[33mA imprimir a lista de \e[32m%u\e[33m items:\e[0m", itList.numIt);
                for (i= 0; i < itList.numIt; i++)
                {
                        printf("\n\e[32mItem\e[0m %u: \e[32mID:\e[0m %s \e[32mDatatype:\e[0m %s \e[32mHeaderfile:\e[0m %s \e[32mPeriod:\e[0m %u", itList.items[i].num, itList.items[i].id, itList.items[i].datatype, itList.items[i].headerfile, itList.items[i].period);
                }
                printf("\n\e[33mFim da lista de items.\e[0m\n");

                //Print Schemas List
                printf("\n\e[33mA imprimir a lista de \e[32m%u\e[33m Esquemas:\e[0m", schemaList.numSc);
                for (i= 0; i < schemaList.numSc; i++)
                {
                        printf("\n\e[33mA imprimir lista de \e[32m%u\e[33m sharedItems do esquema \e[32m%s\e[33m na posicao \e[32m%u\e[33m da lista de esquemas:\e[0m", schemaList.schemas[i].sharedItems.numIt, schemaList.schemas[i].id, i);
                        //Print shared items list
                        for (j= 0; j < schemaList.schemas[i].sharedItems.numIt; j++)
                        {
                                printf("\n\e[32mItem\e[0m %u: \e[32mID:\e[0m %s \e[32mDatatype:\e[0m %s \e[32mHeaderfile:\e[0m %s \e[32mPeriod:\e[0m %u", schemaList.schemas[i].sharedItems.items[j].num, schemaList.schemas[i].sharedItems.items[j].id, schemaList.schemas[i].sharedItems.items[j].datatype, schemaList.schemas[i].sharedItems.items[j].headerfile, schemaList.schemas[i].sharedItems.items[j].period);
                        }
                        printf("\n\e[33mA imprimir lista de \e[32m%u\e[33m local Items do esquema \e[32m%s\e[33m na posicao \e[32m%u\e[33m da lista de esquemas:\e[0m", schemaList.schemas[i].localItems.numIt, schemaList.schemas[i].id, i);
                        //Print local items list
                        for (j= 0; j < schemaList.schemas[i].localItems.numIt; j++)
                        {
                                printf("\n\e[32mItem\e[0m %u: \e[32mID:\e[0m %s \e[32mDatatype:\e[0m %s \e[32mHeaderfile:\e[0m %s \e[32mPeriod:\e[0m %u", schemaList.schemas[i].localItems.items[j].num, schemaList.schemas[i].localItems.items[j].id, schemaList.schemas[i].localItems.items[j].datatype, schemaList.schemas[i].localItems.items[j].headerfile, schemaList.schemas[i].localItems.items[j].period);
                        }
                }
                printf("\n\e[33mFim da lista de Esquemas.\e[0m\n");

                //Print Assignments list        
                printf("\n\e[33mA imprimir lista de \e[32m%u\e[33m Assignments:\e[0m", assignList.numAs);
                for (i= 0; i < assignList.numAs; i++)
                {
                        printf("\n\e[33mO assignment na posição \e[32m%u\e[33m da lista tem o esquema \e[32m%s\e[0m\e[0m", i, assignList.asList[i].schema->id);
                        printf("\n\e[33mA imprimir lista de \e[32m%u\e[33m agentes do assignment:\e[0m", assignList.asList[i].agentList.numAg);
                        for (j= 0; j < assignList.asList[i].agentList.numAg; j++)
                        {
                                printf("\n\e[32mIndex:\e[0m %u \e[32mAgente: \e[0m%u\e[32m ID:\e[0m %s", j, assignList.asList[i].agentList.agents[j].num, assignList.asList[i].agentList.agents[j].id);
                        }
                        printf("\n");
                }
                printf("\e[33mFim da lista de Assignments.\e[0m\n");

                printf("\n\n\e[32mFIM do dump das estruturas guardadas em memória!\e[0m\n\n");

        }

        //Vars to store return values of printUserFile() and printIniFile()
        int userFileStatus, iniFileStatus; 

        printf("\nA criar o ficheiro \e[32mrtdb_user.h\e[0m\n");

        //Call printUserFile() to generate the rtdb_user.h file
        userFileStatus= printUserFile(itList, agList);

        //Check return value of printUserFile() and output according message
        switch (userFileStatus)
        {
                case 0: {
                        printf("\nFicheiro \e[32mrtdb_user.h\e[0m criado com sucesso!\n");
                        break;
                        }
                case 1: {
                        printf("\nCriação do ficheiro \e[32mrtdb_user.h\e[0m cancelada pelo utilizador.\n");
                        break;
                        }
                case 2: {
                        printf("\n\e[33mERRO\e[0m a criar o ficheiro \e[32mrtdb_user.h\e[0m. Não foi possível abrir o ficheiro para escrita.\n");
                        break;
                        }
                default: printf("\n\e[33mERRO\e[0m inesperado a criar o ficheiro \e[32mrtdb_user.h\e[0m!\n");
        }

        //Check if there are assignments defined
        if (assignList.numAs == 0)
        {
                abortOnError("Não é possível escrever o ficheiro \e[32mrtdb.ini\e[0m!\nNão foi definido nenhum \e[33mAssignment\e[0m no ficheiro de configuração!");
        }

        printf("\nA criar o ficheiro \e[32mrtdb.ini\e[0m\n");

        //Call printIniFile() to generate the rtdb.ini file
        iniFileStatus= printIniFile(agList, assignList);

        //Check return value of printIniFile() and output according message
        switch (iniFileStatus)
        {
                case 0: {
                        printf("\nFicheiro \e[32mrtdb.ini\e[0m criado com sucesso!\n");
                        break;
                        }
                case 1: {
                        printf("\nCriação do ficheiro \e[32mrtdb.ini\e[0m cancelada pelo utilizador.\n");
                        break;
                        }
                case 2: {
                        printf("\n\e[33mERRO\e[0m a criar o ficheiro \e[32mrtdb.ini\e[0m. Não foi possível abrir o ficheiro para escrita.\n");
                        break;
                        }
                default: printf("\n\e[33mERRO\e[0m inesperado a criar o ficheiro \e[32mrtdb.ini\e[0m!\n");
        }

        //Free the dynamically allocated vars
        free(agList.agents); free(itList.items);
        free(schemaList.schemas); free(assignList.asList);

        //Free the file pointer
        free(yyin);

        //End program returning 0, meaning everything is OK!
        return 0;
}

/* Bison built-in error handling function */
void yyerror(char* s)
{
    raiseError(nline, s);
}

/* EOF: xrtdb.y */
