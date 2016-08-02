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
#include "rtdb_errors.h"

/* Parsing Errors handling function */
void raiseError (unsigned numln, char* msg)
{
	/* Output to the standard error stream (normally stdout) a message with an error message
	informing the error line number and an error description message */ 
	fprintf(stderr, "\n\e[33mErro\e[0m no ficheiro de configuração na \e[33mlinha %u\e[0m: \n\e[32m%s\e[0m\n", numln, msg);
	/* Abort the program execution */
	exit(2);
}

/* Structure definitions Errors handling function */
void abortOnError(char* errMsg)
{
	/* Output to the standard error stream (normally stdout) a message with an error message */
	fprintf(stderr, "\n\e[33mA abortar! Erro\e[0m: \e[32m%s\e[0m\n", errMsg);
	/* Abort the program execution */
	exit(3);
}

/* EOF: rtdb_errors.c */
