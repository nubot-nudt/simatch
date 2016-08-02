/*
 * Frederico Miguel Santos - frederico.miguel.santos@gmail.com
 * CAMBADA robotic soccer team - www.ieeta.pt/atri/cambada
 * University of Aveiro
 * Copyright (C) 2009
 *
 * This file is part of RTDB middleware.
 * http://code.google.com/p/rtdb/
 *
 * RTDB middleware is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RTDB middleware is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RTDB middleware.  If not, see <http://www.gnu.org/licenses/>.
 */


#ifndef __RTDBDEFS_H
#define __RTDBDEFS_H

#include <string>
// #define DEBUG

#define SELF 3333

#define CONFIG_FILE  "/home/nubot9/nubot_ws/src/nubot/world_model/config/rtdb.ini"

#define SHMEM_KEY 0x2000
#define SHMEM_SECOND_TEAM_KEY 0x3000

// definicoes hard-coded
// alterar de acordo com a utilizacao pretendida

#define MAX_AGENTS 10	// numero maximo de agentes
#define MAX_RECS 100	// numero maximo de 'variaveis' (shared + local)

// fim das definicoes hard-coded

typedef struct
{
	int id;				// identificador da 'variavel'
	int size;			// tamanho de dados
	int period;			// periodicidade de refrescamento via wireless
} RTDBconf_var;

#endif
