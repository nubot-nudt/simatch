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


#ifndef __RTDB_API_H
#define __RTDB_API_H

#include "rtdb_defs.h"

//	*************************
//	DB_init: Aloca acesso a base de dados
//
//	Saida:
//		0 = OK
//		-1 = erro
//
int DB_init (void);


//	*************************
//	DB_free: Liberta acesso a base de dados
//
void DB_free (void);


//	*************************
//	DB_put: Escreve na base de dados do proprio agente
//
//	Entrada:
//		int _id = identificador da 'variavel'
//		void *_value = ponteiro com os dados
//	Saida:
//		0 = OK
//		-1 = erro
//
int DB_put (int _id, void *_value);


//	*************************
//	DB_get: Le da base de dados
//
//	Entrada:
//		int _agent = numero do agente
//		int _id = identificador da 'variavel'
//		void *_value = ponteiro para onde sao copiados os dados
//	Saida:
//		int life = tempo de vida da 'variavel' em ms
//			-1 se erro
//
int DB_get (int _from_agent, int _id, void *_value);


//	*************************
//	Whoami: identifica o agente onde esta a correr
//
//	Saida:
//		int agent_number = numero do agente
//
int Whoami(void);


#endif
