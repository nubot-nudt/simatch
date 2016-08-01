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


#ifndef __RTDB_COMM_H
#define __RTDB_COMM_H

#include "rtdb_api.h"


//	*************************
//	DB_comm_put: Escreve na base de dados - apenas para outros agentes!
//
//	Entrada:
//		int _agent = numero do agente
//		int _id = identificador da 'variavel'
//		void *_value = ponteiro com os dados
//		int life = tempo de vida da 'variavel' em ms
//	Saida:
//		0 = OK
//		-1 = erro
//
int DB_comm_put (int _agent, int _id, int _size, void *_value, int life);


//	*************************
//	DB_comm_ini: 
//
//	Entrada:
//		int _id = array com identificadores da 'variavel'
//		int _size = array com tamanhos da 'variavel'
//		int _period = array com periodos da 'variavel'
//	Saida:
//		int n_shared_recs = numero de 'variaveis' shared
//		-1 = erro
//
int DB_comm_ini(RTDBconf_var *rec);


#endif
