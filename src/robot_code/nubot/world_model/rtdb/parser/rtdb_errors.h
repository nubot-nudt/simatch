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

#ifndef _RTDB_ERRORS_H_
#define _RTDB_ERRORS_H_

/* Parsing Error messages definition */
#define _ERR_LEX_ "Erro Lexical! Sequência de caracteres inválida!"
#define _ERR_INITIAL_ "À espera de uma declaração de um tipo válido!"
#define _ERR_AGENTS_ "Agentes mal declarados! À espera de uma lista de agntes válida!"
#define _ERR_ITEMOPEN_ "Item mal declarado! À espera de \"{\""
#define _ERR_ITEMFIELD_ "Item mal declarado! À espera de \"datatype =\" id, \"period =\" num, \"headerfile =\" ficheiro.h ou \"}\""
#define _ERR_ITEMAFTERFIELD_ "Item mal declarado! À espera de \";\", fim de linha ou \"}\""
#define _ERR_SCHEMAOPEN_ "Esquema mal declarado! À espera de \"{\""
#define _ERR_SCHEMAFIELD_ "Esquema mal declarado! À espera de \"shared =\" ListaItems, \"local =\" ListaItems ou \"}\""
#define _ERR_ITEMSLIST_ "Esquema mal declarado! À espera de uma lista de items válida!"
#define _ERR_ASSIGNMENTOPEN_ "Atribuição mal declarada! À espera de \"{\""
#define _ERR_ASSIGNMENT_ "Atribuição mal declarada! À espera de \"schema =\", \"agents =\" ou \"}\""
#define _ERR_AGENTSLIST_ "Atribuição mal declarada! À espera de uma lista de agentes válida!"


/* Parsing Errors handling function */
void raiseError (unsigned, char*);

/* Structure definitions Errors handling function */
void abortOnError(char*);

#endif

/* EOF: rtdb_errors.h */
