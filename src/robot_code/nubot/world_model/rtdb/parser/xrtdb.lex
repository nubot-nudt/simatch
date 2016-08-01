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
%{
#include <string.h>
#include "rtdb_errors.h"
#include "xrtdb.tab.h"
#define YY_DECL \
        int yylex(char **p)

//Variable to track the line number for error messages
unsigned lineNumber= 1;
%}

%option noyywrap never-interactive pointer nounput

%x COMMENT

id [_a-zA-Z][_a-zA-Z0-9]*
headerfl {id}\.h
blanks [ \t]*
digit [0-9]
number {digit}+

%%
<INITIAL>{
	{blanks}        /* Ignore white space */
        \n	        { lineNumber++; return eol; }
	#	        { BEGIN(COMMENT); }
        =               { return equal; }
        ;               { return semicomma; }
        ,               { return comma; }
        \{              { return openbrace; }
        \}              { return closebrace; }
	AGENTS	        { return agentsDECL; }
        ITEM            { return itemDECL; }
	SCHEMA	        { return schemaDECL; }
	ASSIGNMENT	{ return assignmentDECL; }
        datatype        { return datatypeFIELD; }
        period          { return periodFIELD; }
        headerfile      { return headerfileFIELD; }
        shared          { return sharedFIELD; }
        local           { return localFIELD; }
        schema          { return schemaFIELD; }
        agents          { return agentsFIELD; }
        {id}            { *p= strdup(yytext); return identifier; }
        {headerfl}      { *p= strdup(yytext); return headerfl; }
        {number}        { *p= strdup(yytext); return integer; }
	.		{ raiseError(lineNumber, _ERR_LEX_); }
}
<COMMENT>{
	\n	{ lineNumber++; BEGIN(INITIAL); return eol; }
	.	/* Ignoring all chars except \n */
}

<*><<EOF>>	{ return eof; }
%%

/* EOF: xrtdb.lex */
