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


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <time.h>
#include <errno.h>


#include "rtdb/rtdb_defs.h"
#include "rtdb/rtdb_user.h"
#include <sstream>

//#define DEBUG

#define PERRNO(txt) \
	printf("ERROR: (%s / %s): " txt ": %s\n", __FILE__, __FUNCTION__, strerror(errno))

#define PERR(txt, par...) \
	printf("ERROR: (%s / %s): " txt "\n", __FILE__, __FUNCTION__, ## par)

#ifdef DEBUG
#define PDEBUG(txt, par...) \
	printf("DEBUG: (%s / %s): " txt "\n", __FILE__, __FUNCTION__, ## par)
#else
#define PDEBUG(txt, par...)
#endif


typedef struct
{
	int id;							// id da 'variavel'
	int size;						// sizeof da 'variavel'
	int period;						// refresh period for broadcast
	int offset;						// offset para o campo de dados da 'variavel'
	int read_bank;					// variavel mais actual
	struct timeval timestamp[2];	// relogio da maquina local
} TRec;


typedef struct
{
	int self_agent;						// numero do agente onde esta a correr
	int n_agents;						// numero total de agentes registados
	int n_shared_recs[MAX_AGENTS];		// numero de 'variaveis' shared
	int n_local_recs;					// numero de 'variaveis' local
	int shared_mem_size[MAX_AGENTS];	// tamanho das areas shared
	int local_mem_size;					// tamanho da area local
	int rec_lut[MAX_AGENTS][MAX_RECS];	// lookuptable
} RTDBdef;


typedef struct
{
	int n_shared_recs;
	RTDBconf_var shared[MAX_RECS];
	int n_local_recs;
	RTDBconf_var local[MAX_RECS];
} RTDBconf_agents;


int def_shmid[MAX_AGENTS];					// identificador da area de definicoes
RTDBdef *p_def[MAX_AGENTS];					// ponteiro para a area de definicoes

int shared_shmid[MAX_AGENTS][MAX_AGENTS];	// array de identificadores das areas shared
void *p_shared_mem[MAX_AGENTS][MAX_AGENTS];	// array de ponteiros para as areas shared

int local_shmid[MAX_AGENTS];				// identificador da area local
void *p_local_mem[MAX_AGENTS];				// ponteiro para a area local

int __agent = -1;


//	*************************
//	read_configuration: CONFIG_FILE parser
//
//	output:
//		number of agents
//		-1 = error
//
int read_configuration(RTDBconf_agents *conf)
{
	FILE *f_def;
	int rc;
	char s[100];
	int n_agents = -1;
	int agent;
	int id, size, period;
	char type;

    char * home;
    home=getenv("HOME");
    std::stringstream sh;
    sh<<home;

    std::string ini_config_file=sh.str()+"/nubot_ws/src/nubot/world_model/config/rtdb.ini";
    if ((f_def = fopen(ini_config_file.c_str(), "r")) == NULL)
	{
		PERRNO("fopen");
		return -1;
	}

	do
	{
		rc = fscanf(f_def, "%100[^\n]", s);
		fgetc(f_def);		
	
		if (rc == -1)
			break;

		if (rc != 0)
		{
			if (s[0] != '#')
			{
				sscanf(s, "%d\t%d\t%d\t%c\n", &id, &size, &period, &type);
				if (type == 's')
				{
					conf[agent].shared[conf[agent].n_shared_recs].id = id;
					conf[agent].shared[conf[agent].n_shared_recs].size = size;
					conf[agent].shared[conf[agent].n_shared_recs].period = period;
					conf[agent].n_shared_recs ++;
				}
				else
				{
					conf[agent].local[conf[agent].n_local_recs].id = id;
					conf[agent].local[conf[agent].n_local_recs].size = size;
					conf[agent].local[conf[agent].n_local_recs].period = period;
					conf[agent].n_local_recs ++;
				}
				if ((conf[agent].n_shared_recs + conf[agent].n_local_recs) > MAX_RECS)
				{
					PERR("Increase MAX_RECS");
					return -1;
				}
			}
			else
			{
				if (s[1] != '#')
				{
					sscanf(s+1, "%d\n", &agent);
					if (agent >= MAX_AGENTS)
					{
						PERR("Increase MAX_AGENTS");
						return -1;
					}
					n_agents ++;
					conf[n_agents].n_shared_recs = 0;
					conf[n_agents].n_local_recs = 0;
				}
			}
		}

	} while (rc != -1);

	fclose(f_def);
	n_agents ++;

#ifdef DEBUG

	int i, j;

	for (i = 0; i < n_agents; i++)
	{
		PDEBUG("Agent : %d", i);
		for (j = 0; j < conf[i].n_shared_recs; j++)
			PDEBUG("  Shared: id: %d, size: %d, period: %d", conf[i].shared[j].id, conf[i].shared[j].size, conf[i].shared[j].period);
	}
#endif

	return (n_agents);
}



//	*************************
//	_DB_free: free RTDB
//
//	input:
//		int _agent = agent memory
//
void _DB_free (int _agent)
{
	int i;
	struct shmid_ds shmem_status;

	printf ("RTDB free in agent %d\n", _agent);

	// shared memory
	for (i = 0; i < p_def[_agent]->n_agents; i++)
	{
		shmdt(p_shared_mem[_agent][i]);
		
		// if it is the last
		if (shmctl(shared_shmid[_agent][i], IPC_STAT, &shmem_status) == -1)
			PERRNO("shmctl");
		printf ("-- shared %d ---- %d ---\n", i, (int)shmem_status.shm_nattch);
		if (shmem_status.shm_nattch == 0)
			shmctl(shared_shmid[_agent][i], IPC_RMID, NULL);
	}

	// local memory
	shmdt(p_local_mem[_agent]);
	// if it is the last
	if (shmctl(local_shmid[_agent], IPC_STAT, &shmem_status) == -1)
		PERRNO("shmctl");
	printf ("-- local ---- %d ---\n", (int)shmem_status.shm_nattch);
	if (shmem_status.shm_nattch == 0)
		shmctl(local_shmid[_agent], IPC_RMID, NULL);

	// def memory
	shmdt(p_def[_agent]);
	// se e o ultimo
	if (shmctl(def_shmid[_agent], IPC_STAT, &shmem_status) == -1)
		PERRNO("shmctl");
	printf ("-- def ---- %d ---\n", (int)shmem_status.shm_nattch);
	if (shmem_status.shm_nattch == 0)
		shmctl(def_shmid[_agent], IPC_RMID, NULL);
}



void DB_free (void)
{
	if (__agent != -1)
		_DB_free (__agent);
}



void DB_free_all (void)
{
	int i;

	for (i=0; i<MAX_AGENTS; i++)
		_DB_free (i);
}



//	*************************
//	DB_initialization: RTDB init
//
//	input:
//		int _agent = agent memory
//	output:
//		0 = OK
//		-1 = error
//
int DB_initialization (int _agent, int _second_rtdb)
{
	int i, j;
	int key;
	int offset;
	TRec *p_rec;
	struct shmid_ds shmem_status;
	RTDBconf_agents rtdb_conf[MAX_AGENTS];

	// malloc
  if(_second_rtdb == 0)
  	key = SHMEM_KEY + (_agent * MAX_AGENTS * 2);
  else
  	key = SHMEM_SECOND_TEAM_KEY + (_agent * MAX_AGENTS * 2);

	def_shmid[_agent] = shmget(key, sizeof(RTDBdef), 0644 | IPC_CREAT);
	if (def_shmid[_agent] == -1)
	{
		PERRNO("shmget");
		return -1;
	}
	key++;
	p_def[_agent] = (RTDBdef*)shmat(def_shmid[_agent], (void *)0, 0);
	if ((char *)p_def[_agent] == (char *)(-1))
	{
		PERRNO("shmat");
		_DB_free(_agent);
		return -1;
	}

	// if it is the first
	if (shmctl(def_shmid[_agent], IPC_STAT, &shmem_status) == -1)
	{
		PERRNO("shmctl");
		_DB_free(_agent);
		return -1;
	}
	if (shmem_status.shm_nattch == 1)
	{
		// load defines
		if ((p_def[_agent]->n_agents = read_configuration(rtdb_conf)) < 1)
		{
			PERR("read_configuration");
			_DB_free(_agent);
			return -1;
		}

		p_def[_agent]->self_agent = _agent;
	
		for (i = 0; i < p_def[_agent]->n_agents; i++)
		{

			if (i == p_def[_agent]->self_agent)
			{
				p_def[_agent]->n_local_recs = rtdb_conf[i].n_local_recs;
				for (j = 0; j < p_def[_agent]->n_local_recs; j++)
					p_def[_agent]->local_mem_size += rtdb_conf[i].local[j].size;
				p_def[_agent]->local_mem_size = p_def[_agent]->local_mem_size * 2 + sizeof (TRec) * p_def[_agent]->n_local_recs;
			}
			p_def[_agent]->n_shared_recs[i] = rtdb_conf[i].n_shared_recs;
			for (j = 0; j < p_def[_agent]->n_shared_recs[i]; j++)
				p_def[_agent]->shared_mem_size[i] += rtdb_conf[i].shared[j].size;

			// sizeof memory to alloc
			p_def[_agent]->shared_mem_size[i] = p_def[_agent]->shared_mem_size[i] * 2 + sizeof (TRec) * p_def[_agent]->n_shared_recs[i];
		}
	}

	// alloc of shared memory
	for (i = 0; i < p_def[_agent]->n_agents; i++)
	{
		shared_shmid[_agent][i] = shmget(key, p_def[_agent]->shared_mem_size[i], 0644 | IPC_CREAT);
		if (shared_shmid[_agent][i] == -1)
		{
			PERRNO("shmget");
			_DB_free(_agent);
			return -1;
		}
		key ++;
		p_shared_mem[_agent][i] = shmat(shared_shmid[_agent][i], (void *)0, 0);
		if (p_shared_mem[_agent][i] == (char *)(-1))
		{
			PERRNO("shmat");
			_DB_free(_agent);
			return -1;
		}
	}
	
	// alloc of local memory
	if (p_def[_agent]->local_mem_size == 0)
		local_shmid[_agent] = shmget(key, 1, 0644 | IPC_CREAT);
	else
		local_shmid[_agent] = shmget(key, p_def[_agent]->local_mem_size, 0644 | IPC_CREAT);
	if (local_shmid[_agent] == -1)
	{
		PERRNO("shmget");
		_DB_free(_agent);
		return -1;
	}
	p_local_mem[_agent] = shmat(local_shmid[_agent], (void *)0, 0);
	if (p_local_mem[_agent] == (char *)(-1))
	{
		PERRNO("shmat");
		_DB_free(_agent);
		return -1;
	}

	// if it is the first, memory initialization
	if (shmctl(local_shmid[_agent], IPC_STAT, &shmem_status) == -1)
	{
		PERRNO("shmctl");
		_DB_free(_agent);
		return -1;
	}
	if (shmem_status.shm_nattch > 1)
	{
		PDEBUG("Memory already configurated");
		return 0;
	}

	for (i = 0; i < MAX_AGENTS; i++)
		for (j = 0; j < MAX_RECS; j++)
			p_def[_agent]->rec_lut[i][j] = -1;

	for (i = 0; i < p_def[_agent]->n_agents; i++)
	{
		offset = p_def[_agent]->n_shared_recs[i] * sizeof(TRec);
		for (j = 0; j < p_def[_agent]->n_shared_recs[i]; j++)
		{
			p_rec = (TRec*)((char*)(p_shared_mem[_agent][i]) + j * sizeof(TRec));
			p_rec->id = rtdb_conf[i].shared[j].id;
			p_rec->size = rtdb_conf[i].shared[j].size;
			p_rec->offset = offset;
			p_rec->period = rtdb_conf[i].shared[j].period;
			p_rec->read_bank = 0;
			p_def[_agent]->rec_lut[i][p_rec->id] = j;
			offset = offset + (p_rec->size * 2) - sizeof(TRec);

			PDEBUG("agent: %d, shared: %d, size: %d, offset:%d, period: %d, lut: %d", i, p_rec->id, p_rec->size, p_rec->offset, p_rec->period, j);
		}
	}
	
	offset = p_def[_agent]->n_local_recs * sizeof(TRec);
	for (j = 0; j < p_def[_agent]->n_local_recs; j++)
	{
		p_rec = (TRec*)((char*)(p_local_mem[_agent]) + j * sizeof(TRec));
		p_rec->id = rtdb_conf[p_def[_agent]->self_agent].local[j].id;
		p_rec->size = rtdb_conf[p_def[_agent]->self_agent].local[j].size;
		p_rec->offset = offset;
		p_rec->period = rtdb_conf[p_def[_agent]->self_agent].local[j].period;
		p_rec->read_bank = 0;
		p_def[_agent]->rec_lut[p_def[_agent]->self_agent][p_rec->id] = MAX_RECS + j;
		offset = offset + (p_rec->size * 2) - sizeof(TRec);
		
		PDEBUG("local: %d, size: %d, offset:%d, period: %d, lut: %d", p_rec->id, p_rec->size, p_rec->offset, p_rec->period, MAX_RECS + j);
		PDEBUG("lut = %d",p_def[_agent]->rec_lut[p_def[_agent]->self_agent][p_rec->id]);
	}

	return 0;
}


//	*************************
//	DB_init: RTDB init
//
//	output:
//		0 = OK
//		-1 = error
//
int DB_init(void)
{
	char *environment;
	int agent;
    int second_rtdb = 0;

	// retrieve agent number
	if((environment = getenv("AGENT")) == NULL)
	{
		PERR("getenv");
		return -1;
	}
	agent = atoi(environment);
	__agent = agent;
	PDEBUG("agent = %d", agent);

	// 
	if((environment = getenv("SECOND_RTDB")) == NULL)
	{
		second_rtdb = 0;
  }
  else
  {
  	second_rtdb = atoi(environment);
  }

	return (DB_initialization (agent, second_rtdb));
}


//	*************************
//	DB_init_sim: RTDB init for all agent (use only in simulator)
//
//	output:
//		0 = OK
//		-1 = error
//
int DB_init_all (int second_rtdb)
{
	int i;

	for (i=0; i<MAX_AGENTS; i++)
		if(DB_initialization(i, second_rtdb) == -1)
			return (-1);

	return (0);
}



//	*************************
//	DB_put_in: write in RTDB
//		note: it can write in any area (use with caution!)
//
//	imput:
//		int _agent
//		int _to_agent = agent number
//		int _id = identificador da 'variavel'
//		void *_value = ponteiro com os dados
//		int life = tempo de vida da 'variavel' em ms
//	Saida:
//		int size = size of record data
//		-1 = erro
//
int DB_put_in (int _agent, int _to_agent, int _id, void *_value, int life)
{
	int lut;
	TRec *p_rec;
	void *p_data;
	int write_bank;
	struct timeval time;

	if ((lut = p_def[_agent]->rec_lut[_to_agent][_id]) == -1)
	{
		PERR("Unknown record %d for agent %d", _id, _to_agent);
		return -1;
	}

	if (lut < MAX_RECS)
		p_rec = (TRec*)((char*)(p_shared_mem[_agent][_to_agent]) + lut * sizeof(TRec));
	else
		p_rec = (TRec*)((char*)(p_local_mem[_agent]) + (lut - MAX_RECS) * sizeof(TRec));

	write_bank = (p_rec->read_bank + 1) % 2;

	p_data = (void*)((char*)(p_rec) + p_rec->offset + write_bank * p_rec->size);
	memcpy(p_data, _value, p_rec->size);

	gettimeofday(&time, NULL);
	p_rec->timestamp[write_bank].tv_sec = time.tv_sec - life / 1000;
	p_rec->timestamp[write_bank].tv_usec = time.tv_usec - (life % 1000) * 1000;

	p_rec->read_bank = write_bank;

	PDEBUG("agent: %d, id: %d, lut: %d, size: %d, write_bank: %d, previous life: %umsec", _to_agent, p_rec->id, lut, p_rec->size, p_rec->read_bank, life);
	
	return p_rec->size;
}




//	*************************
//	DB_comm_put: Escreve na base de dados
//		atencao: permite escrever em qualquer area!
//
//	Entrada:
//		int _to_agent = numero do agente
//		int _id = identificador da 'variavel'
//		void *_value = ponteiro com os dados
//		int _life = tempo de vida da 'variavel' em ms
//	Saida:
//		int size = size of record data
//		-1 = erro
//
int DB_comm_put (int _to_agent, int _id, int _size, void *_value, int _life)
{
	if ((_to_agent == SELF) || (_to_agent == __agent))
	{
		PERR("Impossible to write in the running agent!");
		return -1;
	}

	if (p_def[__agent]->rec_lut[_to_agent][_id] >= MAX_RECS)
	{
		PERR("Impossible to write local records!");
		return -1;
	}

	return DB_put_in(__agent, _to_agent, _id, _value, _life);
}



//	*************************
//	DB_put: Escreve na base de dados do proprio agente
//
//	Entrada:
//		int _id = identificador da 'variavel'
//		void *_value = ponteiro com os dados
//	Saida:
//		int size = size of record data
//		-1 = error
//
int DB_put (int _id, void *_value)
{
	if (__agent == -1)
		return (-1);
	return DB_put_in(__agent, __agent, _id, _value, 0);
}



//	*************************
//	DB_get_from: Le da base de dados
//
//	Entrada:
//		int _agent
//		int _from_agent = numero do agente
//		int _id = identificador da 'variavel'
//		void *_value = ponteiro para onde sao copiados os dados
//	Saida:
//		int life = tempo de vida da 'variavel' em ms
//			-1 se erro
//
int DB_get_from (int _agent, int _from_agent, int _id, void *_value)
{
	int lut;
	TRec *p_rec;
	void *p_data;
	struct timeval time;
	int life;

	if (_from_agent == SELF)
		_from_agent = p_def[_agent]->self_agent;

	if((lut = p_def[_agent]->rec_lut[_from_agent][_id]) == -1)
	{
		PERR("Unknown record %d for agent %d", _id, _from_agent);
		return -1;
	}

	if (lut < MAX_RECS)
		p_rec = (TRec*)((char*)(p_shared_mem[_agent][_from_agent]) + lut * sizeof(TRec));
	else
		p_rec = (TRec*)((char*)(p_local_mem[_agent]) + (lut - MAX_RECS) * sizeof(TRec));
	
	p_data = (void *)((char *)(p_rec) + p_rec->offset);

	memcpy(_value, (char *)p_data + (p_rec->read_bank * p_rec->size), p_rec->size);

	gettimeofday(&time, NULL);
	life = (int)(((time.tv_sec - (p_rec->timestamp[p_rec->read_bank]).tv_sec) * 1E3) + ((time.tv_usec - (p_rec->timestamp[p_rec->read_bank]).tv_usec) / 1E3));

	PDEBUG("agent: %d, from_agent: %d, id: %d, read_bank: %d, life: %umsec", _agent, _from_agent, p_rec->id, p_rec->read_bank, life);

	return (life);
}



//	*************************
//	DB_get_from: Le da base de dados
//
//	Entrada:
//		int _from_agent = numero do agente
//		int _id = identificador da 'variavel'
//		void *_value = ponteiro para onde sao copiados os dados
//	Saida:
//		tempo de vida da 'variavel' em ms
//			-1 = error
//
int DB_get (int _from_agent, int _id, void *_value)
{
	if (__agent == -1)
		return (-1);
	return (DB_get_from (__agent, _from_agent, _id, _value));
}



//	*************************
//	Whoami: identifica o agente onde esta a correr
//
//	Saida:
//		int agent_number = numero do agente
//		-1 = error
//
int Whoami(void)
{
	return (__agent);
}



//	*************************
//	DB_comm_ini: 
//
//	Entrada:
//		int _id = array com identificadores da 'variavel'
//		int _size = array com temanhos da 'variavel'
//		int _period = array com periodos da 'variavel'
//	Saida:
//		int n_shared_recs = numero de 'variaveis' shared
//		-1 = erro
//
int DB_comm_ini(RTDBconf_var *rec)
{
	int n_shared_recs;
	int i;
	TRec *p_rec;

	if (__agent == -1)
		return (-1);
	n_shared_recs = p_def[__agent]->n_shared_recs[__agent];
	for (i = 0; i < n_shared_recs; i++)
	{
		p_rec = (TRec*)((char*)(p_shared_mem[__agent][__agent]) + i * sizeof(TRec));
		rec[i].id = p_rec->id;
		rec[i].size = p_rec->size;
		rec[i].period = p_rec->period;
	}

	return n_shared_recs;
}
