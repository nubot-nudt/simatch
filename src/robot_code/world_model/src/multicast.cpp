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


#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/if.h>
#include <unistd.h>
#include <linux/if_ether.h>

#include "nubot/world_model/multicast.h"


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


struct sockaddr_in destAddress;


int if_NameToIndex(char *ifname, char *address)
{
	int	fd;
	struct ifreq if_info;
	int if_index;

	memset(&if_info, 0, sizeof(if_info));
	strncpy(if_info.ifr_name, ifname, IFNAMSIZ-1);

	if ((fd=socket(AF_INET, SOCK_DGRAM, 0)) == -1)
	{
		PERRNO("socket");
		return (-1);
	}
	if (ioctl(fd, SIOCGIFINDEX, &if_info) == -1)
	{
		PERRNO("ioctl");
		close(fd);
		return (-1);
	}
	if_index = if_info.ifr_ifindex;

	if (ioctl(fd, SIOCGIFADDR, &if_info) == -1)
	{
		PERRNO("ioctl");
		close(fd);
		return (-1);
	}
	
	close(fd);

	sprintf(address, "%d.%d.%d.%d\n",
		(int) ((unsigned char *) if_info.ifr_hwaddr.sa_data)[2],
		(int) ((unsigned char *) if_info.ifr_hwaddr.sa_data)[3],
		(int) ((unsigned char *) if_info.ifr_hwaddr.sa_data)[4],
		(int) ((unsigned char *) if_info.ifr_hwaddr.sa_data)[5]);

	printf("**** Using device %s -> Ethernet %s\n ****", if_info.ifr_name, address);

	return (if_index);
}


//	*************************
//  Open Socket
//
int openSocket(char* interface)
{
  struct sockaddr_in multicastAddress;
  struct ip_mreqn mreqn;
  struct ip_mreq mreq;
	int multiSocket;
	int opt;
	char address[20];

  bzero(&multicastAddress, sizeof(struct sockaddr_in));
  multicastAddress.sin_family = AF_INET;
  multicastAddress.sin_port = htons(MULTICAST_PORT);
  multicastAddress.sin_addr.s_addr = INADDR_ANY;

	bzero(&destAddress, sizeof(struct sockaddr_in));
	destAddress.sin_family = AF_INET;
	destAddress.sin_port = htons(MULTICAST_PORT);
	destAddress.sin_addr.s_addr = inet_addr(MULTICAST_IP);

	if((multiSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
	{
		PERRNO("socket");
		return (-1);
	}

	memset((void *) &mreqn, 0, sizeof(mreqn));
	mreqn.imr_ifindex=if_NameToIndex(interface, address);
	if((setsockopt(multiSocket, SOL_IP, IP_MULTICAST_IF, &mreqn, sizeof(mreqn))) == -1)
	{
	  PERRNO("setsockopt");
		return (-1);
	}

	opt = 1;
	if((setsockopt(multiSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) == -1)
  {
    PERRNO("setsockopt");
		return (-1);
  }
 
	memset((void *) &mreq, 0, sizeof(mreq));
	mreq.imr_multiaddr.s_addr = inet_addr(MULTICAST_IP);
	mreq.imr_interface.s_addr = inet_addr(address);

	if((setsockopt(multiSocket, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq))) == -1)
	{
	  PERRNO("setsockopt");
		return (-1);
	}
						
	/* Disable reception of our own multicast */
	opt = RECEIVE_OUR_DATA;
	if((setsockopt(multiSocket, IPPROTO_IP, IP_MULTICAST_LOOP, &opt, sizeof(opt))) == -1)
	{
		PERRNO("setsockopt");
		return (-1);
	}

	if(bind(multiSocket, (struct sockaddr *) &multicastAddress, sizeof(struct sockaddr_in)) == -1)
	{
		PERRNO("bind");
		return (-1);
	}

	return (multiSocket);
}



//	*************************
//  Close Socket
//
void closeSocket(int multiSocket)
{
	if(multiSocket != -1)
		shutdown(multiSocket, SHUT_RDWR);
}



//	*************************
//  Send Data
//
int sendData(int multiSocket, void* data, int dataSize)
{
	return (sendto(multiSocket, data, dataSize, 0, (struct sockaddr *)&destAddress, sizeof (struct sockaddr)));
}



//	*************************
//  Receive Data
//
int receiveData(int multiSocket, void* buffer, int bufferSize)
{
	return (recv(multiSocket, buffer, bufferSize, 0));
}
