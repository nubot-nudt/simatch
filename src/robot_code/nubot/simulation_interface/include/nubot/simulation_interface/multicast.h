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


#define MULTICAST_IP	"224.16.32.53" //"225.17.21.24" //
#define MULTICAST_PORT	5081
#define TTL				64

#define RECEIVE_OUR_DATA 0


//	*************************
//  Open Socket
//
//	Input:
//		const char* = interface name {eth0, wlan0, ...}
//	Output:
//		int multiSocket = socket descriptor
//
int openSocket(char* interface);



//	*************************
//  Close Socket
//
//  Input:
//		int multiSocket = socket descriptor
//
void closeSocket(int multiSocket);



//	*************************
//  Send Data
//
//  Input:
//		int multiSocket = socket descriptor
//		void* data = pointer to buffer with data
//		int dataSize = number of data bytes in buffer
//
int sendData(int multiSocket, void* data, int dataSize);



//	*************************
//  Receive Data
//
//  Input:
//		int multiSocket = socket descriptor
//		void* buffer = pointer to buffer
//		int bufferSize = total size of buffer
//
int receiveData(int multiSocket, void* buffer, int bufferSize);
