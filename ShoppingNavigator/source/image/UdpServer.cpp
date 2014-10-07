/**********************************************************************************
 *  The MIT License (MIT)                                                        *
 *                                                                                *
 *  Copyright (c) 2014 Carnegie Mellon University                                 *
 *                                                                                *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy  *
 *  of this software and associated documentation files (the "Software"), to deal *
 *  in the Software without restriction, including without limitation the rights  *
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell     *
 *  copies of the Software, and to permit persons to whom the Software is         *
 *  furnished to do so, subject to the following conditions:                      *
 *                                                                                *
 *  The above copyright notice and this permission notice shall be included in    *
 *  all copies or substantial portions of the Software.                           *
 *                                                                                *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      *
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE   *
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER        *
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN     *
 *  THE SOFTWARE.                                                                 *
 **********************************************************************************/
/**
 * @file   UdpServer.cpp
 * @brief  A simple UDP server to send data to MOOS and other processes
 * @author M. George
 */

#include "UdpServer.h"

UdpServer::UdpServer(const char* in_IP,int in_Port)
{
    m_connected = false;
    m_socklength = sizeof(struct sockaddr_in);
    m_IP = in_IP;
    m_Port = in_Port;
}

bool UdpServer::connect()
{
	m_sdata = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (m_sdata == -1) {
		 std::cerr << "Error (UdpServer): Error creating socket." << std::endl;
		 return false;
	}

	memset(&m_sa, 0, sizeof(m_sa));
	m_sa.sin_family = AF_INET;
	m_sa.sin_addr.s_addr = inet_addr(m_IP);
	m_sa.sin_port = htons(m_Port);

	return true;
}

bool UdpServer::disconnect()
{
	m_connected = close(m_sdata);
	return !m_connected;
}

bool UdpServer::send(const unsigned char* in_data, unsigned int len)
{
	memcpy(&m_buf[0],in_data,len);
	unsigned int sl=sendto(m_sdata,m_buf,len,0,(struct sockaddr *) &m_sa,sizeof(m_sa));
    if (sl != len)
	{
        std::cerr<<"UdpServer Error Sending: " << sl << " " <<strerror( errno )<<std::endl;
        return false;	
	} 
	return true;
}
