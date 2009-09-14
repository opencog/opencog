/*
 * opencog/embodiment/Control/OperationalPetController/NLGenClient.h
 *
 * Copyright (C) 2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Fabricio Silva 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef NLGENCLIENT_H
#define NLGENCLIENT_H

#include <Sockets/TcpSocket.h>
#include <Sockets/ISocketHandler.h>
#include <string>


namespace OperationalPetController
{
    class NLGenClient : public TcpSocket {
        public:
            NLGenClient(ISocketHandler& h) : TcpSocket(h) {}
            NLGenClient(ISocketHandler& h, const std::string& data) : TcpSocket(h), m_data(data) {}
            void OnConnect() {
                connected = true;
        		SetLineProtocol();
		        if (m_data.empty()) {
		           logger().error("NLGenClient::%s - trying to send empty data to NLGen server socket.",__FUNCTION__); 
                } else {
                    logger().debug("NLGenClient::%s - sending data %s to NLGen server socket.",__FUNCTION__,m_data.c_str());
        			Send(m_data + "\n");
                }
	        }
            //XXX If NLGen returns more than one sentence as cand idate, just
            //the last one is considered. This case occurs when relex has more
            //than one parser for one sentence, and will cause no problem in
            //this case because the sentences candidates will be the same. Just
            //the case when two sentences originate the same relex output is a
            //problem case, which I think will be rare or impossible
            void OnLine(const std::string& line){
                logger().debug("NLGenClient::%s - Incoming Line: %s", __FUNCTION__, line.c_str() );
                output_data = line;
            }
            std::string getOutput(){
               return output_data;
            }
            bool isConnected(){
                return connected;
            }

        private:
        	std::string m_data;
            std::string output_data;
            bool connected;
            
    };
};

#endif //NLGENCLIENT_H

