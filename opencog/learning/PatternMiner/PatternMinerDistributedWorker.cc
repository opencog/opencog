/*
 * opencog/learning/PatternMiner/PatternMinerDistributedWorker.cc
 *
 * Copyright (C) 2012 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Shujing Ke <rainkekekeke@gmail.com> in 2015
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

#include <math.h>
#include <stdlib.h>

#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <vector>
#include <sstream>
#include <thread>
#include <ifaddrs.h>

#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/Handle.h>
#include <opencog/atomspace/atom_types.h>
#include <opencog/spacetime/atom_types.h>
#include <opencog/embodiment/AtomSpaceExtensions/atom_types.h>
#include <opencog/query/BindLinkAPI.h>
#include <opencog/util/Config.h>
#include <opencog/util/StringManipulator.h>

#include <cpprest/http_client.h>
#include <cpprest/filestream.h>

#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include "HTree.h"
#include "PatternMiner.h"

using namespace utility;                    // Common utilities like string conversions
using namespace web;                        // Common features like URIs.
using namespace web::http;                  // Common HTTP functionality
using namespace web::http::client;          // HTTP client features
using namespace concurrency::streams;       // Asynchronous streams


using namespace opencog::PatternMining;
using namespace opencog;


void PatternMiner::launchADistributedWorker()
{
    run_as_distributed_worker = true;

    centralServerIP = config().get("PMCentralServerIP");
    centralServerPort = config().get("PMCentralServerPort");
    centralServerBaseURL = "http://" +  centralServerIP + ":" + centralServerPort + "/PatternMinerServer";

    boost::uuids::random_generator uuidGen;
    boost::uuids::uuid uid = uuidGen();
    std::stringstream ssuid;
    ssuid << uid;
    clientWorkerUID = ssuid.str();
    std::cout<<"Current client UID = "<< clientWorkerUID << std::endl;
    std::cout<<"Registering to the central server: "<< centralServerIP << std::endl;

    // Create http_client to send the request.
    httpClient = new http_client(U(centralServerBaseURL.c_str()));

    // Build request URI and start the request.
    uri_builder builder(U("/RegisterNewWorker"));
    builder.append_query(U("ClientUID"), U(clientWorkerUID));


    httpClient->request(methods::GET, builder.to_string()).then([=](http_response response)
    {
        std::cout << response.to_string() << std::endl;
        if (response.status_code() == status_codes::OK)
        {
            std::cout << "Registered to the central server successfully! " << std::endl;
            startMiningWork();
        }
        else
        {
            std::cout << "Registered to the central server failed! Please check network and the the state of the central server." << std::endl;
        }

        return;

    });


    std::cout << "Registered to the central server failed! Please check network and the the state of the central server." << std::endl;


}

void PatternMiner::startMiningWork()
{
    // Note: Breadth_First mining is not maintained anymore. Only Depth_First is used in distributed mining.
    //    Pattern_mining_mode = config().get("Pattern_mining_mode"); // option: Breadth_First , Depth_First
    //    assert( (Pattern_mining_mode == "Breadth_First") || (Pattern_mining_mode == "Depth_First"));
        Pattern_mining_mode = "Depth_First";

        std::cout<<"Start pattern mining work! Max gram = " + toString(this->MAX_GRAM) << ", mode = Depth_First" << std::endl;

        int start_time = time(NULL);

        originalAtomSpace->get_handles_by_type(back_inserter(allLinks), (Type) LINK, true );

        allLinkNumber = (int)(allLinks.size());
        atomspaceSizeFloat = (float)(allLinkNumber);

        runPatternMinerDepthFirst();

        int end_time = time(NULL);
        printf("Pattern Mining Finish one round! Total time: %d seconds. \n", end_time - start_time);
        std::cout<< THREAD_NUM << " threads used. \n";
        std::cout<<"Corpus size: "<< allLinkNumber << " links in total. \n";
}
