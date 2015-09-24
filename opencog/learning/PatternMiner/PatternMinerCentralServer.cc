/*
 * opencog/learning/PatternMiner/PatternMinerCentralServer.cc
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
#include <set>
#include <string>
#include <functional>


#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/Handle.h>
#include <opencog/atomspace/atom_types.h>
#include <opencog/spacetime/atom_types.h>
#include <opencog/embodiment/AtomSpaceExtensions/atom_types.h>
#include <opencog/query/BindLinkAPI.h>
#include <opencog/util/Config.h>
#include <opencog/util/StringManipulator.h>

#include "HTree.h"
#include "PatternMiner.h"

#include <cpprest/http_listener.h>
#include <cpprest/http_msg.h>
#include <cpprest/json.h>


using namespace opencog::PatternMining;
using namespace opencog;

using namespace std;
using namespace web;
using namespace web::http;
using namespace web::http::experimental::listener;
using namespace utility;


void PatternMiner::launchCentralServer()
{
   centralServerPort = Config().get("PMCentralServerPort");

   serverListener = new http_listener( utility::string_t("http://localhost:" + centralServerPort +"/PatternMinerServer") );

   serverListener->support(methods::GET, std::bind(&PatternMiner::handleGet, this,  std::placeholders::_1));

   try
   {
      serverListener->open()
         .then([](pplx::task<void> t){})
         .wait();

      while (true);
   }
   catch (exception const & e)
   {
      cout << e.what() << endl;
   }

   cout <<"\nPattern Miner central server started!\n";

   cout << "\nPattern Miner central server stopped!\n";

}

void PatternMiner::handleGet(http_request request)
{
    cout << "Got request: \n" << request.to_string() << std::endl;
    string path = request.relative_uri().path();
    if (path == "RegisterNewWorker")
    {
        handleRegisterNewWorker(request);
    }
    else
    {
        json::value answer = json::value::object();

        answer["msg"] = json::value("Unknown request command!");

        request.reply(status_codes::NotFound, answer);
    }


}


void PatternMiner::handleRegisterNewWorker(http_request request)
{

   cout << "Got request to RegisterNewWorker: \n" << request.to_string() << std::endl;
   json::value answer = json::value::object();

   answer["msg"] = json::value("Worker registered successfully!");

   request.reply(status_codes::OK, answer);

}

void PatternMiner::handleReportWorkerStop(http_request request)
{


}

void PatternMiner::handleFindANewPattern(http_request request)
{

}
