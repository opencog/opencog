/*
 * opencog/cogserver/server/ParseSentenceRequest.cc
 *
 * Copyright (C) 2008 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Curtis Faith
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

#include <fstream>

#include "ParseSentenceRequest.h"

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atoms/base/ClassServer.h>
#include <opencog/atoms/base/types.h>
#include <opencog/cogserver/server/CogServer.h>

#include <opencog/nlp/learn/ObserveSentence.h>
#include <opencog/nlp/learn/ParseSentence.h>

using namespace opencog;

// Don't forget to change help in ParseSentenceRequest.h if you change the
// default delimiter below. It can't be programmatically updated because of
// the Request macro handling.

#define DEFAULT_DELIMITER   " "


ParseSentenceRequest::ParseSentenceRequest(CogServer& cs) : Request(cs)
{
}

ParseSentenceRequest::~ParseSentenceRequest()
{
    logger().debug("[ParseSentenceRequest] destructor");
}

bool ParseSentenceRequest::syntaxError()
{
    _error << "invalid syntax" << std::endl;
    sendError();
    return false;
} 

static std::ofstream* output_stream = nullptr;
static int sentence_count = 0;

bool ParseSentenceRequest::execute()
{
    std::list<std::string>::const_iterator it;
    std::string delimiter = DEFAULT_DELIMITER;

    if (_parameters.empty())
    {
        _error << "Error: learn - a sentence is required" << std::endl;
        sendError();
        return false;
    }

    std::ostringstream oss;
    std::string dump_file_name;

    // Extract the parameters.
    int parameter_count = 0;
    int pair_distance = DEFAULT_PAIR_DISTANCE;
    bool no_parse = false;
    bool verbose = false;
    bool dump_weights = false;
    bool check_pairs = false;
    bool test_reply = false;
    for (it = _parameters.begin(); it != _parameters.end(); ++it)
    {
        parameter_count++;

        // -open_parse - open a file for subsequent parse output
        if (*it == "-open_parse")
        { 
            sentence_count = 0;
            if (it == _parameters.end()) return syntaxError();
            ++it;
            std::string file_name = *it;
            output_stream = new std::ofstream;

             // Set exceptions to be thrown on failure
            output_stream->exceptions(std::ifstream::failbit | std::ifstream::badbit);
            try
            {
                printf("\nopening parse file '%s'\n", file_name.c_str());
                output_stream->open(file_name);

            } catch (std::system_error& e) {
                _error << e.code().message() << std::endl;
                sendError();
                return false;
            }
            no_parse = true;
        }
        // -close_parse - close the open parse file
        else if (*it == "-close_parse")
        { 
            if (output_stream)
            {
                printf("\nclosing parse file\n");
                output_stream->close();
                delete output_stream;
                output_stream = nullptr;
            }
            no_parse = true;
        }
        // -check_pairs - check the pairs for existence
        else if (*it == "-check_pairs")
        {
            check_pairs = true;
            no_parse = true;
        }
        // -test - if test is set then we return an answer
        else if (*it == "-test_reply")
        {
            test_reply = true;
        }
        // -delimiter <string> - use this string for a delimiter
        else if (*it == "-delimiter")
        {
            ++it;
            if (it == _parameters.end()) return syntaxError();
            delimiter = *it;       
        }
        // -pair_distance <limit> - limit distance of word pairs
        else if (*it == "-pair_distance")
        {
            ++it;
            if (it == _parameters.end()) return syntaxError();
            pair_distance = atoi(it->c_str());       
        }
        // -dump_weights - open a file and dump the weights for the pairs
        if (*it == "-dump_weights")
        { 
            if (it == _parameters.end()) return syntaxError();
            ++it;
            dump_file_name = *it;
            printf("\ndumping weights to: '%s'\n", dump_file_name.c_str());
            dump_weights = true;
        }
        // -noop - do not perform any operations
        else if (*it == "-noop")
        { 
            no_parse = true;
        }
        // -verbose - send return confirmation of sentence processed
        else if (*it == "-verbose")
        { 
            verbose = true;
        }
        // This should be a sentence.
        else
        {
            _sentence = *it;
        }
    }

    // Dump the pair weights.
    if (dump_weights)
    {
        AtomSpace& atomspace = _cogserver.getAtomSpace();
        AtomSpace* as = &atomspace;
        std::string error;
        std::string message = "Dumping weights to: ";

        message += dump_file_name;
        message += "\n";
        send(message);
        if (!dump_pair_weights(as, dump_file_name, _sentence, pair_distance, error))
        {
            _error << error << std::endl;
            sendError();
            return false;
        }
    }

    AtomSpace& atomspace = _cogserver.getAtomSpace();
    AtomSpace* as = &atomspace;
    WordVector words;

    // Break the sentence up into words if we are checking or parsing...
    if (check_pairs || !no_parse)
        break_sentence_into_words(_sentence, words);

    // Check the pairs for existence.
    if (check_pairs)
    {
        try
        {
            // Check the sentence for sentence pairs to be used later in parsing.
            // std::cerr << "Checking " << sentence_count << ": " << _sentence << std::endl;
            if (check_parse_pairs(as, words, pair_distance))
            {
                if (test_reply)
                    send("<pairs present>\n");
            }
            else
            {
                _error << "CHECK PAIRS failed for sentence: '" << _sentence << "'" << std::endl;
                std::cerr << _error.str();
                send(_error.str());
            }
        }
        catch (...)
        {
            _error << "ERROR checking sentence: '" << _sentence << "'" << std::endl;
            std::cerr << _error.str();
            send(_error.str());
        }
    }

    // Do the actual parse.
    if (!no_parse)
    {
        ParseVector parse_results;
        try
        {
            // Parse the words.
            parse_words(as, words, pair_distance, parse_results);
        }
        catch (...)
        {
            _error << "ERROR parsing sentence: '" << _sentence << "'" << std::endl;
            std::cerr << _error.str();
            send(_error.str());
        }

        // Print out the parse results.
        std::ostringstream  parse_string_stream;
        std::ostream*       parse_stream;
        if (output_stream)
            parse_stream = output_stream;
        else
            parse_stream = &parse_string_stream;

        sentence_count++;

        // Construct the parse output.
        for (auto & pair : parse_results)
        {
             *parse_stream << sentence_count << delimiter << pair.left_index + 1 << delimiter <<
                    words[pair.left_index] << delimiter << pair.right_index  + 1 << delimiter <<
                    words[pair.right_index] << std::endl;
        }

        if (verbose && not output_stream)
            _parse = ((std::ostringstream*) parse_stream)->str();
    }
    if (verbose)
        sendOutput();
    return true;
}


void ParseSentenceRequest::sendOutput()
{
    std::ostringstream oss;
    oss << "Processed sentence: \"" << _sentence << "\"" << std::endl;

    if (not output_stream)
        oss << "Parse: " << std::endl << _parse << std::endl;
           
    send(oss.str());
}

void ParseSentenceRequest::sendError()
{
    _error << "Format: parse [-pair_distance <limit>] \"<sentence-in-quotes>\"" << std::endl;
    _error << "Supported options:" << std::endl;
    _error << "    -open_parse <file_name>  Open the file <file_name> for output." << std::endl;
    _error << "    -close_parse             Close the output file." << std::endl;
    _error << "    -dump_weights <file>     Dump the word pair weights for the sentence to" << std::endl;
    _error << "                             <file> as a C++ struct." << std::endl;
    _error << "    -delimiter <string>      Use <string> to delimit fields in output (default '" << DEFAULT_DELIMITER << "')." << std::endl;
    _error << "    -check_pairs             Check pair sentence observations." << std::endl;
    _error << "    -pair_distance <limit>   Create pairs up to <limit> distance apart (default " << DEFAULT_PAIR_DISTANCE << ")." << std::endl;
    _error << "    -quiet                   Do not return status over telnet." << std::endl;
    _error << "    -noop                    Perform no op-erations (useful for timing)." << std::endl;
    _error << "Options may be combined" << std::endl << std::endl;
    send(_error.str());
}
 
