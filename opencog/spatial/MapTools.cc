/*
 * opencog/spatial/MapTools.cc
 *
 * Copyright (C) 2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Samir Araujo
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

#include <opencog/spatial/MapTools.h>

#include <iostream>
#include <fstream>
#include <csignal>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/parsers.hpp>

#include <boost/array.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <opencog/spatial/MapExplorer.h>
#include <opencog/spatial/MapExplorerServer.h>

using namespace opencog;
using namespace opencog::spatial;

MapTools::MapTools( unsigned int videoMode ) :
    windowWidth( 0 ), windowHeight( 0 ), fullScreen( false )
{    
    switch( videoMode ) {
    case 1:
        this->windowWidth = 800;
        this->windowHeight = 600;
        break;
    case 2:
        this->windowWidth = 800;
        this->windowHeight = 600;
        this->fullScreen = true;
        break;
    case 3:
        this->windowWidth = 1024;
        this->windowHeight = 768;
        break;
    case 4:
    default:
        this->windowWidth = 1024;
        this->windowHeight = 768;
        this->fullScreen = true;
        break;
    }; // switch    
}

int MapTools::runLocalMode( const std::string& fileName ) 
{
    spatial::Octree3DMapManager* map = NULL;
    std::ifstream file( fileName.c_str( ) );
    if ( !file.good( ) ) {
        std::cerr << "Cannot open file '" << fileName << "'" << std::endl;
        return 1;
    } // if

    std::stringstream text;
    text << file.rdbuf( );
    file.close( );
    map = spatial::Octree3DMapManager::fromString( text.str( ) );
    
    try {
        MapExplorer explorer( map, windowWidth, windowHeight, fullScreen );
        explorer.start( );
        explorer.wait( );
        
    } catch ( const opencog::StandardException& ex ) {
        std::cerr << "Error: " << ex.get_message( ) << std::endl;
        return 1;
    } // catch  

    return 0;
}

int MapTools::runRemoteMode( const std::string& host, const std::string& port, bool& externalInterruption )
{
    try {
        boost::asio::ip::tcp::resolver resolver(this->visualDebuggerClientService);
        boost::asio::ip::tcp::resolver::query query( host, port );
        boost::asio::ip::tcp::endpoint endpoint = *resolver.resolve(query);
        
        boost::asio::ip::tcp::socket socket(this->visualDebuggerClientService);
        
        boost::system::error_code error = boost::asio::error::host_not_found;
        
        socket.connect(endpoint, error);
        if (error) {
            throw boost::system::system_error(error);
        } // if
        
        MapExplorer* explorer = NULL;
        spatial::Octree3DMapManager* oldMap = NULL;
        bool running = true;
        while( running && !externalInterruption ) {

            size_t bytesToRead = socket.available( );
            if ( bytesToRead > 0 ) {
                char buffer[bytesToRead+1];
                size_t len = socket.read_some(boost::asio::buffer(buffer, bytesToRead), error);
                buffer[len] = '\0';

                if (error == boost::asio::error::eof) {
                    running = false;
                } else if (error) {
                    throw boost::system::system_error(error); // Some other error.
                } // else if
                
                spatial::Octree3DMapManager* incomingMap = spatial::Octree3DMapManager::fromString( buffer );
                if ( explorer == NULL ) {
                    explorer = new MapExplorer( incomingMap, windowWidth, windowHeight, fullScreen );
                    explorer->start( );
                    oldMap = incomingMap;
                } else {
                    explorer->updateMap( incomingMap );
                    delete oldMap;
                    oldMap = incomingMap;
                } // else

            } else {
                boost::thread::sleep( boost::get_system_time() + boost::posix_time::milliseconds(250) );
            } // else

            if ( explorer && !explorer->isRunning( ) ) {
                running = false;
            } // if

        } // while

        
        boost::asio::write(socket, boost::asio::buffer( "quit" ) );
        socket.close( error );
        if (error) {
            throw boost::system::system_error(error);
        } // if

        if ( explorer != NULL ) {            
            explorer->stop( );
            delete explorer;
            explorer = NULL;
        } // if

        if ( oldMap != NULL ) {
            delete oldMap;
            oldMap = NULL;
        } // if

    } catch ( const opencog::StandardException& ex ) {
        std::cerr << "Error: " << ex.getMessage( ) << std::endl;
        return 1;
    } catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    } // catch

    std::cout << "bye!" << std::endl;

    return 0;
}


int MapTools::runServerMode( const std::string& host, const std::string& port, 
                             unsigned int mapVisualizationTimeout,
                             const std::vector<std::string>& mapFileNames,
                             bool& externalInterruption ) 
{

    if ( mapFileNames.size( ) == 0 ) {
        std::cerr << "Error: At least one map file must be given" << std::endl;
        return 1;
    } // if
    
    std::vector<spatial::Octree3DMapManager*> maps( mapFileNames.size( ) );
    unsigned int i;
    for( i = 0; i < mapFileNames.size( ); ++i ) {
        std::ifstream file( mapFileNames[i].c_str( ) );
        if ( !file.good( ) ) {
            std::cerr << "Cannot open file '" << mapFileNames[i] << "'" << std::endl;
            return 1;
        } // if
    
        std::stringstream text;
        text << file.rdbuf( );
        file.close( );
        maps[i] = spatial::Octree3DMapManager::fromString( text.str( ) );
    } // for

    std::cout << "Number of successfully loaded maps: " << maps.size( ) << std::endl;

    opencog::spatial::MapExplorerServer server( host, port );
    server.start( );

    std::cout << "Visual Debugger Server is now running on " << host << ":" << port << std::endl;
    std::cout << "Press ctrl+c to stop" << std::endl;
    
    bool running = server.isRunning( );
    unsigned int currentMapIndex = 0;
    unsigned int previousMapIndex = 1;
    while ( running ) {
        
        if ( previousMapIndex != currentMapIndex ) {
            server.sendMap( *maps[currentMapIndex] );
            previousMapIndex = currentMapIndex;
        } // if
        currentMapIndex = (currentMapIndex+1) % maps.size( );
        
        boost::thread::sleep( boost::get_system_time() + boost::posix_time::milliseconds(mapVisualizationTimeout) );
        running = server.isRunning( ) && !externalInterruption;
    } // while

    server.stop( );

    for( i = 0; i < maps.size( ); ++i ) {
        delete maps[i];
    } // for

    std::cout << "bye!" << std::endl;
    
    return 0;
}


bool interruption = false;

void handleCtrlC( int code )
{
    std::cout << std::endl << "Ctrl+c pressed. Now closing MapTools..." << std::endl;
    interruption = true;
}

int main( int argc, char* argv[] )
{   

    boost::program_options::options_description help("Usage");
    std::stringstream videoModeHelp;
    videoModeHelp << "window resolution and format"<< std::endl;
    videoModeHelp << "   Available Modes:" << std::endl;
    videoModeHelp << "   1 = 800x600 windowed" << std::endl;
    videoModeHelp << "   2 = 800x600 fullscreen" << std::endl;
    videoModeHelp << "   3 = 1024x768 windowed" << std::endl;
    videoModeHelp << "   4 = 1024x768 fullscreen" << std::endl;        

    help.add_options()
        ("help", "display this help and exit")
        ("help-mode", boost::program_options::value<std::string>(), "display help for a given mode: local, remote or server")
        ("mode", boost::program_options::value<std::string>(), "operation mode: local, remote or server")
        ;

    boost::program_options::options_description local("Local mode");
    local.add_options()
        ("mapfile", boost::program_options::value<std::string>(), "Map dump file in ASCII format")
        ("video-mode", boost::program_options::value<unsigned int>(), videoModeHelp.str( ).c_str( ) )
        ;

    boost::program_options::options_description remote("Remote mode");
    remote.add_options()
        ("host", boost::program_options::value<std::string>(), "ip or host address of the Visual Debugger server")
        ("port", boost::program_options::value<std::string>(), "port of the Visual Debugger server")
        ("video-mode", boost::program_options::value<unsigned int>(), videoModeHelp.str( ).c_str( ) )
        ;

    boost::program_options::options_description server("Server mode");
    server.add_options()
        ("host", boost::program_options::value<std::string>(), "ip or host address used to get up the server")
        ("port", boost::program_options::value<std::string>(), "port used by the server")
        ("mapfile-list", boost::program_options::value<std::string>(), "List of map dump files formated in ASCII separate by colons. i.e. file1.map,file2.map;file3.map" )
        ("map-timeout", boost::program_options::value<unsigned int>(), "Time in milliseconds used to show each map" )
        ;

    boost::program_options::options_description all("Options");
    all.add(help).add(local).add(remote).add(server);

    boost::program_options::variables_map vm;
    try {        
        boost::program_options::store(boost::program_options::parse_command_line(argc, argv, all), vm);
        boost::program_options::notify(vm);


        if (vm.count("help")) {
            std::cout << help << std::endl;
            return 1;
        } else if ( vm.count("help-mode") ) {
            const std::string& mode = vm["help-mode"].as<std::string>();
            if ( mode == "local" ) {
                std::cout << local << std::endl;
            } else if ( mode == "remote" ) {
                std::cout << remote << std::endl;
            } else if ( mode == "server" ) {
                std::cout << server << std::endl;
            } else {
                std::cout << "Unknown mode '" 
                          << mode 
                          << "' in the --help-mode option" 
                          << std::endl;
                return 1;
            } // else
 
        } else if ( vm.count( "mode" ) ) {
            const std::string& mode = vm["mode"].as<std::string>();

            unsigned int videoMode = 0;
            if ( mode == "local" || mode == "remote" ) {
                if ( !vm.count( "video-mode" ) ) {
                    std::cout << "Argument '--video-mode' must be informed" << std::endl;
                    return 1;            
                } // if

                videoMode = vm["video-mode"].as<unsigned int>();
                if ( videoMode < 1 || videoMode > 4 ) {
                    std::stringstream errorMsg;
                    errorMsg << "argument --vide-mode only accept a value between 1 and 4, but '" << videoMode << "' was given";
                    throw boost::program_options::invalid_option_value( errorMsg.str( ) );
                } // if
            } // if

            if ( mode == "local" ) {
                if ( !vm.count( "mapfile" ) ) {
                    std::cout << "Argument '--mapfile' must be informed" << std::endl;
                    return 1;
                } // if
                MapTools tools( videoMode );
                return tools.runLocalMode( vm["mapfile"].as<std::string>( ) );
            } else if ( mode == "remote" ) {
                unsigned int i;
                for( i = 0; i < remote.options( ).size( ); ++i ) {
                    if ( !vm.count( remote.options( )[i]->long_name( ) ) ) {
                        std::cout << "Argument '--" 
                                  << remote.options( )[i]->long_name( ) 
                                  << "' must be informed" << std::endl;
                        return 1;
                    } // if
                } // for
                std::string host = vm["host"].as<std::string>( );
                std::string port = vm["port"].as<std::string>( );

                signal(SIGINT, handleCtrlC );
                MapTools tools( videoMode );
                exit( tools.runRemoteMode( host, port, interruption ) );

            } else if ( mode == "server" ) {
                unsigned int i;
                for( i = 0; i < server.options( ).size( ); ++i ) {
                    if ( !vm.count( server.options( )[i]->long_name( ) ) ) {
                        std::cout << "Argument '--" 
                                  << server.options( )[i]->long_name( ) 
                                  << "' must be informed" << std::endl;
                        return 1;
                    } // if
                } // for
                std::string host = vm["host"].as<std::string>( );
                std::string port = vm["port"].as<std::string>( );
                std::string mapfileList = vm["mapfile-list"].as<std::string>( );
                unsigned int mapTimeout = vm["map-timeout"].as<unsigned int>( );
                
                std::vector<std::string> mapFileNames;
                boost::split( mapFileNames, mapfileList, boost::is_any_of(",") );

                signal(SIGINT, handleCtrlC );
                MapTools tools;
                exit( tools.runServerMode( host, port, mapTimeout, mapFileNames, interruption ) );
            } else {
                std::cout << "Unknown mode '" 
                          << mode 
                          << "' in the --mode option" 
                          << std::endl;
                return 1;
            } // else                                
        } else {
            std::cout << "Try `" << argv[0] << " --help' for more information" << std::endl;
            return 1;
        } // else    

    } catch ( std::exception& ex ) {
        std::cerr << "Error: " << ex.what( ) << std::endl;
        std::cout << "Try `" << argv[0] << " --help' for more information" << std::endl;
        return 1;
    } // catch    

    return 0;
}
