/*
 * opencog/spatial/MapTools.h
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

#ifndef MAPTOOLS_H
#define MAPTOOLS_H

#include <string>
#include <boost/asio.hpp>

namespace opencog 
{
/** \addtogroup grp_spatial
 *  @{
 */
    namespace spatial 
    {
        class MapTools 
        {
        public:
            
            MapTools( unsigned int videoMode = 4 );
            
            int runRemoteMode( const std::string& host, const std::string& port, 
                               bool& externalInterruption );

            int runLocalMode( const std::string& fileName );

            int runServerMode( const std::string& host, const std::string& port, 
                               unsigned int mapVisualizationTimeout,
                               const std::vector<std::string>& mapFileNames,
                               bool& externalInterruption );

        private:            
            
            void remoteModeClientLoop( void );

            unsigned int windowWidth;
            unsigned int windowHeight;
            bool fullScreen;            

            boost::asio::io_service visualDebuggerClientService;
        };

    } // spatial
/** @}*/
} // opencog

#endif // MAPTOOLS
