/*
 * SCMAtomsParser.h
 *
 * Copyright (C) 2016 OpenCog Foundation
 *
 * Author: Andre Senna <https://github.com/andre-senna>
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

#ifndef _OPENCOG_SCMATOMSPARSER_H
#define _OPENCOG_SCMATOMSPARSER_H


namespace opencog
{

/**
 * 
 */
class SCMAtomsParser 
{

public:

    ~SCMAtomsParser();
    SCMAtomsParser(const char *fileName);

private:

    std::vector<AtomPtr> atomsList;

};

}

#endif // _OPENCOG_SCMATOMSPARSER_H
