/*
 * opencog/persist/file/TimeServerSavable.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Welter Silva <welter@vettalabs.com>
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

#include "TimeServerSavable.h"
#include "TemporalTableFile.h"

#include <opencog/util/Logger.h>

using namespace opencog;

TimeServerSavable::TimeServerSavable()
{
    timeserver = NULL;
}

TimeServerSavable::~TimeServerSavable()
{
    timeserver = NULL;
}

const char* TimeServerSavable::getId() const
{
    static const char* id = "TimeServer";
    return id;
}

void TimeServerSavable::saveRepository(FILE* fp) const
{
	logger().debug("Saving %s (%ld)\n", getId(), ftell(fp));
    //Saves TemporalTable
    size_t size=(timeserver->temporalTableMap).size();
	fwrite(&size,sizeof(size_t),1,fp);
	for(auto it=(timeserver->temporalTableMap).begin();it!=(timeserver->temporalTableMap).end();it++)
	{
		const string timeDomainName=it->first;
        char chartimeDomainName[128];
        memset(chartimeDomainName, '\0',strlen(timeDomainName.c_str()));
        strcpy(chartimeDomainName,timeDomainName.c_str());
		logger().error("timeserver save repo timedomain name: %s",chartimeDomainName);
        fwrite(&chartimeDomainName, 128,1,fp);

		TemporalTableFile ttf;
		ttf.save(fp,it->second);
	}
}

void TimeServerSavable::loadRepository(FILE* fp, HandMapPtr conv)
{
	logger().debug("Loading %s (%ld)\n", getId(), ftell(fp));
    bool b_read = true;
	size_t size=0;
    FREAD_CK(&size, sizeof(size_t), 1, fp);
    // Loads the TemporalTable
	for(unsigned int i=0;i!=size;i++)
	{
		char chartimeDomainName[128];
        FREAD_CK(&chartimeDomainName, 128, 1, fp);
		std::string timeDomainName(chartimeDomainName);
		logger().error("load timeserver repo: timedomainname %s",chartimeDomainName);
		(timeserver->temporalTableMap)[timeDomainName]=new TemporalTable();

		TemporalTableFile ttf;
		ttf.load(fp,(timeserver->temporalTableMap)[timeDomainName], conv);
	}
	CHECK_FREAD
}

void TimeServerSavable::clear()
{
    timeserver->clear();
}

