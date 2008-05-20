#include "TimeServer.h"
#include "Logger.h"

int TimeServer::timeServerEntrys = 0;
// USED TO SEEK MEMORY LEAK
//std::set<Temporal> TimeServer::temporalSet;

void TimeServer::init() {
    table = new TemporalTable();
    latestTimestamp = 0;
}

TimeServer::TimeServer() {
    init();
}

TimeServer::~TimeServer() {
    delete table;
}
    
void TimeServer::add(Handle h, const Temporal& t) {
    // USED TO SEEK MEMORY LEAK
    //++timeServerEntrys;
    //cout << "Total timeServerEntrys: " << timeServerEntrys << endl;

    //if(temporalSet.find(t) == temporalSet.end()){
    //   temporalSet.insert(t);
    //   cout << "Total unique entrys: " << temporalSet.size() << endl;
    //}

    table->add(h, t);
    if (t.getUpperBound() > latestTimestamp) {
        latestTimestamp = t.getUpperBound();
    }
}
    
bool TimeServer::remove(Handle h, const Temporal& t, TemporalTable::TemporalRelationship criterion)
{
    return table->remove(h, t, criterion);
}

const char* TimeServer::getId() const
{
    static const char* id = "TimeServer";
    return id;
}

void TimeServer::saveRepository(FILE *fp) const
{
    MAIN_LOGGER.log(Util::Logger::DEBUG, "Saving %s (%ld)\n", getId(), ftell(fp));
    // Saves TemporalTable
    table->save(fp);
}

void TimeServer::loadRepository(FILE *fp, HandleMap<Atom *> *conv)
{
    MAIN_LOGGER.log(Util::Logger::DEBUG, "Loading %s (%ld)\n", getId(), ftell(fp));
    // Loads the TemporalTable
    table->load(fp, conv);
}

void TimeServer::clear()
{
    delete table;
    init();
}

unsigned long TimeServer::getLatestTimestamp() const
{
    return latestTimestamp;
}
