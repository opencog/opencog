#include <fstream>
#include <iostream>

#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>

#ifdef WIN32
#include <direct.h>
#define  mkdir _mkdir    
#endif

#define USER_FLAG "$USER"

bool fileExists(const char* filename){     
    std::fstream dumpFile(filename, std::ios::in);
    dumpFile.close();
        
    if(dumpFile.fail()){
        dumpFile.clear(std::ios_base::failbit);
        return false;
    }
    return true;
}

void expandPath(std::string& path){
    
    size_t user_index = path.find(USER_FLAG, 0);
    if (user_index != std::string::npos) {
        const char* username = getenv("LOGNAME");

        if (username == NULL){
            username = "unknown_user";
        }
        path.replace(user_index, strlen(USER_FLAG), username);
    }
    
    return;
}

bool createDirectory(const char* directory){

#ifdef WIN32    
    if(mkdir(directory) == 0 || errno == EEXIST){
#else
    if(mkdir(directory, S_IRWXU | S_IRWXG | S_IRWXO) == 0|| errno == EEXIST){
#endif       
        return true;
    }
    return false;    
}

bool appendFileContent(const char* filename, std::string &s)
{
    std::ifstream in(filename);
    if (!in.is_open())
        return false;

    char c;
    std::string str;
    while (in.get(c))
        str += c;

    if (!in.eof())
        return false;

    s = str;
    return true;
}

