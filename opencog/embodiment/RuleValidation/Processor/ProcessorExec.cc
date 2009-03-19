#include "RuleProcessor.h"
#include "util/files.h"
#include "util/exceptions.h"
#include <SystemParameters.h>

int main(int argc, char * argv[]) {

    if(argc != 3){
        fprintf(stdout, "processor <scenario-file> <type: pet or humanoid>\n");
        return (1);
    }

    Control::SystemParameters parameters;
    if(fileExists(parameters.get("CONFIG_FILE").c_str())){
    	parameters.loadFromFile(parameters.get("CONFIG_FILE"));
    }

    if((strcmp(argv[2], "pet") != 0) &&
       (strcmp(argv[2], "humanoid") != 0)){
        fprintf(stdout, "processor <scenario-file> <type: pet or humanoid>. Got '%s'.\n", argv[2]);
        return (1);
    }

    Processor::RuleProcessor rp(parameters,std::string(argv[2]));

    try{
        rp.evaluateRules(std::string(argv[1]));
    } catch(...){
        fprintf(stdout, "An error has occured while evaluating rules. Check log.\n");
        return (1);
    }
    return (0);
}
