#include "SCMLoader.h"
#include <opencog/guile/SchemeEval.h>


using namespace opencog;
using namespace std;

bool SCMLoader::load(const std::string &fileName, AtomSpace &atomSpace)
{
    bool exitValue = false;

    logger().set_sync_flag(true);
    std::fstream fin(fileName, std::fstream::in);
    if (fin.good()) {
        ifstream fcount(fileName, ios::binary | ios::ate);
        int fileSize = fcount.tellg();
        fcount.close();
        logger().info("Loading file: \"%s\"", fileName.c_str());
        parseFile(fin, atomSpace, fileSize);
        logger().info("Loading finished.");
        fin.close();
    } else {
        logger().error("Could not open file: \"%s\"", fileName.c_str());
        exitValue = true;
    }

    return exitValue;
}

void SCMLoader::parseFile(std::fstream &fin, AtomSpace &atomSpace, int inputFileSize)
{
    SchemeEval *schemeEval = SchemeEval::get_evaluator(&atomSpace);
    int percentDone = 0;
    int inputFileCharCount = 0;
    bool firstCharInLine = true;
    char c;
    std::string line = "";
    while (fin >> std::noskipws >> c) {
        inputFileCharCount++;
        if (c == '\n') {
            firstCharInLine = true;
            if (inputFileSize > 0) {
                // Logs current % done
                int n = (((float) inputFileCharCount) / inputFileSize) * 100;
                if ((n > percentDone) && (n < 100)) {
                    percentDone = n;
                    logger().info("Loading file. %d%% done.", percentDone);
                }
            }
        } else {
            line += c;
            if (firstCharInLine && (c == ')')) {
                schemeEval->eval(line);
                line = "";
            }
            firstCharInLine = false;
        }
    }
    std::string output = schemeEval->eval("(count-all)");
    logger().info("Atom count after loading: %s", output.c_str());
}
