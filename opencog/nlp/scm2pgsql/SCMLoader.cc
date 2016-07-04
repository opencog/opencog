#include "SCMLoader.h"
#include <opencog/guile/SchemeEval.h>


using namespace opencog;
using namespace std;

bool SCMLoader::load(const std::string &fileName, AtomSpace &atomSpace)
{
    bool exitValue = false;

    std::fstream fin(fileName, std::fstream::in);
    if (fin.good()) {
        /*
        std::fstream fcount(fileName, std::fstream::in);
        int lineCount = 0;
        std::string line;
        while (std::getline(fcount, line)) {
            ++lineCount;
        }
        fcount.close();
        */
        logger().info("Parsing file: \"%s\"", fileName.c_str());
        //parseFile(fin, atomSpace, lineCount);
        parseFile(fin, atomSpace, -1);
        logger().info("Parse done.");
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
    int inputFileLineCount = 0;
    int level = 0;
    char c;
    std::string line = "";
    while (fin >> std::noskipws >> c) {
        if (c == '\n') {
            if (inputFileSize > 0) {
                // Logs current % done
                inputFileLineCount++;
                int n = (((float) inputFileLineCount) / inputFileSize) * 100;
                if (n > percentDone) {
                    logger().info("%d%% done", percentDone);
                    percentDone = n;
                }
            }
        } else {
            line += c;
        }
        switch (c) {
            case '(': {
                level++;
                break;
            }
            case ')': {
                if (--level == 0) {
                    //printf("%s\n", line.c_str());
                    //line = "(WordInstanceNode \"-\")";
                    //line = "(WordInstanceNode \"ola\")";
                    std::string result = schemeEval->eval(line);
                    result = schemeEval->eval("(count-all)");
                    printf("%s\n", result.c_str());
                    line = "";
                    //return;
                }
                break;
            }
            default: {
            }
        }
    }
}
