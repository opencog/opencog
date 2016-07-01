#include <SCMLoader.h>

using opencog;

bool load(const std::string &fileName, AtomSpace &atomSpace)
{
    bool exitValue = false;

    std::fstream fin(fileName, std::fstream::in);
    if (fin.good()) {
        logger().info("Parsing file: \"%s\"", fileName);
        parseFile(fin, atomSpace);
        fin.close();
    } else {
        logger().error("Could not open file: \"%s\"", fileName);
        exitValue = true;
    }

    return exitValue;
}

void parseFile(std::fstream &fin, AtomSpace &atomSpace)
{
    SchemeEval *schemeEval = SchemeEval::get_evaluator(atomSpace);
    int level = 0;
    char c;
    std::string line = "";
    while (fin >> std::noskipws >> c) {
        if (c != '\n') {
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
                    schemeEval->eval(line);
                    line = "";
                }
                break;
            }
            default: {
            }
        }
    }
}
