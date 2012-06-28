#ifndef DESTIN_CUDA_H
#define DESTIN_CUDA_H

#include "AdviceData.h"
#include "DestinData.h"
#include "DestinKernel.h"
#include "DestinIterationFinishedCallback.h"

//pugiXML read/writer
#include "pugixml/pugixml.hpp"

#include <iostream>
#include <stdio.h>
#include <vector>
#include <fstream>
#include <ctime>
#include <sstream>
#include <sys/stat.h>
#include <math.h>
// CUDA Lib
#include <curand.h>

#ifdef _WIN32
#include <direct.h>
#include <string>
#else
// Linux only requirements...
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#endif

struct CommandArgsStuc {

    string strDestinNetworkFileToRead;
    string strTesting;
    string strDestinTrainingFileName;
    int MAX_CNT;
    int seed;
    string sCodeWord;
    string ParametersFileName;
};

class DestinCuda {
    
private:
    DestinKernel * DKernel; //array of DestinKernel objects, one for each layer.
    
    
    string GetNextFileForDiagnostic();
    
    void GetParameters( const char* cFilename, int& NumberOfLayers, double*& dcMu, double*& dcSigma, double*& dcRho,
                    int*& NumberOfStates, bool& bAveraging,bool& bFFT,bool& bBinaryPOS,int* DistanceMeasureArray,
                    bool& bUseStarvationTrace,int& PSSAUpdateDelay,bool& bIgnoreAdvice,
                    int**& SEQ, int& SEQ_LENGTH, string& sFileContents, int& iBlocksToProcess,
                    bool& bBasicOnlineClustering,
                    bool& bClanDestin, bool& bInitialLayerIsTransformOnly,bool& bUseGoodPOSMethod,
                    int*& RowsPerLayer, float*& FixedLearningRateLayer, bool*& bSelfAndUpperFeedback, int& LastLayerInputX, int& LastLayerInputY );
    
    bool FileExists(string strFilename);
    
    bool CreateDestinOnTheFly(string ParametersFileName, int& NumberOfLayers, DestinKernel*& DKernel,
                          DestinData& DataSourceForTraining, int& SEQ_LENGTH, int**& SEQ,
                          int*& ImageInput);
    
public:
    int parseCommandArgs( int argc, char* argv[], CommandArgsStuc &out);

    
    void PrintHelp();
    int MainDestinExperiments(CommandArgsStuc & argsOut);
};      

#endif
