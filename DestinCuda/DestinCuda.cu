#include "AdviceData.h"
#include "DestinData.h"
#include "DestinKernel.h"

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

using namespace std;

// TODO: This main file is still pretty messy.
/*
 * It's almost a copy of the original main of DeSTIN where the custom file reader is replaced by a XML reader(pugixml)
 * Because of the quite basic understanding of the code and a more overall understanding of DeSTIN there are a lot of variables not been used.
 * Also cause there is still work in progress.
 * It still might be a idea to just replace the command line complete and do everything from the configuration
 * This will save a lot of code to analyze the input arguments
 */
void PrintHelp()
{
    // ***************************
    // Print out how to use DeSTIN
    // ***************************

    cout << "Usage: DestinCuda CodeWord MAXCNT LayerToShow ParamsFile TrainingDataFile DestinOutputFile TargetDirectory [OutputDistillationLevel]" << endl;
    cout << "Where:" << endl;
    cout << "    CodeWord must have 11 digits RRRRXXYYYYY" << endl;
    cout << "        RRRR  = 0000 to 9999 where 0000 is real random time." << endl;
    cout << "        XX    = 01 to 99 number of classes will not be higher then training file." << endl;
    cout << "        YYYYY = 00000 to 99999 number of examples of each class." << endl;
    cout << "                00000 means RANDOMLY PICK EXAMPLES until we finish clustering, period, up to max iterations." << endl;
    cout << "    MAXCNT is the number of digits we show it to train the unsupervised DeSTIN architecture" << endl;
    cout << "    LayerToShow = layer written to output file; it is given as S:E:O:P:T where " << endl;
    cout << "        S = first layer to write" << endl;
    cout << "        E = last layer to write" << endl;
    cout << "        O = offset for movements to write" << endl;
    cout << "        P = period of movements to write" << endl;
    cout << "        T = type.  Nothing (and no !) is beliefs.  Type can be: " << endl;
    cout << "            A is belief in advice states computed by tabular method." << endl;
    cout << "            N is belief in advice states computed by neural network function approximator." << endl;
    cout << "            L is belief in advice states computed by linear function approximator." << endl;
    cout << "    ParamsFile is a file that has the run parameters" << endl;
    cout << "    TrainingDataFile is the binary data file for training.  A testing file with the SAME NAME and appended with _TESTING is assumed" << endl;
    cout << "    DestinOutputFile is the name of the DeSTIN network output file for saving." << endl;
    cout << "         Use -D as default, which is the experiment number with a .xml at the end, in the TargetDirectory directory" << endl;
    cout << "    TargetDirectory is where we want to put the MAIN OUTPUT DATA FILES.  We ALWAYS write an experiment marker to the " << endl;
    cout << "        ../DiagnosticData area.  But if you are writing out a lot of data you can specify another directory." << endl;
    cout << "        Put D for default which is the ../DiagnosticData area." << endl;
    cout << "    [OutputDistillationLevel] is optional.  If this exists it must be a number and currently its got to be 0.  "<<endl;
    cout << "        0 = regular outputs with a lot of details about movements and processing: this is our input to SampleAndStack"<<endl;
    cout << "        1 = outputs compatible with the regular distilled output of SampleAndStack. If you use this you can skip SampleAndStack.exe" << endl;
    cout << endl;
    cout << "-OR-" << endl;
    cout << endl;
    cout << "Usage: DestinCuda -F InputNetworkFile LayerToShow ParamsFile TrainingDataFile DestinOutputFile TargetDirectory [OutputDistillationLevel]" << endl;
    cout << "Where:" << endl;
    cout << "    -F signifies use a saved DeSTIN network file " << endl;
    cout << "    InputNetworkFile is the NAME of the saved DeSTIN network file" << endl;
    cout << "    All others are as in first usage type" << endl;
    cout << endl;
}

bool FileExists(string strFilename)
{
    // **************************
    // Does the given file exists
    // **************************
    // For detailed information look the return values of stat

    struct stat stFileInfo;
    bool blnReturn;
    int intStat;

    // Attempt to get the file attributes
    intStat = stat(strFilename.c_str(),&stFileInfo);
    if(intStat == 0) {
        // File exists
        blnReturn = true;
    }
    else
    {
        // File not exists or no permission
        blnReturn = false;
    }

    return(blnReturn);
}

string GetNextFileForDiagnostic()
{
    // *************************************
    // Find next available experimental file
    // *************************************
    // Check if there is a previous experiment inside ../DiagnosticData

    string strFileName;
    int iExperimentNumber=-1;
    bool bFileFound = true;
    while ( bFileFound )
    {
        iExperimentNumber++;
        stringstream buffer;
        buffer << "../DiagnosticData/DestinDiagnostics" << iExperimentNumber << "-0.xml";
        strFileName =  buffer.str();

        bFileFound = FileExists(strFileName);
        strFileName.erase(strFileName.length()-6,2);
    }
    strFileName = strFileName.substr(18);

    return strFileName;
}

void GetParameters( const char* cFilename, int& NumberOfLayers, double*& dcMu, double*& dcSigma, double*& dcRho,
                    int*& NumberOfStates, bool& bAveraging,bool& bFFT,bool& bBinaryPOS,int* DistanceMeasureArray,
                    bool& bUseStarvationTrace,int& PSSAUpdateDelay,bool& bIgnoreAdvice,
                    int**& SEQ, int& SEQ_LENGTH, string& sFileContents, int& iBlocksToProcess,
                    bool& bBasicOnlineClustering,
                    bool& bClanDestin, bool& bInitialLayerIsTransformOnly,bool& bUseGoodPOSMethod,
                    int*& RowsPerLayer, float*& FixedLearningRateLayer, bool*& bSelfAndUpperFeedback, int& LastLayerInputX, int& LastLayerInputY )
{
    // ******************************************
    // Read the XML config file (parameters file)
    // ******************************************
    // This function is rewritten and is not backwards compatible with the DestinPort one.
    // Instead of a txt file its now a XML file pugixml is used for parsing it.
    ifstream stmInput(cFilename);
    string sBuffer;
    // Put the config file into a vector and as one big string back to sFileCOntents
    while ( getline(stmInput, sBuffer) )
    {
        sFileContents = sFileContents + "~" + sBuffer + "\n";
    }
    stmInput.close();

    pugi::xml_document xFile;

    pugi::xml_parse_result result = xFile.load_file(cFilename);
    std::cout << "XML config file Load result: " << result.description() << endl;
    if ( result )
    {
        // Root node is destin
        pugi::xml_node root = xFile.child("destin");

        // Retrieve SEQ_LENGTH
        pugi::xml_node seq = root.child("seq");
        SEQ_LENGTH = seq.attribute("length").as_int();

        // Retrieve all steps
        SEQ = new int*[SEQ_LENGTH];
        pugi::xml_node step = seq.child("step");
        for( int iStep = 0; iStep < SEQ_LENGTH; iStep++ )
        {
            SEQ[iStep]=new int[2];
            SEQ[iStep][0] = step.attribute("x").as_int();
            SEQ[iStep][1] = step.attribute("y").as_int();
            step = step.next_sibling("step");
        }

        // Retrieve amount of layers
        pugi::xml_node layers = root.child("layers");
        NumberOfLayers = layers.attribute("value").as_int();
        LastLayerInputX = layers.attribute("inputX").as_int();
        LastLayerInputY = layers.attribute("inputY").as_int();

        // Retrieve configuration each layer
        dcMu = new double[NumberOfLayers];
        dcSigma = new double[NumberOfLayers];
        dcRho = new double[NumberOfLayers];
        NumberOfStates = new int[NumberOfLayers];
        DistanceMeasureArray = new int[NumberOfLayers];
        RowsPerLayer = new int[NumberOfLayers];
        FixedLearningRateLayer = new float[NumberOfLayers];
        bSelfAndUpperFeedback = new bool[NumberOfLayers];
        pugi::xml_node layer = layers.child("layer");
        // Loop true each layer configuration
        for( int iLayer = 0; iLayer < NumberOfLayers; iLayer++ )
        {
            dcMu[iLayer] = layer.attribute("mu").as_double();
            dcSigma[iLayer] = layer.attribute("sigma").as_double();
            dcRho[iLayer] = layer.attribute("rho").as_double();
            NumberOfStates[iLayer] = layer.attribute("states").as_int();
            DistanceMeasureArray[iLayer] = layer.attribute("distance").as_int();
            RowsPerLayer[iLayer] = layer.attribute("rowsColmsPerLayer").as_int();
            FixedLearningRateLayer[iLayer] = layer.attribute("fixedLearningRate").as_float();
            bSelfAndUpperFeedback[iLayer] = layer.attribute("selfAndUpperFeedback").as_bool();
            layer = layer.next_sibling("layer");
        }

        // Retrieve settings for overal DeSTIN
        pugi::xml_node settings = root.child("settings");
        bAveraging = settings.child("averaging").attribute("value").as_bool();
        bFFT = settings.child("fft").attribute("value").as_bool();
        bBinaryPOS = settings.child("binaryPos").attribute("value").as_bool();
        bUseStarvationTrace = settings.child("starvationTrace").attribute("value").as_bool();
        PSSAUpdateDelay = settings.child("pssaDelay").attribute("value").as_int();
        bIgnoreAdvice = settings.child("ignoreAdvice").attribute("value").as_bool();
        iBlocksToProcess = settings.child("processingBlockSize").attribute("value").as_int();
        bBasicOnlineClustering = settings.child("basicOnlineClustering").attribute("value").as_bool();
        bClanDestin = settings.child("clanDestin").attribute("value").as_bool();
        bInitialLayerIsTransformOnly = settings.child("initialLayerIsTransformOnly").attribute("value").as_bool();
        bUseGoodPOSMethod = settings.child("useGoodPOSMethod").attribute("value").as_bool();
    }
    else
    {
        std::cout << "Error description: " << result.description() << "\n";
        std::cout << "Error offset: " << result.offset << " (error at [..." << (cFilename + result.offset) << "]\n\n";
    }
    cout << "------------------" << endl;
}

bool CreateDestinOnTheFly(string ParametersFileName, int& NumberOfLayers, DestinKernel*& DKernel,
                          DestinData& DataSourceForTraining, int& SEQ_LENGTH, int**& SEQ,
                          int*& ImageInput)

{
    // *********************
    // Create DeSTIN network
    // *********************

    double* dcMu;
    double* dcSigma;
    double* dcRho;
    int* NumberOfCentroids;
    bool bAveraging;
    bool bFFT;
    bool bBinaryPOS;
    int DistanceMeasureArray[128];
    bool bUseStarvationTrace;
    int PSSAUpdateDelay;
    bool bIgnoreAdvice;
    string sParametersFileContents;
    int iBlocksToProcess;
    bool bBasicOnlineClustering;
    bool bClanDestin;
    bool bInitialLayerIsTransformOnly;
    bool bDoGoodPOS;
    int* RowsPerLayer;
    float* FixedLearningRateLayer;
    bool* bSelfAndUpperFeedback;
    ImageInput = new int[2];

    GetParameters( ParametersFileName.c_str(), NumberOfLayers, dcMu, dcSigma, dcRho, NumberOfCentroids,
                   bAveraging, bFFT, bBinaryPOS, DistanceMeasureArray,
                   bUseStarvationTrace, PSSAUpdateDelay, bIgnoreAdvice, SEQ, SEQ_LENGTH,
                   sParametersFileContents, iBlocksToProcess,
                   bBasicOnlineClustering, bClanDestin, bInitialLayerIsTransformOnly, bDoGoodPOS,
                   RowsPerLayer, FixedLearningRateLayer, bSelfAndUpperFeedback, ImageInput[0], ImageInput[1]);

    // The name and loop looks like it is giving to option to save steps of the movements.
    vector<bool> vectorOfMovementsToSave;
    for( int c=0;c<SEQ_LENGTH;c++ )
    {
        vectorOfMovementsToSave.push_back(false);
    }

    DKernel = new DestinKernel[NumberOfLayers];
    int* ColsPerLayer = new int[NumberOfLayers];
    int* NumberOfParentStates = new int[NumberOfLayers];
    int* InputDimensionality = new int[NumberOfLayers];
    int* OffsetSelf = new int[NumberOfLayers];
    int* OffsetSelfFeedback = new int[NumberOfLayers];

    int MovementsForClusteringOption = 1;
    if ( bClanDestin )
    {
        MovementsForClusteringOption=SEQ_LENGTH; // use this if you want one clustering engine per movement
    }

    // These are changed inside the XML file
    if ( bFFT )
    {
        InputDimensionality[0] = 10;  //4x4 FFT has 10 unique magnitude values...
    }
    else
    {
        InputDimensionality[0] = 16;  //4x4 has 16 inputs
    }

    if ( bInitialLayerIsTransformOnly )
    {
        InputDimensionality[1] = NumberOfCentroids[0];
        NumberOfParentStates[0] = NumberOfCentroids[1];
        RowsPerLayer[1] = RowsPerLayer[0];
        for( int Layer=2; Layer<NumberOfLayers; Layer++ )
        {
            InputDimensionality[Layer] = 4*NumberOfCentroids[Layer-1];
            NumberOfParentStates[Layer-1] = NumberOfCentroids[Layer];
            RowsPerLayer[Layer] = RowsPerLayer[Layer-1]/2;
        }
    }
    else
    {
        for(int Layer=1; Layer<NumberOfLayers; Layer++ )
        {
            InputDimensionality[Layer] = 4*NumberOfCentroids[Layer-1];
            NumberOfParentStates[Layer-1] = NumberOfCentroids[Layer];
        }
    }
    NumberOfParentStates[NumberOfLayers-1]=1;
    //if you want to FORCE stability, an exponential / gaussian decay will start at iDecayPoint
    bool bUseDecayLR = false;
    int DigitToStartDecay=1000;
    int iDecayPoint = SEQ_LENGTH*DigitToStartDecay;
    float fRhoThreshold = (float)(1e-2);
    bool bUseRhoDerivative = false;
    int MaxNumberOfInputs=-1;
    int MaxNumberOfOutputs=-1;

    // curandGenerator_t is a CUDA version of rand
    // This fills the whole memory block with number between 0.0 and 1.0
    curandGenerator_t gen;
    curandCreateGenerator(&gen, CURAND_RNG_PSEUDO_DEFAULT);
    // TODO: Add seed code instead of 1
    // This is the right place to do this saves the most time creating numbers. (Inside layer increase the time by +/- 5 times)
    curandSetPseudoRandomGeneratorSeed( gen, 1 );

    // Here we put the first image into the device memory
    for( int Layer=0; Layer<NumberOfLayers; Layer++ )
    {
        bool bTopNode = false;
        bool bAveragingLayer = false;
        bool bConstrainInitialCentroids = true;

        OffsetSelf[Layer] = InputDimensionality[Layer]; // the basic value.
        ColsPerLayer[Layer] = RowsPerLayer[Layer];
        // increase the input dimensionality if you are using the self-upper feedback
        if ( bSelfAndUpperFeedback[Layer] )
        {
            InputDimensionality[Layer] = InputDimensionality[Layer]+NumberOfCentroids[Layer];  // layers get self-feedback
            OffsetSelfFeedback[Layer] = InputDimensionality[Layer];
            if ( Layer<NumberOfLayers-1 )
            {
                InputDimensionality[Layer] = InputDimensionality[Layer]+NumberOfCentroids[Layer+1];  // all layers but the top get feedback from above
            }
        }
        // Initial layer does this a little different (input is raw instead of centroids)
        if ( Layer==0 )
        {
            bAveragingLayer=bAveraging;
            bConstrainInitialCentroids=false;
        }
        if ( Layer==NumberOfLayers-1)
        {
            // Yes you are the top layer
            bTopNode = true;
        }

        DKernel[Layer].Create( Layer, RowsPerLayer[Layer], ColsPerLayer[Layer], NumberOfCentroids[Layer], InputDimensionality[Layer], FixedLearningRateLayer[Layer], gen);
        // Assign Childeren and Parrents of nodes
        if ( NumberOfCentroids[Layer] > MaxNumberOfOutputs )
        {
            MaxNumberOfOutputs=NumberOfCentroids[Layer];
        }
        if ( InputDimensionality[Layer] > MaxNumberOfInputs )
        {
            MaxNumberOfInputs=InputDimensionality[Layer];
        }
    }
    // The generator have to be destroyed after use.
    curandDestroyGenerator( gen );
    cout << "------------------" << endl;
    return 0;
}

struct CommandArgsStuc {

	bool bCreateFromFile;
        string strDestinNetworkFileToRead;
};

/**
 *  parseCommandArgs
 *
 *  Parses the program command line arguments and fills
 *  the out CommandArgsStuc appropriately ( which you can
 *  see right now only consists of bCreateFromFile ).
 *
 *  This almost doesn'thave any side affects at the moment
 *  because most of the declared variables in this function
 *  are not used because most of the functionality it represents
 *  is not yet  ported from the single cpu version of Destin.
 */
int parseCommandArgs(  string strDiagnosticFileName, int argc, char* argv[], CommandArgsStuc &out){
    // Argument: TargetDirectory
    // A given location instead or default
    string strDiagnosticDirectoryForData;
    string strArg7 = argv[7];
    if ( strArg7 == "D" )
    {
        strDiagnosticDirectoryForData = "../DiagnosticData/";
    }
    else
    {
        // Buffer with path + filename where to put diagnostic data
        stringstream buffer;
        buffer << strArg7.c_str() << "/";
        strDiagnosticDirectoryForData = buffer.str();
    }

    // Argument: DestinOutputFile or InputNetworkFile

    string strDestinNetworkFileToWrite;
    string FirstArg = argv[1];
    if ( FirstArg=="-F" )
    {
        // Argument: InputNetworkFile
        out.bCreateFromFile = true;
        out.strDestinNetworkFileToRead = argv[2];  // we read from this file...

        if ( !FileExists( out.strDestinNetworkFileToRead ) )
        {
            cout << "designated input network file named " << out.strDestinNetworkFileToRead.c_str() << " does not exist" << endl;
            return 0;
        }
        cout << "Writing destin file to: " << strDestinNetworkFileToWrite << endl;
    }
    else
    {
        // Argument: DestinOutputFile
        out.bCreateFromFile = false;
        strDestinNetworkFileToWrite = argv[6]; // we write to this file, and then we read from it too!!
        if ( strDestinNetworkFileToWrite == "-D" )
        {
            // If given -D
            strDestinNetworkFileToWrite= strDiagnosticDirectoryForData + strDiagnosticFileName;
            cout << "Writing default destin file to: " << strDestinNetworkFileToWrite << endl;
        }
        out.strDestinNetworkFileToRead = strDestinNetworkFileToWrite;
    }


    // Create the old variable sDiagnosticFileNameForMarking
    // it have the location based on TargetDirectory + FileName
    string sDiagnosticFileNameForMarking;
    sDiagnosticFileNameForMarking = strDiagnosticDirectoryForData + strDiagnosticFileName;
    // Argument: LayerToShow
    // Structure of processing S:E:O:P:T
    // List of default values
    int FirstLayerToShowHECK = 3;
    int LastLayerToShow = FirstLayerToShowHECK;
    int iMovementOutputOffset = 0;
    int iMovementOutputPeriod = 1;
    OutputTypes eTypeOfOutput = eBeliefs;

    string sLayerSpecs = argv[3];
    int iColon = sLayerSpecs.find(":");
    if ( iColon == -1 || sLayerSpecs.substr(iColon).empty() )  //first layer = last layer, and no sampling specified.
    {
        // S
        FirstLayerToShowHECK=atoi(sLayerSpecs.c_str());
        LastLayerToShow=FirstLayerToShowHECK;
    }
    else
    {
        // S:E
        FirstLayerToShowHECK=atoi(sLayerSpecs.substr(0,1).c_str());
        LastLayerToShow=atoi(sLayerSpecs.substr(iColon+1,1).c_str());
        sLayerSpecs = sLayerSpecs.substr(iColon+1);
        iColon = sLayerSpecs.find(":");
        if ( iColon!=-1 && !( sLayerSpecs.substr(iColon).empty() ) )
        {
            //S:E:O
            sLayerSpecs = sLayerSpecs.substr(iColon+1);
            iMovementOutputOffset = atoi(sLayerSpecs.substr(0,1).c_str());
            iColon = sLayerSpecs.find(":");
            if ( iColon!=-1 && !( sLayerSpecs.substr(iColon).empty() ) )
            {
                //S:E:O:P
                sLayerSpecs = sLayerSpecs.substr(iColon+1);
                iMovementOutputPeriod = atoi(sLayerSpecs.substr(0,1).c_str());
                iColon = sLayerSpecs.find(":");
                if ( iColon!=-1 && !( sLayerSpecs.substr(iColon).empty() ) )
                {
                    //S:E:O:P:T
                    sLayerSpecs = sLayerSpecs.substr(iColon+1);
                    if ( sLayerSpecs.substr(0,1)=="A" )
                    {
                        eTypeOfOutput = eBeliefInAdviceTabular;
                    }
                    else if ( sLayerSpecs.substr(0,1)=="B" )
                    {
                        eTypeOfOutput = eBeliefs;
                    }
                    else if ( sLayerSpecs.substr(0,1)=="N" )
                    {
                        eTypeOfOutput = eBeliefInAdviceNNFA;
                    }
                    else if ( sLayerSpecs.substr(0,1)=="L" )
                    {
                        eTypeOfOutput = eBeliefInAdviceLinearFA;
                    }
                    else
                    {
                        cout << "Do not understand the output type " << sLayerSpecs.c_str() << endl;
                        return 1;
                    }
                }
            }
        }
    }

    // Optional argument: OutputDistillationLevel
    // This will write out a distilled movement log file this movement log matches that what SampleAndStack would produce.
    int OutputDistillationLevel = 0; //default level
    if ( argc == 9 )
    {
        OutputDistillationLevel = atoi(argv[8]);
    }


    return 0;
}


int MainDestinExperiments(int argc, char* argv[])
{
    time_t destinStart = time(NULL);
    // ********************************************
    // Main experiment of DeSTIN (Also called main)
    // ********************************************

    // File for diagnostic
    string strDiagnosticFileName;
    strDiagnosticFileName = GetNextFileForDiagnostic();

    // arguments processing

    // For debug information we output the command line to our Diagnostic file.
    string strCommandLineData = "";
    for( int i=0; i<argc; i++ )
    {
        strCommandLineData += argv[i];
        strCommandLineData += " ";
    }


    CommandArgsStuc argsStruc;
    if(parseCommandArgs(strDiagnosticFileName, argc, argv, argsStruc)!=0){
    	return 1;
    }


    // **********************
    // Loading data source(s)
    // **********************
    // Arguments: TrainingDataFile
    // Load the training file for DeSTIN
    string strDestinTrainingFileName = argv[5];

    // Data object containing source (training)
    DestinData DataSourceForTraining;

    int NumberOfUniqueLabels;
    DataSourceForTraining.LoadFile(strDestinTrainingFileName.c_str());
    NumberOfUniqueLabels = DataSourceForTraining.GetNumberOfUniqueLabels();
    if ( NumberOfUniqueLabels==0 )
    {
        cout << "There seems to be something off with data source " << strDestinTrainingFileName.c_str() << endl;
        return 0;
    }

    // A vector with all the labels of the data source
    vector<int> vLabelList;
    DataSourceForTraining.GetUniqueLabels(vLabelList);

    // Load the test file for DeSTIN
    string strTesting = strDestinTrainingFileName;
    strTesting = strTesting + "_TESTING";
    // Data object of test source
    DestinData DataSourceForTesting;

    DataSourceForTesting.LoadFile((char*)(strTesting.c_str()));
    if ( DataSourceForTesting.GetNumberOfUniqueLabels()!=NumberOfUniqueLabels )
    {
        cout << "Test set does not have the same number of labels as train set " << endl;
        return 0;
    }

    // **************************
    // Preparing working data set
    // **************************
    // now get the data set creation parameters
    int NumberOfUniqueLabelsToUse;
    int MAX_CNT = 1000;
    int iTestSequence = 0;
    string ParametersFileName;
    vector< pair<int,int> > vIndicesAndGTLabelToUse;

    if (argsStruc.bCreateFromFile==false )
    {
        // Argument: MAXCNT
        MAX_CNT=atoi(argv[2]);
        // Argument: CodeWord
        iTestSequence=atoi(argv[1]);
        string sCodeWord=argv[1];
        if (sCodeWord.length() != 11 )
        {
            PrintHelp();
            return 0;
        }
        // First part of code word RRRR = for time seeding
        string sNumInp;
        sNumInp= sCodeWord.substr(0,4);

        // if the first 4 digits are 0000 make a TRUE random, otherwise use the complete number.
        int iReserve = atoi( sNumInp.c_str() );
        if ( iReserve == 0 )
        {
            srand( time(NULL) );
        }
        else
        {
            int iRandSeed = iTestSequence;
            srand( (unsigned int)iRandSeed );
        }

        // Second part of code word XX = number of inputs
        sNumInp = sCodeWord.substr(4,2);
        NumberOfUniqueLabelsToUse = atoi( sNumInp.c_str() );

        // Last part of code word YYYYY
        int iNumberOfExamplesFromEachLabel;
        sNumInp = sCodeWord.substr(6,5);
        iNumberOfExamplesFromEachLabel=atoi( sNumInp.c_str() );

        // if iNumberOfExamplesFromEachLabel is 0 we randomly pick examples from the available
        // classes and only show them ONE TIME
        // Generate the examples from the dictates given here.
        vector< pair<int,int> > LabelsAndIndicesForUse;
        cout << "------------------" << endl;
        int DestinTrainSampleStep = 1;
        if(iNumberOfExamplesFromEachLabel == 0)
        {
            DestinTrainSampleStep = 25;
        }
        for(int iLabel=0;iLabel<NumberOfUniqueLabelsToUse;iLabel++)
        {
            int cnt = 0;
            vector<int> IndicesForThisLabel;
            DataSourceForTraining.GetIndicesForThisLabel(iLabel,IndicesForThisLabel);
            if ( IndicesForThisLabel.size() > iNumberOfExamplesFromEachLabel && iNumberOfExamplesFromEachLabel != 0)
            {
                for(int jj=0;jj<iNumberOfExamplesFromEachLabel;jj++)
                {
                    cnt++;
                    pair<int,int> P;
                    P.first = IndicesForThisLabel[jj];
                    P.second = iLabel;
                    LabelsAndIndicesForUse.push_back(P);
                }
            }
            else
            {
                for(int jj=0;jj<IndicesForThisLabel.size();jj=jj+DestinTrainSampleStep)
                {
                    cnt++;
                    pair<int,int> P;
                    P.first = IndicesForThisLabel[jj];
                    P.second = iLabel;
                    LabelsAndIndicesForUse.push_back(P);
                }

            }
            cout << "Label: " << iLabel << " got " << cnt << " unique sample(s)." << endl;
        }
        iNumberOfExamplesFromEachLabel = LabelsAndIndicesForUse.size()/NumberOfUniqueLabelsToUse;

        // Now generate MAX_CNT+1000 random numbers from 0 to LabelsAndIndicesForUse-1
        // and use these to populate vIndicesAndGTLabelToUse

        // Debug list of labels to be used
        int * Picked;
        Picked = (int *) malloc(sizeof(int) * NumberOfUniqueLabels);

        for(int jj=0;jj<NumberOfUniqueLabels;jj++)
        {
            Picked[jj]=0;
        }

        int Digit;
        int iChoice;
        for(int jj=0;jj<MAX_CNT;jj++)
        {
            //pick the digit first...
            Digit = rand() % NumberOfUniqueLabelsToUse;
            iChoice = Digit * iNumberOfExamplesFromEachLabel;
            iChoice = iChoice+rand() % iNumberOfExamplesFromEachLabel;

            pair<int,int> P;
            P = LabelsAndIndicesForUse[iChoice];

            vIndicesAndGTLabelToUse.push_back( P );
            // Debug counter of labels used by label
            Picked[P.second] += 1;
        }

        // Debug information on amount of examples we use each label
        cout << "------------------" << endl;
        for(int jj=0;jj<NumberOfUniqueLabels;jj++)
        {
            cout << "Label: " << jj << " will show " << Picked[jj] << " sample(s)." << endl;
        }
        free( Picked);
        cout << "------------------" << endl;
    }  //check on bCreateFromFile==false
    else
    {
        // TODO: We want to create the network from an INPUT FILE!
        cout << "We want to create the network from an INPUT FILE!" << endl;
    }

    // Argument: ParamsFile
    // A configuration file for DeSTIN
    ParametersFileName=argv[4];
    if ( !FileExists(ParametersFileName) )
    {
        // According to the help the ParamsFile is always used? Maybe some vital information on how to load data?
        // Or some testing to see how the network reacts when expanding or shrinking the network.
        cout << "Parameters file name does not exist" << endl;
        return 0;
    }

    // ***********************
    // Creating DeSTIN network
    // ***********************
    // Yes its going to happen we going to create the network where we are waiting for.
    int SEQ_LENGTH = 0;
    int** SEQ;
    int* ImageInput;

    DestinKernel* DKernel=NULL;
    map<int,int> LabelsUsedToCreateNetwork;
    map<int,int> IndicesUsedToCreateNetwork;
    int NumberOfLayers=4;
    if ( !argsStruc.bCreateFromFile)
    {
        int LayerToShow=-1;   //normally this should be -1 for regular operation.  For debugging, set it to 0 to look at the particular input for layer 0
        int RowToShowInputs=3;
        int ColToShowInputs=3;
        CreateDestinOnTheFly(ParametersFileName, NumberOfLayers, DKernel,
                             DataSourceForTraining, SEQ_LENGTH, SEQ, ImageInput);

        for (int i=0; i<NumberOfLayers;i++)
        {
            cout << "DeSTIN Layer information" << endl;
            cout << "Layer: " << DKernel[i].GetID() << endl;
            cout << "Dimension (row, col): " << DKernel[i].GetNumberOfRows() << " X " << DKernel[i].GetNumberOfCols() << endl;
            cout << "Input each node: " << DKernel[i].GetNumberOfInputDimensionlity() << endl;
            cout << "Centroids: " << DKernel[i].GetNumberOfStates() << endl;
            cout << endl;
        }
    }
    else
    {
        // even if you don't create the file here, we want to mark the experiment number so make a dummy file...
        ofstream stmDummy;
        stmDummy.open(strDiagnosticFileName.c_str(),ios::out);
        stmDummy << strCommandLineData.c_str() << endl;
        stmDummy << "DummyHeader" << endl;
        stmDummy.close();
    }

    cout << "------------------" << endl;
    cout << "Run Destin" << endl;
    cout << "Images to be processed: " << MAX_CNT << endl;
    cout << "Each image moves: " << SEQ_LENGTH << " times." << endl;

    double procces = 0.1;
    for(int i=0;i<MAX_CNT;i++)
    {
        if(i > (MAX_CNT-1)*procces)
        {
            cout << procces*100 << "%" << endl;
            procces+=0.1;
        }
        stringstream xml;
        xml << "<destin>" << endl;

        pair<int,int> element = vIndicesAndGTLabelToUse[i];
        int indexOfExample = element.first;
        int label = element.second;
        time_t iStart = time(NULL);
        for(int seq=0;seq<SEQ_LENGTH;seq++)
        {
            stringstream xmlLayer;
            // Run lowest layer (Kernel)

            time_t lStart = time(NULL);
            DataSourceForTraining.SetShiftedDeviceImage(indexOfExample, SEQ[seq][0], SEQ[seq][1], ImageInput[0], ImageInput[1]);
            DKernel[0].DoDestin(DataSourceForTraining.GetPointerDeviceImage(),xmlLayer);
            for(int i=1;i<NumberOfLayers;i++)
            {
                DKernel[i].DoDestin(DKernel[i-1].GetDevicePointerOutput(),xmlLayer);
            }
            time_t lStop = time(NULL);
            xmlLayer << "<layerRuntime>" << lStop-lStart << "</layerRuntime>" << endl;
            if(seq == SEQ_LENGTH-1)
            {
                xml << xmlLayer.str().c_str();
            }
            xmlLayer.clear();
        }
        time_t iStop = time(NULL);
        xml << "<image id=\"" << i << "\" label=\"" << label << "\" labelIndex=\"" << indexOfExample << "\" runtime=\"" << iStop-iStart << "\" />" << endl;
        xml << "</destin>" << endl;
        if(i == MAX_CNT-1)
        {
            pugi::xml_document outputFile;
            outputFile.load(xml.str().c_str());
            string file = argsStruc.strDestinNetworkFileToRead;
            stringstream num;
            num << "-" << i;
            file.insert(file.length()-4, num.str());
            outputFile.save_file(file.c_str());
        }
    }
    time_t destinStop = time(NULL);
    cout << "Time run: " << destinStop-destinStart << endl;

    free(DKernel);

    return 0;
}

// Simple run command: destinCuda(.exe) 00010100000 120 2:3 ./config.xml ../../data/MNISTTraining32 -D D
int main(int argc, char* argv[])
{
    // ********************
    // Startup check DeSTIN
    // ********************
    // There should be 8 or 9 arguments at this time if not show how to use DeSTIN
    if ( argc==8 || argc==9 )
    {
        cout << "Starting DeSTIN" << endl;
        cout << "------------------" << endl;
        return MainDestinExperiments(argc,argv);
    }
    else
    {
        PrintHelp();
        return 0;
    }
}
