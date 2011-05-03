#include "DestinData.h"

#include <iostream>
#include <stdio.h>
#include <vector>
#include <fstream>
#include <ctime>
#include <sstream>
#include <sys/stat.h>
#include <math.h>

#ifdef _WIN32
#include <direct.h>
#else
// Linux only requirements...
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#endif

using namespace std;

void PrintHelp()
{
    // ***************************
    // Print out how to use DeSTIN
    // ***************************

    cout << "Usage: DestinCuda CodeWord MAXCNT LayerToShow ParamsFile TrainingDataFile DestinOutputFile TargetDirectory [OutputDistillationLevel]" << endl;
    cout << "Where:" << endl;
    cout << "    CodeWord is an 11-digit value of the form." << endl;
    cout << "        RRRRXXYYYYY where RRRR is reserved, XX is number of classes, YYYYY is number of examples" << endl;
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
    cout << "         Use -D as default, which is the experiment number with a .dat at the end, in the ../DiagnosticData directory" << endl;
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
        buffer << "../DiagnosticData/DestinDiagnostics" << iExperimentNumber << ".csv";
        strFileName =  buffer.str();

        bFileFound = FileExists(strFileName);
    }

    return strFileName;
}

int MainDestinExperiments(int argc, char* argv[])
{
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

    // Argument: DestinOutputFile or InputNetworkFile
    bool bCreateFromFile;
    string strDestinNetworkFileToRead;
    string strDestinNetworkFileToWrite;
    string FirstArg = argv[1];
    if ( FirstArg=="-F" )
    {
        // Argument: InputNetworkFile
        bCreateFromFile = true;
        strDestinNetworkFileToRead = argv[2];  // we read from this file...

        if ( !FileExists( strDestinNetworkFileToRead ) )
        {
            cout << "designated input network file named " << strDestinNetworkFileToRead.c_str() << " does not exist" << endl;
            return 0;
        }
    }
    else
    {
        // Argument: DestinOutputFile
        bCreateFromFile = false;
        strDestinNetworkFileToWrite = argv[6]; // we write to this file, and then we read from it too!!
        if ( strDestinNetworkFileToWrite == "-D" )
        {
            // If given -D
            strDestinNetworkFileToWrite=strDiagnosticFileName + "DestinNetwork.dat";
            cout << "Writing to default destin file name..." << endl;
        }
        strDestinNetworkFileToRead = strDestinNetworkFileToWrite;
    }

    // Argument: TargetDirectory
    // A given location instead or default
    string strDiagnosticFileNameForData;
    string strArg7 = argv[7];
    if ( strArg7 == "D" )
    {
        strDiagnosticFileNameForData = strDiagnosticFileName;
    }
    else
    {
        // Buffer with path + filename where to put diagnostic data
        stringstream buffer;
        buffer << strArg7.c_str() << "/" << strDiagnosticFileName;
        strDiagnosticFileNameForData = buffer.str();
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

int main(int argc, char* argv[])
{
    // ********************
    // Startup check DeSTIN
    // ********************

    if ( argc==8 || argc==9 )
    {
        return MainDestinExperiments(argc,argv);
    }
    else
    {
        PrintHelp();
        return 0;
    }
}
