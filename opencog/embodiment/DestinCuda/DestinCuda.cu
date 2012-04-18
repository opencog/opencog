#include "DestinCuda.h"


using namespace std;

// TODO: This main file is still pretty messy.
/*
 * It's almost a copy of the original main of DeSTIN where the custom file reader is replaced by a XML reader(pugixml)
 * Because of the quite basic understanding of the code and a more overall understanding of DeSTIN there are a lot of variables not been used.
 * Also cause there is still work in progress.
 * It still might be a idea to just replace the command line complete and do everything from the configuration
 * This will save a lot of code to analyze the input arguments
 */
void DestinCuda::PrintHelp()
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
}

bool DestinCuda::FileExists(string strFilename)
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

string DestinCuda::GetNextFileForDiagnostic()
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

void DestinCuda::GetParameters( const char* cFilename, int& NumberOfLayers, double*& dcMu, double*& dcSigma, double*& dcRho,
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

bool DestinCuda::CreateDestinOnTheFly(string ParametersFileName, int& NumberOfLayers, DestinKernel*& DKernel,
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


    DKernel = new DestinKernel[NumberOfLayers];
    //TODO: I think all these "new" declarations are not being deleted and potentially leaking memory
    int* ColsPerLayer = new int[NumberOfLayers];
    int* InputDimensionality = new int[NumberOfLayers];

    InputDimensionality[0] = 16; //4x4 has 16 inputs

    for(int Layer=1; Layer<NumberOfLayers; Layer++ ){
        InputDimensionality[Layer] = 4*NumberOfCentroids[Layer-1];
    }

    // curandGenerator_t is a CUDA version of rand
    // This fills the whole memory block with number between 0.0 and 1.0
    curandGenerator_t gen;
    curandCreateGenerator(&gen, CURAND_RNG_PSEUDO_DEFAULT);
    // TODO: Add seed code instead of 1
    // This is the right place to do this saves the most time creating numbers. (Inside layer increase the time by +/- 5 times)
    curandSetPseudoRandomGeneratorSeed( gen, 1 );

    for( int Layer = NumberOfLayers - 1; Layer>=0 ; Layer-- ){
        ColsPerLayer[Layer] = RowsPerLayer[Layer];
        int parentStates = Layer == NumberOfLayers - 1 ? 1 : NumberOfCentroids[Layer + 1];
        DKernel[Layer].Create(
        		Layer, RowsPerLayer[Layer], ColsPerLayer[Layer],
        		NumberOfCentroids[Layer],parentStates,  InputDimensionality[Layer],
        		FixedLearningRateLayer[Layer], gen);

        //give the child layers their parents' advice
        if(Layer == NumberOfLayers - 1){
        	DKernel[Layer].SetInputAdvice(NULL);
        }else{
            DKernel[Layer].SetInputAdvice(DKernel[Layer+1].GetOutputAdvice());
        }
    }

    // The generator have to be destroyed after use.
    curandDestroyGenerator( gen );
    cout << "------------------" << endl;
    return 0;
}
/**
 *  parseCommandArgs
 *
 *  Parses the program command line arguments and fills
 *  the out CommandArgsStuc appropriately
 *      
 */

CommandArgsStuc createCommandArgsStuc(string sCodeWord, int MAX_CNT, string ParametersFileName){
    CommandArgsStuc cas;
    cas.sCodeWord = sCodeWord;
    
    return cas;
}

int DestinCuda::parseCommandArgs(int argc, char* argv[], CommandArgsStuc &out) {

 
    out.sCodeWord = argv[1];
    if (out.sCodeWord.length() != 11) {
        PrintHelp();
        return 1;
    }
    out.seed = atoi(argv[1]);
    out.MAX_CNT = atoi(argv[2]);
    
    
    // Argument: ParamsFile
    // A configuration file for DeSTIN
    out.ParametersFileName = argv[4];
    if (!FileExists(out.ParametersFileName)) {
        // According to the help the ParamsFile is always used? Maybe some vital information on how to load data?
        // Or some testing to see how the network reacts when expanding or shrinking the network.
        cout << "Parameters file name does not exist" << endl;
        return 1;
    }

     out.strDestinTrainingFileName = argv[5];
     
     
    // Argument: TargetDirectory
    // A given location instead or default
    string strDiagnosticDirectoryForData;
    string strArg7 = argv[7];
    if (strArg7 == "D") {
        strDiagnosticDirectoryForData = "../DiagnosticData/";
    } else {
        // Buffer with path + filename where to put diagnostic data
        stringstream buffer;
        buffer << strArg7.c_str() << "/";
        strDiagnosticDirectoryForData = buffer.str();
    }
    
   

    // Argument: DestinOutputFile
    string strDestinNetworkFileToWrite = argv[6]; // we write to this file, and then we read from it too!!
    if (strDestinNetworkFileToWrite == "-D") {
        // If given -D
        strDestinNetworkFileToWrite = strDiagnosticDirectoryForData + GetNextFileForDiagnostic();
        cout << "Writing default destin file to: " << strDestinNetworkFileToWrite << endl;
    }
    out.strDestinNetworkFileToRead = strDestinNetworkFileToWrite;

    

    out.strTesting = out.strDestinTrainingFileName + "_TESTING";
    return 0;
}


int DestinCuda::MainDestinExperiments(CommandArgsStuc & argsStruc)
{
    time_t destinStart = time(NULL);
    // ********************************************
    // Main experiment of DeSTIN (Also called main)
    // ********************************************


    // **********************
    // Loading data source(s)
    // **********************
    // Arguments: TrainingDataFile
    // Load the training file for DeSTIN
    

    // Data object containing source (training)
    DestinData DataSourceForTraining;

    DataSourceForTraining.LoadFile(argsStruc.strDestinTrainingFileName.c_str());
    int NumberOfUniqueLabels = DataSourceForTraining.GetNumberOfUniqueLabels();
    if ( NumberOfUniqueLabels==0 )
    {
        cout << "There seems to be something off with data source " << argsStruc.strDestinTrainingFileName.c_str() << endl;
        return 0;
    }

    // Data object of test source
    DestinData DataSourceForTesting;

    DataSourceForTesting.LoadFile((char*)(argsStruc.strTesting.c_str()));
    if ( DataSourceForTesting.GetNumberOfUniqueLabels()!=NumberOfUniqueLabels )
    {
        cout << "Test set does not have the same number of labels as train set " << endl;
        return 0;
    }

    // **************************
    // Preparing working data set
    // **************************
    // now get the data set creation parameters
   
    vector< pair<int,int> > vIndicesAndGTLabelToUse;


  

        // First part of code word RRRR = for time seeding
        string sNumInp = argsStruc.sCodeWord.substr(0,4);

        // if the first 4 digits are 0000 make a TRUE random, otherwise use the complete number.
        int iReserve = atoi( sNumInp.c_str() );
        if ( iReserve == 0 )
        {
            srand( time(NULL) );
        }
        else
        {
            srand( (unsigned int)argsStruc.seed );
        }

        // Second part of code word XX = number of inputs
        sNumInp = argsStruc.sCodeWord.substr(4,2);
        int NumberOfUniqueLabelsToUse = atoi( sNumInp.c_str() );

        // Last part of code word YYYYY
        sNumInp = argsStruc.sCodeWord.substr(6,5);
        int iNumberOfExamplesFromEachLabel=atoi( sNumInp.c_str() );

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
        for(int jj=0;jj<argsStruc.MAX_CNT;jj++)
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




    // ***********************
    // Creating DeSTIN network
    // ***********************
    int SEQ_LENGTH = 0;
    int** SEQ;
    int* ImageInput;
    int NumberOfLayers;

    CreateDestinOnTheFly(argsStruc.ParametersFileName, NumberOfLayers, DKernel,
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

    cout << "------------------" << endl;
    cout << "Run Destin" << endl;
    cout << "Images to be processed: " << argsStruc.MAX_CNT << endl;
    cout << "Each image moves: " << SEQ_LENGTH << " times." << endl;

	RunningInfo run_info;
    double procces = 0.1;
    for(int i=0;i< argsStruc.MAX_CNT;i++)
    {
        if(i > (argsStruc.MAX_CNT-1)*procces)
        {
            cout << procces*100 << "%" << endl;
            procces+=0.1;
        }
        stringstream xml;
        xml << "<destin>" << endl;

        pair<int,int> element = vIndicesAndGTLabelToUse[i];
        int indexOfExample = element.first;
        int label = element.second;
		run_info.image_label = label;
		
        time_t iStart = time(NULL);
		run_info.image_count = i;
		
        for(int seq=0;seq<SEQ_LENGTH;seq++)
        {
			run_info.sequence_step = seq;
            stringstream xmlLayer;
            // Run lowest layer (Kernel)

            time_t lStart = time(NULL);
			
            DataSourceForTraining.SetShiftedDeviceImage(indexOfExample, SEQ[seq][0], SEQ[seq][1], ImageInput[0], ImageInput[1]);
            DKernel[0].DoDestin(DataSourceForTraining.GetPointerDeviceImage(),&xmlLayer);
			run_info.layer = 0;
            if(this->callback!=NULL){
                this->callback->callback(run_info, DKernel[0] );
            }
            //TODO: is the order of layer evaluation going in the right order?
            for(int l=1;l<NumberOfLayers;l++)
            {
				run_info.layer = l;
                DKernel[l].DoDestin(DKernel[l-1].GetDevicePointerBeliefs(),&xmlLayer);
                if(this->callback!=NULL){
                     this->callback->callback(run_info, DKernel[l] );
                }
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
        if(i == argsStruc.MAX_CNT-1)
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

    delete [] DKernel; 

    return 0;
}

// Simple run command: destinCuda(.exe) 00010100000 120 2:3 ./config.xml ../../data/MNISTTraining32 -D D
int main(int argc, char* argv[])
{
    // ********************
    // Startup check DeSTIN
    // ********************
    // There should be 8 or 9 arguments at this time if not show how to use DeSTIN
    
    DestinCuda dc;
    
    // arguments processing
    CommandArgsStuc argsStruc;
    if(dc.parseCommandArgs( argc, argv, argsStruc)!=0){
    	return 1;
    }
    
    
    if ( argc==8 || argc==9 )
    {
        cout << "Starting DeSTIN" << endl;
        cout << "------------------" << endl;
        
        return dc.MainDestinExperiments(argsStruc);
    }
    else
    {
        dc.PrintHelp();
        return 0;
    }
}
