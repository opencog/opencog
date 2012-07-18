#include "destin.h"
#include "macros.h"
#include "node.h"

#include<stdio.h>
#include<stdlib.h>
#include<time.h>

// create a destin instantiation from a file
Destin * CreateDestin( char *filename ) {
    Destin *newDestin;

    FILE *configFile;
    // open file
    configFile = fopen(filename, "r");
    if( !configFile ) {
        fprintf(stderr, "Cannot open config file %s\n", filename);
        exit(1);
    }
    
    uint ni, nl, i, nMovements;
    uint *nb;

    // parse config file

    // get number of movements per digit
    fscanf(configFile, "%d", &nMovements);

    // get input dimensionality
    fscanf(configFile, "%d", &ni);

    // get number of layers
    fscanf(configFile, "%d", &nl);
    nb = (uint *) malloc(sizeof(uint) * nl);

    printf("allocating new destin(%d,", ni);

    // get layer beliefs
    for(i=0; i < nl; i++) {
        fscanf(configFile, "%d", &nb[i]);
        printf("%d,", nb[i]);
    }
    printf("\b)\n");

    newDestin = InitDestin(ni, nl, nb, nMovements);

    fclose(configFile);

    free(nb);

    return newDestin;
}

int main(int argc, char **argv) {
    Destin *d;

    float *beliefTrain, *beliefTest;

    srand(time(NULL));

    // check arg count
    if( argc != 7 ) {
        fprintf(stderr, "Usage: destin configFile trainSet nnTrainIn nnTrainOut nnTestIn nnTestOut\n");
        exit(1);
    }

    d = CreateDestin( argv[1] );

    // train network
    printf("training network...\n");
    RunDestin( d, argv[2], true );

    // extract beliefs for nn train set
    printf("extracting beliefs for train set...\n");
    beliefTrain = RunDestin( d, argv[3], false );

    FILE *beliefTrainOut = fopen(argv[4], "w");
    fwrite( beliefTrain, sizeof(float), d->nBeliefs * 3 * 60000, beliefTrainOut );
    fclose( beliefTrainOut );

    // extract beliefs for nn test set
    printf("extracting beliefs for test set...\n");
    beliefTest = RunDestin( d, argv[5], false );

    FILE *beliefTestOut = fopen(argv[6], "w");
    fwrite( beliefTest, sizeof(float), d->nBeliefs * 3 * 10000, beliefTestOut );
    fclose( beliefTestOut );

    // free destin resources
    free( beliefTrain );
    free( beliefTest );
    DestroyDestin(d);
}
