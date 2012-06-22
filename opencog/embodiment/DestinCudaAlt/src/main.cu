#include "macros.h"
#include "destin.h"
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
    
    int ni, nl, i, nMovements;
    int *nb;

    // parse config file

    // get number of movements per digit
    fscanf(configFile, "%d", &nMovements);

    // get input dimensionality
    fscanf(configFile, "%d", &ni);

    // get number of layers
    fscanf(configFile, "%d", &nl);
    nb = (int *) malloc(sizeof(int) * nl);

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

    srand(time(NULL));


    // check arg count
    if( argc != 7 ) {
        fprintf(stderr, "Usage: destin configfile trainset-in nntrain-in trainbeliefs-out nntest-in testbeliefs-out\n");
        exit(1);
    }

    d = CreateDestin( argv[1] );

    // train network
    printf("training network...\n");
    RunDestin( d, argv[2], NULL, true );

    // extract beliefs for nn train set
    printf("extracting beliefs for train set...\n");
    RunDestin( d, argv[3], argv[4], false );

    // extract beliefs for nn test set
    printf("extracting beliefs for test set...\n");
    RunDestin( d, argv[5], argv[6], false );

    // free destin resources
    DestroyDestin(d);
}
