#include "destin.h"
#include "macros.h"
#include "node.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include<stdio.h>
#include<stdlib.h>
#include<time.h>


int main(int argc, char **argv) {
    Destin *d = NULL;

    srand(time(NULL));

    // check arg count
    if( argc != 7 ) {
        fprintf(stderr, "Usage: destin train/test (configFile) destinFile trainData trainLabels (generative/output)\n");
        exit(1);
    }


    // train network
    if( strcmp(argv[1], "train") == 0  )
    {
        d = CreateDestin( argv[2] );
        TrainDestin( d, argv[4], argv[5] );
        SaveDestin( d, argv[3] );
    } else if( strcmp(argv[1], "test") == 0 ) {
        d = LoadDestin( d, argv[3] );

        if( atoi(argv[6]) == 1 )
        {
            TestDestin( d, argv[4], argv[5], true );
        } else
        {
            TestDestin( d, argv[4], argv[5], false );
        }
    } else {
        fprintf(stderr, "Usage: destin train/test (configFile) destinFile trainData trainLabels (generative/output)\n");
        exit(1);
    }

    DestroyDestin(d);
}
