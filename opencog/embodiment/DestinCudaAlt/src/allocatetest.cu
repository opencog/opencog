

#include "destin.h"
#include <stdio.h>

int main(int argc, char ** argv){
    //uint dims[] = {26,22,18,16,12,8,6,4,2};
    uint dims[] = {26,22,18,16,12,8,6,4,2};

    Destin * d = InitDestin(16, 8, dims, 16);
    printf("Max number of beliefs: %i\n", d->maxNb);

}
