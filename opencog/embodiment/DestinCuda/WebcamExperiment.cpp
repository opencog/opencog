
#include "VideoSource.h"
#include "DestinNetworkAlt.h"
#include "Transporter.h"
#include "stdio.h"

int main(int argc, char ** argv){
    VideoSource vs(true, "");

    SupportedImageWidths siw = W512;

    uint centroid_counts[]  = {20,16,14,12,10,8,4,2};

    destin_network_alt network(siw, 8, centroid_counts);

    vs.enableDisplayWindow();

    Transporter t(512 * 512);



    float * beliefs;
    while(vs.grab()){

        t.setHostSourceImage(vs.getOutput());
        t.transport(); //move video from host to card

        network.doDestin(t.getDeviceDest());

        beliefs = network.getNodeBeliefs(7,0,0);

        //printf("%c[2A", 27); //earase two lines so window doesn't scroll
        printf("0: %f\n", beliefs[0]);
        printf("1: %f\n", beliefs[1]);
    }

    return 0;
}
