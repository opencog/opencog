
#include "VideoSource.h"
#include "DestinNetworkAlt.h"

int main(int argc, char ** argv){
    VideoSource vs(true, "");

    SupportedImageWidths siw = W512;

    uint centroid_counts[]  = {20,16,14,12,10,8,4,2};

    destin_network_alt network(siw, 8, centroid_counts);

    vs.enableDisplayWindow();
    while(vs.grab()){
        network.doDestin(vs. getOutput());
    }

    return 0;
}
