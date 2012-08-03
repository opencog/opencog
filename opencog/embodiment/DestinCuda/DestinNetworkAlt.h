/*
 * destinnetworkalt.h
 *
 *  Created on: Jun 23, 2012
 *      Author: ted
 */

#ifndef DESTINNETWORKALT_H_
#define DESTINNETWORKALT_H_

#include "INetwork.h"
#include "destin.h"
#include "DestinIterationFinishedCallback.h"
#include <stdexcept>
#include <vector>
#include <iostream>

enum SupportedImageWidths {
    W4 = 4,     //1
    W8 = 8,     //2
    W16 = 16,   //3
    W32 = 32,   //4
    W64 = 64,   //5
    W128 = 128, //6
    W256 = 256, //7
    W512 = 512  //8 layers needed
};
#define MAX_IMAGE_WIDTH 512

class destin_network_alt: public INetwork {

private:
    Destin * destin;
    bool training;
    DestinIterationFinishedCallback * callback;
public:
    destin_network_alt(SupportedImageWidths width, uint layers,
            uint centroid_counts [] ) :
            training(true) {

        uint input_dimensionality = 16;
        bool supported = false;
        uint c, l;
        callback = NULL;

        //figure out how many layers are needed to support the given
        //image width.
        for (c = 4, l = 1; c <= MAX_IMAGE_WIDTH ; c *= 2, l++) {
            if (c == width) {
                supported = true;
                break;
            }
        }

        if (supported) {
            if (layers == l) {
                int num_movements = 0; //this class does not use movements
                destin = InitDestin(input_dimensionality, layers,
                        centroid_counts, num_movements);
            } else {
                throw std::logic_error("Image width does not match the given number of layers.");
            }
        }else{
            throw std::logic_error("given image width is not supported.");
        }

    }

    virtual ~destin_network_alt() {
        if(destin!=NULL){
            DestroyDestin(destin);
            destin = NULL;
        }
    }

    void doDestin( //run destin with the given input
            float * input_dev //pointer to input memory on device
            ) {
        FormulateBelief(destin, training, input_dev);

        if(this->callback != NULL){
            this->callback->callback(*this );
        }
    }

    void isTraining(bool isTraining) {
        this->training = isTraining;
    }

    bool isTraining() {
        return training;
    }

    void free() {
        DestroyDestin(destin);
        destin = NULL;
    }

    void setIsPOSTraining(bool training) {
        isTraining(training);
    }

    void setIsPSSATraining(bool no_op) {
        //this network doesn't do pssa training
        std::cout << "DestinNetworkAlt setIsPSSATraining called. Currently a noop.\n";
    }

    void setIterationFinishedCallback(DestinIterationFinishedCallback * callback){
        this->callback = callback;
    }

    int getBeliefsPerNode(int layer){
        return destin->nBeliefsPerNode[layer];
    }

    float * getNodeBeliefs(int layer, int row, int col){

        Node * hostnode = GetNodeFromDestin(destin, layer, row, col);
        CopyNodeFromDevice(hostnode);
        return hostnode->beliefEuc;
    }
};

#endif /* DESTINNETWORKALT_H_ */
