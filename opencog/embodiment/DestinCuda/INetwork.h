#ifndef INETWORK_H
#define INETWORK_H

class DestinIterationFinishedCallback;

class INetwork {
public:

    virtual ~INetwork(){}
    virtual void doDestin( float * dInput)=0;
    virtual void setIterationFinishedCallback(DestinIterationFinishedCallback * callback)=0;
    virtual void free()=0;
    virtual void setIsPSSATraining(bool training)=0;
    virtual void setIsPOSTraining(bool training)=0;
    virtual int getBeliefsPerNode(int layer)=0;
    virtual float * getNodeBeliefs(int layer, int row, int col)=0;

};
#endif
