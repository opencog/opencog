#ifndef POLEBALANCINGINC
#define POLEBALANCINGINC

#include <opencog/comboreduct/combo/simple_nn.h>

namespace opencog { 

class CartPole {
public:
    CartPole(bool randomize,bool velocity);
    virtual void simplifyTask();  
    virtual void nextTask();
    virtual double evalNet(combo::ann *net);
    double maxFitness;
    bool MARKOV;

    bool last_hundred;
    bool nmarkov_long;  //Flag that we are looking at the champ
    bool generalization_test;  //Flag we are testing champ's generalization

    double state[6];

    double jigglestep[1000];

protected:
    virtual void init(bool randomize);

private:

    void performAction(double output,int stepnum);
    void step(double action, double *state, double *derivs);
    void rk4(double f, double y[], double dydx[], double yout[]);
    bool outsideBounds(); 

    const static int NUM_INPUTS=7;
    const static double MUP = 0.000002;
    const static double MUC = 0.0005;
    const static double GRAVITY= -9.8;
    const static double MASSCART= 1.0;
    const static double MASSPOLE_1= 0.1;

    const static double LENGTH_1= 0.5;		  /* actually half the pole's length */

    const static double FORCE_MAG= 10.0;
    const static double TAU= 0.01;		  //seconds between state updates 

    const static double one_degree= 0.0174532;	/* 2pi/360 */
    const static double six_degrees= 0.1047192;
    const static double twelve_degrees= 0.2094384;
    const static double fifteen_degrees= 0.2617993;
    const static double thirty_six_degrees= 0.628329;
    const static double fifty_degrees= 0.87266;

    double LENGTH_2;
    double MASSPOLE_2;
    double MIN_INC;
    double POLE_INC;
    double MASS_INC;

    //Queues used for Gruau's fitness which damps oscillations
    int balanced_sum;
    double cartpos_sum;
    double cartv_sum;
    double polepos_sum;
    double polev_sum;
};

} // ~namespace opencog

#endif
