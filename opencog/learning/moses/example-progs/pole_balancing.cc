#include "pole_balancing.h"

using namespace opencog;
using namespace combo;
using namespace std;

const int CartPole::NUM_INPUTS=7;
const double CartPole::MUP = 0.000002;
const double CartPole::MUC = 0.0005;
const double CartPole::GRAVITY= -9.8;
const double CartPole::MASSCART= 1.0;
const double CartPole::MASSPOLE_1= 0.1;

const double CartPole::LENGTH_1= 0.5;		  /* actually half the pole's length */

const double CartPole::FORCE_MAG= 10.0;
const double CartPole::TAU= 0.01;		  //seconds between state updates 

const double CartPole::one_degree= 0.0174532;	/* 2pi/360 */
const double CartPole::six_degrees= 0.1047192;
const double CartPole::twelve_degrees= 0.2094384;
const double CartPole::fifteen_degrees= 0.2617993;
const double CartPole::thirty_six_degrees= 0.628329;
const double CartPole::fifty_degrees= 0.87266;


CartPole::CartPole(bool randomize,bool velocity)
{
    maxFitness = 100000;

    MARKOV=velocity;

    MIN_INC = 0.001;
    POLE_INC = 0.05;
    MASS_INC = 0.01;

    LENGTH_2 = 0.05;
    MASSPOLE_2 = 0.01;

    // CartPole::reset() which is called here
}

//Faustino Gomez wrote this physics code using the differential equations from 
//Alexis Weiland's paper and added the Runge-Kutta himself.
double CartPole::evalNet(ann *net)
{
    int steps=0;
    double input[NUM_INPUTS];
    double output;

    int nmarkovmax;  

    //init(randomize);		// restart at some point
  
    if (nmarkov_long) nmarkovmax=100000;
    else if (generalization_test) nmarkovmax=1000;
    else nmarkovmax=1000;


    init(0);

    if (MARKOV) {
        while (steps++ < maxFitness) {
      
         
            input[0] = state[0] / 4.8;
            input[1] = state[1] /2;
            input[2] = state[2]  / 0.52;
            input[3] = state[3] /2;
            input[4] = state[4] / 0.52;
            input[5] = state[5] /2;
            input[6] = .5;
      
            net->load_inputs(input);
      
            //Activate the net
            //If it loops, exit returning only fitness of 1 step
            int depth = net->feedforward_depth();
            for(int x=0;x<depth;++x)
                net->propagate();

            output=net->outputs[0]->activation;
      
            performAction(output,steps);
      
            if (outsideBounds())	// if failure
                break;			// stop it now
        }
        return (double) steps;
    }
    else {  //NON MARKOV CASE

        while (steps++ < nmarkovmax) {
      

            //Do special parameter summing on last hundred
            //if ((steps==900)&&(!nmarkov_long)) last_hundred=true;

            /*
              input[0] = state[0] / 4.8;
              input[1] = 0.0;
              input[2] = state[2]  / 0.52;
              input[3] = 0.0;
              input[4] = state[4] / 0.52;
              input[5] = 0.0;
              input[6] = .5;
            */

            //cout<<"nmarkov_long: "<<nmarkov_long<<endl;

            //if (nmarkov_long)
            //cout<<"step: "<<steps<<endl;

            input[0] = state[0] / 4.8;
            input[1] = state[2]  / 0.52;
            input[2] = state[4] / 0.52;
            input[3] = .5;
      
            net->load_inputs(input);

            //cout<<"inputs: "<<input[0]<<" "<<input[1]<<" "<<input[2]<<" "<<input[3]<<endl;

            //Activate the net
            //If it loops, exit returning only fitness of 1 step
            int depth = net->feedforward_depth();
            for(int x=0;x<depth;++x)
                net->propagate();

            output=net->outputs[0]->activation;

            //cout<<"output: "<<output<<endl;

            performAction(output,steps);

            if (outsideBounds())	// if failure
                break;			// stop it now

            if (nmarkov_long&&(outsideBounds()))	// if failure
                break;			// stop it now
        }

        //If we are generalizing we just need to balance it a while
        if (generalization_test)
            return (double) balanced_sum;
 
        //Sum last 100
        double jiggletotal = 0; //total jiggle in last 100
        if ((steps>100)&&(!nmarkov_long)) {
            cout<<"step "<<steps-99-2<<" to step "<<steps-2<<endl;
            //Adjust for array bounds and count
            for(int count = steps-99-2; count<=steps-2; ++count)
                jiggletotal += jigglestep[count];
        }

        if (!nmarkov_long) {
            double nmarkov_fitness;
            if (balanced_sum>100) 
                nmarkov_fitness=((0.1*(((double) balanced_sum)/1000.0))+
                                 (0.9*(0.75/(jiggletotal))));
            else nmarkov_fitness=(0.1*(((double) balanced_sum)/1000.0));

            //cout <<"fitness: " << nmarkov_fitness << endl;
            //#ifndef NO_SCREEN_OUTR
            //     cout<<"Balanced:  "<<balanced_sum<<" jiggle: "<<jiggletotal<<" ***"<<endl;
            //#endif

            return nmarkov_fitness * 100000;
        }
        else return (double) steps;

    }

}

void CartPole::init(bool randomize)
{
    static int first_time = 1;

    if (!MARKOV) {
        //Clear all fitness records
        cartpos_sum=0.0;
        cartv_sum=0.0;
        polepos_sum=0.0;
        polev_sum=0.0;
    }

    balanced_sum=0; //Always count # balanced

    last_hundred=false;

    /*if (randomize) {
      state[0] = (lrand48()%4800)/1000.0 - 2.4;
      state[1] = (lrand48()%2000)/1000.0 - 1;
      state[2] = (lrand48()%400)/1000.0 - 0.2;
      state[3] = (lrand48()%400)/1000.0 - 0.2;
      state[4] = (lrand48()%3000)/1000.0 - 1.5;
      state[5] = (lrand48()%3000)/1000.0 - 1.5;
      }
      else {*/


    if (!generalization_test) {
        state[0] = state[1] = state[3] = state[4] = state[5] = 0;
        state[2] = 0.07; // one_degree;
    }
    else {
        state[4] = state[5] = 0;
    }

    //}
    if(first_time){
        cout<<"Initial Long pole angle = %f\n"<<state[2]<<endl;;
        cout<<"Initial Short pole length = %f\n"<<LENGTH_2<<endl;
        first_time = 0;
    }
}

void CartPole::performAction(double output, int stepnum)
{ 
  
    int i;
    double  dydx[6];

    const bool RK4=true; //Set to Runge-Kutta 4th order integration method
    const double EULER_TAU= TAU/4;
 
    /*random start state for long pole*/
    /*state[2]= drand48();   */
     
    /*--- Apply action to the simulated cart-pole ---*/

    if(RK4){
        for(i=0;i<2;++i){
            dydx[0] = state[1];
            dydx[2] = state[3];
            dydx[4] = state[5];
            step(output,state,dydx);
            rk4(output,state,dydx,state);
        }
    }
    else{
        for(i=0;i<8;++i){
            step(output,state,dydx);
            state[0] += EULER_TAU * dydx[0];
            state[1] += EULER_TAU * dydx[1];
            state[2] += EULER_TAU * dydx[2];
            state[3] += EULER_TAU * dydx[3];
            state[4] += EULER_TAU * dydx[4];
            state[5] += EULER_TAU * dydx[5];
        }
    }

    //Record this state
    cartpos_sum+=fabs(state[0]);
    cartv_sum+=fabs(state[1]);
    polepos_sum+=fabs(state[2]);
    polev_sum+=fabs(state[3]);
    if (stepnum<=1000)
        jigglestep[stepnum-1]=fabs(state[0])+fabs(state[1])+fabs(state[2])+fabs(state[3]);

    if (false) {
        //cout<<"[ x: "<<state[0]<<" xv: "<<state[1]<<" t1: "<<state[2]<<" t1v: "<<state[3]<<" t2:"<<state[4]<<" t2v: "<<state[5]<<" ] "<<
        //cartpos_sum+cartv_sum+polepos_sum+polepos_sum+polev_sum<<endl;
        if (!(outsideBounds())) {
            if (balanced_sum<1000) {
                cout<<".";
                ++balanced_sum;
            }
        }
        else {
            if (balanced_sum==1000)
                balanced_sum=1000;
            else balanced_sum=0;
        }
    }
    else if (!(outsideBounds()))
        ++balanced_sum;

}

void CartPole::step(double action, double *st, double *derivs)
{
    double force,costheta_1,costheta_2,sintheta_1,sintheta_2,
        gsintheta_1,gsintheta_2,temp_1,temp_2,ml_1,ml_2,fi_1,fi_2,mi_1,mi_2;

    force =  (action - 0.5) * FORCE_MAG * 2;
    costheta_1 = cos(st[2]);
    sintheta_1 = sin(st[2]);
    gsintheta_1 = GRAVITY * sintheta_1;
    costheta_2 = cos(st[4]);
    sintheta_2 = sin(st[4]);
    gsintheta_2 = GRAVITY * sintheta_2;
    
    ml_1 = LENGTH_1 * MASSPOLE_1;
    ml_2 = LENGTH_2 * MASSPOLE_2;
    temp_1 = MUP * st[3] / ml_1;
    temp_2 = MUP * st[5] / ml_2;
    fi_1 = (ml_1 * st[3] * st[3] * sintheta_1) +
        (0.75 * MASSPOLE_1 * costheta_1 * (temp_1 + gsintheta_1));
    fi_2 = (ml_2 * st[5] * st[5] * sintheta_2) +
        (0.75 * MASSPOLE_2 * costheta_2 * (temp_2 + gsintheta_2));
    mi_1 = MASSPOLE_1 * (1 - (0.75 * costheta_1 * costheta_1));
    mi_2 = MASSPOLE_2 * (1 - (0.75 * costheta_2 * costheta_2));
    
    derivs[1] = (force + fi_1 + fi_2)
        / (mi_1 + mi_2 + MASSCART);
    
    derivs[3] = -0.75 * (derivs[1] * costheta_1 + gsintheta_1 + temp_1)
        / LENGTH_1;
    derivs[5] = -0.75 * (derivs[1] * costheta_2 + gsintheta_2 + temp_2)
        / LENGTH_2;

}

void CartPole::rk4(double f, double y[], double dydx[], double yout[])
{

	int i;

	double hh,h6,dym[6],dyt[6],yt[6];


	hh=TAU*0.5;
	h6=TAU/6.0;
	for (i=0;i<=5;++i) yt[i]=y[i]+hh*dydx[i];
	step(f,yt,dyt);
	dyt[0] = yt[1];
	dyt[2] = yt[3];
	dyt[4] = yt[5];
	for (i=0;i<=5;++i) yt[i]=y[i]+hh*dyt[i];
	step(f,yt,dym);
	dym[0] = yt[1];
	dym[2] = yt[3];
	dym[4] = yt[5];
	for (i=0;i<=5;++i) {
		yt[i]=y[i]+TAU*dym[i];
		dym[i] += dyt[i];
	}
	step(f,yt,dyt);
	dyt[0] = yt[1];
	dyt[2] = yt[3];
	dyt[4] = yt[5];
	for (i=0;i<=5;++i)
		yout[i]=y[i]+h6*(dydx[i]+dyt[i]+2.0*dym[i]);
}

bool CartPole::outsideBounds()
{
    const double failureAngle = thirty_six_degrees; 

    return 
        state[0] < -2.4              || 
        state[0] > 2.4               || 
        state[2] < -failureAngle     ||
        state[2] > failureAngle      ||
        state[4] < -failureAngle     ||
        state[4] > failureAngle;  
}

void CartPole::nextTask()
{

    LENGTH_2 += POLE_INC;   /* LENGTH_2 * INCREASE;   */
    MASSPOLE_2 += MASS_INC; /* MASSPOLE_2 * INCREASE; */
    //  ++new_task;
    cout<<"#Pole Length %2.4f\n"<<LENGTH_2<<endl;
}

void CartPole::simplifyTask()
{
    if(POLE_INC > MIN_INC) {
        POLE_INC = POLE_INC/2;
        MASS_INC = MASS_INC/2;
        LENGTH_2 -= POLE_INC;
        MASSPOLE_2 -= MASS_INC;
        cout<<"#SIMPLIFY\n"<<endl;
        cout<<"#Pole Length %2.4f\n"<<LENGTH_2;
    }
    else
        {
            cout<<"#NO TASK CHANGE\n"<<endl;
        }
}
