/*
 * opencog/learning/moses/example-progs/pole_scoring.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Joel Lehman
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#ifndef _POLE_SCORING_H
#define _POLE_SCORING_H

#include <opencog/util/numeric.h>
#include <opencog/util/mt19937ar.h>

#include <opencog/comboreduct/combo/vertex.h>
#include <opencog/comboreduct/combo/simple_nn.h>
#include <opencog/comboreduct/reduct/ann_rules.h>

#include "../scoring/scoring.h"
#include "pole_balancing.h"

using namespace opencog;
using namespace combo;
using namespace std;
using namespace moses;
#define MIN_FITNESS -1.0e10

struct AnnPole2NVFitnessFunction : public unary_function<combo_tree, double>
{
    result_type operator()(argument_type tr) const
    {
        bool velocity = false;
        if (tr.empty())
            return MIN_FITNESS;
        tree_transform tt;
        ann nn = tt.decodify_tree(tr);
        
        CartPole the_cart(true,velocity);
        the_cart.nmarkov_long=false;
        the_cart.generalization_test=false;
        double fitness = -100000.0+the_cart.evalNet(&nn);
        return fitness;
    }

};

struct AnnPole2FitnessFunction : public unary_function<combo_tree, double>
{
    result_type operator()(argument_type tr) const
    {
        bool velocity = true;
        if (tr.empty())
            return MIN_FITNESS;
        tree_transform tt;
        ann nn = tt.decodify_tree(tr);
        
        CartPole the_cart(true,velocity);
        the_cart.nmarkov_long=false;
        the_cart.generalization_test=false;
        double fitness = -100000+the_cart.evalNet(&nn);
        return fitness;
    }
};

struct AnnPoleFitnessFunction : public unary_function<combo_tree, double>
{
    result_type operator()(argument_type tr) const
    {
        if (tr.empty())
            return MIN_FITNESS;
        
        tree_transform tt;
        ann nn = tt.decodify_tree(tr);
        
        return go_cart(&nn,100000);
    }

    //     cart_and_pole() was take directly from the pole simulator written
    //     by Richard Sutton and Charles Anderson.
    int go_cart(ann *net,int max_steps) const
    {
        float x,			/* cart position, meters */
            x_dot,			/* cart velocity */
            theta,			/* pole angle, radians */
            theta_dot;		/* pole angular velocity */
        int steps=0,y;
        
        int random_start=1;
        
        double in[5];  //Input loading array
        
        double out1;
        double out2;
        
        //     double one_degree= 0.0174532;	/* 2pi/360 */
        //     double six_degrees=0.1047192;
        double twelve_degrees=0.2094384;
        //     double thirty_six_degrees= 0.628329;
        //     double fifty_degrees=0.87266;
        
        if (random_start) {
            /*set up random start state*/
            x = (lrand48()%4800)/1000.0 - 2.4;
            x_dot = (lrand48()%2000)/1000.0 - 1;
            theta = (lrand48()%400)/1000.0 - .2;
            theta_dot = (lrand48()%3000)/1000.0 - 1.5;
        }
        else 
            x = x_dot = theta = theta_dot = 0.0;
        
        /*--- Iterate through the action-learn loop. ---*/
        while (steps++ < max_steps) {
            
            /*-- setup the input layer based on the four iputs --*/
            //setup_input(net,x,x_dot,theta,theta_dot);
            in[0]=1.0;  //Bias
            in[1]=(x + 2.4) / 4.8;;
            in[2]=(x_dot + .75) / 1.5;
            in[3]=(theta + twelve_degrees) / .41;
            in[4]=(theta_dot + 1.0) / 2.0;
            net->load_inputs(in);
            
            int depth = net->feedforward_depth();
            dorepeat(depth)
                net->propagate();
            
            /*-- decide which way to push via which output unit is greater --*/
            out1=net->outputs[0]->activation;
            out2=net->outputs[1]->activation;
            
            if (out1 > out2)
                y = 0;
            else
                y = 1;
            
            /*--- Apply action to the simulated cart-pole ---*/
            cart_pole(y, &x, &x_dot, &theta, &theta_dot);
            
            /*--- Check for failure.  If so, return steps ---*/
            if (x < -2.4 || x > 2.4  || theta < -twelve_degrees ||
                                                theta > twelve_degrees) 
                return steps;             
        }
        
        return steps;
    } 


    //     cart_and_pole() was take directly from the pole simulator written
    //     by Richard Sutton and Charles Anderson.
    //     This simulator uses normalized, continous inputs instead of 
    //    discretizing the input space.
    /*----------------------------------------------------------------------
      cart_pole:  Takes an action (0 or 1) and the current values of the
      four state variables and updates their values by estimating the state
      TAU seconds later.
      ----------------------------------------------------------------------*/
    void cart_pole(int action, float *x,float *x_dot, float *theta, float *theta_dot) const {
        float xacc,thetaacc,force,costheta,sintheta,temp;
  
        const float GRAVITY=9.8;
        const float MASSCART=1.0;
        const float MASSPOLE=0.1;
        const float TOTAL_MASS=(MASSPOLE + MASSCART);
        const float LENGTH=0.5;	  /* actually half the pole's length */
        const float POLEMASS_LENGTH=(MASSPOLE * LENGTH);
        const float FORCE_MAG=10.0;
        const float TAU=0.02;	  /* seconds between state updates */
        const float FOURTHIRDS=1.3333333333333;

        force = (action>0)? FORCE_MAG : -FORCE_MAG;
        costheta = cos(*theta);
        sintheta = sin(*theta);
  
        temp = (force + POLEMASS_LENGTH * *theta_dot * *theta_dot * sintheta)
            / TOTAL_MASS;
  
        thetaacc = (GRAVITY * sintheta - costheta* temp)
            / (LENGTH * (FOURTHIRDS - MASSPOLE * costheta * costheta
                         / TOTAL_MASS));
  
        xacc  = temp - POLEMASS_LENGTH * thetaacc* costheta / TOTAL_MASS;
  
        /*** Update the four state variables, using Euler's method. ***/
  
        *x  += TAU * *x_dot;
        *x_dot += TAU * xacc;
        *theta += TAU * *theta_dot;
        *theta_dot += TAU * thetaacc;
    }
};


// This is what the original zource had this as, but its not
// obviously correct, to me.
#define CPXY_RATIO 1.0

struct ann_pole2nv_cscore : public cscore_base
{
    composite_score operator()(const combo_tree& tr) const
    {
        complexity_t cpxy = tr.size();
        return composite_score(p2ff(tr), cpxy, cpxy/CPXY_RATIO);
    }
    AnnPole2NVFitnessFunction p2ff;
};

struct ann_pole2nv_bscore : public bscore_base
{
    result_type operator()(const combo_tree& tr) const
    {
        composite_score cs(ann_pole2nv_cscore()(tr));

        behavioral_score bs;
        bs.push_back(cs.get_score());
        bs.push_back(cs.get_penalty());
        return bs;
    }
    behavioral_score best_possible_bscore() const
    {
        return {0.0};
    }
};

struct ann_pole2_cscore : public cscore_base
{
    result_type operator()(const combo_tree& tr) const
    {
        complexity_t sz = tr.size();
        return composite_score(p2ff(tr), sz, sz/CPXY_RATIO);
    }
    AnnPole2FitnessFunction p2ff;
};

struct ann_pole2_bscore : public bscore_base
{
    result_type operator()(const combo_tree& tr) const
    {
        composite_score cs(ann_pole2_cscore()(tr));

        behavioral_score bs;
        bs.push_back(cs.get_score());
        bs.push_back(cs.get_penalty());
        return bs;
    }
    behavioral_score best_possible_bscore() const
    {
        return {0.0};
    }
};

struct ann_pole_cscore  : public cscore_base
{
    composite_score operator()(const combo_tree& tr) const
    {
        complexity_t cpxy = tr.size();
        return composite_score(pff.operator()(tr), cpxy, cpxy/CPXY_RATIO);
    }
    AnnPoleFitnessFunction pff;
};

struct ann_pole_bscore : public bscore_base
{
    behavioral_score operator()(const combo_tree& tr) const
    {
        composite_score cs(ann_pole_cscore()(tr));

        behavioral_score bs;
        bs.push_back(cs.get_score());
        bs.push_back(cs.get_penalty());
        return bs;
    }
    behavioral_score best_possible_bscore() const
    {
        return {0.0};
    }
};


#endif
