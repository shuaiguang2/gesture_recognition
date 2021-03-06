#ifndef _RECOGNIZER_H
#define _RECOGNIZER_H

#define QUEUELEN 15

#include <list>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stack>
#include "hmm.h"
#include "ardrone_control.h"

class Recognizer{
public:
    Recognizer();
    void AddObservation(int directionCode);
    void GestureRecognize();	
    void Init(ros::NodeHandle n);
    

private:
    std::list<int> seq;
    std::stack<int> commands;

    void EndDetection();
    int FindMax();
    bool CmpCmd(int first_cmd, int last_cmd);
    void ExecuteCmd();
    //The following variables are referred to testfor.c with the same name.
    int t, T;
    int O[QUEUELEN];
    HMM hmm_left_right, hmm_right_left, hmm_down_up, hmm_up_down, hmm_left_top, hmm_right_top, hmm_circle, hmm_threshold;
    double **alpha_Q5, **alpha_Q8, **alpha_threshold;
    //double *scale;
    double proba[8];

    //The following is for ardrone control
    ArdroneControl ardrone_control;

};

#endif
