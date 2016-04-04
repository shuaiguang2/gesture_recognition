#define QUEUELEN 15
#define THRESHOLDMODEL -1;

#include <iostream>
using std::cout;
using std::endl;

#include "recognizer.h"
const char* addr1 = "left_right.hmm";
const char* addr2 = "right_left.hmm";
const char* addr3 = "down_up.hmm";
const char* addr4 = "up_down.hmm";
const char* addr5 = "left_top.hmm";
const char* addr6 = "right_top.hmm";
const char* addr7 = "circle.hmm";
const char* addr8 = "threshold.hmm";
Recognizer::Recognizer(){}

void Recognizer::Init(ros::NodeHandle n){
    
    FILE *fp;
    fp = fopen(addr1, "r");
    if(fp == NULL){
	fprintf(stderr, "Error: File %s not found\n", addr1);
	exit(1);
    }
    ReadHMM(fp, &hmm_left_right);
    fclose(fp);

    fp = fopen(addr2, "r");
    if(fp == NULL){
        fprintf(stderr, "Error: File %s not found\n", addr2);
        exit(1);
    }
    ReadHMM(fp, &hmm_right_left);
    fclose(fp);

    fp = fopen(addr3, "r");
    if(fp == NULL){
        fprintf(stderr, "Error: File %s not found\n", addr3);
        exit(1);
    }
    ReadHMM(fp, &hmm_down_up);
    fclose(fp);

    fp = fopen(addr4, "r");
    if(fp == NULL){
        fprintf(stderr, "Error: File %s not found\n", addr3);
        exit(1);
    }
    ReadHMM(fp, &hmm_up_down);
    fclose(fp);

    fp = fopen(addr5, "r");
    if(fp == NULL){
        fprintf(stderr, "Error: File %s not found\n", addr3);
        exit(1);
    }
    ReadHMM(fp, &hmm_left_top);
    fclose(fp);

    fp = fopen(addr6, "r");
    if(fp == NULL){
        fprintf(stderr, "Error: File %s not found\n", addr3);
        exit(1);
    }
    ReadHMM(fp, &hmm_right_top);
    fclose(fp);

    fp = fopen(addr7, "r");
    if(fp == NULL){
        fprintf(stderr, "Error: File %s not found\n", addr3);
        exit(1);
    }
    ReadHMM(fp, &hmm_circle);
    fclose(fp);

    fp = fopen(addr8, "r");
    if(fp == NULL){
        fprintf(stderr, "Error: File %s not found\n", addr3);
        exit(1);
    }
    ReadHMM(fp, &hmm_threshold);
    fclose(fp);

    T = QUEUELEN;
    alpha_Q5 = dmatrix(1, T, 1, 5); 
    alpha_Q8 = dmatrix(1, T, 1, 8);
    alpha_threshold = dmatrix(1, T, 1, hmm_threshold.N);

    //Initialize the ardrone control
    ardrone_control.init(n);
}

void Recognizer::AddObservation(int directionCode){
    if( directionCode < 0){
	//printf("Unvalid input\n");
	if(!seq.empty())
	    seq.pop_front();
	return;
    }//if

    if(seq.size() < QUEUELEN){
	seq.push_back(directionCode);
    }//if size

    else{
 	seq.pop_front();
	seq.push_back(directionCode);
    }//else
}

void Recognizer::GestureRecognize(){
    if(seq.size() < QUEUELEN){
	//printf("too few features\n");
	if( 0 == seq.size())
	    ExecuteCmd();
	return;
    }//if size
    int i=0;
    for(std::list<int>::iterator itr = seq.begin(); itr !=seq.end(); itr++){
	O[i++] = *itr;
	//cout << O[i-1]<<" ";
    }
   
    Forward(&hmm_left_right, T, O, alpha_Q5, proba);
    Forward(&hmm_right_left, T, O, alpha_Q5, proba+1);
    Forward(&hmm_down_up, T, O, alpha_Q5, proba+2);
    Forward(&hmm_up_down, T, O, alpha_Q5, proba+3);
    Forward(&hmm_left_top, T, O, alpha_Q8, proba+4);
    Forward(&hmm_right_top, T, O, alpha_Q8, proba+5);
    Forward(&hmm_circle, T, O, alpha_Q8, proba+6);
    Forward(&hmm_threshold, T, O, alpha_threshold, proba+7);
  
    EndDetection();
    

}

void Recognizer::EndDetection(){
    int max_num = FindMax();
    //cout <<endl<< "max_num"<<max_num<<endl;
    if( -1 == max_num ) return;
    else{
	if(commands.empty())
	    commands.push(max_num);

	else{
	    if( max_num == commands.top())
		return;
	    else{
		int exe_cmd = commands.top();
		commands.pop();
 		commands.push(max_num);
		//cout<<"exeCom "<<exeCom<<"max_num "<<max_num<<endl;
	    }//else
	}//else	
    }//else
}//EndDetection

int Recognizer::FindMax(){
    int Pos;
    double max;
    max = proba[0];
    Pos = 0;
    for(int i = 0; i<8; i++){
	for(int j = i+1; j < 8; j++){
	    if(max<proba[j]){
	    	max = proba[j];
		Pos = j;
	    }	    
	}
    }
    //set Pos=-1 if it is threshold model.
    if(7==Pos) Pos=-1;
    return Pos;
}


bool Recognizer::CmpCmd(int first_cmd, int last_cmd){
    //judge whether first_cmd includes last_cmd
    //circle(6) includes down_up(2)
    
    if(6 == first_cmd && 2 == last_cmd )
   	return true;
    else 
	return false;

}

void Recognizer::ExecuteCmd(){
    if( !commands.empty() ){
	ardrone_control.sendCommand(commands.top());
	commands.pop();
    }//if
}
