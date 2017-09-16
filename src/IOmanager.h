#pragma once
#include <iostream>

 enum HEURISTIC
  {
    MIN_DISTANCE_FROM_GOAL, 
    MIN_DISTANCE_FROM_STRAIGTH_LINE
  }; 

  enum NEIGHBORHOOD
  {
    KNN, 
    RADIUS
  }; 


class Params
{

public: 
  static void setFile(std::string  file); 
  static std::string getFile(); 

  static bool RANDOM; 
  static int  RANDOM_SIZE; 
  static HEURISTIC H_type; 
  static NEIGHBORHOOD N_type; 
  static double N_param; 
  static int goals_n;
  static bool is2D; 
  static bool PREPROC; 

private: 
    static std::string FILE; 

};


class IOmanager
{

public:
  IOmanager(void);
  ~IOmanager(void);

};

