#pragma once
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

 enum HEURISTIC
  {
    MIN_DISTANCE_FROM_GOAL, 
    MIN_DISTANCE_FROM_STRAIGTH_LINE
  }; 

class Params
{
public: 
  static std::string FILE; 
  static int K_SIZE;
  static HEURISTIC H_TYPE; 
private: 
    static std::string FILE_cat;

};

class IOmanager
{

public:
  IOmanager(void);
  ~IOmanager(void);

};


