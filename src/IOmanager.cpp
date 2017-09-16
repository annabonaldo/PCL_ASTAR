#include "IOmanager.h"

//std::string Params::FILE_cat = "C:\\pointclouds\\ism_test_cat.pcd";



#include <fstream>
#include<iostream>

std::string Params::FILE; 

void Params::setFile(std::string  file)
{
  Params::RANDOM  = false; 
  FILE = file; 
}
std::string Params::getFile()
{
  return FILE; 
}

bool          Params::RANDOM = true;  
int           Params::RANDOM_SIZE = 100;
HEURISTIC     Params::H_type = HEURISTIC::MIN_DISTANCE_FROM_GOAL; 
NEIGHBORHOOD  Params::N_type = NEIGHBORHOOD::KNN; 
double        Params::N_param = 10.0; 
int           Params::goals_n = 1;
bool          Params::is2D = false; 
bool          Params:: PREPROC = false; 


IOmanager::IOmanager(void)
{
}

IOmanager::~IOmanager(void)
{
}
