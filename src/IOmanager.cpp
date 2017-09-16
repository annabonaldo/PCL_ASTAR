#include "IOmanager.h"

//std::string Params::FILE_cat = "C:\\pointclouds\\ism_test_cat.pcd";



#include <fstream>
#include<iostream>
#include <stdio.h>
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
bool          Params::PREPROC = false; 
bool          Params::deleteFile = false; 
bool          Params::deleteAstarLogFile = false;  
bool          Params::deleteGraphLogFile = false;  

void Params::removeFiles()
{
  if (deleteFile) std::remove(Params::getFile().c_str()); 
  Params::deleteFile = false; 

  if (deleteAstarLogFile) std::remove("logAstar.txt"); 
  Params::deleteAstarLogFile = false; 

  if (deleteGraphLogFile) std::remove("logGraph2D.txt"); 
  Params::deleteGraphLogFile = false; 
}

IOmanager::IOmanager(void)
{
}

IOmanager::~IOmanager(void)
{
}
