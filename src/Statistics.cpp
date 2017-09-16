#include "Statistics.h"
#include "IOmanager.h"; 
#include <iostream>
#include <fstream>
std::ofstream Statistics::csv_file;  
Statistics::Statistics(void)
{
  itr=0; 
  cloud_dim.push_back(0);
  cloud_dim.push_back(0);
  cloud_dim.push_back(0);
}

Statistics::~Statistics(void)
{
}

void Statistics::log()
{
  if(!csv_file.is_open())
    csv_file.open("statistics.csv", std::ios::out |std::ios::app ); 
 
        // INPUT PARAMETERS 
  /* a*/ csv_file << cloud_size <<";";
  /* b*/ csv_file << cloud_dim[0] << "; " << cloud_dim[1] <<";" << cloud_dim[2]<<";";
  /* c*/ if(is2D) csv_file << "2D;" ; 
  /*  */ else     csv_file << "3D;" ; 
 
  /* d*/ if(Params::RANDOM) csv_file <<"Random; "; 
         else csv_file << Params::getFile <<";"; 
  /* e*/ csv_file << Params::H_type  <<";"; 
  /* f*/ if(Params::PREPROC) csv_file << "false;"; 
  /*  */ else                csv_file << "true;"; 

  /* g*/ csv_file << Params::N_type  <<";";
  /* h*/ csv_file << Params::N_param <<";";
  /* i*/ csv_file << Params::goals_n <<";";


  // RESULT OF COMPUTATION
  csv_file << this->start_goal_distance  <<";";// 1
  csv_file << this->queue_mean_size      <<";";// 2
  csv_file << this->n_open_nodes         <<";";// 3
  csv_file << this->n_closed_nodes       <<";";// 4
  csv_file << this->path_lenght          <<";";// 5
  csv_file << this->execution_time       <<";";// 6
  csv_file << this->itr                  <<";";// 7
  csv_file << std::endl; 
  
}

void Statistics::logHeaders()
{
  return; 
  if(!csv_file.is_open())
    csv_file.open("statistics.csv", std::ios::out |std::ios::app ); 


         std::string header; 
  /* a */ header +="CLOUD SIZE;"; 
  /* b */ header += "sizeX; sizeY; sizeZ;";
  /* c */ header += "2D/3D;";
  /* d */ header += "FILE;";
  /* e */ header += "H;";
  /* f */ header += "PREPROC;";
  /* g */ header += "NGB;";
  /* h */ header += "NGP_PAR;";
  /* i */ header += "GOALS; ";           

  header += "dist(start-goal);";    // 1
  header += "Q_MEAN_SIZE;";         // 2
  header += "Q_OPEN;";              // 3
  header += "Q_CLOSE;" ;            // 4
  header += "PATH SIZE;";            // 5
  header += "EXEC TIME;";            // 6
  header += "TOT NODES;";            // 6

  csv_file << header.c_str() <<std::endl;


}