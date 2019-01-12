###########################################################################################
#
#   <This program loads the experiment dataset and compares the implementation of the  
#   BAR-F neural network to the Kalman Filter (KF), and the Augmented Monte Carlo (A-MCL) 
#   algorithms>
#
#   Copyright (C) <2018>  <Hendry Ferreira Chame>
#
#   This program is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, version 3.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#   You should have received a copy of the GNU General Public License
#   along with this program.  If not, see <https://www.gnu.org/licenses/>.
#    
#   Programmed by: Hendry Ferreira Chame
#   Institution: Federal University of Rio Grande (FURG), Rio Grande, BRAZIL
#   E-mail: hendrychame@furg.br, hendryf@gmail.com
#     
#   Paper:  Reliable fusion of black-box estimates of underwater localization
#   Authors: Hendry Ferreira Chame, Matheus Machado dos Santos, Sílvia Silva da Costa Botelho
#   2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). doi: 10.1109/IROS.2018.8593593
#
#   Description: This is the main program for executing the filters implemented
#
###########################################################################################

display("#################################################################################################");
display("")
display("  <GNU Octave Implementation of BAR-F>  Copyright (C) <2018>  <Hendry Ferreira Chame>           ");
display("  This program comes with ABSOLUTELY NO WARRANTY. This is free software, and you are welcome    "); 
display("  to redistribute it under certain conditions; for details consult the file 'LICENSE.txt'.      ");
display("")
display("  For more details about the framework, please consult the following reference:                 "); 
display("")
display(" \"Reliable fusion of black-box estimates of underwater localization\" ");  
display("  Hendry Ferreira Chame, Matheus Machado dos Santos, Sílvia Silva da Costa Botelho              ");
display("  2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)               ");
display("")
display("#################################################################################################");
display("")

###########################################################################################
#
# Description: Main loop, data is loaded from 'data' folder. 
#              A filter implementation must be selected at once.
#              Results are data matrices and plots saved to the 'output' folder 
#              -------------------------------------------------------------------------
#              Filters are assumed to implement the following methods:
#
#              filterInfo() .................... Displays the filter name by console 
#              filterInitialize( data ) ........ Initializes the filter parameters
#              filterReadCompass( data) ........ Input compass data to the filter
#              filterReadOdometry(mesurement) .. Input odometry data to the filter   
#              filterReadUsbl(mesurement) ...... Input USBL data to the filter   
#              filterReadDgps(mesurement) ...... Input DGPS data to the filter   
#              filterProcessing(t) ............. Filter's loop processing 
#              filterPlotData() ................ Filter's plot generation 
#              filterRecordData() .............. Filter's data saving
#             
###########################################################################################

clear all;
close all;

display("Main program start");

#### Step One: Loading data 

load("data/compassData.mat");
load("data/odometryData.mat");
load("data/usblData.mat");
load("data/dgpsData.mat");

display("Data loaded!");
  
#### Step Two: Plese select a filter implementation

display("Please select a choice");
fflush(stdout); 

choice = menu ("Filter select a fusion algorithm:", {"1) Kalman Filter (KF)", "2) KF (post-processed)","3) Dead Reckoning", "4) Augmented  MCL", "5) BAR-F Neural network"});

switch (choice)
  case {1}
    source("filters/filterKF.m");
  case {2}
    source("filters/filterKF_post.m");
  case {3}
    source("filters/filterDeadReckoning.m");
  case {4}
    source("filters/filterA_MCL.m");
  case {5}      
    source("filters/filterBAR-F.m");
  otherwise
    display("Invalid choice!");
    choice = 0;
endswitch 

if choice > 0    
  
  filterInfo();
  fflush(stdout); 

  nCompass = size(compassData,1);
  nOdom = size(odometryData,1);
  nUsbl = size(usblData,1);
  nDgps = size(dgpsData,1);

  times = odometryData(:,1);

  % Chosing the desired experiment time interval
  iInitial = 1;
  iFinal = nOdom;
  
  iOdom = 1;
  iDgps = 1;
  iUsbl = 1;
  iCompass = 1;

  tInitial = times(iInitial);

  idx = find(odometryData(:, 1)>=tInitial);
  iOdom = idx(1);

  idx = find(dgpsData(:, 1)>=tInitial);
  iDgps = idx(1);

  idx = find(usblData(:, 1)>=tInitial);
  iUsbl = idx(1);

  idx = find(compassData(:, 1)>=tInitial);
  iCompass = idx(1);

  thetaIni = compassData(iCompass,2);
  muIni = usblData(iUsbl,:);

  tCompass = compassData(iCompass,1);
  tOdom = odometryData(iOdom,1);
  tUsbl = usblData(iUsbl,1);
  tDgps = dgpsData(iDgps,1);

  filterInitialize(muIni, thetaIni);

  #### Step Three: Experimet loop
  c1 = time();

  for i=iInitial: iFinal

    t = times(i);  
      
    if (tCompass <= t)&&(iCompass < nCompass)
    
      measurement = wrapTo2Pi(compassData(iCompass,2)-thetaIni);    

      filterReadCompass(measurement);
      
      iCompass = iCompass + 1;        
      tCompass = compassData(iCompass, 1);

  endif

      
    if (tOdom <= t)&&(iOdom < nOdom)
      
      measurement = odometryData(iOdom,:);    
     
      filterReadOdometry(measurement);   
                            
      iOdom = iOdom + 1;
      tOdom = odometryData(iOdom, 1);
      
    endif

    if (tUsbl <= t)&&(iUsbl < nUsbl)
    
      measurement = usblData(iUsbl,:);
      
      filterReadUsbl(measurement);   

      iUsbl = iUsbl + 1;
      tUsbl = usblData(iUsbl, 1);
      
    endif

    if (tDgps <= t)&&(iDgps < nDgps)
        
        measurement = dgpsData(iDgps,:);
        
        filterReadDgps(measurement);   
        
        iDgps = iDgps + 1;
        tDgps = dgpsData(iDgps, 1);  
        
    endif

    filterProcessing(t);
    
  endfor
  c2 = time();
  #### Step Four: Plot generation

  filterPlotData();
  fflush(stdout); 

  #### Step Five: Data recording

  filterRecordData();
  fflush(stdout); 

  #### Step Six: Program termination
  display(strcat("Total execution time: ", num2str(c2-c1)));
  fflush(stdout); 
endif
  
#### Call to the method comparison plots
if  choice>0&&exist("output/mat/filterDgps.mat")&&exist("output/mat/filterUsbl.mat")&&exist("output/mat/filterCompass.mat")&&exist("output/mat/filterDeadReckoning.mat")&&exist("output/mat/filterA_MCL.mat")&&exist("output/mat/filterKF.mat")&&exist("output/mat/filterKF_post.mat")&&exist("output/mat/filterBAR_F.mat")
  doPlots
endif

display("Main program end");
