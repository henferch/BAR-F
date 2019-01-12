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
#   Description: This script generates the additional plots for data visualization
#   Requirement: This program must be executed after generating data with the script 
#   "main.m" for all the implemented filters in folder "filters".
#
###########################################################################################

clear all;
close all;

  
### Step One: check wheter data has been generated

if ~(exist("output/mat/filterBAR_F.mat"))
  display("Error: Please run 'filterBAR-F.m'");
elseif  ~(exist("output/mat/filterKF.mat"))
    display("Error: Please run 'filterKF.m'");
elseif  ~(exist("output/mat/filterKF_post.mat"))
    display("Error: Please run 'filterKF_post.m'");
elseif  ~(exist("output/mat/filterA_MCL.mat"))
    display("Error: Please run 'filterA_MCL.m'");
elseif  ~(exist("output/mat/filterDgps.mat"))
    display("Error: Please run 'filterDgps.m'");
elseif  ~(exist("output/mat/filterCompass.mat"))
    display("Error: Please run 'filterDeadReckoning.m'");
elseif  ~(exist("output/mat/filterUsbl.mat"))
    display("Error: Please run 'filterUsbl.m'");
elseif  ~(exist("output/mat/filterDeadReckoning.mat"))
    display("Error: Please run 'filterDeadReckoning.m'");
else
  load("output/mat/filterDgps.mat");
  load("output/mat/filterUsbl.mat"); 
  load("output/mat/filterCompass.mat");
  load("output/mat/filterDeadReckoning.mat");
  load("output/mat/filterBAR_F.mat");
  load("output/mat/filterA_MCL.mat");
  load("output/mat/filterKF.mat");
  load("output/mat/filterKF_post.mat");
  
  nCompass = size(allCompass, 1);
  nUsbl = size(allUsbl,1);
  nDgps = size(allDgps,1);
  nDeadReckoning = size(allDeadReckoning,1);
  nA_MCL = size(allA_MCL,1);
  nKF = size(allKF,1);
  nKF_post = size(allKF_post,1);
  nBAR_F = size(allBAR_F,1);
  
  ### Step Two: prepare data for visualization

  times = allDgps(:,1);
  errorUsbl = [];
  for i = 1 : nUsbl
    t = allUsbl(i,1);
    [m ix] = min(abs(times.-t));
    errorUsbl = [errorUsbl; [allDgps(ix,:) allUsbl(i,:)]];
  endfor

  errorUsbl(:,7) = ((errorUsbl(:,2)-errorUsbl(:,5)).^2 + (errorUsbl(:,3)-errorUsbl(:,6)).^2).^0.5;

  errorKF = [];
  for i = 1 : nKF
    t = allKF(i,1);
    [m ix] = min(abs(times.-t));
    errorKF = [errorKF; [allDgps(ix,:) allKF(i,:)]];
  endfor

  errorKF(:,7) = ((errorKF(:,2)-errorKF(:,5)).^2 + (errorKF(:,3)-errorKF(:,6)).^2).^0.5;

  errorKF_post = [];
  for i = 1 : nKF_post
    t = allKF_post(i,1);
    [m ix] = min(abs(times.-t));
    errorKF_post = [errorKF_post; [allDgps(ix,:) allKF_post(i,:)]];
  endfor

  errorKF_post(:,7) = ((errorKF_post(:,2)-errorKF_post(:,5)).^2 + (errorKF_post(:,3)-errorKF_post(:,6)).^2).^0.5;

  errorA_MCL = [];
  for i = 1 : nA_MCL
    t = allA_MCL(i,1);
    [m ix] = min(abs(times.-t));
    errorA_MCL = [errorA_MCL; [allDgps(ix,:) allA_MCL(i,:)]];
  endfor

  errorA_MCL(:,7) = ((errorA_MCL(:,2)-errorA_MCL(:,5)).^2 + (errorA_MCL(:,3)-errorA_MCL(:,6)).^2).^0.5;

  errorDeadReckoning = [];
  for i = 1 : nDeadReckoning
    t = allDeadReckoning(i,1);
    [m ix] = min(abs(times.-t));
    errorDeadReckoning = [errorDeadReckoning; [allDgps(ix,:) allDeadReckoning(i,:)]];
  endfor
   
  errorDeadReckoning(:,7) = ((errorDeadReckoning(:,2)-errorDeadReckoning(:,5)).^2 + (errorDeadReckoning(:,3)-errorDeadReckoning(:,6)).^2).^0.5;

  errorBAR_F = [];
  for i = 1 : nBAR_F
    t = allBAR_F(i,1);
    [m ix] = min(abs(times.-t));
    errorBAR_F = [errorBAR_F; [allDgps(ix,:) allBAR_F(i,:)]];
  endfor

  errorBAR_F(:,7) = ((errorBAR_F(:,2)-errorBAR_F(:,5)).^2 + (errorBAR_F(:,3)-errorBAR_F(:,6)).^2).^0.5;

  
  ### Step Three: Chosing the desired plots

  % options: 
  % 1) Error vs time
  % 2) Accumumulated error vs time
  % 3) USBL error vs time

  plots= [0 0 0] ; % set here the desired choice position to '1', or '0' otherwise

  choice = menu ("Which plot do you want to generate?", {"1) Error vs time", "2) Accum. error vs time","3) USBL error vs time", "4) All available plots", "5) None"});

  switch (choice)
    case 1
      plots(1) = 1;
    case 2
      plots(2) = 1;
    case 3
      plots(3) = 1;
    case 4 
      plots(1:3) = 1;
    case 5
      display("No plot choice selected.");
    otherwise
      display("Invalid option selections!");
  endswitch  
 
 if plots(1)>0  

    figure(1);
    hold on;
    plot (errorUsbl(:,4)./60, errorUsbl(:,7), 'r');
    plot (errorBAR_F(:,4)./60, errorBAR_F(:,7), 'b');
    plot (errorKF(:,4)./60, errorKF(:,7), 'k');
    plot (errorKF_post(:,4)./60, errorKF_post(:,7), 'm');
    plot (errorA_MCL(:,4)./60, errorA_MCL(:,7), 'g');
    title('Error vs time');
    xlabel("time in min");
    ylabel("Error in m");
    legend(" Usbl ", " BAR-F ", " KF", " KF Post", " MCL ");
    %axis("equal","tight");
    axis([0 62 0 80]);
    print("-deps", "-color", "output/plots/ErrorVsTime.eps");
    
  endif

   
  ### Step Four: Chosing the desired plots
  if plots(2)>0  
    
    figure(2);
    hold on;
    pe = cumsum(errorDeadReckoning(:,7));
    tt = errorDeadReckoning(:,4)./60;
    [id v] = find(pe<65000);
    cropedErrorDeadReckoning = [tt(id) pe(id)];
    
    merrorKF = [errorKF(:,4)./60 cumsum(errorKF(:,7))];
    merrorKF_post = [errorKF_post(:,4)./60 cumsum(errorKF_post(:,7))];
    merrorA_MCL = [errorA_MCL(:,4)./60 cumsum(errorA_MCL(:,7))];
    merrorBAR_F = [errorBAR_F(:,4)./60 cumsum(errorBAR_F(:,7))];
    
    plot (cropedErrorDeadReckoning(:,1), cropedErrorDeadReckoning(:,2), 'r');
    plot (merrorKF(:,1), merrorKF(:,2), 'k');
    plot (merrorA_MCL(:,1), merrorA_MCL(:,2), 'g');  
    plot (merrorKF_post(:,1), merrorKF_post(:,2), 'm');
    plot (merrorBAR_F(:,1), merrorBAR_F(:,2), 'b');
    title('Accumulated error vs time');
    xlabel("time in min");
    ylabel("Error in m");
    legend("DR", "KF", "A-MCL", "KF-P", "BAR-F",   "location", "northeast", "orientation", "horizontal");
    axis([0 62 0 70000]);
    print("-deps", "-color", "output/plots/AccumErrorVsTime.eps");
    
   
  endif
   
  if plots(3)>0
    figure(3);
    plot (errorUsbl(:,4)./60, errorUsbl(:,7), 'r');
    title('USBL error vs time');
    xlabel("time in min");
    ylabel("Error in m");
    axis([0 62]);
    print("-deps", "-color", "output/plots/USBLErrorVsTime.eps");
  endif 

  
  ### Step Five: displaying the mean and standard deviation of 'error vs time' data
   
  display("Filters error comparisson:"); 
  display(strcat("DeadReckoning \t mean: ", num2str(mean(errorDeadReckoning(:,7))), ", std dev: ", num2str(std(errorDeadReckoning(:,7)))," "));
  display(strcat("KF \t\t mean: ", num2str(mean(errorKF(:,7))), ", std dev: ", num2str(std(errorKF(:,7)))," "));
  display(strcat("A_MCL \t\t mean: ", num2str(mean(errorA_MCL(:,7))), ", std dev: ", num2str(std(errorA_MCL(:,7)))," "));
  display(strcat("KF_post \t mean: ", num2str(mean(errorKF_post(:,7))), ", std dev: ", num2str(std(errorKF_post(:,7)))," "));
  display(strcat("BAR-F\t\t mean: ", num2str(mean(errorBAR_F(:,7))), ", std dev: ", num2str(std(errorBAR_F(:,7)))," "));

endif

