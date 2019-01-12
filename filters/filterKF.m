###########################################################################################
#
#   <This program loads the experiment dataset and compares the implementation of the  
#   BAR-F neural network to the Kalman Filter (KF), the Augmented Monte Carlo (A-MCL) 
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
#   Description: Implelemtation of the Kalman Filter algorithm.
#              -------------------------------------------------------------------------
#              Filters are assumed to implement the following methods:
#
#              filterInfo() ................ Displays the filter name by console 
#              filterInitialize( data ) .... Initializes the filter parameters
#              filterReadCompass( data ) ... Input compass data to the filter
#              filterReadOdometry( data ) .. Input odometry data to the filter   
#              filterReadUsbl( data ) ...... Input USBL data to the filter   
#              filterReadDgps( data ) ...... Input DGPS data to the filter   
#              filterProcessing( time ) .... Filter's loop processing 
#              filterPlotData() ............ Filter's plot generation 
#              filterRecordData() .......... Filter's data saving
#             
# Requirement: This program must be loaded within the 'main.m' program
#
###########################################################################################


ns = 3; % state dimension
muUSBL = muODOM = zeros(1,ns);

### kallman filter 1: theta estimate
global ns1 = 1; % theta

sigmaCompass = [0.02]*(pi/180); % deg
sigmaThetaODOM = [0.7]*(pi/180); % deg

global R1 = eye(ns1)*sigmaCompass; %; observation noise covariance 
global Q1 = eye(ns1)*sigmaThetaODOM; %; process noise covariance 

global F1 = eye(ns1); % transition state model
global B1 = eye(ns1); % control input model
global H1 = eye(ns1); % observation model
global P1 = R1;

global x1 = 0;

###  kallman filter 2: rho theta estimate
global ns2 = 3; % x y theta

sigmaUSBL = [0.1 0.1 sigmaCompass]; %[m  deg] [distance, bearing]
sigmaODOM = [0.5 0.5 sigmaThetaODOM]; % m/sec

global R2 = eye(ns2).*sigmaUSBL; %; observation noise covariance 
global Q2 = eye(ns2).*sigmaODOM; %; process noise covariance 
global H2 = eye(ns2); % observation model
global P2 = R2;

global F2 = eye(ns2); % transition state model
F2 = [1 0 0; 0 1 0; 0 0 0]; % transition state model
global B2 = eye(ns2); % control input model

global x2 = mu = muODOM';

### data containers 

global allKF = [];
global allUsbl = [];
global allDgps = [];
global allCompass  = [];

### Filter interface functions 

function filterInfo()
  display("KF filter loaded!");
endfunction
  
  
function filterInitialize(mu, theta)
  
  muODOM  = muUSBL = [mu(2:3) 0];
  x2 = muODOM';
  x1 = thetaIni = theta;

endfunction


function filterReadCompass(mesurement)

  global H1 P1 R1 x1 ns1 allCompass;   
    % correction step filter 1
    theta = mesurement;
    z1 = theta;
    y1 = z1 - H1*x1;
    S1 = H1*P1*H1' + R1;
    K1 = P1*H1'*inv(S1);
    x1 = x1 + K1*y1;
    P1 = (eye(ns1)-K1*H1)*P1;
    allCompass = [allCompass ; mesurement];

endfunction

function filterReadOdometry(mesurement)
       
    global x1 F1 B1 P1 Q1 F2 P2 B2 P2 Q2 x2 allKF;   
    
    tOdom = mesurement(1);
    mes = mesurement(2:3)';    
    dTheta  = mesurement(4);
    
    angle = wrapTo2Pi(x1);
    bTr = [cos(angle) -sin(angle); sin(angle) cos(angle)];           
    %frameAngle = -pi;
    frameAngle = pi/2;
    rTs = [cos(frameAngle) -sin(frameAngle); sin(frameAngle) cos(frameAngle)];
    wTb = [1 0; 0 -1];
    dX = (wTb*bTr*rTs*mes)';
    
    %prediction step filter 1
    u1 = [dTheta];
    x1 = F1*x1 + B1*u1;
    P1 = F1*P1*F1' + Q1; % predicted covariance    

    %prediction step filter 2
    u2 = [dX x1]';
    x2 = F2*x2 + B2*u2;
    P2 = F2*P2*F2' + Q2; % predicted covariance
    allKF = [allKF ; [tOdom x2']];
    
endfunction

function filterReadUsbl(mesurement)

    global H2 P2 R2 x1 x2 ns2 allUsbl;   
    muUSBL = mesurement(2:3);
    
   % if (diff(1)^2 + diff(2)^2).^0.5 > 0.5
      
    % correction step filter 2
    z2 = [ muUSBL(1:2) x1]'; 
    y2 = z2 - H2*x2;
    S2 = H2*P2*H2' + R2;
    K2 = P2*H2'*inv(S2);
    x2 = x2 + K2*y2;
    P2 = (eye(ns2)-K2*H2)*P2;
    allUsbl = [allUsbl ; mesurement];
    
endfunction

function filterReadDgps(mesurement)

  global allDgps;  
  
  allDgps = [allDgps; mesurement];
  
endfunction

function filterProcessing(t)
  % no neet to do anything
endfunction

function filterPlotData()

  global allDgps allKF allUsbl;

  figure(1);
  hold on;
  title("Kalman Filter");    
  plot (allDgps(:,2),  allDgps(:,3), 'k',"LineWidth", 1);
  plot (allKF(:,2),  allKF(:,3), 'm',"LineWidth", 1);
  plot (allUsbl(:,2), allUsbl(:,3), '.b',"LineWidth", 1);
  axis("equal", "tight");  
  print("-deps", "-color", "output/plots/filterKF.eps");
  

endfunction

function filterRecordData()

  global allKF;

  save 'output/mat/filterKF.mat' allKF;

endfunction


