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
#   Description: Implementation of the neural network BAR-F 
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

global ns = 2; % state dimension


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

### kallman filter 2: rho theta estimate
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

global x2 = zeros(ns2,1);

### BAR-N neural network parameters

global nb = 2; % Behaviour profiles number
global ne = 2; % Estimator processes available 
global b = [1 0]'; % Layer B (behavior)
global a = zeros(ne,1); % Layer A (activity)
global r = zeros(ne,1); % Layer R (reliability)
global f = zeros(ne,1); % Layer R (fusion)
global Wbf = [0.1 0.9; 0.6 0.5];
global Waf = eye(ne); % activation matrix
global Wrf = eye(ne); % reliability matrix
global Wbff = zeros(nb, ne, ne);

global tEstimators = zeros(1,ne);

%dSmallSigE1 = 1/1000; % heuristically set Tau = mean (time usbl)
dSmallSigE1 = 1/(10*2.5161); % heuristically set Tau = mean (time usbl)
dSmallSigE2 = 1; % 

global deltaSmallSigmas = [dSmallSigE1, dSmallSigE2]; 

global bigSigE1Init = 8*eye(ns); % in meters
global bigSigE2Init = 0.1*eye(ns);
global bigSigmasInit = [bigSigE1Init bigSigE2Init];
global bigSigmas = bigSigmasInit;
global smallSigmas = ones(1,ne);
global lastTimeOdometry = 0
global phi1 = 0.2; % reliability threshold phi in [0,1]
global phi2 = 13; % steepness of the sigmoid 
global crb = zeros(1,ne);
global mus = zeros(ne,ns);


### Data containers

global allBAR_F = [];
global allUsbl = [];
global allDgps = [];
global allCompass  = [];


### BAR-F Neural network functions


# Calculates gamma( ) conforming to Equation (10)
function [result] = gamma(sigma, SIGMA, x, phi)
  
  [V lambda] = eig(SIGMA);
  G = sigma * lambda;
  f = x * V * inv(G) * V' * x'; 
  result = (1 / (1 + exp(-phi.*(sigma - f))));
 
endfunction

# Calculates Psy( ) conforming to Equation (8)
function [mu, sigma, SIGMA, dt] = Psy(leftBrotherId, mus, smallSigmas, bigSigmas, tEstimators)
 
 global ns;
 
 sigma = smallSigmas(leftBrotherId);
 SIGMA = bigSigmas(1:ns,ns*(leftBrotherId-1)+1:ns*leftBrotherId);
 mu = mus(leftBrotherId,:);
 dt = tEstimators(leftBrotherId);
     
endfunction 
 
 
### Filter interface functions 

function filterInfo()
  display("BAR-F filter loaded!");
endfunction
  
  
function filterInitialize(mu, theta)
  
  global  Wbff Wbf x1 x2 nb ne ns b crb mus;
  
  x2 = [mu(2:3) 0]';
  x1 = thetaIni = theta;
  mus = ones(ne,ns).*mu(2:3);
  
  % initializing the network
  maxf = max(Wbf, [], 2);
  minf = min(Wbf, [], 2);

  for k = 1: nb
    for i = 1: ne
      pivot = Wbf(k,i);  
      for j = 1: ne
          if i == j
            Wbff(k,i,j)  =  0;
          else
            diff = pivot - Wbf(k,j);
            x = diff/minf(k);
            Wbff(k,i,j)  =  1/x;      
          endif
        endfor
    endfor
  endfor
  ib = find(b>=1);
  pWbff = reshape(Wbff(ib,:,:),ne,ne);

  % clic pair vector  
  [v c] = max(pWbff');
  crb = c;  

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
       
    global x1 F1 B1 P1 Q1 F2 P2 B2 P2 Q2 x2 lastTimeOdometry;   
    global mus tEstimators smallSigmas deltaSmallSigmas bigSigmas;
    global ns a;
        
    tOdom = mesurement(1);
    mes = mesurement(2:3)';    
    dTheta  = mesurement(4);
    
    angle = wrapTo2Pi(x1);
    bTr = [cos(angle) -sin(angle); sin(angle) cos(angle)];           
    frameAngle = pi/2;
    rTs = [cos(frameAngle) -sin(frameAngle); sin(frameAngle) cos(frameAngle)];
    wTb = [1 0; 0 -1];
    dX = (wTb*bTr*rTs*mes)';
    
    %prediction step KF 1
    u1 = [dTheta];
    x1 = F1*x1 + B1*u1;
    P1 = F1*P1*F1' + Q1; % predicted covariance    

    %prediction step KF 2
    u2 = [dX x1]';        
    x2 = F2*x2 + B2*u2;
    P2 = F2*P2*F2' + Q2; % predicted covariance
    
    %% estimator #1
    
    estimatorId = 1;
    a(estimatorId) = 1;    
    
    %tE1 = tEstimators(estimatorId);
    
    dt = tOdom - lastTimeOdometry;
    lastTimeOdometry = tOdom;
    
    %smallSigmas(estimatorId) = smallSigmas(estimatorId) + tE1*deltaSmallSigmas(estimatorId);
    smallSigmas(estimatorId) = smallSigmas(estimatorId) + dt*deltaSmallSigmas(estimatorId);
             
    mus(estimatorId,:) = mus(estimatorId,:) + dX;
           
    noiseE1 = abs(dX).*eye(ns); % noise added from motion
    siODOMc = bigSigmas(1:ns,ns*(estimatorId-1)+1:ns*estimatorId);              
    bigSigmas(1:ns,ns*(estimatorId-1)+1:ns*estimatorId) = siODOMc + noiseE1;
        
    tEstimators(estimatorId) = tEstimators(estimatorId)+ dt;
    
    %% estimator #2 is left inactive for efficiency
        
    
    %## draw the ellipse 
    %elli = cov2ellipse (siODOMc(1:2,1:2));
    %elli(1:2) = muODOM(1:2);
    %## plot the ellipse
    %drawEllipse(elli, 'g', "LineWidth", 1);      
    %axis("equal");
    
    
endfunction

function filterReadUsbl(mesurement)

    global H2 P2 R2 x1 x2 ns2 allUsbl a mus;   
    muUSBL = mesurement(2:3); 
  
    %diff = muUSBL - [13.177 -0.596];     
    % if (diff(1)^2 + diff(2)^2).^0.5 > 0.5
  
    % correction step filter 2
    z2 = [ muUSBL x1]'; 
    y2 = z2 - H2*x2;
    S2 = H2*P2*H2' + R2;
    K2 = P2*H2'*inv(S2);
    x2 = x2 + K2*y2;
    P2 = (eye(ns2)-K2*H2)*P2;
    
    % Estimator #2
    
    estimatorId = 2;
    a(estimatorId) = 1;
    e2Mu = x2(1:2)';
    mus(estimatorId,:) = e2Mu;
       
    %% estimator #1 DOES NOTHING (only dead reckoning)
       
    allUsbl = [allUsbl; mesurement];    
    
       
    %endif
    
    
endfunction

function filterReadDgps(mesurement)

  global allDgps;  
  
  allDgps = [allDgps; mesurement];
  
endfunction

function filterProcessing(t)
 
 global Wbff Wbf Waf Wrf a r b crb allBAR_F ne ns;
 global smallSigmas bigSigmasInit tEstimators bigSigmas mus phi1 phi2;
 
 if sum(a) > 0  
        
    % Layer R activation    
    for i = 1:ne 
    
      if (a(i) == 1) % just to optimize       
        
        leftBrotherId = crb(i);      
        [muLeft, sigmaLeft, SIGMALeft] = Psy(leftBrotherId, mus, smallSigmas, bigSigmas, tEstimators);
        r(i) = max( gamma(sigmaLeft, SIGMALeft, mus(i,:)-muLeft, phi2) - phi1, 0);
      
      endif
    endfor        
    
    % Layer F activation
    
    Gamma = Wbf*Waf*Wrf*diag(a);
    beta = Gamma*r;
    f = (inv(diag(beta))*b)'*Gamma*diag(r);        
    
    % Network fusion         
    mu = f*mus;           
    allBAR_F = [allBAR_F; [t mu]];
    
    
    
    % Reset signal
    for i = 1:ne 
    
       if r(i) > 0% 
          
        id = i;
        leftBrotherId = crb(i); 
        
        % reset the whole left neighborhood 
        while (id ~= leftBrotherId)           
          
          mus(leftBrotherId,:) = mu;          
        
          smallSigmas(leftBrotherId) = 1;
          tEstimators(leftBrotherId) = 0;
        
          bigSigmas(1:ns,ns*(leftBrotherId-1)+1:ns*leftBrotherId) = bigSigmasInit(1:ns,ns*(leftBrotherId-1)+1:ns*leftBrotherId);
          
          id = leftBrotherId;
          leftBrotherId = crb(id);
          
        endwhile           
        
      endif 
      
    endfor      
    
    a = zeros(ne,1);
    r = zeros(ne,1);
        
  endif
  
endfunction
    
    
function filterPlotData()

  global allDgps allBAR_F allUsbl;

  figure(1);
  hold on;
  title("BAR-F Neural Network");    
  plot (allDgps(:,2),  allDgps(:,3), 'k',"LineWidth", 1);
  plot (allBAR_F(:,2),  allBAR_F(:,3), 'm',"LineWidth", 1);
  plot (allUsbl(:,2), allUsbl(:,3), '.b',"LineWidth", 1);
  axis("equal", "tight");
  print("-deps", "-color", "output/plots/filterBAR_F.eps");

endfunction

function filterRecordData()

  global allBAR_F;

  save 'output/mat/filterBAR_F.mat' allBAR_F;

endfunction


