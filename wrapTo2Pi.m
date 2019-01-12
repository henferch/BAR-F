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
#   Description: Thus function wraps an angle between 0 and 2pi
#
###########################################################################################


function xwrap = wrapTo2Pi(x)
  xwrap = rem (x-pi, 2*pi);
  idx = find (abs (xwrap) > pi);
  xwrap(idx) -= 2*pi * sign (xwrap(idx));
  xwrap += pi;
endfunction
