%{
 * Copyright (C) 2013-2025, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/) 
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may 
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 * 
 * AUTHOR: Bruce JK Huang (bjhuang[at]umich.edu)
 * WEBSITE: https://www.brucerobot.com/
 %}


theta_o = 20;
phi_o = 10;
rho_o = 5;

theta = linspace(-90+theta_o, 90+theta_o);
phi = linspace(-90+phi_o, 90+phi_o);
rho = inline('rho_o*(tand(theta) + tand(phi))', 'rho_o', 'theta', 'phi');


%%
theta_o = 20;
phi_o = 10;
rho_o = 5;
theta = linspace(-90+theta_o, 90+theta_o);
phi = linspace(-90+phi_o, 90+phi_o);
mygrid = @(rho_o, theta, phi) rho_o*(tand(theta) + tand(phi));

[x,y] = mygrid(pi,2*pi);


%%
theta_o = 20;
phi_o = 10;
rho_o = 100;
theta = linspace(0, 360, 1e5);
phi = linspace(-90+phi_o, 90+phi_o, 1e5);

R = test(rho_o, theta, phi);
[X,Y,Z]=sph2cart(deg2rad(theta), deg2rad(phi), R);
% Z=R*sin(Phi);
% X=R*cos(Phi).*cos(Theta);
% Y=R*cos(Phi).*sin(Theta);
scatter3(0,0,0, 'o')
hold on 
scatter3(X,Y,Z, '*');
axis equal
hold off

function rho = test(rho_o, theta, phi)
    rho = rho_o*(tand(theta) + tand(phi));
end
