function [weight] = create_gc_weights(xDim, yDim, zDim, xVar, yVar, zVar)
%     NeuroSLAM System Copyright (C) 2018-2019 
%     NeuroSLAM: A Brain inspired SLAM System for 3D Environments
%
%     Fangwen Yu (www.yufangwen.com), Jianga Shang, Youjian Hu, Michael Milford(www.michaelmilford.com) 
%
%     The NeuroSLAM V1.0 (MATLAB) was developed based on the OpenRatSLAM (David et al. 2013). 
%     The RatSLAM V0.3 (MATLAB) developed by David Ball, Michael Milford and Gordon Wyeth in 2008.
% 
%     Reference:
%     Ball, David, Scott Heath, Janet Wiles, Gordon Wyeth, Peter Corke, and Michael Milford.
%     "OpenRatSLAM: an open source brain-based SLAM system." Autonomous Robots 34, no. 3 (2013): 149-176.
% 
%     This program is free software: you can redistribute it and/or modify
%     it under the terms of the GNU General Public License as published by
%     the Free Software Foundation, either version 3 of the License, or
%     (at your option) any later version.
% 
%     This program is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU General Public License for more details.
% 
%     You should have received a copy of the GNU General Public License
%     along with this program.  If not, see <http://www.gnu.org/licenses/>.


    % Creates a 3D normalised distributio of size dimension^3 with a variance of var.

    xDimCentre = floor(xDim / 2) + 1;
    yDimCentre = floor(yDim / 2) + 1;
    zDimCentre = floor(zDim / 2) + 1;

    weight = zeros(xDim, yDim, zDim);
    
    for z = 1 : zDim  
        for x = 1 : xDim
            for y = 1 : yDim
               weight(x,y,z) = 1/(xVar*sqrt(2*pi))*exp((-(x - xDimCentre) ^ 2) / (2 * xVar ^ 2)) ...
                   * 1/(yVar*sqrt(2*pi))*exp((-(y - yDimCentre) ^ 2) / (2 * yVar ^ 2)) ...
                   * 1/(zVar*sqrt(2*pi))*exp((-(z - zDimCentre) ^ 2) / (2 * zVar ^ 2)); 
            end
        end
    end

    % ensure that it is normalised
    total = sum(sum(sum(weight)));
    weight = weight./total;       

end