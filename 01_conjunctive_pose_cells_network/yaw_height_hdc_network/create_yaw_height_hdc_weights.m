function [weight] = create_yaw_height_hdc_weights(yawDim, heightDim, yawVar, heightVar)
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
    
    % Creates a 2D normalised distributio of size dim^2 with a 
    % variance of var.

    yawDimCentre = floor(yawDim / 2) + 1;
    heightDimCentre = floor(heightDim / 2) + 1;

    weight = zeros(yawDim, heightDim);

    for h = 1 : heightDim
        for y = 1 : yawDim
            weight(y, h) =  1/(yawVar*sqrt(2*pi))*exp((-(y - yawDimCentre) ^ 2 ) / (2 * yawVar ^ 2)) ...
               * 1/(heightVar*sqrt(2*pi))* exp((-(h - heightDimCentre) ^ 2) / (2 * heightVar ^ 2)); 
        end
    end

    % ensure that it is normalised
    total = sum(sum(sum(weight)));
    weight = weight./total;            

end

