function [gcX, gcY, gcZ] = get_gc_xyz()
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

    % The 3D Grid Cells Network
    global GRIDCELLS;
    
    % The x, y, z dimension of 3D Grid Cells Model (3D CAN) 
    global GC_X_DIM;
    global GC_Y_DIM;
    global GC_Z_DIM;


    % The x, y, z theta size of each unit in radian, 2*pi/ x_dim
    % radian e.g. 2*pi/36 = 0.175
    global GC_X_TH_SIZE;   
    global GC_Y_TH_SIZE;   
    global GC_Z_TH_SIZE;   
    

    % these are the lookups for finding the centre of the gccell in GRIDCELLS by
    % get_gc_xyz()

    global GC_X_SUM_SIN_LOOKUP;
    global GC_X_SUM_COS_LOOKUP;
    global GC_Y_SUM_SIN_LOOKUP;
    global GC_Y_SUM_COS_LOOKUP;
    global GC_Z_SUM_SIN_LOOKUP;
    global GC_Z_SUM_COS_LOOKUP;

    % packet size for wrap, the left and right activity cells near
    % center of best activity packet, eg. = 5
    global GC_PACKET_SIZE;

    % The wrap for finding maximum activity packet 
    global GC_MAX_X_WRAP;
    global GC_MAX_Y_WRAP;
    global GC_MAX_Z_WRAP;


    % find the max activated cell
    indexes = find(GRIDCELLS);
    [value, index] = max(GRIDCELLS(indexes));
    [x y z] = ind2sub(size(GRIDCELLS), indexes(index));

    % take the max activated cell +- AVG_CELL in 3d space
    tempGridcells = zeros(GC_X_DIM, GC_X_DIM, GC_Z_DIM);

    tempGridcells(GC_MAX_X_WRAP(x : x + GC_PACKET_SIZE * 2), ...
        GC_MAX_Y_WRAP(y : y + GC_PACKET_SIZE * 2), ...
        GC_MAX_Z_WRAP(z : z + GC_PACKET_SIZE * 2)) = ...
        GRIDCELLS(GC_MAX_X_WRAP(x : x + GC_PACKET_SIZE * 2), ...
        GC_MAX_Y_WRAP(y : y + GC_PACKET_SIZE * 2), ...
        GC_MAX_Z_WRAP(z : z + GC_PACKET_SIZE * 2));

    xSumSin = sum(GC_X_SUM_SIN_LOOKUP * sum(sum(tempGridcells,2),3));
    xSumCos = sum(GC_X_SUM_COS_LOOKUP * sum(sum(tempGridcells,2),3));

    ySumSin = sum(GC_Y_SUM_SIN_LOOKUP * sum(sum(tempGridcells,1),3)');
    ySumCos = sum(GC_Y_SUM_COS_LOOKUP * sum(sum(tempGridcells,1),3)');

    tempZSum = sum(sum(tempGridcells,1),2);

    for i = 1: GC_Z_DIM
        zSum(i)= tempZSum(1,1,i);
    end

    zSumSin = sum(GC_Z_SUM_SIN_LOOKUP * zSum');
    zSumCos = sum(GC_Z_SUM_COS_LOOKUP * zSum');

    gcX = mod(atan2(xSumSin, xSumCos) / GC_X_TH_SIZE, GC_X_DIM);
    gcY = mod(atan2(ySumSin, ySumCos) / GC_Y_TH_SIZE, GC_Y_DIM);
    gcZ = mod(atan2(zSumSin, zSumCos) / GC_Z_TH_SIZE, GC_Z_DIM);

end