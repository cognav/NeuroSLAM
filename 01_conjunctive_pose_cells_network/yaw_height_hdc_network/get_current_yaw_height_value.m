function [outYawTheta, outHeightValue] = get_current_yaw_height_value()
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
    
    % Returns the approximate averaged centre of the most active activity
    % packet. This implementation averages the cells around the maximally
    % activated cell.
    % Pupulation Vector Decoding http://blog.yufangwen.com/?p=878

    % The HD cells of yaw and height conjunctively
    global YAW_HEIGHT_HDC;
    
    % The dimension of yaw in yaw_height_hdc network
    global YAW_HEIGHT_HDC_Y_DIM;
    
    % The dimension of height in yaw_height_hdc network
    global YAW_HEIGHT_HDC_H_DIM;

    % The wrap for finding maximum activity packet 
    global YAW_HEIGHT_HDC_MAX_Y_WRAP;
    global YAW_HEIGHT_HDC_MAX_H_WRAP;
    
    % for finding maximum activity packet
    global YAW_HEIGHT_HDC_Y_SUM_SIN_LOOKUP;
    global YAW_HEIGHT_HDC_Y_SUM_COS_LOOKUP;
    global YAW_HEIGHT_HDC_H_SUM_SIN_LOOKUP;
    global YAW_HEIGHT_HDC_H_SUM_COS_LOOKUP;
    
    % packet size for wrap, the left and right activity cells near
    % center of best activity packet, eg. = 5
    global YAW_HEIGHT_HDC_PACKET_SIZE;
    
    % The yaw theta size of each unit in radian, 2*pi/ YAW_HEIGHT_HDC_H_DIM
    % radian e.g. 2*pi/360 = 0.0175
    global YAW_HEIGHT_HDC_Y_TH_SIZE;                      
    global YAW_HEIGHT_HDC_H_SIZE;     

    % find the max activated cell
    indexes = find(YAW_HEIGHT_HDC);
    [value, index] = max(YAW_HEIGHT_HDC(indexes));
    [y, h] = ind2sub(size(YAW_HEIGHT_HDC), indexes(index));

    % take the max activated cell +- AVG_CELL in 2d space
    tempYawHeightHdc = zeros(YAW_HEIGHT_HDC_Y_DIM, YAW_HEIGHT_HDC_H_DIM);

    tempYawHeightHdc(YAW_HEIGHT_HDC_MAX_Y_WRAP(y : y + YAW_HEIGHT_HDC_PACKET_SIZE * 2), YAW_HEIGHT_HDC_MAX_H_WRAP(h : h + YAW_HEIGHT_HDC_PACKET_SIZE * 2)) = ...
        YAW_HEIGHT_HDC(YAW_HEIGHT_HDC_MAX_Y_WRAP(y : y + YAW_HEIGHT_HDC_PACKET_SIZE * 2), YAW_HEIGHT_HDC_MAX_H_WRAP(h : h + YAW_HEIGHT_HDC_PACKET_SIZE * 2));

    yawSumSin = sum(YAW_HEIGHT_HDC_Y_SUM_SIN_LOOKUP * sum(tempYawHeightHdc,2));
    yawSumCos = sum(YAW_HEIGHT_HDC_Y_SUM_COS_LOOKUP * sum(tempYawHeightHdc,2));

    heightSumSin = sum(YAW_HEIGHT_HDC_H_SUM_SIN_LOOKUP * (sum(tempYawHeightHdc,1))');
    heightSumCos = sum(YAW_HEIGHT_HDC_H_SUM_COS_LOOKUP * (sum(tempYawHeightHdc,1))');

   
    outYawTheta = mod(atan2(yawSumSin, yawSumCos) / YAW_HEIGHT_HDC_Y_TH_SIZE, YAW_HEIGHT_HDC_Y_DIM);
    outHeightValue = mod(atan2(heightSumSin, heightSumCos) / YAW_HEIGHT_HDC_H_SIZE, YAW_HEIGHT_HDC_H_DIM);
    
end
