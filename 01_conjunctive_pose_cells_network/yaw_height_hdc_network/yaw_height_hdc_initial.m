function yaw_height_hdc_initial(varargin)
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

    % The HD cells of yaw and height conjunctively
    global YAW_HEIGHT_HDC;
       
    % The dimension of yaw in yaw_height_hdc network
    global YAW_HEIGHT_HDC_Y_DIM;
    
    % The dimension of height in yaw_height_hdc network
    global YAW_HEIGHT_HDC_H_DIM;
    
    % The dimension of local excitation weight matrix for yaw
    global YAW_HEIGHT_HDC_EXCIT_Y_DIM;
    
    % The dimension of local excitation weight matrix for height
    global YAW_HEIGHT_HDC_EXCIT_H_DIM;
    
    % The dimension of local inhibition weight matrix for yaw
    global YAW_HEIGHT_HDC_INHIB_Y_DIM;
    
    % The dimension of local inhibition weight matrix for height
    global YAW_HEIGHT_HDC_INHIB_H_DIM;
    
    % The global inhibition value
    global YAW_HEIGHT_HDC_GLOBAL_INHIB;
    
    
    % The amount of energy injected when a view template is re-seen
    global YAW_HEIGHT_HDC_VT_INJECT_ENERGY; 
    
    % Variance of Excitation and Inhibition in XY and THETA respectively
    global YAW_HEIGHT_HDC_EXCIT_Y_VAR;
    global YAW_HEIGHT_HDC_EXCIT_H_VAR;
    global YAW_HEIGHT_HDC_INHIB_Y_VAR;
    global YAW_HEIGHT_HDC_INHIB_H_VAR;
      
    % The scale of rotation velocity of yaw
    global YAW_ROT_V_SCALE;
    
    % The scale of rotation velocity of height
    global HEIGHT_V_SCALE;
    
    % packet size for wrap, the left and right activity cells near
    % center of best activity packet, eg. = 5
    global YAW_HEIGHT_HDC_PACKET_SIZE;

    % Process the parameters
    for i=1:(nargin-1)
        if ischar(varargin{i})
            switch varargin{i}
                case 'YAW_HEIGHT_HDC_Y_DIM', YAW_HEIGHT_HDC_Y_DIM = varargin{i+1};
                case 'YAW_HEIGHT_HDC_H_DIM', YAW_HEIGHT_HDC_H_DIM = varargin{i+1};
                
                case 'YAW_HEIGHT_HDC_EXCIT_Y_DIM', YAW_HEIGHT_HDC_EXCIT_Y_DIM = varargin{i+1};
                case 'YAW_HEIGHT_HDC_EXCIT_H_DIM', YAW_HEIGHT_HDC_EXCIT_H_DIM = varargin{i+1};
                
                case 'YAW_HEIGHT_HDC_INHIB_Y_DIM', YAW_HEIGHT_HDC_INHIB_Y_DIM = varargin{i+1};
                case 'YAW_HEIGHT_HDC_INHIB_H_DIM', YAW_HEIGHT_HDC_INHIB_H_DIM = varargin{i+1};
                    
                case 'YAW_HEIGHT_HDC_EXCIT_Y_VAR', YAW_HEIGHT_HDC_EXCIT_Y_VAR = varargin{i+1};
                case 'YAW_HEIGHT_HDC_EXCIT_H_VAR', YAW_HEIGHT_HDC_EXCIT_H_VAR = varargin{i+1};
                    
                case 'YAW_HEIGHT_HDC_INHIB_Y_VAR', YAW_HEIGHT_HDC_INHIB_Y_VAR = varargin{i+1};
                case 'YAW_HEIGHT_HDC_INHIB_H_VAR', YAW_HEIGHT_HDC_INHIB_H_VAR = varargin{i+1};
                    
                case 'YAW_HEIGHT_HDC_GLOBAL_INHIB', YAW_HEIGHT_HDC_GLOBAL_INHIB = varargin{i+1};
                                                    
                case 'YAW_HEIGHT_HDC_VT_INJECT_ENERGY', YAW_HEIGHT_HDC_VT_INJECT_ENERGY = varargin{i+1};
        
                case 'YAW_ROT_V_SCALE', YAW_ROT_V_SCALE = varargin{i+1};
                case 'HEIGHT_V_SCALE', HEIGHT_V_SCALE = varargin{i+1};
                    
                case 'YAW_HEIGHT_HDC_PACKET_SIZE', YAW_HEIGHT_HDC_PACKET_SIZE = varargin{i+1};
   
             end
        end
    end
   
    % The weight of excitation in yaw_height_hdc network
    global YAW_HEIGHT_HDC_EXCIT_WEIGHT;
    YAW_HEIGHT_HDC_EXCIT_WEIGHT = create_yaw_height_hdc_weights(YAW_HEIGHT_HDC_EXCIT_Y_DIM, ...
        YAW_HEIGHT_HDC_EXCIT_H_DIM, YAW_HEIGHT_HDC_EXCIT_Y_VAR, YAW_HEIGHT_HDC_EXCIT_H_VAR);
    
    % The weight of inhibition in yaw_height_hdc network
    global YAW_HEIGHT_HDC_INHIB_WEIGHT;
    YAW_HEIGHT_HDC_INHIB_WEIGHT = create_yaw_height_hdc_weights(YAW_HEIGHT_HDC_INHIB_Y_DIM, ...
        YAW_HEIGHT_HDC_INHIB_H_DIM, YAW_HEIGHT_HDC_INHIB_Y_VAR, YAW_HEIGHT_HDC_INHIB_Y_VAR);

    % convienience constants
    % The half dimension of local excitation weight matrix for yaw
    global YAW_HEIGHT_HDC_EXCIT_Y_DIM_HALF;
    
    % The half dimension of local excitation weight matrix for height
    global YAW_HEIGHT_HDC_EXCIT_H_DIM_HALF;
    
    % The half dimension of local inhibition weight matrix for yaw
    global YAW_HEIGHT_HDC_INHIB_Y_DIM_HALF;
    
    % The half dimension of local inhibition weight matrix for height
    global YAW_HEIGHT_HDC_INHIB_H_DIM_HALF;
    
    YAW_HEIGHT_HDC_EXCIT_Y_DIM_HALF = floor(YAW_HEIGHT_HDC_EXCIT_Y_DIM / 2);
    YAW_HEIGHT_HDC_EXCIT_H_DIM_HALF = floor(YAW_HEIGHT_HDC_EXCIT_H_DIM / 2);
    YAW_HEIGHT_HDC_INHIB_Y_DIM_HALF = floor(YAW_HEIGHT_HDC_INHIB_Y_DIM / 2);
    YAW_HEIGHT_HDC_INHIB_H_DIM_HALF = floor(YAW_HEIGHT_HDC_INHIB_H_DIM / 2);
    
    % The yaw theta size of each unit in radian, 2*pi/ YAW_HEIGHT_HDC_Y_DIM
    % radian e.g. 2*pi/360 = 0.0175
    global YAW_HEIGHT_HDC_Y_TH_SIZE;                      
    YAW_HEIGHT_HDC_Y_TH_SIZE = (2 * pi) / YAW_HEIGHT_HDC_Y_DIM;
    
    % The yaw theta size of each unit in radian, 2*pi/ YAW_HEIGHT_HDC_H_DIM
    % radian e.g. 2*pi/36 = 0.1745
    global YAW_HEIGHT_HDC_H_SIZE;     
    YAW_HEIGHT_HDC_H_SIZE = (2 * pi) / YAW_HEIGHT_HDC_H_DIM;
    
    % these are the lookups for finding the centre of the hdcell in YAW_HEIGHT_HDC by
    % get_hdcell_theta()

    global YAW_HEIGHT_HDC_Y_SUM_SIN_LOOKUP;
    global YAW_HEIGHT_HDC_Y_SUM_COS_LOOKUP;
    global YAW_HEIGHT_HDC_H_SUM_SIN_LOOKUP;
    global YAW_HEIGHT_HDC_H_SUM_COS_LOOKUP;

    YAW_HEIGHT_HDC_Y_SUM_SIN_LOOKUP = sin((1 : YAW_HEIGHT_HDC_Y_DIM) .* YAW_HEIGHT_HDC_Y_TH_SIZE);
    YAW_HEIGHT_HDC_Y_SUM_COS_LOOKUP = cos((1 : YAW_HEIGHT_HDC_Y_DIM) .* YAW_HEIGHT_HDC_Y_TH_SIZE);
 
    YAW_HEIGHT_HDC_H_SUM_SIN_LOOKUP = sin((1 : YAW_HEIGHT_HDC_H_DIM) .* YAW_HEIGHT_HDC_H_SIZE);
    YAW_HEIGHT_HDC_H_SUM_COS_LOOKUP = cos((1 : YAW_HEIGHT_HDC_H_DIM) .* YAW_HEIGHT_HDC_H_SIZE);


    % The excit wrap of yaw in yaw_height_hdc network
    global YAW_HEIGHT_HDC_EXCIT_Y_WRAP;
    YAW_HEIGHT_HDC_EXCIT_Y_WRAP = [(YAW_HEIGHT_HDC_Y_DIM - YAW_HEIGHT_HDC_EXCIT_Y_DIM_HALF + 1) ...
        : YAW_HEIGHT_HDC_Y_DIM  1 : YAW_HEIGHT_HDC_Y_DIM  1 : YAW_HEIGHT_HDC_EXCIT_Y_DIM_HALF];

    % The excit wrap of height in yaw_height_hdc network
    global YAW_HEIGHT_HDC_EXCIT_H_WRAP;
    YAW_HEIGHT_HDC_EXCIT_H_WRAP = [(YAW_HEIGHT_HDC_H_DIM - YAW_HEIGHT_HDC_EXCIT_H_DIM_HALF + 1) ...
        : YAW_HEIGHT_HDC_H_DIM  1 : YAW_HEIGHT_HDC_H_DIM  1 : YAW_HEIGHT_HDC_EXCIT_H_DIM_HALF];

    % The inhibit wrap of yaw in yaw_height_hdc network
    global YAW_HEIGHT_HDC_INHIB_Y_WRAP;
    YAW_HEIGHT_HDC_INHIB_Y_WRAP = [(YAW_HEIGHT_HDC_Y_DIM - YAW_HEIGHT_HDC_INHIB_Y_DIM_HALF + 1) ...
        : YAW_HEIGHT_HDC_Y_DIM  1 : YAW_HEIGHT_HDC_Y_DIM  1 : YAW_HEIGHT_HDC_INHIB_Y_DIM_HALF];

    % The inhibit wrap of height in yaw_height_hdc network
    global YAW_HEIGHT_HDC_INHIB_H_WRAP;
    YAW_HEIGHT_HDC_INHIB_H_WRAP = [(YAW_HEIGHT_HDC_H_DIM - YAW_HEIGHT_HDC_INHIB_H_DIM_HALF + 1) ...
        : YAW_HEIGHT_HDC_H_DIM  1 : YAW_HEIGHT_HDC_H_DIM  1 : YAW_HEIGHT_HDC_INHIB_H_DIM_HALF];

    % The wrap for finding maximum activity packet 
    global YAW_HEIGHT_HDC_MAX_Y_WRAP;
    global YAW_HEIGHT_HDC_MAX_H_WRAP;

    YAW_HEIGHT_HDC_MAX_Y_WRAP = [(YAW_HEIGHT_HDC_Y_DIM - YAW_HEIGHT_HDC_PACKET_SIZE + 1) ...
        : YAW_HEIGHT_HDC_Y_DIM  1 : YAW_HEIGHT_HDC_Y_DIM  1 : YAW_HEIGHT_HDC_PACKET_SIZE];
    YAW_HEIGHT_HDC_MAX_H_WRAP = [(YAW_HEIGHT_HDC_H_DIM - YAW_HEIGHT_HDC_PACKET_SIZE + 1) ...
        : YAW_HEIGHT_HDC_H_DIM  1 : YAW_HEIGHT_HDC_H_DIM  1 : YAW_HEIGHT_HDC_PACKET_SIZE];
    
    % set the initial position in the hdcell network
    [curYawTheta, curHeight] = get_hdc_initial_value();
    
    YAW_HEIGHT_HDC = zeros(YAW_HEIGHT_HDC_Y_DIM, YAW_HEIGHT_HDC_H_DIM);
    YAW_HEIGHT_HDC(curYawTheta, curHeight) = 1;

    global MAX_ACTIVE_YAW_HEIGHT_HIS_PATH;
    MAX_ACTIVE_YAW_HEIGHT_HIS_PATH = [curYawTheta curHeight];
    
end
