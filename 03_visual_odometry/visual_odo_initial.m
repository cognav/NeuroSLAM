function visual_odo_initial(varargin)
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

    %% Initial some global variables

    % definition the Y (vertical) range of images for odomentry, including image for
    % translational velocity, image for rotational velocity, and image for
    % picth velocity

    global ODO_IMG_TRANS_Y_RANGE;
    global ODO_IMG_TRANS_X_RANGE;
        
    global ODO_IMG_HEIGHT_V_Y_RANGE;
    global ODO_IMG_HEIGHT_V_X_RANGE;  
    
    global ODO_IMG_YAW_ROT_Y_RANGE;     
    global ODO_IMG_YAW_ROT_X_RANGE;
   
    % define the size of resized images for odo
    global ODO_IMG_TRANS_RESIZE_RANGE;
    global ODO_IMG_YAW_ROT_RESIZE_RANGE;
    global ODO_IMG_HEIGHT_V_RESIZE_RANGE;
   
    % define the scale of translational velocity, rotational velocity, and pitch velocity 
    global ODO_TRANS_V_SCALE;
    
    global ODO_YAW_ROT_V_SCALE;
    global ODO_HEIGHT_V_SCALE;

    % difine the maximum threshold of translational velocity, rotational velocity and pitch velocity
    global MAX_TRANS_V_THRESHOLD;
    global MAX_YAW_ROT_V_THRESHOLD;
    global MAX_HEIGHT_V_THRESHOLD;
    
    % define the veriable for visual odometry shift match in vertical and
    % horizontal 
    global ODO_SHIFT_MATCH_VERT;
    global ODO_SHIFT_MATCH_HORI;

    % define the degree of the field of view in horizontal and vertical. 
    % Field of View (FOV), Degree (DEG) the horizontal, vertical, and diagonal
    % degrees for all FOVs
    global FOV_HORI_DEGREE;
    global FOV_VERT_DEGREE;
     
    global KEY_POINT_SET;
    global ODO_STEP;

    %%% Process the parameters

    for i=1:(nargin-1)
        if ischar(varargin{i})
            switch varargin{i}
                case 'ODO_IMG_TRANS_Y_RANGE', ODO_IMG_TRANS_Y_RANGE = varargin{i+1};
                case 'ODO_IMG_TRANS_X_RANGE', ODO_IMG_TRANS_X_RANGE = varargin{i+1};              
                case 'ODO_IMG_HEIGHT_V_Y_RANGE', ODO_IMG_HEIGHT_V_Y_RANGE = varargin{i+1}; 
                case 'ODO_IMG_HEIGHT_V_X_RANGE', ODO_IMG_HEIGHT_V_X_RANGE = varargin{i+1};     
                case 'ODO_IMG_YAW_ROT_Y_RANGE', ODO_IMG_YAW_ROT_Y_RANGE = varargin{i+1};
                case 'ODO_IMG_YAW_ROT_X_RANGE', ODO_IMG_YAW_ROT_X_RANGE = varargin{i+1};
                
                case 'ODO_IMG_TRANS_RESIZE_RANGE', ODO_IMG_TRANS_RESIZE_RANGE = varargin{i+1};
                case 'ODO_IMG_YAW_ROT_RESIZE_RANGE', ODO_IMG_YAW_ROT_RESIZE_RANGE = varargin{i+1};
                case 'ODO_IMG_HEIGHT_V_RESIZE_RANGE', ODO_IMG_HEIGHT_V_RESIZE_RANGE = varargin{i+1};
                      
                case 'ODO_TRANS_V_SCALE', ODO_TRANS_V_SCALE = varargin{i+1};
                case 'ODO_YAW_ROT_V_SCALE', ODO_YAW_ROT_V_SCALE = varargin{i+1};
                case 'ODO_HEIGHT_V_SCALE', ODO_HEIGHT_V_SCALE = varargin{i+1};
                    
                case 'MAX_TRANS_V_THRESHOLD', MAX_TRANS_V_THRESHOLD = varargin{i+1};         
                case 'MAX_YAW_ROT_V_THRESHOLD', MAX_YAW_ROT_V_THRESHOLD = varargin{i+1};
                case 'MAX_HEIGHT_V_THRESHOLD', MAX_HEIGHT_V_THRESHOLD = varargin{i+1};
                 
                case 'ODO_SHIFT_MATCH_HORI', ODO_SHIFT_MATCH_HORI = varargin{i+1};
                case 'ODO_SHIFT_MATCH_VERT', ODO_SHIFT_MATCH_VERT = varargin{i+1};
 
                case 'FOV_HORI_DEGREE', FOV_HORI_DEGREE = varargin{i+1};
                case 'FOV_VERT_DEGREE', FOV_VERT_DEGREE = varargin{i+1};
               
                case 'KEY_POINT_SET', KEY_POINT_SET = varargin{i+1};
                case 'ODO_STEP', ODO_STEP = varargin{i+1};
                    
            end
        end
    end
    
    % x_sums, sum each column of intensity in the current image and previous
    % image perspectively.
    % form one dimensional vector, 1*N array

    global PREV_TRANS_V_IMG_X_SUMS;
    global PREV_YAW_ROT_V_IMG_X_SUMS;
    global PREV_HEIGHT_V_IMG_Y_SUMS;
    
    PREV_YAW_ROT_V_IMG_X_SUMS = zeros(1,ODO_IMG_TRANS_RESIZE_RANGE(2));
    PREV_HEIGHT_V_IMG_Y_SUMS = zeros(ODO_IMG_HEIGHT_V_RESIZE_RANGE(1), 1);
    PREV_TRANS_V_IMG_X_SUMS = zeros(1,ODO_IMG_TRANS_RESIZE_RANGE(2) - ODO_SHIFT_MATCH_HORI);
    
    % define the previous velocity for keeping stable speed
    global PREV_TRANS_V;
    global PREV_YAW_ROT_V;
    global PREV_HEIGHT_V;
    PREV_TRANS_V = 0.025;    % 0.03
    PREV_YAW_ROT_V = 0;
    PREV_HEIGHT_V = 0;
    
    %%% End up for setting up the visual odometry 
   
end