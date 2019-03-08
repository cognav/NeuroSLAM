function exp_initial(varargin)
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

    %% define some variables of 3d em

    % The number of times to run the experience map correction per frame
    global EXP_LOOPS;

    % The amount to correct each experience on either side of a link ( >0.5 is unstable)
    global EXP_CORRECTION; 

    % The experience delta threshold 
    % The threshold change in pose cell activity to generate a new exp given the same view template
    global DELTA_EXP_GC_HDC_THRESHOLD;

    % Process the parameters
    for i=1:(nargin)
        if ischar(varargin{i})
            switch varargin{i}
                case 'DELTA_EXP_GC_HDC_THRESHOLD', DELTA_EXP_GC_HDC_THRESHOLD = varargin{i+1};
                case 'EXP_LOOPS', EXP_LOOPS = varargin{i+1};
                case 'EXP_CORRECTION', EXP_CORRECTION = varargin{i+1};
             end
        end
    end
       
    % All experiences
    global EXPERIENCES; 
    
    % integrate the delta x, y, z, yaw
    % Rad radian  accumulative
    global ACCUM_DELTA_X; 
    global ACCUM_DELTA_Y; 
    global ACCUM_DELTA_Z;  
    global ACCUM_DELTA_YAW; 
%     global ACCUM_DELTA_HEIGHT;  

    ACCUM_DELTA_X = 0;
    ACCUM_DELTA_Y = 0;
    ACCUM_DELTA_Z = 0;
    ACCUM_DELTA_YAW = 0;
%     ACCUM_DELTA_HEIGHT = 0;

    % The namber of total experiences
    global NUM_EXPS;

    % The current experience ID
    global CUR_EXP_ID;

    % Experience history
    global EXP_HISTORY;
    
    global MIN_DELTA_EM;
    MIN_DELTA_EM(1) = 0;

    %% HDC
    % set the initial position in the hdcell network
    [curYawTheta, curHeightValue] = get_hdc_initial_value();
    
    %% GC
    % set the initial position in the grid cell network
    [gcX, gcY, gcZ] = get_gc_initial_pos();
    
    % Initial the experience
    EXP_HISTORY = 1;
    NUM_EXPS = 1;
    CUR_EXP_ID = 1;
    
    EXPERIENCES(CUR_EXP_ID).x_gc = gcX;
    EXPERIENCES(CUR_EXP_ID).y_gc = gcY;
    EXPERIENCES(CUR_EXP_ID).z_gc = gcZ;

    EXPERIENCES(CUR_EXP_ID).yaw_hdc = curYawTheta;
    EXPERIENCES(CUR_EXP_ID).height_hdc = curHeightValue;

    EXPERIENCES(CUR_EXP_ID).vt_id = 1;
    
    EXPERIENCES(CUR_EXP_ID).x_exp = 0;
    EXPERIENCES(CUR_EXP_ID).y_exp = 0;
    EXPERIENCES(CUR_EXP_ID).z_exp = 0;
    
    EXPERIENCES(CUR_EXP_ID).yaw_exp_rad = 0;
%     EXPERIENCES(CUR_EXP_ID).height_exp = 0;
 
    EXPERIENCES(CUR_EXP_ID).numlinks = 0;
    EXPERIENCES(CUR_EXP_ID).links = [];
 
end
