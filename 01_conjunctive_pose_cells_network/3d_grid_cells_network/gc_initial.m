function gc_initial(varargin)
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

    %% define some variables of 3d gc
    % The 3D Grid Cells Network
    global GRIDCELLS;
    
    % The x, y, z dimension of 3D Grid Cells Model (3D CAN) 
    global GC_X_DIM;
    global GC_Y_DIM;
    global GC_Z_DIM;
    
    % The dimension of local excitation weight matrix for x, y, z
    global GC_EXCIT_X_DIM;
    global GC_EXCIT_Y_DIM;
    global GC_EXCIT_Z_DIM;
    
    % The dimension of local excitation weight matrix for x, y, z
    global GC_INHIB_X_DIM;
    global GC_INHIB_Y_DIM;
    global GC_INHIB_Z_DIM;
    
    % The global inhibition value
    global GC_GLOBAL_INHIB;   
    
    % The amount of energy injected when a view template is re-seen
    global GC_VT_INJECT_ENERGY;

    
    % Variance of Excitation and Inhibition in XY and THETA respectively
    global GC_EXCIT_X_VAR;
    global GC_EXCIT_Y_VAR;
    global GC_EXCIT_Z_VAR;
    
    global GC_INHIB_X_VAR;
    global GC_INHIB_Y_VAR;
    global GC_INHIB_Z_VAR;

      
    % The scale of horizontal translational velocity
    global GC_HORI_TRANS_V_SCALE;
    
    % The scale of vertical translational velocity
    global GC_VERT_TRANS_V_SCALE;
    
    % packet size for wrap, the left and right activity cells near
    % center of best activity packet, eg. = 5
    global GC_PACKET_SIZE;

    % Process the parameters
    for i=1:(nargin-1)
        if ischar(varargin{i})
            switch varargin{i}
                case 'GC_X_DIM', GC_X_DIM = varargin{i+1};
                case 'GC_Y_DIM', GC_Y_DIM = varargin{i+1};
                case 'GC_Z_DIM', GC_Z_DIM = varargin{i+1};
                
                case 'GC_EXCIT_X_DIM', GC_EXCIT_X_DIM = varargin{i+1};
                case 'GC_EXCIT_Y_DIM', GC_EXCIT_Y_DIM = varargin{i+1};
                case 'GC_EXCIT_Z_DIM', GC_EXCIT_Z_DIM = varargin{i+1};
                
                case 'GC_INHIB_X_DIM', GC_INHIB_X_DIM = varargin{i+1};
                case 'GC_INHIB_Y_DIM', GC_INHIB_Y_DIM = varargin{i+1};
                case 'GC_INHIB_Z_DIM', GC_INHIB_Z_DIM = varargin{i+1};
                    
                case 'GC_EXCIT_X_VAR', GC_EXCIT_X_VAR = varargin{i+1};
                case 'GC_EXCIT_Y_VAR', GC_EXCIT_Y_VAR = varargin{i+1};
                case 'GC_EXCIT_Z_VAR', GC_EXCIT_Z_VAR = varargin{i+1};
                    
                case 'GC_INHIB_X_VAR', GC_INHIB_X_VAR = varargin{i+1};
                case 'GC_INHIB_Y_VAR', GC_INHIB_Y_VAR = varargin{i+1};
                case 'GC_INHIB_Z_VAR', GC_INHIB_Z_VAR = varargin{i+1};
                    
                case 'GC_GLOBAL_INHIB', GC_GLOBAL_INHIB = varargin{i+1};
                                    
                case 'GC_VT_INJECT_ENERGY', GC_VT_INJECT_ENERGY = varargin{i+1};
        
                case 'GC_HORI_TRANS_V_SCALE', GC_HORI_TRANS_V_SCALE = varargin{i+1};
                case 'GC_VERT_TRANS_V_SCALE', GC_VERT_TRANS_V_SCALE = varargin{i+1};
                    
                case 'GC_PACKET_SIZE', GC_PACKET_SIZE = varargin{i+1};
   
             end
        end
    end
   
    % The weight of excitation in 3D grid cell network
    global GC_EXCIT_WEIGHT;
    GC_EXCIT_WEIGHT = create_gc_weights(GC_EXCIT_X_DIM, ...
        GC_EXCIT_Y_DIM, GC_EXCIT_Z_DIM, GC_EXCIT_X_VAR, GC_EXCIT_Y_VAR, GC_EXCIT_Z_VAR);
    
    % The weight of inhibition in 3D grid cell network
    global GC_INHIB_WEIGHT;
    GC_INHIB_WEIGHT = create_gc_weights(GC_INHIB_X_DIM, ...
        GC_INHIB_Y_DIM, GC_INHIB_Z_DIM, GC_INHIB_X_VAR, GC_INHIB_X_VAR, GC_INHIB_Z_VAR);

    % convienience constants
    % The half dimension of local excitation weight matrix for x, y, z
    global GC_EXCIT_X_DIM_HALF;
    global GC_EXCIT_Y_DIM_HALF;
    global GC_EXCIT_Z_DIM_HALF;
    
    % The half dimension of local inhibition weight matrix for x, y, z
    global GC_INHIB_X_DIM_HALF;
    global GC_INHIB_Y_DIM_HALF;
    global GC_INHIB_Z_DIM_HALF;
    
    GC_EXCIT_X_DIM_HALF = floor(GC_EXCIT_X_DIM / 2);
    GC_EXCIT_Y_DIM_HALF = floor(GC_EXCIT_Y_DIM / 2);
    GC_EXCIT_Z_DIM_HALF = floor(GC_EXCIT_Z_DIM / 2);
    
    GC_INHIB_X_DIM_HALF = floor(GC_INHIB_X_DIM / 2);
    GC_INHIB_Y_DIM_HALF = floor(GC_INHIB_Y_DIM / 2);
    GC_INHIB_Z_DIM_HALF = floor(GC_INHIB_Z_DIM / 2);
    
     % The excit wrap of x,y,z in 3D grid cell network
    global GC_EXCIT_X_WRAP;
    global GC_EXCIT_Y_WRAP;
    global GC_EXCIT_Z_WRAP;

    GC_EXCIT_X_WRAP = [(GC_X_DIM - GC_EXCIT_X_DIM_HALF + 1) : GC_X_DIM  1 : GC_X_DIM  1 : GC_EXCIT_X_DIM_HALF];
    GC_EXCIT_Y_WRAP = [(GC_Y_DIM - GC_EXCIT_Y_DIM_HALF + 1) : GC_Y_DIM  1 : GC_Y_DIM  1 : GC_EXCIT_Y_DIM_HALF];
    GC_EXCIT_Z_WRAP = [(GC_Z_DIM - GC_EXCIT_Z_DIM_HALF + 1) : GC_Z_DIM  1 : GC_Z_DIM  1 : GC_EXCIT_Z_DIM_HALF];
    
    % The inhibit wrap of x,y,z in 3D grid cell network
    global GC_INHIB_X_WRAP;
    global GC_INHIB_Y_WRAP;
    global GC_INHIB_Z_WRAP;
    GC_INHIB_X_WRAP = [(GC_X_DIM - GC_INHIB_X_DIM_HALF + 1) : GC_X_DIM  1 : GC_X_DIM  1 : GC_INHIB_X_DIM_HALF];
    GC_INHIB_Y_WRAP = [(GC_Y_DIM - GC_INHIB_Y_DIM_HALF + 1) : GC_Y_DIM  1 : GC_Y_DIM  1 : GC_INHIB_Y_DIM_HALF];
    GC_INHIB_Z_WRAP = [(GC_Z_DIM - GC_INHIB_Z_DIM_HALF + 1) : GC_Z_DIM  1 : GC_Z_DIM  1 : GC_INHIB_Z_DIM_HALF];
    
        
    % The x, y, z cell size of each unit in meter or unit
    global GC_X_TH_SIZE;   
    global GC_Y_TH_SIZE;   
    global GC_Z_TH_SIZE;   
    
    GC_X_TH_SIZE = 2*pi / GC_X_DIM;
    GC_Y_TH_SIZE = 2*pi / GC_Y_DIM;
    GC_Z_TH_SIZE = 2*pi / GC_Y_DIM;

   
    % these are the lookups for finding the centre of the gccell in GRIDCELLS by
    % get_gc_xyz()

    global GC_X_SUM_SIN_LOOKUP;
    global GC_X_SUM_COS_LOOKUP;
    global GC_Y_SUM_SIN_LOOKUP;
    global GC_Y_SUM_COS_LOOKUP;
    global GC_Z_SUM_SIN_LOOKUP;
    global GC_Z_SUM_COS_LOOKUP;
    
    GC_X_SUM_SIN_LOOKUP = sin((1 : GC_X_DIM) .* GC_X_TH_SIZE);
    GC_X_SUM_COS_LOOKUP = cos((1 : GC_X_DIM) .* GC_X_TH_SIZE);
 
    GC_Y_SUM_SIN_LOOKUP = sin((1 : GC_Y_DIM) .* GC_Y_TH_SIZE);
    GC_Y_SUM_COS_LOOKUP = cos((1 : GC_Y_DIM) .* GC_Y_TH_SIZE);

    GC_Z_SUM_SIN_LOOKUP = sin((1 : GC_Z_DIM) .* GC_Z_TH_SIZE);
    GC_Z_SUM_COS_LOOKUP = cos((1 : GC_Z_DIM) .* GC_Z_TH_SIZE);
   
    % The wrap for finding maximum activity packet 
    global GC_MAX_X_WRAP;
    global GC_MAX_Y_WRAP;
    global GC_MAX_Z_WRAP;
    
    GC_MAX_X_WRAP = [(GC_X_DIM - GC_PACKET_SIZE + 1) : GC_X_DIM  1 : GC_X_DIM  1 : GC_PACKET_SIZE];
    GC_MAX_Y_WRAP = [(GC_Y_DIM - GC_PACKET_SIZE + 1) : GC_Y_DIM  1 : GC_Y_DIM  1 : GC_PACKET_SIZE];
    GC_MAX_Z_WRAP = [(GC_Y_DIM - GC_PACKET_SIZE + 1) : GC_Y_DIM  1 : GC_Y_DIM  1 : GC_PACKET_SIZE];


    % set the initial position in the grid cell network
    [gcX, gcY, gcZ] = get_gc_initial_pos();
    
    GRIDCELLS = zeros(GC_X_DIM, GC_Y_DIM, GC_Z_DIM);
    GRIDCELLS(gcX, gcY, gcZ) = 1;

    global MAX_ACTIVE_XYZ_PATH;
    MAX_ACTIVE_XYZ_PATH = [gcX gcY gcZ];
    
end

