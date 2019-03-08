function plot_3d_hdc_gtvo(visualDataFile, expMapHistoryFile, odoMapHistoryFile, groundTruthFile,vtHistoryFile, emHistoryFile, varargin)
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
    
    %% Initial some variables
    % Transform the angle from degree to radian
    global DEGREE_TO_RADIAN;
    DEGREE_TO_RADIAN = pi / 180;
   
    % Transform the angle from radian to degree 
    global RADIAN_TO_DEGREE;
    RADIAN_TO_DEGREE = 180 / pi;  
    
    % define the size of block in reading video
    global BLOCK_READ;
    
    % define the render rate for dyawing vo 
    global RENDER_RATE;
    
    global GT_ODO_X_SCALING;
    global GT_ODO_Y_SCALING;
    global GT_ODO_Z_SCALING;
    global GT_EXP_X_SCALING;
    global GT_EXP_Y_SCALING;
    global GT_EXP_Z_SCALING;
    
    global ODO_MAP_X_SCALING;
    global ODO_MAP_Y_SCALING;
    global ODO_MAP_Z_SCALING;
    global EXP_MAP_X_SCALING;
    global EXP_MAP_Y_SCALING;
    global EXP_MAP_Z_SCALING;
    
    % Process the parameters
    for i=1:(nargin - 6)
        if ischar(varargin{i})
            switch varargin{i}
                case 'BLOCK_READ', BLOCK_READ = varargin{i+1};
                case 'RENDER_RATE', RENDER_RATE = varargin{i+1};
                case 'GT_ODO_X_SCALING', GT_ODO_X_SCALING = varargin{i+1};    
                case 'GT_ODO_Y_SCALING', GT_ODO_Y_SCALING = varargin{i+1};    
                case 'GT_ODO_Z_SCALING', GT_ODO_Z_SCALING = varargin{i+1};    
                case 'GT_EXP_X_SCALING', GT_EXP_X_SCALING = varargin{i+1};    
                case 'GT_EXP_Y_SCALING', GT_EXP_Y_SCALING = varargin{i+1};    
                case 'GT_EXP_Z_SCALING', GT_EXP_Z_SCALING = varargin{i+1};   
                case 'ODO_MAP_X_SCALING', ODO_MAP_X_SCALING = varargin{i+1};    
                case 'ODO_MAP_Y_SCALING', ODO_MAP_Y_SCALING = varargin{i+1};    
                case 'ODO_MAP_Z_SCALING', ODO_MAP_Z_SCALING = varargin{i+1};    
                case 'EXP_MAP_X_SCALING', EXP_MAP_X_SCALING = varargin{i+1};    
                case 'EXP_MAP_Y_SCALING', EXP_MAP_Y_SCALING = varargin{i+1};    
                case 'EXP_MAP_Z_SCALING', EXP_MAP_Z_SCALING = varargin{i+1};   
            end
        end
    end
    
    %% vt
    global PREV_VT_ID;
    global SUB_VT_IMG;
    global VT_HISTORY;
    global IMG_TYPE;
    
    %% vo
    % for drawing images for estimating pitch and yaw
    global SUB_HORI_TRANS_IMG;
    global SUB_ROT_IMG;
    global SUB_VERT_TRANS_IMG;
    global SUB_PITCH_IMG;

    % for limit the velocity of gtvo
    global MAX_HORI_TRANS_V_THRESHOLD;
    global MAX_ROT_V_THRESHOLD;
    global MAX_VERT_TRANS_V_THRESHOLD;
    global MAX_PITCH_V_THRESHOLD;
    
    
    %% yaw_pitch_hdc
    global YAW_PITCH_HDC;  
        
    % The dimension of yaw in yaw_pitch_hdc network
    global YAW_PITCH_HDC_Y_DIM;
    
    % The dimension of pitch in yaw_pitch_hdc network
    global YAW_PITCH_HDC_P_DIM;
    
    % The yaw theta size of each unit in radian, 2*pi/ YAW_PITCH_HDC_Y_DIM
    % radian e.g. 2*pi/360 = 0.0175
    global YAW_PITCH_HDC_Y_TH_SIZE;                      
    global YAW_PITCH_HDC_P_TH_SIZE;     
    global MAX_ACTIVE_YAW_PITCH_HIS_PATH;
    
    
    %% 3D gridcells
    global GRIDCELLS;
    
    global GC_X_DIM;
    global GC_Y_DIM;
    global GC_Z_DIM;
    
    % set the initial position in the grid cell network
    [gcX, gcY, gcZ] = get_gc_initial_pos();
   
    global MAX_ACTIVE_XYZ_PATH;
    
    %% 3D EM
    global EXPERIENCES;
        
    % Experience history
    global EXP_HISTORY;
    
    % The namber of total experiences
    global NUM_EXPS;
    
    %% read imgs as VT input

    % get the visual data info
    [subFoldersPathSet, numSubFolders] = get_images_data_info(visualDataFile);

    % initial some variables for drawing pitch and yaw 
    odoPitchTheta = [0 0 0];
    Theta = [0 0 0 0 0 0];
    Rho = [0 0 0 0 0 0];
    startpoint =[0 0];
    endpoint = [0.8 0.8];

    odoYawTheta = [0 0 0];
    
    hdcPitchTheta = [0 0 0];
    
    hdcYawTheta = [0 0 0];
    
    hdcPitchTheta(1,3)= 0;  
    hdcYawTheta(1,3)= 0;  
   
    zAxisHDC = 1:2:36;
    
    % for drawing the activity packet of HDC
    zThirdDim = 36;
    tempYawPitchHDC = zeros(YAW_PITCH_HDC_Y_DIM, YAW_PITCH_HDC_P_DIM, zThirdDim);

    % for drawing the activity packet in toriodal ring
    R = 12;
    r = 6;
    xTorus = nan(36,36);
    yTorus = nan(36,36);
    zTorus = nan(36,36);
    for j = 1 : 36
        for i = 1 : 36
            phi = (i - 1) * 2 * pi / 35;
            theta = (j - 1) * 2 * pi / 35;
            xTorus(i,j)= cos(phi) * (R + cos(theta) * r);
            yTorus(i,j)= sin(phi) * (R + cos(theta) * r);
            zTorus(i,j)= sin(theta) * r;
        end
    end


    %% for drawing contour3 in 3D ring
    % define color map based on activity in 3D ring
%     coMap = zeros(YAW_PITCH_HDC_Y_DIM * YAW_PITCH_HDC_P_DIM, 3);
            
    %% processing two ring model for hd integration
    % x,y,z, yaw, pitch
    expTrajectory(1,1) = 0;
    expTrajectory(1,2) = 0;
    expTrajectory(1,3) = 0;
    expTrajectory(1,4) = 0;
    expTrajectory(1,5) = 0;
    
    odoMapTrajectory(1,1) = 0;
    odoMapTrajectory(1,2) = 0;
    odoMapTrajectory(1,3) = 0;
    odoMapTrajectory(1,4) = 0;
    odoMapTrajectory(1,5) = 0;
    
    transOdoTraj(1,1) = 0;
    transOdoTraj(1,2) = 0;
    transOdoTraj(1,3) = 0;
    
    % load ground truth data
    [frameId, gt_x, gt_y, gt_z, gt_rx, gt_ry, gt_rz] = load_ground_truth_data(groundTruthFile);
%     groundTruthData = [gt_x gt_y gt_z];
    
    horiTransVHis = [];
        
    preHoriTransV = 0;
    preVertTransV = 0;
    preYawRotV = 0;
    prePitchRotV = 0;
    
    % for all frames do
    replace = 0;
    
    % end libviso2
    
    
    %% process
    for iSubFolder = 1 : numSubFolders
        
        [curFolderPath, imgFilesPathList, numImgs] = get_cur_img_files_path_list(subFoldersPathSet, IMG_TYPE, iSubFolder);
        
        if numImgs > 0
            for curFrame = 1 : numImgs
                
                [curImg] = read_current_image(curFolderPath, imgFilesPathList, curFrame);

                % visual templates and visual odo uses intensity so convert to grayscale
                curGrayImg = rgb2gray(curImg);
                curGrayImg = im2double(curGrayImg);

                
                if curFrame > 1

                    tempHoriTransV = sqrt((gt_x(curFrame) - gt_x(curFrame - 1))^2 + ...
                        (gt_y(curFrame) - gt_y(curFrame - 1))^2 );
                    if abs(tempHoriTransV) < MAX_HORI_TRANS_V_THRESHOLD
                        horiTransV = tempHoriTransV;
                        preHoriTransV = horiTransV;
                    else
                        horiTransV = preHoriTransV;
                    end


                    tempYawRotV = get_signed_delta_radian(gt_rz(curFrame), gt_rz(curFrame - 1));

                    if abs(tempYawRotV) < MAX_ROT_V_THRESHOLD
                        yawRotV = tempYawRotV;
                        preYawRotV = yawRotV;
                    else
                        yawRotV = preYawRotV;
                    end

                    tempPitchRotV = get_signed_delta_radian(gt_rx(curFrame),gt_rx(curFrame - 1));

                    if abs(tempPitchRotV) < MAX_PITCH_V_THRESHOLD
                        pitchRotV = tempPitchRotV;
                        prePitchRotV = pitchRotV;
                    else
                        pitchRotV = prePitchRotV;
                    end

                    odoPitchTheta(curFrame, 3) = odoPitchTheta(curFrame - 1, 3) + pitchRotV;
                    odoYawTheta(curFrame, 3) = odoYawTheta(curFrame -1, 3) + yawRotV;

                    tempVertTransV = sqrt((gt_x(curFrame) - gt_x(curFrame - 1))^2 + ...
                        (gt_y(curFrame) - gt_y(curFrame - 1))^2 + ...
                        (gt_z(curFrame) - gt_z(curFrame - 1))^2) * sin(odoPitchTheta(curFrame, 3));

                    if abs(tempVertTransV) < MAX_VERT_TRANS_V_THRESHOLD
                        vertTransV = tempVertTransV;
                        preVertTransV = vertTransV;
                    else
                        vertTransV = preVertTransV;
                    end

                else
                    horiTransV = 0;
                    pitchRotV = 0;
                    vertTransV = 0;
                    yawRotV = 0;

                    odoPitchTheta(curFrame, 3) = pitchRotV;
                    odoYawTheta(curFrame, 3) = yawRotV;
                 
                end

               
                
                %% step3: processing vt
                % get the most active view template
                [vt_id] = visual_template(curGrayImg, gcX, gcY,gcZ, yawRotV, pitchRotV);
                 

                %% step4: processing the integration of yaw_pitch_hdc 
                yaw_pitch_hdc_iteration(vt_id, yawRotV, pitchRotV);

                [curYawTheta, curPitchTheta] = get_current_yaw_pitch_theta();

                % transform to radian
                curYawThetaInRadian = curYawTheta * YAW_PITCH_HDC_Y_TH_SIZE;
                curPitchThetaInRadian = curPitchTheta * YAW_PITCH_HDC_P_TH_SIZE;

                MAX_ACTIVE_YAW_PITCH_HIS_PATH = [MAX_ACTIVE_YAW_PITCH_HIS_PATH; curYawTheta curPitchTheta];

                % 3d grid cells interation
                gc_iteration(vt_id, horiTransV, curYawThetaInRadian, vertTransV, curPitchThetaInRadian);

                [gcX, gcY, gcZ] = get_gc_xyz();

                MAX_ACTIVE_XYZ_PATH = [MAX_ACTIVE_XYZ_PATH; gcX gcY gcZ];  

                % 3d experience map interation
                exp_map_iteration(vt_id, horiTransV, yawRotV, vertTransV, pitchRotV, gcX, gcY, gcZ, curYawTheta, curPitchTheta);


                % for drawing HDC pitch theta
                hdcPitchTheta(curFrame, 1) = cos(curPitchTheta * YAW_PITCH_HDC_P_TH_SIZE);
                hdcPitchTheta(curFrame, 2) = sin(curPitchTheta * YAW_PITCH_HDC_P_TH_SIZE);
                hdcPitchTheta(curFrame, 3) = curPitchTheta * YAW_PITCH_HDC_P_TH_SIZE;

                if curFrame > 1
                    % for drawing odometry pitch theta
                    odoPitchTheta(curFrame, 1) = cos(odoPitchTheta(curFrame, 3));
                    odoPitchTheta(curFrame + 1, 2) = sin(odoPitchTheta(curFrame, 3));
    %                 odoPitchTheta(curFrame + 1, 3) = odoPitchTheta(curFrame, 3);

                    % for drawing HDC yaw theta
                    hdcYawTheta(curFrame, 1) = cos(curYawTheta * YAW_PITCH_HDC_Y_TH_SIZE);
                    hdcYawTheta(curFrame, 2) = sin(curYawTheta * YAW_PITCH_HDC_Y_TH_SIZE);
                    hdcYawTheta(curFrame, 3) = curYawTheta * YAW_PITCH_HDC_Y_TH_SIZE;

                    % for drawing odometry yaw theta
                    odoYawTheta(curFrame, 1) = cos(odoYawTheta(curFrame, 3));
                    odoYawTheta(curFrame, 2) = sin(odoYawTheta(curFrame, 3));
    %                 odoYawTheta(curFrame + 1, 3) = odoYawTheta(curFrame, 3) + yawRotV;

                    odoMapTrajectory(curFrame,1) = odoMapTrajectory(curFrame - 1, 1) + horiTransV * cos(odoYawTheta(curFrame, 3));
                    odoMapTrajectory(curFrame,2) = odoMapTrajectory(curFrame - 1, 2) + horiTransV * sin(odoYawTheta(curFrame, 3));
                    odoMapTrajectory(curFrame,3) = odoMapTrajectory(curFrame - 1, 3) + vertTransV;
                    odoMapTrajectory(curFrame,4) = odoYawTheta(curFrame, 3);
                    odoMapTrajectory(curFrame,5) = odoPitchTheta(curFrame, 3);
    %                 transOdoTraj(curFrame + 1,1) = odoMapTrajectory(curFrame + 1,1);
    %                 transOdoTraj(curFrame + 1,2) = odoMapTrajectory(curFrame + 1,2);
    %                 transOdoTraj(curFrame + 1,3) = odoMapTrajectory(curFrame + 1,3);

                end
                
                for ind = 1 : YAW_PITCH_HDC_Y_DIM
                    tempYawPitchHDC(:,:,ind) = YAW_PITCH_HDC;
                end

                for ind = 1 : NUM_EXPS 
                    expTrajectory(ind,1) = EXPERIENCES(ind).x_exp;
                    expTrajectory(ind,2) = EXPERIENCES(ind).y_exp;
                    expTrajectory(ind,3) = EXPERIENCES(ind).z_exp;
                    expTrajectory(ind,4) = EXPERIENCES(ind).yaw_exp_rad;
                    expTrajectory(ind,5) = EXPERIENCES(ind).pitch_exp_rad;
%                     transExpTraj(ind,1) =   expTrajectory(ind,1);
%                     transExpTraj(ind,2) =   expTrajectory(ind,2);
%                     transExpTraj(ind,3) =   expTrajectory(ind,3);
                end
                
    

                % draw the results
                if (mod(curFrame, RENDER_RATE) == 1)

%                     % draw current activity packet in toroidal ring 
                    figure('color',[1 1 1]);
                    subplot(1, 1, 1,'replace');
%                     cellColor(:,:,1) = zeros(36) .* linspace(0,0,36); % red
%                     cellColor(:,:,2) = zeros(36) .* linspace(0,0,36); % green
%                     cellColor(:,:,3) = zeros(36) .* linspace(0,0,36); % blue
%                     cellColor = zeros(36,36);
                    caxis([10 30]);
                    col = del2(YAW_PITCH_HDC);
%                     for j = 1 : 36
%                         for i = 1 : 36
%                             cellColor(j,i) = col(i,j);
%                         end
%                     end

                    surf(xTorus, yTorus, zTorus, col);
%                     shading interp
                    shading faceted
%                     colormap(coMap);
%                     title('3D HD Cells Activity (Yaw Pitch)');
                    axis on
                    rotate3d on


                    drawnow;

                end
                
                PREV_VT_ID = vt_id;
            end
            
            % save history data (x, y, z, yaw, pitch)
%             save_history_data(expMapHistoryFile, expTrajectory);
%             save_history_data(odoMapHistoryFile, odoMapTrajectory);
%             save_em_history_data(emHistoryFile, VT_HISTORY);
%             save_vt_history_data(vtHistoryFile, EXP_HISTORY);
        end
    end
end