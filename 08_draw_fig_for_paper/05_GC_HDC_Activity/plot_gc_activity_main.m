function plot_gc_activity_main(visualDataFile, groundTruthFile, expMapHistoryFile, odoMapHistoryFile, vtHistoryFile, emHistoryFile, gcTrajFile, hdcTrajFile,varargin)
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
    
    clear EXP_NODES_LINKS
    
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
    for i=1:(nargin - 8)
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
    global VT_STEP;
    
    %% vo
    % for drawing images for estimating pitch and yaw
    global SUB_HORI_TRANS_IMG;
    global SUB_ROT_IMG;
    global SUB_VERT_TRANS_IMG;
    global SUB_PITCH_IMG;
    global KEY_POINT_SET;
    global ODO_STEP;
    
    %% yaw_height_hdc
    global YAW_HEIGHT_HDC;  
        
    % The dimension of yaw in yaw_height_hdc network
    global YAW_HEIGHT_HDC_Y_DIM;
    
    % The dimension of pitch in yaw_height_hdc network
    global YAW_HEIGHT_HDC_H_DIM;
    
    % The yaw theta size of each unit in radian, 2*pi/ YAW_HEIGHT_HDC_Y_DIM
    % radian e.g. 2*pi/360 = 0.0175
    global YAW_HEIGHT_HDC_Y_TH_SIZE;                      
    global MAX_ACTIVE_YAW_HEIGHT_HIS_PATH;
    
    
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
    
    global EXP_CORRECTION;
    global EXP_LOOPS;
    
    %% read imgs as VT input

    % get the visual data info
    [subFoldersPathSet, numSubFolders] = get_images_data_info(visualDataFile);

    % initial some variables for drawing pitch and yaw 
    odoHeightValue = [0 0 0];
    Theta = [0 0 0 0 0 0];
    Rho = [0 0 0 0 0 0];
    startpoint =[0 0];
    endpoint = [0.8 0.8];

    odoYawTheta = [0 0 0];

    hdcYawTheta = [0 0 0];
    
    hdcYawTheta(1,3)= 0;  
   
    zAxisHDC = 1:2:36;
  
      
    % x,y,z, yaw
    expTrajectory(1,1) = 0;
    expTrajectory(1,2) = 0;
    expTrajectory(1,3) = 0;
    expTrajectory(1,4) = 0;
    
    odoMapTrajectory(1,1) = 0;
    odoMapTrajectory(1,2) = 0;
    odoMapTrajectory(1,3) = 0;
    odoMapTrajectory(1,4) = 0;

    % load ground truth data
    [frameId, gt_x, gt_y, gt_z, gt_rx, gt_ry, gt_rz] = load_ground_truth_data(groundTruthFile);
    
    % plane
%     startFrame = 1;

    % up
%     startFrame = 2352;
%     endFrame = 2450;
    
    % down
    startFrame = 4002;
    endFrame = 4100;
    
    curFrame = 0;
    
    vtcurFrame = 1;
    preImg = 0;
%     global EXP_NODES_LINKS;
    EXP_NODES_LINKS.nodes(1) = 1;
    EXP_NODES_LINKS.numlinks(1) = 0;
    EXP_NODES_LINKS.linknodes(1, 1) = 0;
    
    global DELTA_EM;
    
    %% process
    for iSubFolder = 1 : numSubFolders
        
        [curFolderPath, imgFilesPathList, numImgs] = get_cur_img_files_path_list(subFoldersPathSet, IMG_TYPE, iSubFolder);
        
        if numImgs > 0

              if ODO_STEP == 0
                  %             for indFrame = startFrame : endFrame -1
                  for indFrame = startFrame : endFrame -1
                [curImg] = read_current_image(curFolderPath, imgFilesPathList, indFrame);
                                
                % visual templates and visual odo uses intensity so convert to grayscale
                curGrayImg = rgb2gray(curImg);
                curGrayImg = im2double(curGrayImg);
                
                %% step2: processing vo
                % get the odometry from the video
                % computing the 3d odometry based on the current image.
                % yawRotV in degree
                
                % for common use
                %  [transV, yawRotV, heightV] = visual_odometry(curGrayImg);
                
                % for special use
                if length(KEY_POINT_SET) == 2
                    if indFrame < KEY_POINT_SET(1)
                        [transV, yawRotV, heightV] = visual_odometry(curGrayImg);
                    elseif indFrame < KEY_POINT_SET(2)
                        [transV, yawRotV, heightV] = visual_odometry_up(curGrayImg);
                    else 
                        [transV, yawRotV, heightV] = visual_odometry(curGrayImg);
                    end
                    transV = 2;
                else
                    if indFrame < KEY_POINT_SET(1)
                        [transV, yawRotV, heightV] = visual_odometry(curGrayImg);
                    elseif indFrame < KEY_POINT_SET(2) 
                        [transV, yawRotV, heightV] = visual_odometry_up(curGrayImg);
                    elseif indFrame < KEY_POINT_SET(3)
                        [transV, yawRotV, heightV] = visual_odometry(curGrayImg);
                    elseif indFrame < KEY_POINT_SET(4)
                        [transV, yawRotV, heightV] = visual_odometry_down(curGrayImg);
                    else 
                        [transV, yawRotV, heightV] = visual_odometry(curGrayImg);
                    end
                end
                
                yawRotV = yawRotV * DEGREE_TO_RADIAN;  % in radian
                
                      
                %% step3: processing vt
                % get the most active view template
                curFrame = curFrame + 1;
                if VT_STEP == 0
                    vtcurGrayImg = curGrayImg; 
                else
                    if mod(curFrame,VT_STEP) == 1 
                        vtcurGrayImg = curGrayImg; 
                        preImg = vtcurGrayImg;
                    else
                        vtcurGrayImg = preImg;
                    end
                end
                
                [vt_id] = visual_template(vtcurGrayImg, gcX, gcY,gcZ, yawRotV, heightV);
   
                
                %% step4: processing the integration of yaw_height_hdc 
                yaw_height_hdc_iteration(vt_id, yawRotV, heightV);

                [curYawTheta, curHeightValue] = get_current_yaw_height_value();

                % transform to radian
                curYawThetaInRadian = curYawTheta * YAW_HEIGHT_HDC_Y_TH_SIZE;

                MAX_ACTIVE_YAW_HEIGHT_HIS_PATH = [MAX_ACTIVE_YAW_HEIGHT_HIS_PATH; curYawTheta curHeightValue];

                % 3d grid cells interation
                gc_iteration(vt_id, transV, curYawThetaInRadian, heightV);

                [gcX, gcY, gcZ] = get_gc_xyz();

                MAX_ACTIVE_XYZ_PATH = [MAX_ACTIVE_XYZ_PATH; gcX gcY gcZ];  

                % 3d experience map interation
                exp_map_iteration(vt_id, transV, yawRotV, heightV, gcX, gcY, gcZ, curYawTheta, curHeightValue);

                % for drawing odometry yaw theta
                odoYawTheta(curFrame + 1, 1) = cos(odoYawTheta(curFrame, 3) + yawRotV);
                odoYawTheta(curFrame + 1, 2) = sin(odoYawTheta(curFrame, 3) + yawRotV);
                odoYawTheta(curFrame + 1, 3) = odoYawTheta(curFrame, 3) + yawRotV;

                odoMapTrajectory(curFrame + 1,1) = odoMapTrajectory(curFrame,1) + transV * cos(sym(odoYawTheta(curFrame, 3) + yawRotV));
                odoMapTrajectory(curFrame + 1,2) = odoMapTrajectory(curFrame,2) + transV * sin(sym(odoYawTheta(curFrame, 3) + yawRotV));
                odoMapTrajectory(curFrame + 1,3) = odoMapTrajectory(curFrame,3) + heightV;
                odoMapTrajectory(curFrame + 1,4) = odoYawTheta(curFrame + 1, 3);


                % for drawing HDC yaw theta
                hdcYawTheta(curFrame, 1) = cos(curYawTheta * YAW_HEIGHT_HDC_Y_TH_SIZE);
                hdcYawTheta(curFrame, 2) = sin(curYawTheta * YAW_HEIGHT_HDC_Y_TH_SIZE);
                hdcYawTheta(curFrame, 3) = curYawTheta * YAW_HEIGHT_HDC_Y_TH_SIZE;


                for ind = 1 : NUM_EXPS 
                    expTrajectory(ind,1) = EXPERIENCES(ind).x_exp;
                    expTrajectory(ind,2) = EXPERIENCES(ind).y_exp;
                    expTrajectory(ind,3) = EXPERIENCES(ind).z_exp;
                    expTrajectory(ind,4) = EXPERIENCES(ind).yaw_exp_rad;    
                end

                % get the experience nodes and links info
                for ind_exps = 1 : NUM_EXPS

                    EXP_NODES_LINKS.nodes(ind_exps) = ind_exps;
                    EXP_NODES_LINKS.numlinks(ind_exps) = EXPERIENCES(ind_exps).numlinks;

                    for link_id = 1 : EXPERIENCES(ind_exps).numlinks
                        EXP_NODES_LINKS.linknodes(ind_exps, link_id) =  ...
                            EXPERIENCES(ind_exps).links(link_id).exp_id;
                    end

                end
                
                
                % draw the results
                if (mod(curFrame, RENDER_RATE) == 1)
                    figure(1)
                    % drawing the activity of 3D grid cells
                    subplot(1, 1, 1,'replace');
                    edge1_x = [0 0];
                    edge1_y = [0 0];
                    edge1_z = [0 36];

                    edge2_x = [0 0];
                    edge2_y = [0 36];
                    edge2_z = [0 0];

                    edge3_x = [0 36];
                    edge3_y = [0 0];
                    edge3_z = [0 0];

                    edge4_x = [0 36];
                    edge4_y = [36 36];
                    edge4_z = [0 0];

                    edge5_x = [0 0];
                    edge5_y = [36 36];
                    edge5_z = [0 36];

                    edge6_x = [36 36];
                    edge6_y = [0 36];
                    edge6_z = [0 0];

                    edge7_x = [36 36];
                    edge7_y = [0 0];
                    edge7_z = [0 36];

                    edge8_x = [0 0];
                    edge8_y = [0 36];
                    edge8_z = [36 36];

                    edge9_x = [0 36];
                    edge9_y = [0 0];
                    edge9_z = [36 36];

                    edge10_x = [0 36];
                    edge10_y = [36 36];
                    edge10_z = [36 36];

                    edge11_x = [36 36];
                    edge11_y = [0 36];
                    edge11_z = [36 36];

                    edge12_x = [36 36];
                    edge12_y = [36 36];
                    edge12_z = [0 36];

                    plot3(edge1_x,edge1_y,edge1_z, '-k');
                    hold on
                    plot3(edge2_x,edge2_y,edge2_z, '-k');
                    hold on
                    plot3(edge3_x,edge3_y,edge3_z, '-k');
                    hold on
                    plot3(edge4_x,edge4_y,edge4_z, '-k');
                    hold on
                    plot3(edge5_x,edge5_y,edge5_z, '-k');
                    hold on
                    plot3(edge6_x,edge6_y,edge6_z, '-k');
                    hold on
                    plot3(edge7_x,edge7_y,edge7_z, '-k');
                    hold on
                    plot3(edge8_x,edge8_y,edge8_z, '-k');
                    hold on
                    plot3(edge9_x,edge9_y,edge9_z, '-k');
                    hold on
                    plot3(edge10_x,edge10_y,edge10_z, '-k');
                    hold on
                    plot3(edge11_x,edge11_y,edge11_z, '-k');
                    hold on
                    plot3(edge12_x,edge12_y,edge12_z, '-k');
                    
                    view(20,20)
                    
                    

                    phandles = contourslice(GRIDCELLS, 1:2:GC_X_DIM, 1:2:GC_Y_DIM, 1:2:GC_Z_DIM, 10); 
%                     axis([1 GC_X_DIM 1 GC_Y_DIM 1 GC_Z_DIM]);
%                     view(3);
                    set(phandles,'LineWidth',0.5);
%                     grid on;
                    hold on;
                    plot3(MAX_ACTIVE_XYZ_PATH(end,2), MAX_ACTIVE_XYZ_PATH(end,1),  MAX_ACTIVE_XYZ_PATH(end,3), '.m', 'MarkerSize',8);
%                  
%                     plot3(MAX_ACTIVE_XYZ_PATH(:,2), MAX_ACTIVE_XYZ_PATH(:,1),  MAX_ACTIVE_XYZ_PATH(:,3), '.r', 'MarkerSize',8);
%                     plot3([MAX_ACTIVE_XYZ_PATH(end,2) MAX_ACTIVE_XYZ_PATH(end,2)], [MAX_ACTIVE_XYZ_PATH(end,1) MAX_ACTIVE_XYZ_PATH(end,1)], [0 gcZ], 'MarkerSize',8);
%                     plot3([MAX_ACTIVE_XYZ_PATH(end,1) MAX_ACTIVE_XYZ_PATH(end,1)],[MAX_ACTIVE_XYZ_PATH(end,2) MAX_ACTIVE_XYZ_PATH(end,2)],  [0 gcZ], 'MarkerSize',8);

                    hold off;
                    xl = xlabel('y', 'FontSize',24);
                    yl = ylabel('x', 'FontSize',24);
                    z1 = zlabel('z', 'FontSize',24);
                    view(20,20)
                    axis([0 36 0 36 0 36]);
                    set(gca,'xtick',0:9:36)  
                    set(gca,'ytick',0:18:36) 
                    set(gca,'ztick',0:9:36)   
%                     set(xl,'Rotation',15);
%                     set(yl,'Rotation',-30);
                    set(gca,'FontSize',24, 'LineWidth',1.5); % axis font
                    fig = get(groot,'CurrentFigure');
                    set (fig,'Position',[300,300,500,400], 'color','w')
                    
                    path = 'C:\Activity_Fig\GC\GC_Activity\03_Down';
                    imageID = num2str(curFrame);
                    fileType = '.tif';
                    fileName = strcat(imageID,fileType);
                    fileFullPath = fullfile (path, fileName);
                    print(fig, '-dtiff', '-r600', fileFullPath);
                    
                    figure(2)
                    subplot(1, 1, 1,'replace');
                    [x,y]= meshgrid(1:1:36);
                    z = YAW_HEIGHT_HDC ;
                    surf(x,y,z,'EdgeColor','b');
                    % surf(x,y,z,'FaceColor','red','EdgeColor','none');
                    shading interp
                     axis([1 YAW_HEIGHT_HDC_Y_DIM 1 YAW_HEIGHT_HDC_H_DIM 0 0.1]);
                    % meshc(x,y,z);
                    % view(0, 90);
                    xl = xlabel('Height','FontSize',24);
                    yl = ylabel('Yaw','FontSize',24);
                    zlabel('Acitivity','FontSize',24);
                    set(xl,'Rotation',15);
                    set(yl,'Rotation',-30);
%                     title('Yaw Pitch HDCell Activity');
                    set(gca,'xtick',0:9:36)  
                    set(gca,'ytick',0:9:36) 
                    axis on
                    set(gca,'FontSize',24, 'LineWidth',1.5); % axis font
                    fig = get(groot,'CurrentFigure');
                    set (fig,'Position',[900,300,500,400], 'color','w')

                    path = 'C:\Activity_Fig\HDC\hdc_activity\03_Down';
                    imageID = num2str(curFrame);
                    fileType = '.tif';
                    fileName = strcat(imageID,fileType);
                    fileFullPath = fullfile (path, fileName);
                    print(fig, '-dtiff', '-r600', fileFullPath);
                    
                    drawnow;
  
                end
                
                PREV_VT_ID = vt_id;
                
              
                end
              else
                 for indFrame = startFrame :ODO_STEP: numImgs -1
                [curImg] = read_current_image(curFolderPath, imgFilesPathList, indFrame);
                                
                % visual templates and visual odo uses intensity so convert to grayscale
                curGrayImg = rgb2gray(curImg);
                curGrayImg = im2double(curGrayImg);
                
                %% step2: processing vo
                % get the odometry from the video
                % computing the 3d odometry based on the current image.
                % yawRotV in degree
                
                % for common use
                %  [transV, yawRotV, heightV] = visual_odometry(curGrayImg);
                
                % for special use
                if length(KEY_POINT_SET) == 2
                    if indFrame < KEY_POINT_SET(1)
                        [transV, yawRotV, heightV] = visual_odometry(curGrayImg);
                    elseif indFrame < KEY_POINT_SET(2)
                        [transV, yawRotV, heightV] = visual_odometry_up(curGrayImg);
                    else 
                        [transV, yawRotV, heightV] = visual_odometry(curGrayImg);
                    end
                    transV = 2;
                else
                    if indFrame < KEY_POINT_SET(1)
                        [transV, yawRotV, heightV] = visual_odometry(curGrayImg);
                    elseif indFrame < KEY_POINT_SET(2) 
                        [transV, yawRotV, heightV] = visual_odometry_up(curGrayImg);
                    elseif indFrame < KEY_POINT_SET(3)
                        [transV, yawRotV, heightV] = visual_odometry(curGrayImg);
                    elseif indFrame < KEY_POINT_SET(4)
                        [transV, yawRotV, heightV] = visual_odometry_down(curGrayImg);
                    else 
                        [transV, yawRotV, heightV] = visual_odometry(curGrayImg);
                    end
                end
                
                yawRotV = yawRotV * DEGREE_TO_RADIAN;  % in radian
                
                      
                %% step3: processing vt
                % get the most active view template
                curFrame = curFrame + 1;
                if VT_STEP == 0
                    vtcurGrayImg = curGrayImg; 
                else
                    if mod(curFrame,VT_STEP) == 1 
                        vtcurGrayImg = curGrayImg; 
                        preImg = vtcurGrayImg;
                    else
                        vtcurGrayImg = preImg;
                    end
                end
                
                [vt_id] = visual_template(vtcurGrayImg, gcX, gcY,gcZ, yawRotV, heightV);
   
                
                %% step4: processing the integration of yaw_height_hdc 
                yaw_height_hdc_iteration(vt_id, yawRotV, heightV);

                [curYawTheta, curHeightValue] = get_current_yaw_height_value();

                % transform to radian
                curYawThetaInRadian = curYawTheta * YAW_HEIGHT_HDC_Y_TH_SIZE;

                MAX_ACTIVE_YAW_HEIGHT_HIS_PATH = [MAX_ACTIVE_YAW_HEIGHT_HIS_PATH; curYawTheta curHeightValue];

                % 3d grid cells interation
                gc_iteration(vt_id, transV, curYawThetaInRadian, heightV);

                [gcX, gcY, gcZ] = get_gc_xyz();

                MAX_ACTIVE_XYZ_PATH = [MAX_ACTIVE_XYZ_PATH; gcX gcY gcZ];  

                % 3d experience map interation
                exp_map_iteration(vt_id, transV, yawRotV, heightV, gcX, gcY, gcZ, curYawTheta, curHeightValue);

                % for drawing odometry yaw theta
                odoYawTheta(curFrame + 1, 1) = cos(odoYawTheta(curFrame, 3) + yawRotV);
                odoYawTheta(curFrame + 1, 2) = sin(odoYawTheta(curFrame, 3) + yawRotV);
                odoYawTheta(curFrame + 1, 3) = odoYawTheta(curFrame, 3) + yawRotV;

                odoMapTrajectory(curFrame + 1,1) = odoMapTrajectory(curFrame,1) + transV * cos(sym(odoYawTheta(curFrame, 3) + yawRotV));
                odoMapTrajectory(curFrame + 1,2) = odoMapTrajectory(curFrame,2) + transV * sin(sym(odoYawTheta(curFrame, 3) + yawRotV));
                odoMapTrajectory(curFrame + 1,3) = odoMapTrajectory(curFrame,3) + heightV;
                odoMapTrajectory(curFrame + 1,4) = odoYawTheta(curFrame + 1, 3);


                % for drawing HDC yaw theta
                hdcYawTheta(curFrame, 1) = cos(curYawTheta * YAW_HEIGHT_HDC_Y_TH_SIZE);
                hdcYawTheta(curFrame, 2) = sin(curYawTheta * YAW_HEIGHT_HDC_Y_TH_SIZE);
                hdcYawTheta(curFrame, 3) = curYawTheta * YAW_HEIGHT_HDC_Y_TH_SIZE;


                for ind = 1 : NUM_EXPS 
                    expTrajectory(ind,1) = EXPERIENCES(ind).x_exp;
                    expTrajectory(ind,2) = EXPERIENCES(ind).y_exp;
                    expTrajectory(ind,3) = EXPERIENCES(ind).z_exp;
                    expTrajectory(ind,4) = EXPERIENCES(ind).yaw_exp_rad;    
                end

                % get the experience nodes and links info
                for ind_exps = 1 : NUM_EXPS

                    EXP_NODES_LINKS.nodes(ind_exps) = ind_exps;
                    EXP_NODES_LINKS.numlinks(ind_exps) = EXPERIENCES(ind_exps).numlinks;

                    for link_id = 1 : EXPERIENCES(ind_exps).numlinks
                        EXP_NODES_LINKS.linknodes(ind_exps, link_id) =  ...
                            EXPERIENCES(ind_exps).links(link_id).exp_id;
                    end

                end
                
                
                % draw the results
                if (mod(curFrame, RENDER_RATE) == 1)
                    figure(1)
                    % drawing the activity of 3D grid cells
                    subplot(1, 1, 1,'replace');
                    edge1_x = [0 0];
                    edge1_y = [0 0];
                    edge1_z = [0 36];

                    edge2_x = [0 0];
                    edge2_y = [0 36];
                    edge2_z = [0 0];

                    edge3_x = [0 36];
                    edge3_y = [0 0];
                    edge3_z = [0 0];

                    edge4_x = [0 36];
                    edge4_y = [36 36];
                    edge4_z = [0 0];

                    edge5_x = [0 0];
                    edge5_y = [36 36];
                    edge5_z = [0 36];

                    edge6_x = [36 36];
                    edge6_y = [0 36];
                    edge6_z = [0 0];

                    edge7_x = [36 36];
                    edge7_y = [0 0];
                    edge7_z = [0 36];

                    edge8_x = [0 0];
                    edge8_y = [0 36];
                    edge8_z = [36 36];

                    edge9_x = [0 36];
                    edge9_y = [0 0];
                    edge9_z = [36 36];

                    edge10_x = [0 36];
                    edge10_y = [36 36];
                    edge10_z = [36 36];

                    edge11_x = [36 36];
                    edge11_y = [0 36];
                    edge11_z = [36 36];

                    edge12_x = [36 36];
                    edge12_y = [36 36];
                    edge12_z = [0 36];

                    plot3(edge1_x,edge1_y,edge1_z, '-k');
                    hold on
                    plot3(edge2_x,edge2_y,edge2_z, '-k');
                    hold on
                    plot3(edge3_x,edge3_y,edge3_z, '-k');
                    hold on
                    plot3(edge4_x,edge4_y,edge4_z, '-k');
                    hold on
                    plot3(edge5_x,edge5_y,edge5_z, '-k');
                    hold on
                    plot3(edge6_x,edge6_y,edge6_z, '-k');
                    hold on
                    plot3(edge7_x,edge7_y,edge7_z, '-k');
                    hold on
                    plot3(edge8_x,edge8_y,edge8_z, '-k');
                    hold on
                    plot3(edge9_x,edge9_y,edge9_z, '-k');
                    hold on
                    plot3(edge10_x,edge10_y,edge10_z, '-k');
                    hold on
                    plot3(edge11_x,edge11_y,edge11_z, '-k');
                    hold on
                    plot3(edge12_x,edge12_y,edge12_z, '-k');
                    
                    view(20,20)
                    
                    

                    phandles = contourslice(GRIDCELLS, 1:2:GC_X_DIM, 1:2:GC_Y_DIM, 1:2:GC_Z_DIM, 10); 
%                     axis([1 GC_X_DIM 1 GC_Y_DIM 1 GC_Z_DIM]);
%                     view(3);
                    set(phandles,'LineWidth',0.5);
%                     grid on;
                    hold on;
                    plot3(MAX_ACTIVE_XYZ_PATH(end,2), MAX_ACTIVE_XYZ_PATH(end,1),  MAX_ACTIVE_XYZ_PATH(end,3), '.m', 'MarkerSize',8);
%                     plot3([MAX_ACTIVE_XYZ_PATH(end,2) MAX_ACTIVE_XYZ_PATH(end,2)], [MAX_ACTIVE_XYZ_PATH(end,1) MAX_ACTIVE_XYZ_PATH(end,1)], [0 gcZ], 'MarkerSize',8);
%                     plot3([MAX_ACTIVE_XYZ_PATH(end,1) MAX_ACTIVE_XYZ_PATH(end,1)],[MAX_ACTIVE_XYZ_PATH(end,2) MAX_ACTIVE_XYZ_PATH(end,2)],  [0 gcZ], 'MarkerSize',8);

                    hold off;
                    xl = xlabel('y', 'FontSize',24);
                    yl = ylabel('x', 'FontSize',24);
                    z1 = zlabel('z', 'FontSize',24);
                    view(20,20)
                    axis([0 36 0 36 0 36]);
                    set(gca,'xtick',0:9:36)  
                    set(gca,'ytick',0:18:36) 
                    set(gca,'ztick',0:9:36)   
%                     set(xl,'Rotation',15);
%                     set(yl,'Rotation',-30);
                    set(gca,'FontSize',24, 'LineWidth',1.5); % axis font
                    fig = get(groot,'CurrentFigure');
                    set (fig,'Position',[300,300,500,400], 'color','w')

                    
                    figure(2)
                    subplot(1, 1, 1,'replace');
                    [x,y]= meshgrid(1:1:36);
                    z = YAW_HEIGHT_HDC ;
                    surf(x,y,z,'EdgeColor','b');
                    % surf(x,y,z,'FaceColor','red','EdgeColor','none');
                    shading interp
                     axis([1 YAW_HEIGHT_HDC_Y_DIM 1 YAW_HEIGHT_HDC_H_DIM 0 0.2]);
                    % meshc(x,y,z);
                    % view(0, 90);
                    xl = xlabel('Height','FontSize',24);
                    yl = ylabel('Yaw','FontSize',24);
                    zlabel('Acitivity','FontSize',24);
                    set(xl,'Rotation',15);
                    set(yl,'Rotation',-30);
%                     title('Yaw Pitch HDCell Activity');
                    set(gca,'xtick',0:9:36)  
                    set(gca,'ytick',0:9:36) 
                    axis on
                    set(gca,'FontSize',24, 'LineWidth',1.5); % axis font
                    fig = get(groot,'CurrentFigure');
                    set (fig,'Position',[900,300,500,400], 'color','w')

                    drawnow;
                end
                
                PREV_VT_ID = vt_id;
                
              
                 end
             
              end    
     end
  end
end
   
    
%     figure(2)
%     for ind = 1 : 30000
% 
%         % do the experience map correction interatively for all the links in all the experiences
%         for i = 1:EXP_LOOPS
% 
%             for exp_id = 1:NUM_EXPS
% 
%                 for link_id=1:EXPERIENCES(exp_id).numlinks
% 
%                     % experience 0 has a link to experience 1
%                     e0 = exp_id;
%                     e1 = EXPERIENCES(exp_id).links(link_id).exp_id;
% 
%                     % work out where e0 thinks e1 (x,y) should be based on the stored link information
%                     lx = EXPERIENCES(e0).x_exp + EXPERIENCES(e0).links(link_id).d_xy * cos(EXPERIENCES(e0).yaw_exp_rad + EXPERIENCES(e0).links(link_id).heading_yaw_exp_rad);
%                     ly = EXPERIENCES(e0).y_exp + EXPERIENCES(e0).links(link_id).d_xy * sin(EXPERIENCES(e0).yaw_exp_rad + EXPERIENCES(e0).links(link_id).heading_yaw_exp_rad);
%                     lz = EXPERIENCES(e0).z_exp + EXPERIENCES(e0).links(link_id).d_z;  % 
% 
%                     % correct e0 and e1 (x,y) by equal but opposite amounts
%                     % a 0.5 correction parameter means that e0 and e1 will be fully
%                     % corrected based on e0's link information
%                     EXPERIENCES(e0).x_exp = EXPERIENCES(e0).x_exp + (EXPERIENCES(e1).x_exp - lx) * EXP_CORRECTION;
%                     EXPERIENCES(e0).y_exp = EXPERIENCES(e0).y_exp + (EXPERIENCES(e1).y_exp - ly) * EXP_CORRECTION;
%                     EXPERIENCES(e0).z_exp = EXPERIENCES(e0).z_exp + (EXPERIENCES(e1).z_exp - lz) * EXP_CORRECTION;
% 
%                     EXPERIENCES(e1).x_exp = EXPERIENCES(e1).x_exp - (EXPERIENCES(e1).x_exp - lx) * EXP_CORRECTION;
%                     EXPERIENCES(e1).y_exp = EXPERIENCES(e1).y_exp - (EXPERIENCES(e1).y_exp - ly) * EXP_CORRECTION;
%                     EXPERIENCES(e1).z_exp = EXPERIENCES(e1).z_exp - (EXPERIENCES(e1).z_exp - lz) * EXP_CORRECTION;
% 
%                     % determine the angle between where e0 thinks e1's facing
%                     % should be based on the link information
%                     TempDeltaYawFacing = get_signed_delta_radian((EXPERIENCES(e0).yaw_exp_rad + EXPERIENCES(e0).links(link_id).facing_yaw_exp_rad), EXPERIENCES(e1).yaw_exp_rad);
% 
%                     % correct e0 and e1 facing by equal but opposite amounts
%                     % a 0.5 correction parameter means that e0 and e1 will be fully
%                     % corrected based on e0's link information           
%                     EXPERIENCES(e0).yaw_exp_rad = clip_radian_180(EXPERIENCES(e0).yaw_exp_rad + TempDeltaYawFacing * EXP_CORRECTION);
%                     EXPERIENCES(e1).yaw_exp_rad = clip_radian_180(EXPERIENCES(e1).yaw_exp_rad - TempDeltaYawFacing * EXP_CORRECTION);
% 
%                     % determine the angle between where e0 thinks e1's facing
%                     % should be based on the link information
%     %                 TempDeltaHeightFacing = get_signed_delta_radian((EXPERIENCES(e0).height_exp + EXPERIENCES(e0).links(link_id).facing_height_exp), EXPERIENCES(e1).height_exp);
% 
%                     % correct e0 and e1 facing by equal but opposite amounts
%                     % a 0.5 correction parameter means that e0 and e1 will be fully
%                     % corrected based on e0's link information           
%     %                 EXPERIENCES(e0).height_exp = EXPERIENCES(e0).height_exp + TempDeltaHeightFacing * EXP_CORRECTION;
%     %                 EXPERIENCES(e1).height_exp = EXPERIENCES(e1).height_exp - TempDeltaHeightFacing * EXP_CORRECTION;
%     %                 
%                 end
%             end
% 
%         end
% 
%     % keep a frame by frame history of which experience was currently active
% 
%         for ind = 1 : NUM_EXPS 
%             expTrajectory(ind,1) = EXPERIENCES(ind).x_exp;
%             expTrajectory(ind,2) = EXPERIENCES(ind).y_exp;
%             expTrajectory(ind,3) = EXPERIENCES(ind).z_exp;
%             expTrajectory(ind,4) = EXPERIENCES(ind).yaw_exp_rad;    
%         end
%     
%         
%         
%         subplot(1, 2, 1, 'replace');
%         hold on
%         plot3((expTrajectory(:,2)) * EXP_MAP_X_SCALING, ...
%             (expTrajectory(:,1) ) * EXP_MAP_Y_SCALING, ...
%             expTrajectory(:,3) * EXP_MAP_Z_SCALING, '.b');
% 
% %         plot3((gt_x(1:curFrame)-gt_x(1)) * GT_EXP_X_SCALING, ...
% %             (gt_y(1:curFrame)-gt_y(1)) * GT_EXP_Y_SCALING, ...
% %             (gt_z(1:curFrame)-gt_z(1)) * GT_EXP_Z_SCALING, '.r');
%         hold off;
%         grid on
%         view(3)
% %                   view(0, 90);
%         xl = xlabel('exp-x');
%         yl = ylabel('exp-y');
%         zlabel('exp-z');
%         set(xl,'Rotation',15);
%         set(yl,'Rotation',-30);
%         title('Multilayered Experience Map');
% %                     legend('Result','Truth' ,'1');
% %                     axis([-5 110 -5 100 -2 5]);
% %                     axis equal                    
%         axis on
%         rotate3d on
%                
%         
%         subplot(1, 2, 2, 'replace');
%         hold on
%         plot3((expTrajectory(:,2)) * EXP_MAP_X_SCALING , ...
%             (expTrajectory(:,1) ) * EXP_MAP_Y_SCALING, ...
%             expTrajectory(:,3) * EXP_MAP_Z_SCALING, '.b');
% 
% %         plot3((gt_x(1:curFrame)-gt_x(1)) * GT_EXP_X_SCALING, ...
% %             (gt_y(1:curFrame)-gt_y(1)) * GT_EXP_Y_SCALING, ...
% %             (gt_z(1:curFrame)-gt_z(1)) * GT_EXP_Z_SCALING, '.r');
%         hold off;
%         grid on;
%         view(0, 90);
%         xlabel('exp-x');
%         ylabel('exp-y');
%         zlabel('exp-z');
%         title('Multilayered Experience Map (top view)');
% %                     legend('Result','Truth' ,'1');
% %                     axis([-5 110 -5 100]);
%         axis on
%         
%         drawnow;
%     end
% end

