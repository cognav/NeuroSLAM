function visual_odo_main(visualDataFile,groundTruthFile)
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

    %% vo
    % for drawing images for estimating pitch and yaw
    global SUB_TRANS_IMG;
    global SUB_YAW_ROT_IMG;
    global SUB_HEIGHT_V_IMG;

    global ODO_IMG_TRANS_RESIZE_RANGE;
    global ODO_IMG_YAW_ROT_RESIZE_RANGE;
    global ODO_IMG_HEIGHT_V_RESIZE_RANGE;
        
    global PREV_TRANS_V_IMG_X_SUMS;
    global PREV_YAW_ROT_V_IMG_X_SUMS;
    global PREV_HEIGHT_V_IMG_Y_SUMS;
    
    %% read imgs as VT input

    % get the visual data info
    [subFoldersPathSet, numSubFolders] = get_images_data_info(visualDataFile);

    % load ground truth data
    [frameId, gt_x, gt_y, gt_z, gt_rx, gt_ry, gt_rz] = load_ground_truth_data(groundTruthFile);
    groundTruthData = [gt_x gt_y gt_z];
    
    % define temp variables of previous profiles
    preProfilesTransImg = zeros(1,ODO_IMG_TRANS_RESIZE_RANGE(2));
    preProfilesYawRotImg = zeros(1,ODO_IMG_YAW_ROT_RESIZE_RANGE(2));
    preProfilesPitchRotImg = zeros(ODO_IMG_HEIGHT_V_RESIZE_RANGE(2), 1);
    
    % For drawing the current rot direction
    curRotDir = [0 0 0];
    curRotDir(1,3) = 0;

       
    % The total rotational angle
    sumRotAngle = 0;
    
        
    % for drawing the direction theta (rotational angle)
    theta = [0 0 0 0 0 0];
    rho = [0 0 0 0 0 0];
    startPoint =[0 0];
    endPoint = [0.8 0.8];

    % For drawing the velocity
    subRotVel = [];
    transVelVector = [];
    heightVelVector = [];
    sumHeight = [];
    transVelVector(1) = 0;
    heightVelVector(1) = 0;
    subRotVel(1) = 0;
    sumHeight = 0;
    
     % Transform the angle from degree to radian
    global DEGREE_TO_RADIAN;
    DEGREE_TO_RADIAN = pi / 180;
   
    % Transform the angle from radian to degree 
    global RADIAN_TO_DEGREE;
    RADIAN_TO_DEGREE = 180 / pi;  
    
    RENDER_RATE = 2;
    
    IMG_TYPE = '*.png';
          
    global OFFSET_YAW_ROT; 
    offset_yaw_rot_vector = [];
    offset_yaw_rot_vector(1) = 0;
    
    global OFFSET_HEIGHT_V;
    offset_height_V_vector = [];
    offset_height_V_vector(1) = 0;
    
    odoMapTrajectory(1,1) = 0;
    odoMapTrajectory(1,2) = 0;
    odoMapTrajectory(1,3) = 0;
    odoMapTrajectory(1,4) = 0;
    
    ODO_MAP_X_SCALING = 1;
    ODO_MAP_Y_SCALING = 1;
    ODO_MAP_Z_SCALING = 1;
    GT_ODO_X_SCALING = 1;
    GT_ODO_Y_SCALING = 1;
    GT_ODO_Z_SCALING = 1;
    
    curFrame = 0;
%     step = 1;

%     %  up
    startFrame = 1;
    endFrame = 3000;

    % down
%     startFrame = 2484;
%     endFrame = 2621;

    % level_2
%     startFrame = 925;
%     endFrame = 2483;

    %% processing visual odometry
    for iSubFolder = 1 : numSubFolders
        
        [curFolderPath, imgFilesPathList, numImgs] = get_cur_img_files_path_list(subFoldersPathSet, IMG_TYPE, iSubFolder);
        
        if numImgs > 0
            for indFrame = startFrame : endFrame
                curFrame = curFrame + 1;
                [curImg] = read_current_image(curFolderPath, imgFilesPathList, indFrame);

                % visual templates and visual odo uses intensity so convert to grayscale
                curGrayImg = rgb2gray(curImg);
                curGrayImg = im2double(curGrayImg);
                
                % for drawing previous intensity
                preProfilesTransImg = PREV_TRANS_V_IMG_X_SUMS;
                preProfilesYawRotImg = PREV_YAW_ROT_V_IMG_X_SUMS;
                preProfilesHeightVImg = PREV_HEIGHT_V_IMG_Y_SUMS;
     
                %% step2: processing vo
                % get the odometry from the video
                % computing the 3d odometry based on the current image.
                % yawRotV and pitchRotV in degree
                [transV, yawRotV, heightV] = visual_odometry(curGrayImg);


                subRotVel(curFrame) = yawRotV;
                transVelVector(curFrame) = transV;
                heightVelVector(curFrame) = heightV;
                
                offset_yaw_rot_vector(curFrame) = OFFSET_YAW_ROT;
                offset_height_V_vector(curFrame) = OFFSET_HEIGHT_V;
                
                sumHeight(curFrame + 1) = sumHeight(curFrame) + heightV;
                
                % For drawing the total rotational angle
                if  sumRotAngle + yawRotV >= 360
                    sumRotAngle = mod(sumRotAngle + yawRotV, 360);
                elseif sumRotAngle + yawRotV <= -360
                    sumRotAngle = sumRotAngle + yawRotV + 360;
                else
                    sumRotAngle = sumRotAngle + yawRotV;
                end


               
                % Transform the vrot from degree to radian
                yawRotV = yawRotV * DEGREE_TO_RADIAN;  

                % Direction by raw odometry
                curRotDir(curFrame + 1, 1) = cos(curRotDir(curFrame, 3) + yawRotV );
                curRotDir(curFrame + 1, 2) = sin(curRotDir(curFrame, 3) + yawRotV );
                curRotDir(curFrame + 1, 3) = curRotDir(curFrame, 3) + yawRotV;
 
                odoMapTrajectory(curFrame + 1,1) = odoMapTrajectory(curFrame,1) + transV * cos(sym(curRotDir(curFrame + 1, 3)));
                odoMapTrajectory(curFrame + 1,2) = odoMapTrajectory(curFrame,2) + transV * sin(sym(curRotDir(curFrame + 1, 3)));
                odoMapTrajectory(curFrame + 1,3) = odoMapTrajectory(curFrame,3) + heightV;
                odoMapTrajectory(curFrame + 1,4) = curRotDir(curFrame + 1, 3);
                
                % render results information
                if (mod(curFrame, RENDER_RATE) == 0)

                    % computing the normalized profile of horizontal translational image
                    profilesTransImg = sum(SUB_TRANS_IMG);
                    avgIntensity = sum(profilesTransImg) / size(profilesTransImg, 2);
                    profilesTransImg = profilesTransImg / avgIntensity; 

                    
                    % computing the normalized profile of rotational image
                    profilesYawRotImg = sum(SUB_YAW_ROT_IMG);
                    avgIntensity = sum(profilesYawRotImg) / size(profilesYawRotImg, 2);
                    profilesYawRotImg = profilesYawRotImg / avgIntensity;  

%                     diffYawRotImgs = profilesYawRotImg - preProfilesYawRotImg;
                    % computing the normalized profile of vertical transV image
                    profilesHeightVImg = sum(SUB_HEIGHT_V_IMG,2);
                    avgIntensity = sum(profilesHeightVImg) / size(profilesHeightVImg, 1);
                    profilesHeightVImg = profilesHeightVImg / avgIntensity; 
                    
                    diffHeightVImgs = profilesHeightVImg - preProfilesHeightVImg;
                    
                    
                    figure(1)
                                       
                    % draw odo map
                    subplot(1, 1, 1, 'replace');
                    hold on
                    plot3((odoMapTrajectory(:,2))* ODO_MAP_X_SCALING , ...
                        (odoMapTrajectory(:,1)) * ODO_MAP_Y_SCALING, ...
                        (odoMapTrajectory(:,3) ) * ODO_MAP_Z_SCALING, '.b');
                    
%                     plot3((gt_x(1:curFrame) - gt_x(1)) * GT_ODO_X_SCALING, ...
%                         (gt_y(1:curFrame) - gt_y(1)) * GT_ODO_Y_SCALING, ...
%                         (gt_z(1:curFrame) - gt_z(1)) * GT_ODO_Z_SCALING, '.r');
                    hold off;
                    view(3);
                    grid on;
%                     view(0, 90);
                    xl = xlabel('odo-map-x');
                    yl = ylabel('odo-map-y');
                    zlabel('odo-map-z');
                    set(xl,'Rotation',15);
                    set(yl,'Rotation',-30);
%                     title('Multilayered Odometry Map');
%                     legend('Result','Truth' ,'1');
%                     axis([-5 110 -5 100 -2 5]);
                    axis on
                    rotate3d on
                    
                    % draw odo map
%                     subplot(1, 2, 2, 'replace');
%                     hold on
%                     plot3((odoMapTrajectory(:,2))* ODO_MAP_X_SCALING , ...
%                         (odoMapTrajectory(:,1)) * ODO_MAP_Y_SCALING, ...
%                         (odoMapTrajectory(:,3) ) * ODO_MAP_Z_SCALING, '.b');
%                     
% %                     plot3((gt_x(1:curFrame) - gt_x(1)) * GT_ODO_X_SCALING, ...
% %                         (gt_y(1:curFrame) - gt_y(1)) * GT_ODO_Y_SCALING, ...
% %                         (gt_z(1:curFrame) - gt_z(1)) * GT_ODO_Z_SCALING, '.r');
%                     hold off;
%                     grid on;
%                     view(0, 90);
%                     xlabel('odo-map-x');
%                     ylabel('odo-map-y');
%                     zlabel('odo-map-z');
% %                     title('Multilayered Odometry Map (top view)');
% %                     legend('Result','Truth' ,'1');
% %                      axis([-5 110 -5 100]);
%                     axis on
%                     
                    
                    figure(2)
                    % render the horizontal translational velocity
                    subplot(1, 1, 1, 'replace');
                    plot(subRotVel);
%                     title('Yaw rotational velocity');
                    xlabel('Time');
                    ylabel('vYawRot');
                    axis on
                    
                    figure(3)
                     % render current rot direction 
                    subplot(1, 1, 1, 'replace');
                    polar(theta,rho);
                    hold on
                    plot([startPoint(1) curRotDir(curFrame,1)],[startPoint(2) curRotDir(curFrame,2)],'linewidth',1,'color',[0.9 0 0]);
%                     title('Current rotational direction');
                    
                    figure(4)
                    % render the sub image for estimating horizontal translational velocity
                    subplot(3, 4, 1, 'replace');
                    imshow(SUB_TRANS_IMG, []);
                    title('Sub image (vHoriTrans)');
                    xlabel('Width');
                    ylabel('Height');
                    axis on

                    % draw the the normalized profile of horizontal translational image
                    subplot(3,4,5, 'replace');
                    hold on
                    plot(1: size(profilesTransImg,2),profilesTransImg, 'r');
                    plot(1: size(preProfilesTransImg,2),preProfilesTransImg, 'g');
                    hold off
%                     legend('curr', 'g: prev');
                    title('The intensity profiles');
                    xlabel('Width');
                    ylabel('Normalized profile');
                    axis on

                    % render the horizontal translational velocity
                    subplot(3, 4, 9, 'replace');
                    plot(transVelVector);
                    title('Translational velocity');
                    xlabel('Time');
                    ylabel('vTrans');
                    axis on
                    
                    % render the sub image for estimating rotational velocity
                    subplot(3, 4, 2, 'replace');
                    imshow(SUB_YAW_ROT_IMG, []);
                    title('Sub image (vYawRot)');
                    xlabel('Width');
                    ylabel('Height');
                    axis on

                   
                    % draw the the normalized profile of rotational image
                    subplot(3,4,6, 'replace');
                    hold on 
                    plot(1: size(profilesYawRotImg,2),profilesYawRotImg, 'r');
                    plot(1: size(preProfilesYawRotImg,2),preProfilesYawRotImg, 'g');
                    hold off
%                     legend('r: curr', 'g: prev');
                    title('The intensity profiles');
                    xlabel('Width');
                    ylabel('Normalized profile');
                    axis on

                    
                    
                                        % render the sub image for estimating pitch velocity
                    subplot(3, 4, 3, 'replace');
                    imshow(SUB_HEIGHT_V_IMG, []);
                    title('Sub image (vPitchRot)');
                    xlabel('Width');
                    ylabel('Height');
                    axis on
                    
                    % draw the the normalized profile of height v image
                    subplot(3,4,7, 'replace');
                    hold on 
                    plot(profilesHeightVImg, 1: size(profilesHeightVImg,1), 'r');
                    plot(preProfilesHeightVImg, 1: size(preProfilesHeightVImg,1), 'g');
                    hold off
%                     legend('r: curr', 'g: prev');
                    title('The intensity profiles');
                    xlabel('Width');
                    ylabel('Normalized profile');
                    axis on
                    
                    % render the horizontal translational velocity
                    subplot(3, 4, 11, 'replace');
                    plot(heightVelVector);
                    title('Vertical translational velocity');
                    xlabel('Time');
                    ylabel('vVertTrans');
                    axis on
                    
                   

                    % render the horizontal translational velocity
                    subplot(3, 4, 8, 'replace');
                    plot(sumHeight);
                    title('Height Change');
                    xlabel('Time');
                    ylabel('Height');
                    axis on
                    
                    % render the horizontal translational velocity
                    subplot(3, 4, 12, 'replace');
                    hold on
                    plot(offset_yaw_rot_vector, 'r');
                    plot(offset_height_V_vector, 'g');
                    hold off
                    title('Offset yaw rotation');
                    xlabel('Time');
                    ylabel('Offset');
                    axis on                   
                    
%                     figure(3) 
%                     
%                      subplot(1,1,1, 'replace');
%                     hold on 
%                     plot(1: size(diffHeightVImgs,1),diffHeightVImgs, 'r');
%                     hold off
% %                     legend('r: curr', 'g: prev');
%                     title('The diff between two intensity profiles');
%                     xlabel('Width');
%                     ylabel('Diff');
%                     axis on
                    
                    drawnow
                end
            end
        end
    end
end
