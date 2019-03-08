function [transV, yawRotV, heightV] = visual_odometry_down(rawImg)
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
   
    % The simple visual odometry with scanline intensity profile algorithm.
    % the input is a raw image
    % the output including horizontal translational velocity, rotational velocity,
    % vertical translational velocity (vertical)


    %% start to set up the visual odometry 

    % define the veriable for drawing sub images used by estimating translational, 
    % rotational, and pitch velocity 
    global SUB_TRANS_IMG;
    global SUB_YAW_ROT_IMG;
    global SUB_HEIGHT_V_IMG;

          
    % definition the Y (vertical) range of images for odomentry, including image for
    % translational velocity, image for rotational velocity, and image for
    % height change velocity
    global ODO_IMG_HEIGHT_V_Y_RANGE;
    global ODO_IMG_YAW_ROT_Y_RANGE;

    % definition of X (horizontal) range of the image for odometry
    global ODO_IMG_YAW_ROT_X_RANGE;
    global ODO_IMG_HEIGHT_V_X_RANGE;

    global ODO_IMG_TRANS_Y_RANGE;
    global ODO_IMG_TRANS_X_RANGE;
    
    % define the size of resized images for odo
    global ODO_IMG_YAW_ROT_RESIZE_RANGE;
    global ODO_IMG_HEIGHT_V_RESIZE_RANGE;
    global ODO_IMG_TRANS_RESIZE_RANGE;
   
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
          
    % x_sums, sum each column of intensity in the current image and previous
    % image perspectively.
    % form one dimensional vector, 1*N array
    global PREV_TRANS_V_IMG_X_SUMS;
    global PREV_YAW_ROT_V_IMG_X_SUMS;
    global PREV_HEIGHT_V_IMG_Y_SUMS;
    
    % define the previous velocity for keeping stable speed
    global PREV_TRANS_V;
    global PREV_YAW_ROT_V;
    global PREV_HEIGHT_V;
    
    global DEGREE_TO_RADIAN;
    
    %%% End up for setting up the visual odometry 
    
    global OFFSET_YAW_ROT;
    global OFFSET_HEIGHT_V;
    
    %% start to compute the horizontal rotational velocity (yaw)

    % get the sub_image for rotational velocity from raw image with range constrait
    subRawImg = rawImg(ODO_IMG_YAW_ROT_Y_RANGE, ODO_IMG_YAW_ROT_X_RANGE);
    subRawImg = imresize(subRawImg, ODO_IMG_YAW_ROT_RESIZE_RANGE); 
    horiDegPerPixel = FOV_HORI_DEGREE / size(subRawImg, 2);
    
    SUB_YAW_ROT_IMG = subRawImg;
    SUB_TRANS_IMG = subRawImg;
    
%     % get the size of template image after resized 
%     ySizeODOImg = ODO_IMG_YAW_ROT_RESIZE_RANGE(1);
%     xSizeODOImg = ODO_IMG_YAW_ROT_RESIZE_RANGE(2);
%     ySizeNormImg = ySizeODOImg;
%     
%     PATCH_SIZE_Y_K = 5;
%     PATCH_SIZE_X_K = 5;
%     
%     % define a temp variable for patch normalization
%     % extent the dimension of raw image for patch normalization (extODOImg, extension sub image of vtResizedImg)
%     extODOImg = zeros(ySizeODOImg + PATCH_SIZE_Y_K - 1, xSizeODOImg + PATCH_SIZE_X_K - 1);
%     extODOImg(fix((PATCH_SIZE_Y_K + 1 )/2) : fix((PATCH_SIZE_Y_K + 1 )/2) ...
%         + ySizeNormImg - 1 ,  fix((PATCH_SIZE_X_K + 1 )/2) : fix((PATCH_SIZE_X_K + 1 )/2) ...
%         + xSizeODOImg - 1 ) = subRawImg;
%     
%     %% patch normalisation is applied to compensate for changes in lighting condition
%     for v = 1: ySizeNormImg 
%         for u = 1 : xSizeODOImg 
%             % get patch image
%             patchImg = extODOImg(v : v + PATCH_SIZE_Y_K - 1, u : u + PATCH_SIZE_X_K -1);        
% 
%             % Find the average of the matrix patch image
%             meanPatchImg = mean2(patchImg);
% 
%             % Find the standard deviation of the matrix patch image
%             stdPatchIMG = std2(patchImg);
%             
%             % Normalize the sub raw image
%             % normODOImg(v,u) = 11 * 11 * (vtResizedImg(v,u) - meanPatchImg ) / stdPatchIMG ;
%             % normODOImg(v,u) =  PATCH_SIZE_Y_K * PATCH_SIZE_X_K * (vtResizedImg(v,u) - meanPatchImg ) / stdPatchIMG ;
%             normODOImg(v,u) =  (subRawImg(v,u) - meanPatchImg ) / stdPatchIMG/ 255;
%             
%         end
%     end
%     
%     SUB_YAW_ROT_IMG = normODOImg;
%     SUB_TRANS_IMG = normODOImg;
    
    % get the x_sum of average sum intensity values in every column of image
    imgXSums = sum(subRawImg);
    avgIntensity = sum(imgXSums) / size(imgXSums, 2);
    imgXSums = imgXSums / avgIntensity;

    % compare the current image with the previous image
    % get the minimum offset and minimum difference of intensity between two images 
    [minOffsetYawRot, minDiffIntensityRot] = compare_segments(imgXSums, PREV_YAW_ROT_V_IMG_X_SUMS, ODO_SHIFT_MATCH_HORI, size(imgXSums, 2));  

    OFFSET_YAW_ROT = minOffsetYawRot;
    yawRotV = ODO_YAW_ROT_V_SCALE * minOffsetYawRot * horiDegPerPixel;  % in deg

    if abs(yawRotV) > MAX_YAW_ROT_V_THRESHOLD
        yawRotV = PREV_YAW_ROT_V;
    else
        PREV_YAW_ROT_V = yawRotV;
    end

    PREV_YAW_ROT_V_IMG_X_SUMS = imgXSums;
    PREV_TRANS_V_IMG_X_SUMS = imgXSums;
    %%% end up to compute the translational velocity
    
   
    %% start to compute total translational velocity

    % principle
    % speeds are estimates based on the rate of image change. 
    % the speed measure v is obtained from the filtered average absolute
    % intensity difference between consecutive scanline intensity profiles at
    % the best match for rotation with best offest in yaw and pitch shift
 
    transV = minDiffIntensityRot * ODO_TRANS_V_SCALE;
    
    
    if transV > MAX_TRANS_V_THRESHOLD
       transV = PREV_TRANS_V;
    else
       PREV_TRANS_V = transV;
    end
    
    
    %% start to compute the height change velocity

    % get the sub_image for pitch velocity from raw image with range constrait
    subRawImg = rawImg(ODO_IMG_HEIGHT_V_Y_RANGE, ODO_IMG_HEIGHT_V_X_RANGE);
    subRawImg = imresize(subRawImg, ODO_IMG_HEIGHT_V_RESIZE_RANGE); 
    vertDegPerPixel = FOV_VERT_DEGREE / size(subRawImg, 1);
    

    if minOffsetYawRot > 0
        subRawImg = subRawImg(:, minOffsetYawRot + 1 : end);
    else
        subRawImg = subRawImg(:, 1 : end -(-minOffsetYawRot));
    end

    SUB_HEIGHT_V_IMG = subRawImg;

    imageYSums = sum(subRawImg,2);
    avgIntensity = sum(imageYSums) / size(imageYSums, 1);
    imageYSums = imageYSums / avgIntensity; 

    [minOffsetHeightV, minDiffIntensityHeight] = compare_segments(imageYSums, PREV_HEIGHT_V_IMG_Y_SUMS, ODO_SHIFT_MATCH_VERT, size(imageYSums, 1));
    
    
    if minOffsetHeightV < 0
        minDiffIntensityHeight = - minDiffIntensityHeight;
    end
    
    OFFSET_HEIGHT_V = minOffsetHeightV;

    % covert the perceptual speed into a physical speed with an empirically
    % determined constant TRANSLATIONAL_VELOCITY_SCALE

%     if abs(minOffsetYawRot) < 2
%         heightV = ODO_HEIGHT_V_SCALE * minDiffIntensityHeight;
%     else 
%         heightV = 0;
%     end


%     if abs(minOffsetYawRot) < 2
%         heightV = minOffsetHeightV * 0.1;
%     else 
%         heightV = 0;
%     end
%     
%     if abs(minOffsetHeightV) < 2
%         heightV = 0;
%     end
    if minOffsetHeightV < 0
        heightV = ODO_HEIGHT_V_SCALE * minDiffIntensityHeight;
    elseif PREV_HEIGHT_V < 0
        heightV = PREV_HEIGHT_V;
    else
        heightV = 0;
    end
    
    if abs(heightV) > MAX_HEIGHT_V_THRESHOLD
        heightV = PREV_HEIGHT_V;
    else
        PREV_HEIGHT_V = heightV;
    end
    
%   if abs(minOffsetHeightV) > 1
%         heightV = ODO_HEIGHT_V_SCALE * minDiffIntensityHeight;
%             
%     else 
%         heightV = 0;
%   end
%     
%     if abs(minOffsetHeightV) > 1 && abs(minOffsetYawRot) < 2
%         heightV = ODO_HEIGHT_V_SCALE * minDiffIntensityHeight;
%             
%     else 
%         heightV = 0;
%     end
    
    % to detect excessively large translational velocity
    % the threshold Vmax ensured that spuriously high image differences were
    % not used. Large image differences could be caused by sudden illumination
    % changes such as when travelling uphill facing directly into the sun.

    % define a maximum velocity threshold according to the average motion speed

   
    
%     if abs(minOffsetHeightV) >3  
%         transV = 0;
%     end
    
    PREV_HEIGHT_V_IMG_Y_SUMS = imageYSums;
    %%% end up to compute the pitch velocity
    
end
