function [vt_id] = visual_template(rawImg, x, y,z, yaw, height)
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
    
    %% start to define some variables for visual template
    
    % define a variable of visual template
    global VT; 
        
    % define the number of visual templates
    global NUM_VT;
    
    % define history of visual template
    global VT_HISTORY;
    global VT_HISTORY_FIRST;
    global VT_HISTORY_OLD;
    
    % define a variable of previous vt id
    global PREV_VT_ID;
    
    % define the x, y range of visual template image
    global VT_IMG_CROP_Y_RANGE; % Row range
    global VT_IMG_CROP_X_RANGE; % Column range
       
    % define x, y shift range for template matching
    % This determines how many +- pixels (therefore rotation) will be tested for match
    global VT_IMG_X_SHIFT;
    global VT_IMG_Y_SHIFT;
    
    % define half offset for reversed matching
    global VT_IMG_HALF_OFFSET; 
    
    % define the threshold for vt matching
    % This threshold determines whether a new view template is generated
    global VT_MATCH_THRESHOLD;
        
    % define decay of vt for updating vts
    % VT_GLOBAL_DECAY is subtracted for all the view templates at each time step
    % VT_ACTIVE_DECAY is added the best matching view template
    global VT_GLOBAL_DECAY;
    global VT_ACTIVE_DECAY; 

    % for showing the comparing result between current images with templates
    global MIN_DIFF_CURR_IMG_VTS;
    global DIFFS_ALL_IMGS_VTS;
    
    % define a variable for showing sub vt imgage
    global SUB_VT_IMG;
    
    % define the patch size in x, y for patch normalisation
    global PATCH_SIZE_Y_K;
    global PATCH_SIZE_X_K;
        
    % Define resized range of visual template image for reducing resolution
    global VT_IMG_RESIZE_X_RANGE;
    global VT_IMG_RESIZE_Y_RANGE;
    
    % Define panoramic camera 
    global VT_PANORAMIC;
    %%% end to define some variables for visual template
    
    %% start to initial some variables
    
    % resize the raw image with constrait range
    subImg = rawImg(VT_IMG_CROP_Y_RANGE, VT_IMG_CROP_X_RANGE);
    vtResizedImg = imresize(subImg, [VT_IMG_RESIZE_Y_RANGE VT_IMG_RESIZE_X_RANGE]);

    % get the size of template image after resized 
    ySizeVtImg = VT_IMG_RESIZE_Y_RANGE;
    xSizeVtImg = VT_IMG_RESIZE_X_RANGE;
    ySizeNormImg = ySizeVtImg;
    
    % define a temp variable for patch normalization
    % extent the dimension of raw image for patch normalization (extVtImg, extension sub image of vtResizedImg)
    extVtImg = zeros(ySizeVtImg + PATCH_SIZE_Y_K - 1, xSizeVtImg + PATCH_SIZE_X_K - 1);
    extVtImg(fix((PATCH_SIZE_Y_K + 1 )/2) : fix((PATCH_SIZE_Y_K + 1 )/2) ...
        + ySizeNormImg - 1 ,  fix((PATCH_SIZE_X_K + 1 )/2) : fix((PATCH_SIZE_X_K + 1 )/2) ...
        + xSizeVtImg - 1 ) = vtResizedImg;
    
    %% patch normalisation is applied to compensate for changes in lighting condition
    for v = 1: ySizeNormImg 
        for u = 1 : xSizeVtImg 
            % get patch image
            patchImg = extVtImg(v : v + PATCH_SIZE_Y_K - 1, u : u + PATCH_SIZE_X_K -1);        

            % Find the average of the matrix patch image
            meanPatchImg = mean2(patchImg);

            % Find the standard deviation of the matrix patch image
            stdPatchIMG = std2(patchImg);
            
            % Normalize the sub raw image
            % normVtImg(v,u) = 11 * 11 * (vtResizedImg(v,u) - meanPatchImg ) / stdPatchIMG ;
            % normVtImg(v,u) =  PATCH_SIZE_Y_K * PATCH_SIZE_X_K * (vtResizedImg(v,u) - meanPatchImg ) / stdPatchIMG ;
            normVtImg(v,u) =  (vtResizedImg(v,u) - meanPatchImg ) / stdPatchIMG/ 255;
            
        end
    end

    % when partial normalization, the following can combine the normalized
    % part img to the un-normalized part
    % combNormImgUnNormImg combNormImgUnNormImg(1: ySizeNormImg, :) = vtResizedImg(1:ySizeNormImg, :);
    % combNormImgUnNormImg (1:ySizeVtImg, :) = normVtImg;

    % for drawing normalized vt image
    SUB_VT_IMG = normVtImg;
    
    
    
    % processing the first 20 visual template
    % add the image in to vt directly
    if NUM_VT < 5
        VT(NUM_VT).decay = VT(NUM_VT).decay - VT_GLOBAL_DECAY;
        if VT(NUM_VT).decay < 0
            VT(NUM_VT).decay = 0;
        end
        
        NUM_VT = NUM_VT + 1;
        VT(NUM_VT).id = NUM_VT;
        VT(NUM_VT).template = normVtImg;
        VT(NUM_VT).decay = VT_ACTIVE_DECAY;
        VT(NUM_VT).gc_x = x;
        VT(NUM_VT).gc_y = y;
        VT(NUM_VT).gc_z = z;
        VT(NUM_VT).hdc_yaw = yaw;
        VT(NUM_VT).hdc_height = height;
        VT(NUM_VT).first = 1;           % don't want to inject energy as the vt is been created
        VT(NUM_VT).numExp = 0;
        VT(NUM_VT).exps = [];
        vt_id = NUM_VT;
        VT_HISTORY_FIRST = [VT_HISTORY_FIRST; vt_id];
    else
        
        for k = 2:NUM_VT
            
            VT(k).decay = VT(k).decay - VT_GLOBAL_DECAY;
            
            if VT(k).decay < 0
                VT(k).decay = 0;
            end
            
            [minOffsetY(k), minOffsetX(k), MIN_DIFF_CURR_IMG_VTS(k)] = vt_compare_segments(normVtImg, VT(k).template,   ...
            VT_PANORAMIC, VT_IMG_HALF_OFFSET, VT_IMG_Y_SHIFT, VT_IMG_X_SHIFT, size(normVtImg, 1),size(normVtImg, 2) );
        
        end

        [diff, diff_id] = min(MIN_DIFF_CURR_IMG_VTS);
        DIFFS_ALL_IMGS_VTS = [DIFFS_ALL_IMGS_VTS; diff];
        
        % if this intensity template doesn't match any of the existing templates
        % then create a new template

        if (diff > VT_MATCH_THRESHOLD)
            NUM_VT = NUM_VT + 1;
            VT(NUM_VT).id = NUM_VT;
            VT(NUM_VT).template = normVtImg;
            VT(NUM_VT).decay = VT_ACTIVE_DECAY;
            VT(NUM_VT).gc_x = x;
            VT(NUM_VT).gc_y = y;
            VT(NUM_VT).gc_z = z;
            VT(NUM_VT).hdc_yaw = yaw;
            VT(NUM_VT).hdc_height = height;
            VT(NUM_VT).first = 1;           % don't want to inject energy as the vt is been created
            VT(NUM_VT).numExp = 0;
            VT(NUM_VT).exps = [];
            vt_id = NUM_VT;
            VT_HISTORY_FIRST = [VT_HISTORY_FIRST; vt_id];
  
        else
            vt_id = diff_id;
            
            VT(vt_id).decay = VT(vt_id).decay + VT_ACTIVE_DECAY;
            
            if PREV_VT_ID ~= vt_id
                VT(vt_id).first = 0;
            end
            
            VT_HISTORY_OLD = [VT_HISTORY_OLD; vt_id];
        end
    end
    
    VT_HISTORY = [VT_HISTORY; vt_id];

end