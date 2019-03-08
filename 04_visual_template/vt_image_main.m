function vt_image_main(visualDataFile, imgType, varargin)
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

    %% initial some variables
    % define a variable of visual template
    global VT; 
        
    % define the number of visual templates
    global NUM_VT;
    NUM_VT = 1;
    
    % define a variable of previous vt id
    global PREV_VT_ID;
    PREV_VT_ID = 1;
    
    % define history of visual template
    global VT_HISTORY;
    global VT_HISTORY_FIRST;
    global VT_HISTORY_OLD;
 
    VT_HISTORY = [0];
    
    % for showing the comparing result between current imgs with templates
    global MIN_DIFF_CURR_IMG_VTS;
    
    MIN_DIFF_CURR_IMG_VTS = zeros(NUM_VT, 1);
    
    for indx = 1 : NUM_VT
        MIN_DIFF_CURR_IMG_VTS(indx) = 10;
    end
    
    global DIFFS_ALL_IMGS_VTS;
    DIFFS_ALL_IMGS_VTS = zeros(NUM_VT, 1);
    
    % define a variable for showing sub vt imgage
    global SUB_VT_IMG;
    
   
    
    %% Process the parameters
    % parser some pameters
    
    % define the threshold for vt matching
    % This threshold determines whether a new view template is generated
    global VT_MATCH_THRESHOLD;
    
    % define the x, y range of visual template img
    global VT_IMG_CROP_Y_RANGE; % Row range
    global VT_IMG_CROP_X_RANGE; % Column range
    
    % Define resized range of visual template img for reducing resolution
    global VT_IMG_RESIZE_X_RANGE;
    global VT_IMG_RESIZE_Y_RANGE;
       
    % define x, y shift range for template matching
    % This determines how many +- pixels (therefore rotation) will be tested for match
    global VT_IMG_X_SHIFT;
    global VT_IMG_Y_SHIFT;
        
    % define decay of vt for updating vts
    % VT_GLOBAL_DECAY is subtracted for all the view templates at each time step
    % VT_ACTIVE_DECAY is added the best matching view template
    global VT_GLOBAL_DECAY;
    global VT_ACTIVE_DECAY; 
    
     % define the patch size in x, y for patch normalisation
    global PATCH_SIZE_Y_K;
    global PATCH_SIZE_X_K;
        
    
    for i=1:(nargin - 2)
        if ischar(varargin{i})
            switch varargin{i}
                case 'VT_MATCH_THRESHOLD', VT_MATCH_THRESHOLD = varargin{i+1}; 
                
                case 'VT_IMG_CROP_Y_RANGE', VT_IMG_CROP_Y_RANGE = varargin{i+1};
                case 'VT_IMG_CROP_X_RANGE', VT_IMG_CROP_X_RANGE = varargin{i+1};
                
                case 'VT_IMG_RESIZE_X_RANGE', VT_IMG_RESIZE_X_RANGE = varargin{i+1};
                case 'VT_IMG_RESIZE_Y_RANGE', VT_IMG_RESIZE_Y_RANGE = varargin{i+1};

                case 'VT_IMG_X_SHIFT', VT_IMG_X_SHIFT = varargin{i+1};               
                case 'VT_IMG_Y_SHIFT', VT_IMG_Y_SHIFT = varargin{i+1};      
                    
                case 'VT_GLOBAL_DECAY', VT_GLOBAL_DECAY = varargin{i+1};
                case 'VT_ACTIVE_DECAY', VT_ACTIVE_DECAY = varargin{i+1};  
                                       
                case 'PATCH_SIZE_Y_K', PATCH_SIZE_Y_K = varargin{i+1};
                case 'PATCH_SIZE_X_K', PATCH_SIZE_X_K = varargin{i+1};
   
            end
        end
    end

    % define half offset for reversed matching
    global VT_IMG_HALF_OFFSET; 
    VT_IMG_HALF_OFFSET = [0 floor((VT_IMG_RESIZE_X_RANGE) / 2)];

    gc_x = 1;
    gc_y = 1;
    gc_z = 1;
    
    hdc_yaw = 0;
    hdc_pitch = 0;
    

    VT(1).id = 1;
    VT(1).template(1) = 1;
    VT(1).decay = 0.7;
    VT(1).gc_x = gc_x;
    VT(1).gc_y = gc_y;
    VT(1).gc_z = gc_z;
    VT(1).hdc_yaw = hdc_yaw;
    VT(1).hdc_pitch = hdc_pitch;
    VT(1).first = 1;           % don't want to inject energy as the vt is been created
    VT(1).numExp = 1;
    VT(1).exps(1).id = 1;
        
    VT(1).template = zeros(size(VT_IMG_RESIZE_Y_RANGE, 2), size(VT_IMG_RESIZE_X_RANGE, 2));

    %% read imgs as VT input
    
    % get the visual data info
    [subFoldersPathSet, numSubFolders] = get_images_data_info(visualDataFile);

%  iCurImg = 0 ;
    for iSubFolder = 1 :numSubFolders
        
%         numTotalImg = numTotalImg + numImgs;
        [curFolderPath, imgFilesPathList, numImgs] = get_cur_img_files_path_list(subFoldersPathSet, imgType, iSubFolder);
        
        if numImgs > 0
            for iCurImg = 1 :2: numImgs
%                for iCurImg = 1 :numImgs 
%                 imgName = filePathList(iCurImg).name;
%                 img =  imread(strcat(filePath,imgName));
                [curImg] = read_current_image(curFolderPath, imgFilesPathList, iCurImg);

                % visual templates and visual odo uses intensity so convert to grayscale
                rawImg = rgb2gray(curImg);
                rawImg = im2double(rawImg);
%                 imgResized = imresize(rawImg, [VT_IMG_RESIZE_Y_RANGE VT_IMG_RESIZE_X_RANGE]);

                % get the most active view template
                [vt_id] = visual_template(rawImg, gc_x, gc_y,gc_z, hdc_yaw, hdc_pitch);
                
                % render the raw img
                subplot(3, 3, 1, 'replace');
                imshow(curImg, []);
                title('Raw img');
                axis on

                % render the history of visual templates
                subplot(3, 3, [2 3 5 6], 'replace');
                plot(VT_HISTORY, '.r');
                title('Frame vs View Template');
                axis on

                % render the resized image of visual templates
                subplot(3, 3, 4, 'replace');
                imshow(SUB_VT_IMG, []);
                title('Resized img');
                axis on

                % render the history of visual templates
                subplot(3, 3, 7, 'replace');
                plot(MIN_DIFF_CURR_IMG_VTS, '.');
                title('Min SAD with each template');
                axis on

                % render the history of visual templates
                subplot(3, 3, [8 9], 'replace');
                plot(DIFFS_ALL_IMGS_VTS, '.');
                title('Min SAD of each current img');
                axis on

%                 DIFFS_ALL_IMGS_VTS = vt_id;

                drawnow; 
             end
        end
    end
    
end