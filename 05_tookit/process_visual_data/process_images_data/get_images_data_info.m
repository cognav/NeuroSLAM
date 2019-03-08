function [subFoldersPathSet,numSubFolders] = get_images_data_info(visualDataFile)
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
    
    % eg.  
    % the upper path:  */imgsData
    % the sub folder 1: */imgsData/subFolder1
    % the sub folder 2: */imgsData/subFolder2
    % the sub folder X: */imgsData/subFolderX
    
    % get the root (upper) path of all sub folders 
    rootPathOfData = visualDataFile;
    
    % get all sub folders path as a list
    subFoldersPathList = genpath(rootPathOfData);
    lenSubFolderPathList = size(subFoldersPathList,2);
    
    % all sub folders path set 
    subFoldersPathSet = {};
    temp = [];

    % parser the sub folder path from a path string
    for i = 1 : lenSubFolderPathList 
        if subFoldersPathList(i) ~= ';'
            temp = [temp subFoldersPathList(i)];
        else 
            temp = [temp '\']; 
            subFoldersPathSet = [subFoldersPathSet ; temp];
            temp = [];
        end
    end  
    
    % get the num of all sub folder paths
    numSubFolders = size(subFoldersPathSet,1);   

end


