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

%  2.9 -> 3.1 -> 3.2 ->3.3 -3.4 -3.5   \ best  3.2
vt_image_main('C:\NeuroSLAM_Datasets\01_NeuroSLAM_Datasets\02_SynPanData', '*.png', ...
    'VT_MATCH_THRESHOLD', 3.12 , ...
    'VT_IMG_CROP_Y_RANGE', 1:200, ...
    'VT_IMG_CROP_X_RANGE', 1:480, ...
    'VT_IMG_RESIZE_X_RANGE', 48, ...
    'VT_IMG_RESIZE_Y_RANGE', 20, ...
    'VT_IMG_X_SHIFT', 5, ...
    'VT_IMG_Y_SHIFT', 3, ...
    'VT_GLOBAL_DECAY', 0.1, ...
    'VT_ACTIVE_DECAY', 0.2, ...
    'PATCH_SIZE_Y_K', 5, ...
    'PATCH_SIZE_X_K', 5);




