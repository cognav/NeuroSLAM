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


% Jan 3, 2019
% groundTruthFile = 'C:\Dataset\3dratslam_groud_truth\01_ground_truth_data\03_3d_ground_truth_same_view_perspective_0918.txt';
odoMapFile = 'C:\NeuroSLAM_Datasets\03_NeuroSLAM_Experiments_Results\QUTCarparData\02_odo_map_ml.txt';
%     plot3(odomap_y*1*(-1), odomap_x*1+3, odomap_z*(1), '.b');
%     plot3((gt_x - gt_x(1))*17,(gt_y - gt_y(1))*23, (gt_z - gt_z(1))*(10), '.r');            


plot_3d_odomap_qut(groundTruthFile, odoMapFile, 0.8, -0.8, 0.1, 0, 0.5, 0, 20,20, 20);