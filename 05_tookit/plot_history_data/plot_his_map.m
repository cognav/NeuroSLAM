function plot_his_map(groundTruthFile, expMapFile)
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

    % load ground truth data
    [frameId, gt_x, gt_y, gt_z, gt_rx, gt_ry, gt_rz] = load_ground_truth_data(groundTruthFile);

    % load experience map data
    [expId, exp_x, exp_y, exp_z, exp_yaw, exp_pitch] = load_exp_map_data(expMapFile); 

    hold on
    plot3(exp_x*1*(100), exp_y*100, exp_z*(-200), '.b');
    plot3((gt_x - gt_x(1))*100, (gt_y - gt_y(1))*100, (gt_z - gt_z(1))*(100), '.r');            
    hold off;
    % grid on
    view(3)
    xl = xlabel('exp-x');
    yl = ylabel('exp-y');
    zlabel('exp-z');
    set(xl,'Rotation',15);
    set(yl,'Rotation',-30);
    title('3D Experience Map');
    %                     legend('Result','Truth' ,'1');
    % axis([-10 20 -10 20 -10 20]);
    %                     axis equal                    
    % axis on
    rotate3d on

end
