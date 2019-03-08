function plot_3d_odomap_qut(groundTruthFile, odoMapFile, ...
    xOdoMapScaling, yOdoMapScaling, zOdoMapScaling, ...
    xOdoMapTrans, yOdoMapTrans, zOdoMapTrans, ...
    xGtScaling, yGtScaling, zGtScaling)
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
%     [frameId, gt_x, gt_y, gt_z, gt_rx, gt_ry, gt_rz] = load_ground_truth_data(groundTruthFile);

    % load experience map data
    [id, odomap_x, odomap_y, odomap_z, odomap_yaw] = load_exp_map_data(odoMapFile); 

    figure('color',[1 1 1]);
    hold on
    
    plot3(odomap_y * yOdoMapScaling + yOdoMapTrans, ...
        odomap_x * xOdoMapScaling + xOdoMapTrans, ...
        odomap_z * zOdoMapScaling + zOdoMapTrans, '.b');
%     plot3((gt_x - gt_x(1)) * xGtScaling,(gt_y - gt_y(1)) * yGtScaling, (gt_z - gt_z(1)) * zGtScaling, '.r'); 
    hold off;

    view(3)
% view(0, 90);
    grid on
%     xlabel('x');
%     ylabel('y');
    xl = xlabel('x','FontSize',18);
    yl = ylabel('y','FontSize',18);
    zlabel('z','FontSize',18);
    set(xl,'Rotation',15);
    set(yl,'Rotation',-30);
%     title('3D Experience Map');
    %                     legend('Result','Truth' ,'1');
    % SynPanData
%     axis([-5 10 -10 5 -1 3]);

% SynPerData
% %     axis([-5 20 -20 5 -1 3]);

% qutdata
        axis([-10 48 -40 40 -0.5 3]);
    %                     axis equal    
%     legend('Experience map','Ground truth', 'location', '0');
    hh = legend('OM');

    set(hh,'position',[.76 .82 .05 .05]);
    set(gca,'FontSize',18); % axis font
    set (gcf,'Position',[500,300,400,300], 'color','w')

    axis on
    rotate3d on
end