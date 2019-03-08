function plot_hdc_traj_main(hdcTrajFile)
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

    % load gcTrajData
    [id, hdc_yaw, hdc_height] = load_hdc_traj_data(hdcTrajFile);
    
    
    subplot(1, 1, 1,'replace');
       
    % draw hdc trajectory

    plot(hdc_yaw, hdc_height, '.r', 'MarkerSize',6);

    xl = xlabel('yaw', 'FontSize',24);
    yl = ylabel('height', 'FontSize',24);

    axis([0 36 0 36]);
     
    set(gca,'xtick',0:9:36)  
    set(gca,'ytick',0:9:36) 
    set(gca,'FontSize',24, 'LineWidth',1); % axis font
    
    fig = get(groot,'CurrentFigure');
    set (fig,'Position',[500,300,400,300], 'color','w')
end