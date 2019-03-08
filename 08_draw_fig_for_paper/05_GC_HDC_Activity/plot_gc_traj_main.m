function plot_gc_traj_main(gcTrajFile)
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
    [id, gc_x, gc_y, gc_z] = load_gc_traj_data(gcTrajFile);
    
    
    subplot(1, 1, 1,'replace');
    
    % draw cube
    edge1_x = [0 0];
    edge1_y = [0 0];
    edge1_z = [0 36];

    edge2_x = [0 0];
    edge2_y = [0 36];
    edge2_z = [0 0];

    edge3_x = [0 36];
    edge3_y = [0 0];
    edge3_z = [0 0];

    edge4_x = [0 36];
    edge4_y = [36 36];
    edge4_z = [0 0];

    edge5_x = [0 0];
    edge5_y = [36 36];
    edge5_z = [0 36];

    edge6_x = [36 36];
    edge6_y = [0 36];
    edge6_z = [0 0];

    edge7_x = [36 36];
    edge7_y = [0 0];
    edge7_z = [0 36];

    edge8_x = [0 0];
    edge8_y = [0 36];
    edge8_z = [36 36];

    edge9_x = [0 36];
    edge9_y = [0 0];
    edge9_z = [36 36];

    edge10_x = [0 36];
    edge10_y = [36 36];
    edge10_z = [36 36];

    edge11_x = [36 36];
    edge11_y = [0 36];
    edge11_z = [36 36];

    edge12_x = [36 36];
    edge12_y = [36 36];
    edge12_z = [0 36];

    plot3(edge1_x,edge1_y,edge1_z, '-k');
    hold on
    plot3(edge2_x,edge2_y,edge2_z, '-k');
    hold on
    plot3(edge3_x,edge3_y,edge3_z, '-k');
    hold on
    plot3(edge4_x,edge4_y,edge4_z, '-k');
    hold on
    plot3(edge5_x,edge5_y,edge5_z, '-k');
    hold on
    plot3(edge6_x,edge6_y,edge6_z, '-k');
    hold on
    plot3(edge7_x,edge7_y,edge7_z, '-k');
    hold on
    plot3(edge8_x,edge8_y,edge8_z, '-k');
    hold on
    plot3(edge9_x,edge9_y,edge9_z, '-k');
    hold on
    plot3(edge10_x,edge10_y,edge10_z, '-k');
    hold on
    plot3(edge11_x,edge11_y,edge11_z, '-k');
    hold on
    plot3(edge12_x,edge12_y,edge12_z, '-k');

    view(20,20)

    
    % draw gc trajectory

    hold on;
    plot3(gc_y, gc_x, gc_z, '.r', 'MarkerSize',6);

    hold off;
    xl = xlabel('y', 'FontSize',24);
    yl = ylabel('x', 'FontSize',24);
    z1 = zlabel('z', 'FontSize',24);
    view(33,12)
    
%     set(xl,'Rotation',15);
    set(yl,'Rotation',30);
     
    axis([0 36 0 36 0 36]);
     
    set(gca,'xtick',0:9:36)  
    set(gca,'ytick',0:18:36) 
    set(gca,'ztick',0:9:36) 
    set(gca,'FontSize',24, 'LineWidth',1.5); % axis font
    
    fig = get(groot,'CurrentFigure');
    set (fig,'Position',[500,300,400,300], 'color','w')
end