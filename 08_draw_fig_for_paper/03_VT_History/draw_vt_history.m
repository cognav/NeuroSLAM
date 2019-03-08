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


% % SynPerData
% vtHistoryFile = 'C:\NeuroSLAM_Datasets\03_NeuroSLAM_Experiments_Results\SynPerData\04_vt_history_ml.txt';

% % SynPanData
% vtHistoryFile = 'C:\NeuroSLAM_Datasets\03_NeuroSLAM_Experiments_Results\SynPanData\04_vt_history_ml.txt';

% QUTCarparkData
vtHistoryFile = 'C:\NeuroSLAM_Datasets\03_NeuroSLAM_Experiments_Results\QUTCarparData\04_vt_history_ml.txt';

plot_vt_history(vtHistoryFile);
