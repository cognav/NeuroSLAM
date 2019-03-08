function create_new_exp(curExpId, newExpId, vt_id, xGc, yGc, zGc, curYawHdc, curHeight)
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

    % create a new experience and current experience to it
    global VT;
    global EXPERIENCES;
    global ACCUM_DELTA_X;
    global ACCUM_DELTA_Y;
    global ACCUM_DELTA_Z;
    global ACCUM_DELTA_YAW; 
%     global ACCUM_DELTA_HEIGHT; 
    
    % add link information to the current experience for the new experience
    % including the experience_id, odo distance to the experience, 
    % odo heading (relative to the current experience's facing) to the experience, 
    % odo delta facing (relative to the current expereience's facing).
    
    EXPERIENCES(curExpId).numlinks = EXPERIENCES(curExpId).numlinks + 1;
    EXPERIENCES(curExpId).links(EXPERIENCES(curExpId).numlinks).exp_id = newExpId;
    
    EXPERIENCES(curExpId).links(EXPERIENCES(curExpId).numlinks).d_xy = sqrt(ACCUM_DELTA_X^2 + ACCUM_DELTA_Y^2);
    EXPERIENCES(curExpId).links(EXPERIENCES(curExpId).numlinks).d_z = ACCUM_DELTA_Z;
    
    EXPERIENCES(curExpId).links(EXPERIENCES(curExpId).numlinks).heading_yaw_exp_rad = ... 
        get_signed_delta_radian(EXPERIENCES(curExpId).yaw_exp_rad, -atan2(ACCUM_DELTA_Y, ACCUM_DELTA_X)); % heading is the delta angle between current pose of exp and previous pose of exp
        
    EXPERIENCES(curExpId).links(EXPERIENCES(curExpId).numlinks).facing_yaw_exp_rad = ... % facing is the direction of each exp
        get_signed_delta_radian(EXPERIENCES(curExpId).yaw_exp_rad, ACCUM_DELTA_YAW);
    
    % create the new experience which will have no links to being with
    % associate with 3d gc
    EXPERIENCES(newExpId).x_gc = xGc;
    EXPERIENCES(newExpId).y_gc = yGc;
    EXPERIENCES(newExpId).z_gc = zGc;
    
    % associate with hdc
    EXPERIENCES(newExpId).yaw_hdc = curYawHdc;
    EXPERIENCES(newExpId).height_hdc = curHeight;
    
    % associate with vt
    EXPERIENCES(newExpId).vt_id = vt_id;
    
    % update the coordinate of em (x_exp, y_exp, z_exp, yaw_exp_rad, height_exp)
    EXPERIENCES(newExpId).x_exp = EXPERIENCES(curExpId).x_exp + ACCUM_DELTA_X;
    EXPERIENCES(newExpId).y_exp = EXPERIENCES(curExpId).y_exp + ACCUM_DELTA_Y;
    EXPERIENCES(newExpId).z_exp = EXPERIENCES(curExpId).z_exp + ACCUM_DELTA_Z;
    
    EXPERIENCES(newExpId).yaw_exp_rad = clip_radian_180(ACCUM_DELTA_YAW);
%     EXPERIENCES(newExpId).height_exp = ACCUM_DELTA_HEIGHT;
    
    EXPERIENCES(newExpId).numlinks = 0;
    EXPERIENCES(newExpId).links = [];

    % add this experience id to the vt for efficient lookup
    VT(vt_id).numExp = VT(vt_id).numExp + 1;
    VT(vt_id).EXPERIENCES(VT(vt_id).numExp).id = newExpId;
end

