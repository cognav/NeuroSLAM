function exp_map_iteration(vt_id, transV, yawRotV, heightV, xGc, yGc, zGc, curYawHdc, curHeight)
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

    %% define some variables 
    
    %%% some variables of em
    
    % define a variable of experiences
    global EXPERIENCES; 
     
    % current experience id
    global CUR_EXP_ID;
   
    % the number of experiences
    global NUM_EXPS;
    
    % the min change of em between previous e with current e
    global MIN_DELTA_EM;

    % the experience delta threshold 
    global DELTA_EXP_GC_HDC_THRESHOLD;
    
    
    
    %%% some variables of vt
    
    % define a variable of visual template
    global VT; 
        
    % the previous visual template id
    global PREV_VT_ID; 
 
    
    %%% some variables of 3D GC
    
    % The x, y, z dimension of 3D Grid Cells Model (3D CAN)
    global GC_X_DIM;
    global GC_Y_DIM;
    global GC_Z_DIM;

    
    %%% some variables of 3D HDC
    
    % The dimension of yaw in yaw_height_hdc network
    global YAW_HEIGHT_HDC_Y_DIM;
    
    % The dimension of height in yaw_height_hdc network
    global YAW_HEIGHT_HDC_H_DIM;
    
    
    %%% computing delta x,y,z, yaw, height
    
    % integrate the delta x, y, z, yaw, height
    % Rad radian  accumulative
    global ACCUM_DELTA_X; 
    global ACCUM_DELTA_Y; 
    global ACCUM_DELTA_Z;  
    global ACCUM_DELTA_YAW;   % accum_delta_facing
%     global ACCUM_DELTA_HEIGHT; 
    
    ACCUM_DELTA_YAW = clip_radian_180(ACCUM_DELTA_YAW + yawRotV);
%     ACCUM_DELTA_HEIGHT = mod(ACCUM_DELTA_HEIGHT + heightV, YAW_HEIGHT_HDC_H_DIM);
%     
    ACCUM_DELTA_X = ACCUM_DELTA_X + transV * cos(ACCUM_DELTA_YAW);
    ACCUM_DELTA_Y = ACCUM_DELTA_Y + transV * sin(ACCUM_DELTA_YAW);
    ACCUM_DELTA_Z = ACCUM_DELTA_Z + heightV;
    
    % trajectory of delta of em
    global DELTA_EM;
    
    minDeltaX = get_min_delta(EXPERIENCES(CUR_EXP_ID).x_gc, xGc, GC_X_DIM);
    minDeltaY = get_min_delta(EXPERIENCES(CUR_EXP_ID).y_gc, yGc, GC_Y_DIM);
    minDeltaZ = get_min_delta(EXPERIENCES(CUR_EXP_ID).z_gc, zGc, GC_Z_DIM);
    
    minDeltaYaw = get_min_delta(EXPERIENCES(CUR_EXP_ID).yaw_hdc, curYawHdc, YAW_HEIGHT_HDC_Y_DIM);
    minDeltaHeight = get_min_delta(EXPERIENCES(CUR_EXP_ID).height_hdc, curHeight, YAW_HEIGHT_HDC_H_DIM);
    
    minDeltaYawReversed = get_min_delta(EXPERIENCES(CUR_EXP_ID).yaw_hdc, (YAW_HEIGHT_HDC_Y_DIM /2) - curYawHdc, YAW_HEIGHT_HDC_Y_DIM);
    minDeltaYaw = min(minDeltaYaw, minDeltaYawReversed);
    
%     minDeltaHeightReversed = get_min_delta(EXPERIENCES(CUR_EXP_ID).height_hdc, (YAW_HEIGHT_HDC_H_DIM /2) - curHeight, YAW_HEIGHT_HDC_H_DIM);
%     minDeltaHeight = min(minDeltaHeight, minDeltaHeightReversed);
    
    delta_em = sqrt((minDeltaX).^2 + (minDeltaY).^2 + (minDeltaZ).^2 + (minDeltaYaw).^2 + (minDeltaHeight).^2);
    DELTA_EM = [DELTA_EM delta_em];
    
    % if the visual template is new or the 3d grid cells (x,y,z) and head direction cells (yaw, height)
    % has changed enough create a new experience
    if VT(vt_id).numExp == 0 || delta_em > DELTA_EXP_GC_HDC_THRESHOLD

        NUM_EXPS = NUM_EXPS + 1;
        create_new_exp(CUR_EXP_ID, NUM_EXPS, vt_id, xGc, yGc, zGc, curYawHdc, curHeight);

        PREV_EXP_ID = CUR_EXP_ID;
        CUR_EXP_ID = NUM_EXPS;

        ACCUM_DELTA_X = 0;
        ACCUM_DELTA_Y = 0;
        ACCUM_DELTA_Z = 0;
        
        ACCUM_DELTA_YAW = EXPERIENCES(CUR_EXP_ID).yaw_exp_rad;
%         ACCUM_DELTA_HEIGHT = EXPERIENCES(CUR_EXP_ID).height_hdc;

        % if the visual template has changed (but isn't new) search for the matching experience
    elseif vt_id ~= PREV_VT_ID

        % find the experience associated with the current visual template and that is under the
        % threshold distance to the centre of grid cell and head direction cell activity
        % if multiple experiences are under the threshold then don't match (to reduce hash collisions)
        matched_exp_id = 0;
        matched_exp_count = 0;

        for search_id = 1:VT(vt_id).numExp
             
            minDeltaYaw = get_min_delta(EXPERIENCES(VT(vt_id).EXPERIENCES(search_id).id).yaw_hdc, curYawHdc, YAW_HEIGHT_HDC_Y_DIM);
%             minDeltaYawReversed = get_min_delta(EXPERIENCES(VT(vt_id).EXPERIENCES(search_id).id).yaw_hdc, (YAW_HEIGHT_HDC_Y_DIM /2) - curYawHdc, YAW_HEIGHT_HDC_Y_DIM);
%             minDeltaYaw = min(minDeltaYaw, minDeltaYawReversed);
    
            minDeltaHeight = get_min_delta(EXPERIENCES(VT(vt_id).EXPERIENCES(search_id).id).height_hdc, curHeight, YAW_HEIGHT_HDC_Y_DIM);
%             minDeltaHeightReversed = get_min_delta(EXPERIENCES(VT(vt_id).EXPERIENCES(search_id).id).height_hdc, (YAW_HEIGHT_HDC_Y_DIM /2) - curHeight, YAW_HEIGHT_HDC_H_DIM);
%             minDeltaHeight = min(minDeltaHeight, minDeltaHeightReversed);
            
            delta_em(search_id) = sqrt(get_min_delta(EXPERIENCES(VT(vt_id).EXPERIENCES(search_id).id).x_gc, xGc, GC_X_DIM)^2 ...
            + get_min_delta(EXPERIENCES(VT(vt_id).EXPERIENCES(search_id).id).y_gc, yGc, GC_Y_DIM)^2 ...
            + get_min_delta(EXPERIENCES(VT(vt_id).EXPERIENCES(search_id).id).z_gc, yGc, GC_Z_DIM)^2 ...
            + minDeltaYaw^2 + minDeltaHeight^2) ;
            
            
            if delta_em(search_id) < DELTA_EXP_GC_HDC_THRESHOLD
               matched_exp_count = matched_exp_count + 1; 
            end
        end

        if matched_exp_count > 1
            % this means we aren't sure about which experience is a match due to hash table collision
            % instead of a false posivitive which may create blunder links in
            % the experience map keep the previous experience

        else
            [min_delta, min_delta_id] = min(delta_em);

            MIN_DELTA_EM = [MIN_DELTA_EM; min_delta];
            if min_delta < DELTA_EXP_GC_HDC_THRESHOLD

                matched_exp_id = VT(vt_id).EXPERIENCES(min_delta_id).id;

                % see if the previous experience already has a link to the current experience
                link_exists = 0;
                for link_id = 1 : EXPERIENCES(CUR_EXP_ID).numlinks
                    if EXPERIENCES(CUR_EXP_ID).links(link_id).exp_id == matched_exp_id
                        link_exists = 1;
                        break;
                    end
                end

                % if we didn't find a link then create the link between current
                % experience and the experience for the current visual template
                if link_exists == 0
                    EXPERIENCES(CUR_EXP_ID).numlinks = EXPERIENCES(CUR_EXP_ID).numlinks + 1;
                    EXPERIENCES(CUR_EXP_ID).links(EXPERIENCES(CUR_EXP_ID).numlinks).exp_id = matched_exp_id;
                    %  EXPERIENCES(CUR_EXP_ID).links(EXPERIENCES(CUR_EXP_ID).numlinks).d_xy = sqrt(ACCUM_DELTA_X^2 + ACCUM_DELTA_Y^2 + ACCUM_DELTA_Z^2);
                    EXPERIENCES(CUR_EXP_ID).links(EXPERIENCES(CUR_EXP_ID).numlinks).d_xy = sqrt(ACCUM_DELTA_X^2 + ACCUM_DELTA_Y^2);
                    EXPERIENCES(CUR_EXP_ID).links(EXPERIENCES(CUR_EXP_ID).numlinks).d_z = ACCUM_DELTA_Z;
                    
                    EXPERIENCES(CUR_EXP_ID).links(EXPERIENCES(CUR_EXP_ID).numlinks).heading_yaw_exp_rad = ...
                        get_signed_delta_radian(EXPERIENCES(CUR_EXP_ID).yaw_exp_rad, atan2(ACCUM_DELTA_Y, ACCUM_DELTA_X)); % heading is the delta angle between current pose of exp and previous pose of exp
                   
                    EXPERIENCES(CUR_EXP_ID).links(EXPERIENCES(CUR_EXP_ID).numlinks).facing_yaw_exp_rad = ... % facing is the direction of each exp
                        get_signed_delta_radian(EXPERIENCES(CUR_EXP_ID).yaw_exp_rad, ACCUM_DELTA_YAW);
                                    
                 end

            end

            % if there wasn't an experience with the current visual template and grid cell (x y z) and head direction cell (yaw, height)
            % then create a new experience
            if matched_exp_id == 0
                NUM_EXPS = NUM_EXPS + 1;
                create_new_exp(CUR_EXP_ID, NUM_EXPS, vt_id, xGc, yGc, zGc, curYawHdc, curHeight);
                matched_exp_id = NUM_EXPS;
            end

            PREV_EXP_ID = CUR_EXP_ID;
            CUR_EXP_ID = matched_exp_id;

            ACCUM_DELTA_X = 0;
            ACCUM_DELTA_Y = 0;
            ACCUM_DELTA_Z = 0;
            ACCUM_DELTA_YAW = EXPERIENCES(CUR_EXP_ID).yaw_exp_rad;
%             ACCUM_DELTA_HEIGHT = EXPERIENCES(CUR_EXP_ID).height_hdc;
        end

    end

    global EXP_CORRECTION;
    global EXP_LOOPS;


    % do the experience map correction interatively for all the links in all the experiences
    for i = 1:EXP_LOOPS

        for exp_id = 1:NUM_EXPS

            for link_id=1:EXPERIENCES(exp_id).numlinks

                % experience 0 has a link to experience 1
                e0 = exp_id;
                e1 = EXPERIENCES(exp_id).links(link_id).exp_id;

                % work out where e0 thinks e1 (x,y) should be based on the stored link information
                lx = EXPERIENCES(e0).x_exp + EXPERIENCES(e0).links(link_id).d_xy * cos(EXPERIENCES(e0).yaw_exp_rad + EXPERIENCES(e0).links(link_id).heading_yaw_exp_rad);
                ly = EXPERIENCES(e0).y_exp + EXPERIENCES(e0).links(link_id).d_xy * sin(EXPERIENCES(e0).yaw_exp_rad + EXPERIENCES(e0).links(link_id).heading_yaw_exp_rad);
                lz = EXPERIENCES(e0).z_exp + EXPERIENCES(e0).links(link_id).d_z;  % 

                % correct e0 and e1 (x,y) by equal but opposite amounts
                % a 0.5 correction parameter means that e0 and e1 will be fully
                % corrected based on e0's link information
                EXPERIENCES(e0).x_exp = EXPERIENCES(e0).x_exp + (EXPERIENCES(e1).x_exp - lx) * EXP_CORRECTION;
                EXPERIENCES(e0).y_exp = EXPERIENCES(e0).y_exp + (EXPERIENCES(e1).y_exp - ly) * EXP_CORRECTION;
                EXPERIENCES(e0).z_exp = EXPERIENCES(e0).z_exp + (EXPERIENCES(e1).z_exp - lz) * EXP_CORRECTION;
                
                EXPERIENCES(e1).x_exp = EXPERIENCES(e1).x_exp - (EXPERIENCES(e1).x_exp - lx) * EXP_CORRECTION;
                EXPERIENCES(e1).y_exp = EXPERIENCES(e1).y_exp - (EXPERIENCES(e1).y_exp - ly) * EXP_CORRECTION;
                EXPERIENCES(e1).z_exp = EXPERIENCES(e1).z_exp - (EXPERIENCES(e1).z_exp - lz) * EXP_CORRECTION;

                % determine the angle between where e0 thinks e1's facing
                % should be based on the link information
                TempDeltaYawFacing = get_signed_delta_radian((EXPERIENCES(e0).yaw_exp_rad + EXPERIENCES(e0).links(link_id).facing_yaw_exp_rad), EXPERIENCES(e1).yaw_exp_rad);

                % correct e0 and e1 facing by equal but opposite amounts
                % a 0.5 correction parameter means that e0 and e1 will be fully
                % corrected based on e0's link information           
                EXPERIENCES(e0).yaw_exp_rad = clip_radian_180(EXPERIENCES(e0).yaw_exp_rad + TempDeltaYawFacing * EXP_CORRECTION);
                EXPERIENCES(e1).yaw_exp_rad = clip_radian_180(EXPERIENCES(e1).yaw_exp_rad - TempDeltaYawFacing * EXP_CORRECTION);
                
            end
        end

    end

    % keep a frame by frame history of which experience was currently active
    global EXP_HISTORY;
    EXP_HISTORY = [EXP_HISTORY; CUR_EXP_ID];
end

