function yaw_height_hdc_iteration(vt_id, yawRotV, heightV)
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
    
    % Pose cell update steps
    % 1. Add view template energy
    % 2. Local excitation
    % 3. Local inhibition
    % 4. Global inhibition
    % 5. Normalisation
    % 6. Path Integration (yawRotV then heightV)

   
    % The HD cells of yaw and height conjunctively
    global YAW_HEIGHT_HDC;
    
    % The visual templete
    global VT;
    
    % The dimension of yaw in yaw_height_hdc network
    global YAW_HEIGHT_HDC_Y_DIM;
    
    % The dimension of height in yaw_height_hdc network
    global YAW_HEIGHT_HDC_H_DIM;
    
    % The dimension of local excitation weight matrix for yaw
    global YAW_HEIGHT_HDC_EXCIT_Y_DIM;
    
    % The dimension of local excitation weight matrix for height
    global YAW_HEIGHT_HDC_EXCIT_H_DIM;
    
    % The dimension of local inhibition weight matrix for yaw
    global YAW_HEIGHT_HDC_INHIB_Y_DIM;
    
    % The dimension of local inhibition weight matrix for height
    global YAW_HEIGHT_HDC_INHIB_H_DIM;
    
    % The global inhibition value
    global YAW_HEIGHT_HDC_GLOBAL_INHIB;
    
    
    % The amount of energy injected when a view template is re-seen
    global YAW_HEIGHT_HDC_VT_INJECT_ENERGY; 
    
    % The excit wrap of yaw in yaw_height_hdc network
    global YAW_HEIGHT_HDC_EXCIT_Y_WRAP;
    
    % The excit wrap of height in yaw_height_hdc network
    global YAW_HEIGHT_HDC_EXCIT_H_WRAP;
    
    % The inhibit wrap of yaw in yaw_height_hdc network
    global YAW_HEIGHT_HDC_INHIB_Y_WRAP;
    
    % The inhibit wrap of height in yaw_height_hdc network
    global YAW_HEIGHT_HDC_INHIB_H_WRAP;
    
    % The weight of excitation in yaw_height_hdc network
    global YAW_HEIGHT_HDC_EXCIT_WEIGHT;
    
    % The weight of inhibition in yaw_height_hdc network
    global YAW_HEIGHT_HDC_INHIB_WEIGHT;
    
    % The yaw theta size of each unit in radian, 2*pi/ YAW_HEIGHT_HDC_Y_DIM
    % radian e.g. 2*pi/360 = 0.0175
    global YAW_HEIGHT_HDC_Y_TH_SIZE;   
    global YAW_HEIGHT_HDC_H_SIZE;     


    % if this isn't a new visual template then add the energy at its associated posecell location
    if VT(vt_id).first ~= 1
        act_yaw = min([max([round(VT(vt_id).hdc_yaw), 1]), YAW_HEIGHT_HDC_Y_DIM]);
        act_height = min([max([round(VT(vt_id).hdc_height), 1]), YAW_HEIGHT_HDC_H_DIM]);
        

        % this decays the amount of energy that is injected at the visual template's posecell location
        % this is important as the posecell Posecells will errounously snap 
        % for bad visual template matches that occur over long periods (eg a bad matches that
        % occur while the agent is stationary). This means that multiple visual template's
        % need to be recognised for a snap to happen
        energy = YAW_HEIGHT_HDC_VT_INJECT_ENERGY * 1/30 * (30 - exp(1.2 * VT(vt_id).decay));
        if energy > 0
            YAW_HEIGHT_HDC(act_yaw, act_height) = YAW_HEIGHT_HDC(act_yaw, act_height) + energy;
        end
    end


    % Local excitation: yaw_height_hdc_local_excitation = yaw_height_hdc elements * yaw_height_hdc weights
    yaw_height_hdc_local_excit_new = zeros(YAW_HEIGHT_HDC_Y_DIM, YAW_HEIGHT_HDC_H_DIM);
    for h = 1 : YAW_HEIGHT_HDC_H_DIM
        for y = 1 : YAW_HEIGHT_HDC_Y_DIM
            if YAW_HEIGHT_HDC(y, h) ~= 0
                yaw_height_hdc_local_excit_new(YAW_HEIGHT_HDC_EXCIT_Y_WRAP(y : y + YAW_HEIGHT_HDC_EXCIT_Y_DIM - 1),YAW_HEIGHT_HDC_EXCIT_H_WRAP(h : h + YAW_HEIGHT_HDC_EXCIT_H_DIM - 1)) = ...
                    yaw_height_hdc_local_excit_new(YAW_HEIGHT_HDC_EXCIT_Y_WRAP(y : y + YAW_HEIGHT_HDC_EXCIT_Y_DIM - 1),YAW_HEIGHT_HDC_EXCIT_H_WRAP(h : h + YAW_HEIGHT_HDC_EXCIT_H_DIM - 1)) ...
                        + YAW_HEIGHT_HDC(y,h) .* YAW_HEIGHT_HDC_EXCIT_WEIGHT;
            end    
        end
    end
    YAW_HEIGHT_HDC = yaw_height_hdc_local_excit_new;

    % local inhibition: yaw_height_hdc_local_inhibition = hdc - hdc elements * hdc_inhib weights
    yaw_height_hdc_local_inhib_new = zeros(YAW_HEIGHT_HDC_Y_DIM, YAW_HEIGHT_HDC_H_DIM);  
    for h = 1 : YAW_HEIGHT_HDC_H_DIM
        for y = 1 : YAW_HEIGHT_HDC_Y_DIM
            if YAW_HEIGHT_HDC(y, h) ~= 0
                yaw_height_hdc_local_inhib_new(YAW_HEIGHT_HDC_INHIB_Y_WRAP(y : y + YAW_HEIGHT_HDC_INHIB_Y_DIM - 1),YAW_HEIGHT_HDC_INHIB_H_WRAP(h : h + YAW_HEIGHT_HDC_INHIB_H_DIM - 1)) = ...
                    yaw_height_hdc_local_inhib_new(YAW_HEIGHT_HDC_INHIB_Y_WRAP(y : y + YAW_HEIGHT_HDC_INHIB_Y_DIM - 1),YAW_HEIGHT_HDC_INHIB_H_WRAP(h : h + YAW_HEIGHT_HDC_INHIB_H_DIM - 1)) ...
                    + YAW_HEIGHT_HDC(y, h) .* YAW_HEIGHT_HDC_INHIB_WEIGHT;
            end
        end
    end
    YAW_HEIGHT_HDC = YAW_HEIGHT_HDC - yaw_height_hdc_local_inhib_new;

    % global inhibition - PC_gi = PC_li elements - inhibition
    YAW_HEIGHT_HDC = (YAW_HEIGHT_HDC >= YAW_HEIGHT_HDC_GLOBAL_INHIB) .* (YAW_HEIGHT_HDC - YAW_HEIGHT_HDC_GLOBAL_INHIB);
    
    % normalisation
    total = sum(sum(YAW_HEIGHT_HDC));
    YAW_HEIGHT_HDC = YAW_HEIGHT_HDC./total;       
    
    if yawRotV ~= 0
        % mod to work out the partial shift amount
        weight = mod(abs(yawRotV) / YAW_HEIGHT_HDC_Y_TH_SIZE, 1);
        if weight == 0
            weight = 1.0;
        end
        YAW_HEIGHT_HDC = circshift(YAW_HEIGHT_HDC, ...
            [sign(yawRotV) * floor(mod(abs(yawRotV) / YAW_HEIGHT_HDC_Y_TH_SIZE, YAW_HEIGHT_HDC_Y_DIM)) 0]) * (1.0 - weight) ...
            + circshift(YAW_HEIGHT_HDC, ...
            [sign(yawRotV) * ceil(mod(abs(yawRotV) / YAW_HEIGHT_HDC_Y_TH_SIZE, YAW_HEIGHT_HDC_Y_DIM)) 0]) * (weight);
    end
    
    if heightV ~= 0
        % mod to work out the partial shift amount
        weight = mod(abs(heightV) / YAW_HEIGHT_HDC_H_SIZE, 1);
        if weight == 0
            weight = 1.0;
        end
        YAW_HEIGHT_HDC = circshift(YAW_HEIGHT_HDC, ...
            [0 sign(heightV) * floor(mod(abs(heightV) / YAW_HEIGHT_HDC_H_SIZE, YAW_HEIGHT_HDC_H_DIM))]) * (1.0 - weight) ...
            + circshift(YAW_HEIGHT_HDC, ...
            [0 sign(heightV) * ceil(mod(abs(heightV) / YAW_HEIGHT_HDC_H_SIZE, YAW_HEIGHT_HDC_H_DIM))]) * (weight);
    end

end






