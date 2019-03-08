function [out_minimum_offset, out_minimum_difference_intensity] = compare_segments(seg1, seg2, shift_length, compare_length_of_intensity)
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

    % input parameters
    % seg1, seg2 represent 1D arrays of the intensity profiles of the current image and previous image.
    % shift_length is the range of offsets in pixels to consider i.e. slen = 0 considers only the no offset case
    % compare_length_of_intensity is the length of the intensity profile to actually compare, and must be < than image width – 1 * slen

    % output parameters
    % minimum_offset  the minimum shift offset when the difference of intensity
    % is smaller minimum_difference_intensity the minimum of intensity profile

    % assume a large difference
    minimum_difference_intensity = 1e6;

    % initial the matrix
    differencs = zeros(shift_length);

    % compare two 1 row*N column matrix 
    % for each offset sum the abs difference between the two segments
    for offset = 0:shift_length
        compare_difference_segments = abs(seg1(1 + offset:compare_length_of_intensity) - seg2(1:compare_length_of_intensity - offset));
        sum_compare_difference_segments = sum(compare_difference_segments) / (compare_length_of_intensity - offset);
        differencs(shift_length - offset + 1) = sum_compare_difference_segments;
        if (sum_compare_difference_segments < minimum_difference_intensity)
            minimum_difference_intensity = sum_compare_difference_segments;
            minimum_offset = offset;
        end
    end

    for offset = 1:shift_length
        compare_difference_segments = abs(seg1(1:compare_length_of_intensity - offset) - seg2(1 + offset:compare_length_of_intensity));
        sum_compare_difference_segments = sum(compare_difference_segments) / (compare_length_of_intensity - offset);
        differencs(shift_length + 1 + offset) = sum_compare_difference_segments;
        if (sum_compare_difference_segments < minimum_difference_intensity)
            minimum_difference_intensity = sum_compare_difference_segments;
            minimum_offset = -offset;
        end
    end

    out_minimum_offset =  minimum_offset;
    out_minimum_difference_intensity = minimum_difference_intensity;

end