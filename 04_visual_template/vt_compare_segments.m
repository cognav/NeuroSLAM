function [offsetY, offsetX, sdif] = vt_compare_segments(seg1, seg2, vtPanoramic, halfOffsetRange, slenY, slenX, cwlY, cwlX )
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
    
    % assume a large difference
    mindiff = 1e7;
    minoffsetX = 0;
    minoffsetY = 0;

    % compare two 1 row*N column matrix 
    % for each offset sum the abs difference between the two segments
    
    % slen : shift range for comparing
    if vtPanoramic == 1
        for halfOffset = halfOffsetRange

            seg2 = circshift(seg2,[0 halfOffset]);

            for offsetY = 0:slenY
                for offsetX = 0:slenX
                    cdiff = abs(seg1(1 + offsetY : cwlY , 1 + offsetX : cwlX) - seg2(1 : cwlY - offsetY, 1 : cwlX - offsetX));
                    cdiff = sum(sum(cdiff)) / (cwlY - offsetY) * (cwlX - offsetX) ;
                    if (cdiff < mindiff)
                        mindiff = cdiff;
                        minoffsetX = offsetX;
                        minoffsetY = offsetY;
                    end
                end
            end

            for offsetY = 1:slenY
                for offsetX = 1:slenX
                    cdiff = abs(seg1(1 : cwlY - offsetY, 1 : cwlX - offsetX) - seg2(1 + offsetY : cwlY, 1 + offsetX : cwlX));
                    cdiff = sum(sum(cdiff)) / (cwlY - offsetY) * (cwlX - offsetX) ;
                    if (cdiff < mindiff)
                        mindiff = cdiff;
                        minoffsetX = -offsetX;
                        minoffsetY = -offsetY;
                    end
                end
            end

        end
        offsetX = minoffsetX;
        offsetY = minoffsetY;
        sdif = mindiff;
        
    else

        for offsetY = 0:slenY
            for offsetX = 0:slenX
                cdiff = abs(seg1(1 + offsetY : cwlY , 1 + offsetX : cwlX) - seg2(1 : cwlY - offsetY, 1 : cwlX - offsetX));
                cdiff = sum(sum(cdiff)) / (cwlY - offsetY) * (cwlX - offsetX) ;
                if (cdiff < mindiff)
                    mindiff = cdiff;
                    minoffsetX = offsetX;
                    minoffsetY = offsetY;
                end
            end
        end

        for offsetY = 1:slenY
            for offsetX = 1:slenX
                cdiff = abs(seg1(1 : cwlY - offsetY, 1 : cwlX - offsetX) - seg2(1 + offsetY : cwlY, 1 + offsetX : cwlX));
                cdiff = sum(sum(cdiff)) / (cwlY - offsetY) * (cwlX - offsetX) ;
                if (cdiff < mindiff)
                    mindiff = cdiff;
                    minoffsetX = -offsetX;
                    minoffsetY = -offsetY;
                end
            end
        end

        offsetX = minoffsetX;
        offsetY = minoffsetY;
        sdif = mindiff;
    end

end