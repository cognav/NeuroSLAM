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

[filename,pathname,fileindex] = uigetfile('*.*','Please open a video file');
dname = uigetdir('c:/','Save images to');
dname = [dname '\'];
video_info = VideoReader([pathname filename]);

LEN = 50000;
nfmt = ['%0.' num2str(length(num2str(LEN))) 'd'];
ffmt = 'png';%file format

videoFReader = vision.VideoFileReader([pathname filename]);

Frames = 0;
% Frames = 41443; 
while ~isDone(videoFReader) 
    Frames = Frames+1; 
    disp(['Processing ' num2str(Frames) ' frame']);
    video_image = step(videoFReader); 
    resizeImage = imresize (video_image, [270, 480]);
    imwrite(resizeImage,strcat(dname,num2str(Frames,nfmt),['.' ffmt]),ffmt); %save each frame       
end 
disp('Finised');

