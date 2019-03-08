function file = sortObj(file)  
% Alphanumeric / Natural-Order sort the strings in a cell array of strings (1xN char).
%
% (c) 2012 Stephen Cobeldick

    for i = 1 : length(file)  
        A{i} = file(i).name;  
    end  
    [~, ind] = natsort(A);  

    for j = 1 : length(file)  
        files(j) = file(ind(j));  
    end  
    
clear file;  
file = files';