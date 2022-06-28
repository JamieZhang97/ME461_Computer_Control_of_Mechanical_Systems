function arraytoCformat2(a)
% format arraytoCformat(a)
% this function prints out an array in C-format so that it can be easily cut and
% pasted into a C file.  
[num_rows,num_columns] = size(a);

if num_rows > num_columns
    if double(inputname(1)) > 64 & double(inputname(1)) < 123 
        fprintf(1,'float %s[%d]={',inputname(1),num_rows);
    else 
        fprintf(1,'float array[%d]={',num_rows);        
    end
else 
    if double(inputname(1)) > 64 & double(inputname(1)) < 123
        fprintf(1,'float %s[%d]={',inputname(1),num_columns);
    else
        fprintf(1,'float array[%d]={',num_columns);
    end
end

for i=1:num_rows
%    fprintf(1,'{');
    for j=1:num_columns
        fprintf(1,'\t%10.16e',a(i,j));
        if j~=num_columns
            fprintf(1,',');
        end
    end
%   fprintf(1,'}');
    if i~=num_rows
        fprintf(1,',\n');
    end
end
fprintf(1,'};\n');
