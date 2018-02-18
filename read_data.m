function data=read_data(filename, col_label, data_name)
raw_data = csvread(filename);
data = struct();
if nargin >= 3
    data = setfield(data, 'name', data_name);
end

for idx = 1:length(col_label)
    data = setfield(data, col_label{idx}, raw_data(:,idx));
end
end