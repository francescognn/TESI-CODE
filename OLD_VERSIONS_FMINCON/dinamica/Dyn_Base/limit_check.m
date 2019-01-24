function y=limit_check(u,my_axes)

h = find_system(gcs,'Name', 'Name');

min = get_param(h,[my_axes,'min']);
min = str2num(min{1});
if min > u-1
    set_param(h{1},[my_axes,'min'],num2str(u-1));
end
    
max = get_param(h,[my_axes,'max']);    
max = str2num(max{1});
if max < u+1
    set_param(h{1},[my_axes,'max'],num2str(u+1));
end


y=u;