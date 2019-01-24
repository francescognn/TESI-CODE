function y=axes_set(my_axes,lim,u)

h = find_system(gcs,'Name', 'Name');

set_param(h{1},[my_axes,'min'],num2str(lim(1)));

set_param(h{1},[my_axes,'max'],num2str(lim(2)));

y=u;
end
