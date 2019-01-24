function [varargout] = all_samples(varargin)
  argout_0 = cell(60,1);
  w0 = varargin{1};
  w1 = varargin{2};
  w2 = varargin{3};
unknownN6casadi4CallE
  sp_in_m = 3;
  sp_in_n = 10;
  sp_in_j = [1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10];
  sp_in_i = [1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3];
  sp_in = sparse(sp_in_i, sp_in_j, ones(size(sp_in_i)), sp_in_m, sp_in_n);
  argout_0{1} = w3(sp_in==1);
  sp_in_m = 3;
  sp_in_n = 10;
  sp_in_j = [1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10];
  sp_in_i = [1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3];
  sp_in = sparse(sp_in_i, sp_in_j, ones(size(sp_in_i)), sp_in_m, sp_in_n);
  tmp = w3(sp_in==1);
  sp_m = 3;
  sp_n = 1;
  sp_j = [1, 1, 1];
  sp_i = [1, 2, 3];
  w0 = sparse(sp_i, sp_j, tmp(28:30), sp_m, sp_n);
  w1 = varargin{2};
  w2 = varargin{3};
unknownN6casadi4CallE
  sp_in_m = 3;
  sp_in_n = 10;
  sp_in_j = [1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10];
  sp_in_i = [1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3];
  sp_in = sparse(sp_in_i, sp_in_j, ones(size(sp_in_i)), sp_in_m, sp_in_n);
  argout_0{2} = w3(sp_in==1);
  varargout{1} = reshape(vertcat(argout_0{:}), 3, 20);
end
function y=nonzeros_gen(x)
  if isa(x,'casadi.SX') || isa(x,'casadi.MX') || isa(x,'casadi.DM')
    y = x{:};
  elseif isa(x,'sdpvar')
    b = getbase(x);
    f = find(sum(b~=0,2));
    y = sdpvar(length(f),1,[],getvariables(x),b(f,:));
  else
    y = nonzeros(x);
  end
end
function y=if_else_zero_gen(c,e)
  if isa(c+e,'casadi.SX') || isa(c+e,'casadi.MX') || isa(c+e,'casadi.DM')
    y = if_else(c, e, 0);
  else
    if c
        y = x;
    else
        y = 0;
    end
  end
end
