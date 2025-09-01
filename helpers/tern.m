function out = tern(cond, a, b)
% TERN  simple inline-like ternary operator
% usage: out = tern(condition, value_if_true, value_if_false)
if cond
    out = a;
else
    out = b;
end
end
