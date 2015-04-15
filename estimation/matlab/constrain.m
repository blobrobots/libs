% constrain value
function result = constrain(v, min, max)

if(v < min)
    result = min;
elseif (v > max)
    result = max;
else
    result = v;
end

end