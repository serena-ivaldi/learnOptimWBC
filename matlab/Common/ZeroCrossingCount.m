function count = ZeroCrossingCount(vec)
zeroCrossIndex=diff(vec>0)~=0;
count = sum(zeroCrossIndex);
end