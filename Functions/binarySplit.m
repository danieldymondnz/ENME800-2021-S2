function [lowerArray, middle, upperArray] = binarySplit(in)

    middleIndex = floor((length(in) / 2));
    middle = in(middleIndex);

    lowerArray = in(1:middleIndex-1);
    upperArray = in(middleIndex:length(in));

%     lowerArray = in(1:middleIndex-1);
%     upperArray = in(middleIndex+1:length(in));

end

