function line = drawDesiredPath()
    
    line(1) = plot3([1125 1525], [25 25], [308 308], 'yellow');
    line(2) = plot3([1525 1525], [25 325], [308 308], 'yellow');
    line(3) = plot3([1525 1125], [325 325], [308 308], 'yellow');
    line(4) = plot3([1125 1125], [325 25], [308 308], 'yellow');

end