function line = drawDesiredPath()
    
    line(1) = plot3([1.125 1.525], [0.025 0.025], [0.308 0.308], 'yellow');
    line(2) = plot3([1.525 1.525], [0.025 0.325], [0.308 0.308], 'yellow');
    line(3) = plot3([1.525 1.125], [0.325 0.325], [0.308 0.308], 'yellow');
    line(4) = plot3([1.125 1.125], [0.325 0.025], [0.308 0.308], 'yellow');

end