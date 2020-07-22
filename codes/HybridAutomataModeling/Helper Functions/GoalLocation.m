function Goal = GoalLocation(q,CrossDirection)

if strcmp(CrossDirection,'forward')
    
    switch q
        case 1
            Goal = [0;-3.5];
        case 2
            Goal = [0;-3.5];
        case 3
            Goal = [0;3.5];
        case 4
            Goal = [0;3.5];
        case 5
            Goal = [-10;3.5];
    end
else
    switch q
        case 1
            Goal = [0;3.5];
        case 2
            Goal = [0;3.5];
        case 3
            Goal = [0;-3.5];
        case 4
            Goal = [0;-3.5];
        case 5
            Goal = [-10;-3.5];
    end
end
 




end
            
            
            
            
            
            
            
            










end