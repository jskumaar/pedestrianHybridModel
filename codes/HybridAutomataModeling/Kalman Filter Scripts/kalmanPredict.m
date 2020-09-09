function s = kalmanPredict(s)
    s.x = s.A*s.x;
    s.P = s.A * s.P * s.A' + s.Q; 
end
