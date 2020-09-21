function s = kalmanPredict(s)
<<<<<<< HEAD
    s.x = s.A*s.x;
    s.P = s.A * s.P * s.A' + s.Q; 
=======
    s.x = s.A*s.x';
    s.P = s.A * s.P * s.A' + s.Q; 
    s.x=s.x';
>>>>>>> 1d644f352954a8f506e14ff3e9798122c8d63240
end
