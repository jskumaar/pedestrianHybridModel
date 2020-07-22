function s = kalmanCorrect(s)
    % Compute Kalman gain factor:
    K = s.P * s.H' * inv(s.H * s.P * s.H' + s.R);
    % Correction based on observation:
    s.x = s.x' + K*(s.z' - s.H *s.x');
    s.P = s.P - K*s.H*s.P;
    s.x = s.x';
end