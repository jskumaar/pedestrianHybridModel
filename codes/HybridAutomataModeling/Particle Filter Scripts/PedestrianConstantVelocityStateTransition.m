function predictParticles = PedestrianConstantVelocityStateTransition(pf, prevParticles, sys) 

    N = length(prevParticles);
    
    LQ = chol(sys.Q, 'lower');

    for i = 1:N
        % sample noise
        
        %1) Given in the PF code (Maani)
        w = LQ * randn(4,1);

%         %2) Sample from zero mean and process noise covariance
%         w = normrnd(0,LQ);
        
        % propagate the particle!
        predictParticles(i,:) = sys.f(prevParticles(i,:)', w);
    end
       
end