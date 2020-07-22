   function  likelihood = PedestrianConstantVelocityMeasurementLikelihood(pf, predictParticles, measurement, sys)
       % The measurement contains all state variables
       predictMeasurement = predictParticles;
       N = length(predictParticles);
       % Calculate observed error between predicted and actual measurement
       % NOTE in this example, we don't have full state observation, but only
       % the measurement of current pose, therefore the measurementErrorNorm
       % is only based on the pose error.
%        measurementError = bsxfun(@minus, predictMeasurement(:,1:2), measurement);
       measurementError = predictMeasurement(:,1:2) - repmat(measurement,[1,N])';
       measurementErrorNorm = sqrt(sum(measurementError.^2, 2));
       % Normal-distributed noise of measurement
       % Assuming measurements on all three pose components have the same error distribution 
       measurementNoise = sys.R;
       % Convert error norms into likelihood measure. 
       % Evaluate the PDF of the multivariate normal distribution 
       likelihood = 1/sqrt((2*pi).^3 * det(measurementNoise)) * exp(-0.5 * measurementErrorNorm);
   end