classdef HybridAutomaton_Model_1 < handle
% Vehicle Gaps - summary statistics for real-time predictions
% Kalman filter for continuous state update    
    
    
    properties
        f;              % process model
        h;              % measurement model
        x;              % continuous state vector
        q;              % discrete state vector
        Q;              % process noise covariance
        LQ;             % Cholesky factor of Q
        R;              % measurement noise covariance
        p_thres         % threshold of probability for state change
        KFplant         % plant model of dynamics 'f' for the KF filter
        Classifier;     % Classifier Object
        G;              % Guard states
        
%         PredictedY_Positions
%         Predicted_Actions
%         CV_Noise
%         currentStates
%         nextStates
    end
    
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %class constructor
        
        function obj = HybridAutomaton_Model_1(system,init)
            
            obj.Classifier = ActionClassifier(init);          
            obj.f = system.f;
            obj.Q = system.Q;
            obj.LQ = chol(obj.Q, 'lower');
            obj.h = system.h;
            obj.R = system.R; 
            obj.KFplant = system.KFplant;
            obj.p_thres = system.p_thres;
            obj.G = system.G;
            
            
            %intialize states
            obj.x = init.x;
            obj.q = init.q;
         end
 
        % continuous state propagation
        function obj= pedestrian_motion_cv(obj)
            % A constant velocity random walk motion model
                % sample noise
                w = obj.LQ * randn(4,1);
                % propagate the pedestrian state!
                obj.x = obj.f(obj.x, w);  
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        % discrete state propagation
        
        function pedestrian_action(obj,F)
                switch obj.q
                    case 1 % in Approach state
                        p_q2 = obj.Classifier.LR_Approach_Model_1(obj.x,F);   %approach to wait probability
                        p_q3 = obj.Classifier.LR_Approach_Model_2(obj.x,F);   %approach to cross probability
                        
                        pedestrian_action_update_meas(obj);      %check discrete state from guard conditions
                        qGuard = obj.q;          
                        
                        %during prediction under CV, current state remain in approach unless there
                        %is a strong probability of state change or the pedestrian crosses the road
                        if (qGuard==1 & p_q2.p>p_q3.p & p_q2.p>obj.p_thres)    
                            obj.q=2;
                        elseif(qGuard==1 & p_q3.p>p_q2.p & p_q3.p>obj.p_thres)
                            obj.q=3;
                        else
                            obj.q=qGuard;
                        end
                        
                    case 2  % in Wait state
                        p_q3.p = LR_Wait_Model_1(obj.x,F);
                        pedestrian_action_update(obj);
                        qGuard = obj.q;
                        if (qGuard==2 & p_q3.p>obj.p_thres)
                            obj.q =3;
                        else
                            obj.q=2;
                        end
                        
                    case 3 % in Cross state
                        obj.q = pedestrian_action_update(obj);                        
                end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % continuous state update from measurement
        
        function pedestrian_motion_cv_update(obj,zH)
%            % Kalman update from measurement
%                 [kalmf,L,P,M] = kalman(Plant,Q,R);
             obj.x(1:2,1) = zH(:,end);    %has to be replaced with a Kalman Update soon!   
                                        %velocity and heding updated in the reset function;
              N = size(zH,2);
              if N>1
                  Vx = (zH(end,1)-zH(1,1))/N;
                  Vy = (zH(end,2)-zH(1,2))/N;
                  obj.x(3) = sqrt(Vx^2 + Vy^2);    %average velocity of last N observations
                  obj.x(4) = atan2(Vy,Vx);         %average heading of last N observations
              end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % discrete state update from continuous state measurement
        
        function pedestrian_action_update_meas(obj)
             % Check Guards for state 1
             if (obj.x(3)>obj.G(3) & abs(obj.x(2))>obj.G(2) & obj.q==1)
                obj.q = 1;
                            
             % Check Guards for state 2    
             elseif(obj.x(3)<=obj.G(3) & ...
                     -sign(obj.x(2))*obj.x(4) <= 2*pi/3 & -sign(obj.x(2))*obj.x(4) >= pi/3 & obj.q==1)                
                obj.q = 2;
               
             % Check Guards for state 3    
             elseif(obj.x(3)>obj.G(3) & abs(obj.x(2))<obj.G(2))                 
                obj.q = 3;  
                                  
             else
                obj.q = 4;
                                 
             end          
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % discrete state update from ground truth
        function pedestrian_action_update(obj,z_discrete)
            obj.q = z_discrete;  
        end  
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Reset states
        
%         function pedestrian_state_reset(obj,zH,T)
%                switch obj.q
%                    case 1                     
%                        % check intersection near crosswalk on closest side                       
%                        xlimit = [-1.5 1.5];
%                        if obj.x(2)>0
%                             ylimit = [4.5  3];
%                        else
%                             ylimit = [-4.5  -3];
%                        end
%                        xbox = xlimit([1 1 2 2 1]);
%                        ybox = ylimit([1 2 2 1 1]);
%                        
%                                              
%                        xPath = [obj.x(1) obj.x(1)+100*obj.x(3)*cos(obj.x(4))];
%                        yPath = [obj.x(2) obj.x(2)+100*obj.x(3)*sin(obj.x(4))];
%                        
%                        [xi,yi] = polyxpoly(xPath,yPath,xbox,ybox);
%                        r = sqrt((xi-obj.x).^2+(yi-obj.y).^2);
%                        [~,ind] = max(r);
%                                              
%                    if isempty(ind)  %path does not intersect close to crosswalk
%                        % intersection of path and road edge
%                        xCurb = [-20 20];
%                        if obj.x(2)>0
%                             yCurb = [3.5 3.5];
%                        else
%                             yCurb = [-3.5 -3.5]; 
%                        end
%                      
%                    case 2
%                        
%                        
%                    case 3
%                        
%                        
%                    case 4
%                        
%                        
%                end
%          end
           
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Reset states

           function pedestrian_state_reset(obj,T)
               if obj.q==2
                   obj.x(3) = 0;
                   if obj.x(2)>0
                       obj.x(4) = -pi/2;
                   else
                       obj.x(4) = pi/2;
                   end
               elseif obj.q==3
                   obj.q = atan2((T(2)-obj.x(2)),(T(1)-obj.x(1)));
               elseif obj.q==4
                   if abs(obj.x(4)) > pi/2    %heading towards left
                       obj.x(4) = -sign(obj.x(2))*pi;
                   else
                       obj.x(4) = 0;
                   end
                   
               end              
           end
            
            
        
        
        
             
             
             
             
            
            
            
    end
        
        
        
        
        
end