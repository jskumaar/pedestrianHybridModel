classdef HybridAutomaton_Model_2 < handle
% Probability diistribution based classification  
    
    
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
        p;
        Prob;
        dt;             % time interval
        q_decide;
        
    end
    
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %class constructor
        
        function obj = HybridAutomaton_Model_2(system,init)
                     
            obj.f = system.f;
            obj.Q = system.Q;
            obj.LQ = chol(obj.Q, 'lower');
            obj.h = system.h;
            obj.R = system.R; 
            obj.KFplant = system.KFplant;
            obj.p_thres = system.p_thres;
            obj.G = system.G;
            obj.dt = system.dt;
            
                       
            %intialize states
            obj.x = init.x;
            obj.q = init.q;
            obj.Prob = init.Prob;
            obj.q_decide = 1;
         end
 
        % continuous state propagation
        function obj= pedestrian_motion_cv(obj)
            % A constant velocity random walk motion model
                % sample noise
%                 w = obj.LQ * randn(4,1);
%                 % propagate the pedestrian state!
%                 obj.x = obj.f(obj.x', w)';  

             w = [0;0;0;0];
             obj.x = obj.f(obj.x', w)';  
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        % discrete state propagation
        
        function pedestrian_action_decision(obj,F,T,walk_start_flag)
           
            %% Probability model
                switch obj.q
                        case 1 % in Approach state
                            p_q3 = obj.ApproachToCross(F);   %approach to cross probability
                            p_q2.p = 1-p_q3.p;   %approach to wait probability

                            pedestrian_action_update_meas(obj,T,walk_start_flag);      %check discrete state from guard conditions
                            qGuard = obj.q;          

                            %during prediction under CV, current state remain in approach unless there
                            %is a strong probability of state change or the pedestrian crosses the road
                            if (qGuard==1 & p_q2.p>p_q3.p & p_q2.p>=obj.p_thres)    
                                obj.q_decide=2;
                            elseif(qGuard==1 & p_q3.p>=p_q2.p & p_q3.p>=obj.p_thres)
                                obj.q_decide=3;
                            else
                                obj.q_decide=qGuard;
                            end

                        case 2  % in Wait state
                            p_q3 = obj.WaitToCross(F);
                            pedestrian_action_update_meas(obj,T,walk_start_flag);
                            qGuard = obj.q;
                            if (qGuard==2 & p_q3.p>obj.p_thres)
                                obj.q_decide = 3;
                            else
                                obj.q_decide = 2;
                            end

    %                     case 3 % in Cross state
    %                         pedestrian_action_update_meas(obj,T);                        
                end
        
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % continuous state update from measurement
        
        function pedestrian_motion_cv_update(obj,zH)
%            % Kalman update from measurement
%                 [kalmf,L,P,M] = kalman(Plant,Q,R);
             obj.x(1,1:2) = zH(end,:);    %has to be replaced with a Kalman Update soon!   
                                          %velocity and heading updated in the reset function;
              N = size(zH,1);
              if N>1
                  Vx = (zH(end,1)-zH(1,1))/(obj.dt*(N-1));
                  Vy = (zH(end,2)-zH(1,2))/(obj.dt*(N-1));
%                   obj.x(3) = sqrt(Vx^2 + Vy^2);    %average velocity of last N observations
%                   obj.x(4) = atan2(Vy,Vx);         %average heading of last N observations
                  
                  % cartesian state update
                  obj.x(3) = Vx;
                  obj.x(4) = Vy;
              end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % discrete state update from continuous state measurement
        
        function pedestrian_action_update_meas(obj,T,walk_start_flag)
            
            new_q = obj.q;
            
            AbsoluteVelocity = sqrt(obj.x(3)^2 + obj.x(4)^2);
            CurrentHeading = atan2(obj.x(4),obj.x(3));
%              % Check Guards for state 1
%              if (obj.x(3)>obj.G(3) & abs(obj.x(2))>obj.G(2) & obj.q==1)
%                 obj.q = 1;
%                             
%              % Check Guards for state 2    
%              elseif(obj.x(3)<=obj.G(3) & ...
%                      -sign(obj.x(2))*obj.x(4) <= 2*pi/3 & -sign(obj.x(2))*obj.x(4) >= pi/3 & obj.q==1)                
%                 obj.q = 2;
%                
%              % Check Guards for state 3    
%              elseif(obj.x(3)>obj.G(3) & abs(obj.x(2))<obj.G(2))                 
%                 obj.q = 3;  
%                                   
%              else
%                 obj.q = 4;
%                                  
%              end          

             % Check Guards for state 1
             if ((abs(obj.x(2))>obj.G(2) & obj.q==1))
                new_q = 1;       
             end               
             % Check Guards for state 2    
             if ( (AbsoluteVelocity<=obj.G(3) & -sign(obj.x(2))*CurrentHeading <= 2*pi/3  & -sign(obj.x(2))*CurrentHeading >= 0 & obj.q==1) | (AbsoluteVelocity<= obj.G(3) & abs( obj.x(2))> obj.G(2) & obj.q==3) )                
                new_q = 2;
             end  
             % Check Guards for state 3    
             if(((AbsoluteVelocity>obj.G(3) & abs(obj.x(2))<obj.G(2)) | (AbsoluteVelocity>obj.G(3) & obj.q==2) | (AbsoluteVelocity>obj.G(3) & obj.q==3)))     %started walking and on the road or start walking from wait 
                new_q = 3;                                                                     
             end    
             % Check Guards for state 4  - y: 3.6 m
             if T(3,2)>0
                 if (obj.x(2)>=(obj.G(2)+0.1) & obj.q==3)
                     new_q = 4;
                 end
             else
                 if (obj.x(2)<(-obj.G(2)-0.1) & obj.q==3)
                     new_q = 4;
                 end
             end
             
             % state updates only when the gap is accepted or reamins the
             % same; if this condition is not placed, the state updates
             % solely based on the guards within the prediction horizon
             
             if ( (obj.q_decide==2 & new_q==2) | obj.q==new_q | new_q==4 | (obj.q_decide==3 & new_q==3 & walk_start_flag==1) | (new_q==3 & abs(obj.x(2))<obj.G(2)) )
                    obj.q = new_q;
             end
             
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % discrete state update from ground truth
        function pedestrian_action_update(obj,z_discrete)
            obj.q = z_discrete;  
        end  
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
           
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Reset states

           function pedestrian_state_reset(obj,T)
                          
              TargetHeading = atan2((T(2)-obj.x(2)),(T(1)-obj.x(1)));
              CurrentHeading = atan2(obj.x(4),obj.x(3));
%                if obj.q==2
%                    obj.x(3) = 0;
%                    if obj.x(2)>0
%                        obj.x(4) = -pi/2;
%                    else
%                        obj.x(4) = pi/2;
%                    end
%                elseif obj.q==3
%                    obj.q = atan2((T(2)-obj.x(2)),(T(1)-obj.x(1)));
%                elseif obj.q==4
%                    if abs(obj.x(4)) > pi/2    %heading towards left
%                        obj.x(4) = -sign(obj.x(2))*pi;
%                    else
%                        obj.x(4) = 0;
%                    end
%                    
%                end   

               if obj.q==2
                   obj.x(3) = 0;
                   obj.x(4) = 0;
%                    if obj.x(2)>0
%                        obj.x(4) = -pi/2;
%                    else
%                        obj.x(4) = pi/2;
%                    end
               elseif obj.q==3                                            
                   obj.x(3) = 0.1*rand(1)*cos(TargetHeading);     %random starting velocities when state changes to cross
                   obj.x(4) = rand(1)*sin(TargetHeading);
               
               elseif obj.q==4
                   if abs(CurrentHeading) > pi/2    %heading towards left
                       TargetHeading = -sign(obj.x(2))*pi;
                   else
                       TargetHeading = 0;
                   end
                   obj.x(3) = rand(1)*cos(TargetHeading);     %random starting velocities when state changes to cross
                   obj.x(4) = 0.1*rand(1)*sin(TargetHeading);                                 
               end  



           end
            
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
        function obj = ApproachToCross(obj,GapTest)
                pGap = obj.Prob.AcceptedGap*obj.Prob.AcceptedGapDistribution_Train(GapTest)./obj.Prob.GapDistribution_Train(GapTest);
%                 obj.pGaze = obj.Prob.GazeCross_Train*obj.Prob.CrossedGazeDistribution_Train(F.GazeTest)./obj.Prob.GazeDistribution_Train(F.GazeTest);
%                 obj.pSpeed = obj.Prob.SpeedCross_Train*obj.Prob.CrossedSpeedDistribution_Train(F.SpeedTest)./obj.Prob.SpeedDistribution_Train(F.SpeedTest);
                
                obj.p = pGap;
%                 obj.p = obj.pSpeed;
%                 obj.p = obj.pGap*obj.pGaze;
%                 obj.p = obj.pGap*obj.pSpeed;
%                 obj.p = obj.pGap*obj.pGaze*obj.pSpeed;



        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = WaitToCross(obj,GapTest)
                pGap = obj.Prob.AcceptedGap*obj.Prob.AcceptedGapDistribution_Train(GapTest)./obj.Prob.GapDistribution_Train(GapTest);
%                 obj.pGaze = obj.Prob.GazeAcceptance_Train*obj.Prob.AcceptedGazeDistribution_Train(F.GazeTest)./obj.Prob.GazeDistribution_Train(F.GazeTest);

                obj.p = pGap;
%                 obj.p = obj.pGap*obj.pGaze;

        end
        
            
    end
        
        
        
        
        
end