classdef ScrewTheory < handle
    %SCREWTHEORY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    methods
        function obj = ScrewTheory()
           
        end
    end
    methods (Static)  
        function cartesian_position = ForwardKinematics(joint_position)
            if (size(joint_position,1) == 1)
                joint_position=joint_position';
            end
            % Build TwMag for each point
            TwMag=[IiwaRobot.Twist; joint_position(1:size(IiwaRobot.Twist,2))'];
            % Calculate HstR (4by4 matrix) for that joint position
            HstR = ScrewTheory.expScrew(TwMag(:,1));  
            for j = 2:size(TwMag,2)
                HstR = HstR*ScrewTheory.expScrew(TwMag(:,j));
            end
            % Transform the matrix into [pos; eul] form (1by6 vec)
            Hst=HstR*IiwaRobot.Hst0;
            cartesian_position(1:3) = Hst(1:3, 4);
            cartesian_position(4:6) = rotm2eul(Hst(1:3, 1:3), 'XYZ');
        end
        function H = expScrew(TwMag)
            v = TwMag(1:3); % "vee" component of the TWIST.
            w = TwMag(4:6); % "omega" component of the TWIST.
            t = TwMag(7);   % "theta" magnitude component of the SCREW.
            if norm(w) == 0 % only translation
               r = eye(3);
               p = v*t;
            else
               r = ScrewTheory.expAxAng([w' t]);
               p = (eye(3)-r)*(cross(w,v)); % for only rotation joint.
        %      p = (eye(3)-r)*(cross(w,v))+w*w'*v*t; % for general (rot+tra) screw.     
            end
            H = [r, p; [0 0 0 1]];
        end
        function rotm = expAxAng(AxAng)
            ws = ScrewTheory.axis2skew(AxAng(1:3));
            t = AxAng(1,4);
            rotm = eye(3)+ws*sin(t)+ws*ws*(1-cos(t));
        end
        function r = axis2skew(w)
            r = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
        end
        function JstS = GeoJacobianS(twists, joint_pos)
            JstS = twists;
            PoE = ScrewTheory.screw2tform(twists(:,1), joint_pos(1));
            for i = 2:size(twists,2)
                JstS(:,i) = ScrewTheory.tform2adjoint(PoE)*twists(:,i);
                PoE = PoE*ScrewTheory.screw2tform(twists(:,i), joint_pos(i));
            end
        end
        function Ad = tform2adjoint(T)
            R = tform2rotm(T);
            p = tform2trvec(T);
            Ad = [R ScrewTheory.axis2skew(p)*R; zeros(3) R];
        end
        function H = screw2tform(twist, theta)
            v = twist(1:3); 
            w = twist(4:6);
            if norm(w) == 0 % only translation
               r = eye(3);
               p = v*theta;
            else
                %axang2rotm
                r = axang2rotm([w' theta]);
                %cross
                p = (eye(3)-r)*(cross(w,v)); % for only rotation joint.     
            end
            H = [r, p; [0 0 0 1]];
        end
        
        function axang3A2B_A = axangA2B_A(oriA, oriB)
            %Get rotation matrices that express orientations A and B, which
            %are the rotations from space frame S to frames A or B
            R_SA = eul2rotm(oriA, 'XYZ');
            R_SB = eul2rotm(oriB, 'XYZ');
            %Rotation matrix from A to S will be the transposed of the
            %rotation matrix from S to A
            R_AS = R_SA';
            %Rotation matrix from A to B will be rotation matrix from A to
            %S multiplied by rotation matrix from S to B
            R_AB = R_AS * R_SB;
            %Transform this rotation matrix into the axang4 notation. Three
            %first elements are the unit axis of rotation (in the A frame)
            %and the fourth is the magnitude (angle) of that rotation. 
            axang4A2B_A = rotm2axang(R_AB);
            %Transform this into the axis notation, which will be the
            %unit axis multiplied by the magnitude, expressed in the A
            %frame.
            axang3A2B_A = axang4A2B_A(1:3)*axang4A2B_A(4);
        end
        function screw_A = screwA2B_A (frameA, frameB)
            screw_A(1:3) = frameB(1:3)-frameA(1:3);
            screw_A(4:6) = ScrewTheory.axangA2B_A(frameA(4:6), frameB(4:6));
        end
        function oriB = rotframe_A(oriA, axang_A)
            angle = norm(axang_A);
            if (angle~=0)
                axis_A = axang_A/angle;
                R_AB = axang2rotm([axis_A, angle]);
                R_SA = eul2rotm(oriA, 'XYZ');
                R_SB = R_SA*R_AB;
                oriB = rotm2eul(R_SB, 'XYZ');
            else
                oriB = oriA;
            end
        end
        function frameB = tfframe_A(frameA, screw_A)
            frameB(1:3) = frameA(1:3) + screw_A(1:3);
            frameB(4:6) = ScrewTheory.rotframe_A(frameA(4:6), screw_A(4:6));
        end
        function screw_S = tfscrew_A2S (screw_A, frame_SA)
            %frame_A = screw representing the position and orientation of
            %frame A wrt to frame S
            %screw_A = a screw expressed in the frame A that you want it to
            %be represented in the space frame
            screw_S(4:6) = eul2rotm(frame_SA(4:6), 'XYZ')*screw_A(4:6)'; %screw_A(4:6);%
            screw_S(1:3) = screw_A(1:3) - cross(screw_S(4:6), frame_SA(1:3));
        end
        function screw_A = tfscrew_S2A (screw_S, frame_SA)
            screw_A(1:3) = screw_S(1:3) + cross(screw_S(4:6), frame_SA(1:3));
            screw_A(4:6) = eul2rotm(frame_SA(4:6), 'XYZ')'*screw_S(4:6)';
        end
        function qdot = IDK_point(q_curr, xdot_S)
            %Check sizes
            if (size(xdot_S,2)== 1)
                xdot_S=xdot_S';
            end
            Jst = ScrewTheory.GeoJacobianS(IiwaRobot.Twist, q_curr);
            Jst_i = pinv(Jst); % inv(Jst);%
            qdot = (Jst_i*xdot_S')';
            far_ratio = abs(qdot)./abs(IiwaRobot.ThDotmax);
            farest = max(far_ratio);
            if (farest>1)
                disp('Limited qdot');
                qdot = qdot./(farest);
            end
        end
        function xdot_S = DK_point (q_curr, qdot)
            if (size(qdot,2)==1)
                qdot=qdot';
            end
            Jst = ScrewTheory.GeoJacobianS(IiwaRobot.Twist, q_curr);
            xdot_S = (Jst*qdot')';
        end
        function xdot_A = GetCartesianVelocityStraightLine(qini, xgoal, ttotal)
            xinc_total_A = ScrewTheory.GetCartesianIncrementStraightLine(qini, xgoal);
            xdot_A = xinc_total_A/ttotal; %meters/radians per second
        end
        function xinc_total_A = GetCartesianIncrementStraightLine(qini,xgoal)
            xini = ScrewTheory.ForwardKinematics(qini);
            xinc_total_A = ScrewTheory.screwA2B_A(xini, xgoal);
        end
     
        %% Build trajectory
        function traj = BuildTrapezoidalTrajectory(qini, xgoal, ttotal, control_step_size, name) %6coord
            traj = IiwaTrajectory(name);
            xini= ScrewTheory.ForwardKinematics(qini);
            xinc_A = ScrewTheory.GetCartesianIncrementStraightLine(qini, xgoal);

            axis_rot=xinc_A(4:6)/norm(xinc_A(4:6));
            angle=norm(xinc_A(4:6));
            
            axis_tras=xinc_A(1:3)/norm(xinc_A(1:3));
            dist=norm(xinc_A(1:3));
            
            [a_pos, tacc, tflat] = ScrewTheory.GetTrapezoidalTrajectoryTimeParameters(ttotal, dist, IiwaRobot.CartAccMax, control_step_size);
            a_ori = ScrewTheory.GetTrapezoidalTrajectoryAcceleration(angle, tacc, tflat);
            
            [~, xpos, xdotpos, xdotdotpos] = ScrewTheory.BuildTrapezoidalTrajectory1Coord(tacc, tflat, a_pos, control_step_size);
            [t, xori, xdotori, xdotdotori] = ScrewTheory.BuildTrapezoidalTrajectory1Coord(tacc, tflat, a_ori, control_step_size);
            
            %Deparametrize
            traj.t = t;
            traj.xdot = [xdotpos*axis_tras, xdotori*axis_rot];
            traj.xdotdot = [xdotdotpos*axis_tras, xdotdotori*axis_rot];
            for i=1:size(traj.t,1)
                traj.x(i,:) = ScrewTheory.tfframe_A(xini, [xpos(i)*axis_tras, xori(i)*axis_rot]);
            end
        end
        
        function [t, x, xdot, xdotdot] = BuildTrapezoidalTrajectory1Coord(tacc, tflat, a, step_size)          
            if (isinf(tacc) || isinf(tflat))
                t=0;
                x=0;
                xdot=0;
                xdotdot=0;
                return;
            end
            ttotal = tacc*2 + tflat;
            %Build parameterized trajectory
            ids_acc = 2:round(tacc/step_size)+1;
            ids_flat = round(tacc/step_size)+1:round((tacc+tflat)/step_size)+1;
            ids_dec = round((tacc+tflat)/step_size)+1:round(ttotal/step_size)+1;

            xdotdot(ids_acc,:) = a;
            xdotdot(ids_flat,:) = 0;
            xdotdot(ids_dec(1:end-1),:) = -a;
            xdotdot(ids_dec(end),:) = 0;
            t = (0:step_size:ttotal)';
            x = zeros(size(t,1), 1);
            xdot = zeros(size(t,1), 1);
            x(1) = 0;
            xdot(1) = 0;
            for i=ids_acc(2:end)
                x(i) = 0.5 * a * t(i)*t(i);
                xdot(i) = a*t(i);
            end
            for i=ids_flat(2:end)
               t_=t(i)-t(ids_flat(1));
               x(i) = x(ids_flat(1))+xdot(ids_flat(1))*t_;
               xdot(i)=xdot(ids_flat(1));
            end
            for i=ids_dec(2:end)
                t_=t(i)-t(ids_dec(1));
                x(i) = x(ids_dec(1)) + xdot(ids_dec(1))*t_-0.5*a*t_*t_;
                xdot(i) = xdot(ids_dec(1)) -a*t_;
            end
        end
        
        function a = GetTrapezoidalTrajectoryAcceleration(dtotal, tacc, tflat)
            a = dtotal/(tacc*tflat + tacc*tacc);
        end
        
        function [a, tacc, tflat] = GetTrapezoidalTrajectoryTimeParameters(ttotal, dtotal, a_max, time_step)
            %%First solve for maximum acceleration
            % Opt1:Using the roots function
            %p = [1/a_max, -ttotal, dtotal];
            %vflat_sols = roots(p);
            a_min = (4*dtotal)/(ttotal*ttotal);
            %a_max=a_min;
            % Opt2: Using the formula for 2nd grade polynomials
            vflat_sols(1,:)=ttotal*a_max/2 + a_max*sqrt(ttotal*ttotal-4*dtotal/a_max)/2;
            vflat_sols(2,:)=ttotal*a_max/2 - a_max*sqrt(ttotal*ttotal-4*dtotal/a_max)/2;
            
            if (~isreal(vflat_sols))
                disp('Not enough time or acceleration to reach the position')
                a=inf;
                tacc=inf;
                tflat=0;
                return 
            end
            tflat_sols= ttotal -2*vflat_sols./a_max;
            tacc_sols = vflat_sols./a_max;
            %Select only those with t>0 and v>0
            vflat = vflat_sols(tflat_sols>0 & tacc_sols>0 & vflat_sols>0);
            tflat = tflat_sols(tflat_sols>0 & tacc_sols>0 & vflat_sols>0);
            tacc = tacc_sols(tflat_sols>0 & tacc_sols>0 & vflat_sols>0);
            %Fit the values to the discrete system
            %First find tacc multiple of control_step_size
            tacc = ceil(tacc/time_step)*time_step; %ceil to avoid incrementing a_flat
            %Get tflat for that tacc
            tflat = ttotal - 2*tacc;
            %Find acceleration for these values, which no longer will be
            %IiwaRobot.CartPosAccMax, to fit the trajectory correctly
            a = ScrewTheory.GetTrapezoidalTrajectoryAcceleration(dtotal, tacc, tflat);
            vflat = tacc * a;
%             x1 = vflat*vflat/(2*a)
%             x2 = x1 + vflat*tflat
%             x3 = x2 + vflat*tacc -0.5*a*tacc*tacc
        end
        
        function traj_output = BuildStraightTrajectoryAtGivenVelocity(q_ini, x_goal, control_step_size, velocity, name)
            %velocity is a number from 0 to 1 expressing the percentage of
            %the maximum qdot used in the whole trajectory
            
            %Build a straight trajectory using a fixed total_time (high
            %enough so it doesnt get limited by qdot limit
            time_first_approach = 20;
            tic
            traj_straight = ScrewTheory.BuildTrapezoidalTrajectory(q_ini, x_goal, time_first_approach, control_step_size, 'straight');
            toc
            %Get joint positions and velocities needed to follow that
            %straight trajectory (in theory)
            traj_idk = ScrewTheory.FillJointPositionsFromCartesianPositions(traj_straight, q_ini);
            %For each joint, get the maximum velocity it reaches in the
            %whole trajectory
            max_qdot = max(abs(traj_idk.qdot));
            %Get the percentage of the maximum joint velocity
            percentage_qdot = max_qdot./IiwaRobot.ThDotmax;
            %Get the joint percentage that is nearer its joint velocity limits
            max_percentage = max(percentage_qdot);
            %Therefore, the time may be multiplied by this percentage to
            %find the minimum time needed to follow the trajectory without
            %reaching the limits. Divide it by 0.9 to leave a margin to
            %correct errors
            time_minimum = time_first_approach * (max_percentage/0.9);
            %Now apply the velocity factor to the time
            time_new = time_minimum/velocity;
            %And adjust it the control_step_size
            time_new = ceil(time_new/control_step_size)*control_step_size;
            %Finally recalculate the straight trajectory for this time
            traj_output = ScrewTheory.BuildTrapezoidalTrajectory(q_ini, x_goal, time_new, control_step_size, name);
        end
        %% Complete trajectories with certain data
        function traj = FillJointPositionsFromCartesianPositions(traj, qini)
            traj.q(1,:)=qini;
            if (isempty(traj.xdot))
                bCalcXdot=1;
            else
                bCalcXdot=0;
            end
            for i=1:size(traj.x,1)-1
                if (bCalcXdot)
                    %approximation, not completely correct
                    traj.xdot(i,:) = ScrewTheory.screwA2B_A(traj.x(i,:), traj.x(i+1,:))/(traj.t(i+1)-traj.t(i));
                end
                xdot_S = ScrewTheory.tfscrew_A2S(traj.xdot(i,:), traj.x(i,:));
                traj.qdot(i,:) = ScrewTheory.IDK_point(traj.q(i,:), xdot_S);
                traj.q(i+1,:) = traj.q(i,:) + traj.qdot(i,:)*(traj.t(i+1)-traj.t(i));
            end
            traj.xdot(size(traj.x,1),:)=zeros(6,1);
            traj.qdot(size(traj.x,1),:)=zeros(size(IiwaRobot.Twist,2),1);
        end
        
        function traj = FillCartesianPositionsFromJointPositions(traj)
            for i=1:size(traj.q,1)
                traj.x(i,:)=ScrewTheory.ForwardKinematics(traj.q(i,:));
            end
        end
    end
end

