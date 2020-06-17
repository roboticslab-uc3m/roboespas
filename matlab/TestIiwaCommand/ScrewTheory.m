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
            elseif (norm(v)==0)
               r = ScrewTheory.expAxAng([w' t]);
               p = (eye(3)-r)*(cross(w,v)); % for only rotation joint.
            else
               r = ScrewTheory.expAxAng([w' t]);
               p = (eye(3)-r)*(cross(w,v))+w*w'*v*t; % for general (rot+tra) screw.     
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

