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
            if (size(joint_position) == [1 7])
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
        
        function axang3A2B_A = axangframeA2B_A(oriA, oriB)
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
        function screw = screwframeA2B_S(frameA, frameB, time)
            axang3A2B_A = ScrewTheory.axangframeA2B_A(frameA(1:3), frameB(1:3));
            axang3A2B_S = eul2rotm(frameA(4:6), 'XYZ') * axang3A2B_A';
            %Divide by the time
            screw=axang3A2B_S'/time;
        end
        function screw_A = frameA2B_A (frameA, frameB)
            screw_A(1:3) = frameB(1:3)-frameA(1:3);
            screw_A(4:6) = ScrewTheory.axangframeA2B_A(frameA(4:6), frameB(4:6));
        end
        function oriB = rotframe_A(oriA, axang_A)
            angle = norm(axang_A);
            axis_A = axang_A/angle;
            R_AB = axang2rotm([axis_A, angle]);
            R_SA = eul2rotm(oriA, 'XYZ');
            R_SB = R_SA*R_AB;
            oriB = rotm2eul(R_SB, 'XYZ');
        end
        function frameB = transformframe_A(frameA, screw_A)
            frameB(1:3) = frameA(1:3) + screw_A(1:3);
            frameB(4:6) = ScrewTheory.rotframe_A(frameA(4:6), screw_A(4:6));
        end
        function screw_S = transformscrewA2S (screw_A, frame_A)
            %frame_A = screw representing the position and orientation of
            %frame A wrt to frame S
            %screw_A = a screw expressed in the frame A that you want it to
            %be represented in the space frame
            screw_S(4:6) = eul2rotm(frame_A(4:6), 'XYZ')*screw_A(4:6)';
            screw_S(1:3) = screw_A(1:3) - cross(screw_S(4:6), frame_A(1:3));
        end
        function qdot = IDK_point(q_curr, xdot)
            %TODO: Check sizes
            if (size(xdot)==[6 1])
                xdot=xdot';
            end
            Jst = ScrewTheory.GeoJacobianS(IiwaRobot.Twist, q_curr);
            Jst_i = pinv(Jst);
            qdot = (Jst_i*xdot')';
            far_ratio = abs(qdot)./abs(IiwaRobot.ThDotmax);
            farest = max(far_ratio);
            if (farest>1)
                qdot = qdot./(farest);
            end
       end
        %%
        
        function qdot = InverseDifferentialKinematics_point(q_curr, x_next, time)
            % Calculate current cartesian position using forward kinematics
            x_curr=ScrewTheory.ForwardKinematics(q_curr);
            % Calculate cartesian velocity - Position
            v_xyz=(x_next(1:3)-x_curr(1:3))/time;
            % Calculate cartesian velocity - Orientation
            w_xyz=ScrewTheory.substract_ori(x_curr(4:6), x_next(4:6), time);
            % Transform velocity in screw form
            v_screw=v_xyz-cross(w_xyz,x_curr(1:3));
            w_screw=w_xyz;
            cart_vel_screw=[v_screw, w_screw]';
            qdot = ScrewTheory.IDK_point(q_curr, cart_vel_screw);
%             % Calculate space jacobian
%             Jst=ScrewTheory.GeoJacobianS(IiwaRobot.Twist, q_curr);
%             % Calculate joint velocity with Moore-Penrose pseudojacobian
%             Jst_i=pinv(Jst);%Jst'*inv(Jst*Jst');%pinv(Jst);
%             qdot=(Jst_i*cart_vel_screw)';
%             % Limit velocity to joint_vel_limits
%             far_ratio = abs(qdot)./abs(IiwaRobot.ThDotmax);
%             farest = max(far_ratio);
%             if (farest>1)
%                 qdot = qdot./(farest);
%             end
            %qdot=min(abs(qdot), abs(IiwaRobot.ThDotmax)).*sign(qdot);
            % Calculate joint_position that the robot should achieve moving at that
            % velocity during the given time
           % q_next=q_curr(1:size(qdot,2))+qdot*time;
            % Limit position to limits
            %q_next=min(abs(q_next), abs(IiwaRobot.Thmax)).*sign(q_next);
        end

%         function axang_S = axangframeA2B(oriA, oriB)
%             % Gets the axis (in the space frame) and angle
%             % needed to rotate a frame with orientation oriA into a frame 
%             % with orientation oriB following the smaller path
%             R_SA = eul2rotm(oriA, 'XYZ');
%             R_SB = eul2rotm(oriB, 'XYZ');
%             R_AS = R_SA';
%             R_AB = R_AS * R_SB;
%             axang_AB_A = rotm2axang(R_AB);
%             axis = R_SA * axang_AB_A(1:3)';
%             angle = axang_AB_A(4);
%             axang_S = (axis*angle)';
%         end
% 
%         function frameB = rotateframe(oriA, axang_S)
%             %Axis expressed in the space frame, frame A expressed in the
%             %space frame, frameB expressed in the space frame
%             angle = norm(axang_S);
%             axis_S = axang_S/angle;
%             axis_A = eul2rotm(oriA,'XYZ')' * axis_S';
%             R_AB = axang2rotm([axis_A', angle]);
%             R_SA = eul2rotm(oriA, 'XYZ');
%             R_SB = R_SA*R_AB;
%             frameB = rotm2eul(R_SB, 'XYZ');
%         end
%         function screwA2B = frameA2B (frameA, frameB)
%             v_A = frameB(1:3) - frameA(1:3);
%             w_S = ScrewTheory.axangframeA2B(frameA(4:6), frameB(4:6));
%             v_S = v_A - cross(w_S, frameA(1:3));
%             screwA2B = [v_A w_S];
%         end
%         function frameB = transformframe (frameA, screwA2B)
%             frameB(1:3) = frameA(1:3) + screwA2B(1:3);
%             frameB(4:6) = ScrewTheory.rotateframe(frameA(4:6), screwA2B(4:6));
%         end
%         function x = substract_cartesian(x_ini, x_end)
%             x(1:3)=x_end(1:3)-x_ini(1:3);
%             x(4:6)=substract_ori(x_ini(4:6), x_end(4:6), 1);
%         end
    end
end

