classdef IiwaScrewTheory < handle
    %IiwaScrewTheory Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        MInertiaMode='jsl'; %'aij'; %
        NPotentialMode='gwrench' %'difsym', %'aij'; %
    end
    methods
        function obj = IiwaScrewTheory()
           
        end
    end
    methods (Static, Access='public')  
        function cartesian_position = ForwardKinematics(joint_position)
            if (size(joint_position,1) == 1)
                joint_position=joint_position';
            end
            % Build TwMag for each point
            TwMag=[IiwaRobot.Twist; joint_position(1:size(IiwaRobot.Twist,2))'];
            % Calculate HstR (4by4 matrix) for that joint position
            HstR = IiwaScrewTheory.ForwardKinematicsPOE(TwMag);
            % Transform the matrix into [pos; eul] form (1by6 vec)
            Hst=HstR*IiwaRobot.Hst0;
            cartesian_position(1:3) = Hst(1:3, 4);
            cartesian_position(4:6) = rotm2eul(Hst(1:3, 1:3), 'XYZ');
        end
        function qdot = IDK_point(q_curr, xdot_S)
            %Check sizes
            if (size(xdot_S,2)== 1)
                xdot_S=xdot_S';
            end
            if (size(q_curr,2)~=7)
                q_curr=q_curr';
            end
            Jst = IiwaScrewTheory.GeoJacobianS([IiwaRobot.Twist; q_curr]);
            Jst_i = pinv(Jst); % inv(Jst);%
            qdot = (Jst_i*xdot_S')';
            far_ratio = abs(qdot)./abs(IiwaRobot.ThDotmax);
            farest = max(far_ratio);
            if (farest>1)
                %disp('Limited qdot');
                qdot = qdot./(farest);
            end
        end
        function q = IDK_IK_norot(xyz_obj, q_close)
            xyz_obj = xyz_obj(1:3);
            x_close = IiwaScrewTheory.ForwardKinematics(q_close);
            xyz_close = x_close(1:3);
            xyz_inc = xyz_obj(1:3) - xyz_close;
            points_per_cm = 50; 
            points = ceil(points_per_cm*norm(xyz_inc));
            for i=1:3
                xx(i,:)=linspace(xyz_close(i), xyz_obj(i), points);
            end
            xx(4:6,:) = x_close(4:6)'.*ones(3,points);
            q_curr = q_close;
            for i=1:size(xx,2)-1
                x_curr = IiwaScrewTheory.ForwardKinematics(q_curr);
                xinc_A = IiwaScrewTheory.screwA2B_A(x_curr, xx(:,i+1)');
                xinc_S = IiwaScrewTheory.tfscrew_A2S(xinc_A, x_curr);
                qinc = IiwaScrewTheory.IDK_point(q_curr, xinc_S); %To avoid qdot limits
                q_curr = q_curr + qinc;
            end
            q=q_curr;
            
%             data_output.qdot(:,size(xpos,2))=[0; 0; 0; 0; 0; 0; 0];
%             data_output.t = t;
%             data_output=fill_cartesian(data_output);
%             end
%             t=linspace(0, 10, npoints);
%             xpos=[linspace(xpos_ini(1), xpos_ini(1)+displacement(1), npoints); ...
%                              linspace(xpos_ini(2), xpos_ini(2)+displacement(2), npoints); ...
%                              linspace(xpos_ini(3), xpos_ini(3)+displacement(3), npoints)];
%             % !! Does NOT work for displacements in orientation, use
%             % IDK_point_straightline2 and absolute pos/ori, instead of relative
%             % ones
%             xori=[linspace(xori_ini(1), xori_ini(1)+displacement(4), npoints); ...
%                              linspace(xori_ini(2), xori_ini(2)+displacement(5), npoints); ...
%                              linspace(xori_ini(3), xori_ini(3)+displacement(6), npoints)];
%             data_output=IDK_trajectory(xpos, xori, t, q_close);    
%             q_end = data_output.q(:,end);
        end
        %% Complete trajectories with certain data
        % TODO: Change to IiwaTrajectoryGeneration
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
                    traj.xdot(i,:) = IiwaScrewTheory.screwA2B_A(traj.x(i,:), traj.x(i+1,:))/(traj.t(i+1)-traj.t(i));
                end
                xdot_S = IiwaScrewTheory.tfscrew_A2S(traj.xdot(i,:), traj.x(i,:));
                traj.qdot(i,:) = IiwaScrewTheory.IDK_point(traj.q(i,:), xdot_S);
                traj.q(i+1,:) = traj.q(i,:) + traj.qdot(i,:)*(traj.t(i+1)-traj.t(i));
            end
            traj.xdot(size(traj.x,1),:)=zeros(6,1);
            traj.qdot(size(traj.x,1),:)=zeros(size(IiwaRobot.Twist,2),1);
        end
        function traj = FillCartesianPositionsFromJointPositions(traj)
            for i=1:size(traj.q,1)
                traj.x(i,:)=IiwaScrewTheory.ForwardKinematics(traj.q(i,:));
            end
        end
        function screw_A = screwA2B_A (frameA, frameB)
            screw_A(1:3) = frameB(1:3)-frameA(1:3);
            screw_A(4:6) = IiwaScrewTheory.axangA2B_A(frameA(4:6), frameB(4:6));
        end
        function frameB = tfframe_A(frameA, screw_A)
            frameB(1:3) = frameA(1:3) + screw_A(1:3);
            frameB(4:6) = IiwaScrewTheory.rotframe_A(frameA(4:6), screw_A(4:6));
        end
        function screw_S = tfscrew_A2S (screw_A, frame_SA)
            %frame_A = screw representing the position and orientation of
            %frame A wrt to frame S
            %screw_A = a screw expressed in the frame A that you want it to
            %be represented in the space frame
            screw_S(4:6) = eul2rotm(frame_SA(4:6), 'XYZ')*screw_A(4:6)'; %
            screw_S(1:3) = screw_A(1:3) - cross(screw_S(4:6), frame_SA(1:3));
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
    end
    % Kinematics
    methods(Static, Access='private')
        function Hp = trvP2tform(p)
            Hp = [1 0 0 p(1,1); 0 1 0 p(2,1); 0 0 1 p(3,1); 0 0 0 1];
        end
        function HstR = ForwardKinematicsPOE(TwMag)
            HstR = IiwaScrewTheory.expScrew(TwMag(:,1));  
            for i = 2:size(TwMag,2)
                HstR = HstR*IiwaScrewTheory.expScrew(TwMag(:,i));
            end
        end
        function rotm = expAxAng(AxAng)
            ws = IiwaScrewTheory.axis2skew(AxAng(1:3));
            t = AxAng(1,4);
            rotm = eye(3)+ws*sin(t)+ws*ws*(1-cos(t));
        end
        function r = axis2skew(w)
            r = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
        end
        function JstS = GeoJacobianS(TwMag)
            Twist=TwMag(1:6, :);
            q = TwMag(7,:);
            JstS = Twist;
            PoE = IiwaScrewTheory.screw2tform(Twist(:,1), q(1));
            for i = 2:size(Twist,2)
                JstS(:,i) = IiwaScrewTheory.tform2adjoint(PoE)*Twist(:,i);
                PoE = PoE*IiwaScrewTheory.screw2tform(Twist(:,i), q(i));
            end
        end
        function Ad = tform2adjoint(T)
            R = tform2rotm(T);
            p = tform2trvec(T);
            Ad = [R IiwaScrewTheory.axis2skew(p)*R; zeros(3) R];
        end
        function H = expScrew(TwMag)
            v = TwMag(1:3); % "vee" component of the TWIST.
            w = TwMag(4:6); % "omega" component of the TWIST.
            t = TwMag(7);   % "theta" magnitude component of the SCREW.
            if norm(w) == 0 % only translation
               r = eye(3);
               p = v*t;
            elseif (norm(v)==0)
               r = IiwaScrewTheory.expAxAng([w' t]);
               p = (eye(3)-r)*(cross(w,v)); % for only rotation joint.
            else
               r = IiwaScrewTheory.expAxAng([w' t]);
               p = (eye(3)-r)*(cross(w,v))+w*w'*v*t; % for general (rot+tra) screw.     
            end
            H = [r, p; [0 0 0 1]];
        end
        function w = skew2axis(r)
            w = [r(3,2); r(1,3); r(2,1)];
        end
        function tform = twist2tform(xi)
            wr = IiwaScrewTheory.axis2skew([xi(4,1);xi(5,1);xi(6,1)]);
            tform = [wr(1,1) wr(1,2) wr(1,3) xi(1,1);
                 wr(2,1) wr(2,2) wr(2,3) xi(2,1);
                 wr(3,1) wr(3,2) wr(3,3) xi(3,1); 0 0 0 0];
        end
        function xi = tform2twist(tform)
            s = IiwaScrewTheory.skew2axis([tform(1,1) tform(1,2) tform(1,3);
                 tform(2,1) tform(2,2) tform(2,3);
                 tform(3,1) tform(3,2) tform(3,3)]);
            xi = [tform(1,4);tform(2,4);tform(3,4);s(1,1);s(2,1);s(3,1)];
        end
        function H = screw2tform(twist, theta)
            v = twist(1:3); 
            w = twist(4:6);
            if norm(w) == 0 % only translation
               r = eye(3);
               p = v*theta;
            else
                %axang2rotm
                if (norm(w)~=1)
                    disp('Warning, received a non-unit axis');
                end
                r = axang2rotm([w' theta]);
                %cross
                p = (eye(3)-r)*(cross(w,v)); % for only rotation joint.     
            end
            H = [r, p; [0 0 0 1]];
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
        function screw_A = tfscrew_S2A (screw_S, frame_SA)
            screw_A(1:3) = screw_S(1:3) + cross(screw_S(4:6), frame_SA(1:3));
            screw_A(4:6) = eul2rotm(frame_SA(4:6), 'XYZ')'*screw_S(4:6)';
        end
        function xdot_S = DK_point (q_curr, qdot)
            if (size(qdot,2)==1)
                qdot=qdot';
            end
            if (size(q_curr,2)~=7)
                q_curr=q_curr';
            end
            Jst = IiwaScrewTheory.GeoJacobianS([IiwaRobot.Twist; q_curr]);
            xdot_S = (Jst*qdot')';
        end
        function xdot_A = GetCartesianVelocityStraightLine(qini, xgoal, ttotal)
            xinc_total_A = IiwaScrewTheory.GetCartesianIncrementStraightLine(qini, xgoal);
            xdot_A = xinc_total_A/ttotal; %meters/radians per second
        end
    end
    %Dynamics
    methods (Static, Access='public')
        function Tdyn = InverseDynamics(q, qdot, qdotdot)
            Twist = IiwaRobot.Twist;
            TwMag = [Twist; q];
            LiMas = IiwaRobot.LiMas;
            PoAcc = IiwaRobot.PoAcc;
            MtST24RJsl = MInertiaJsl(TwMag,LiMas);
            CtdtST24RAij = CCoriolisAij(TwMag,LiMas,qdot);
            NtST24RWre = NPotentialWre(TwMag,LiMas,PoAcc);
            Tdyn = MtST24RJsl*qdotdot' + CtdtST24RAij*qdot' + NtST24RWre;
%             Mt = IiwaScrewTheory.MInertia(q);
%             Ctdt = IiwaScrewTheory.CCoriolis(q,qdot);
%             Nt = IiwaScrewTheory.NPotential(q);
%             Tdyn = Mt*qdotdot' + Ctdt*qdot' + Nt;
        end
    	function Mt = MInertia(q)
            LiMas = IiwaRobot.LiMas;
            n_joints = IiwaRobot.n_joints;
            Twist = IiwaRobot.Twist;
            TwMag = [Twist; q];
            if (strcmp(IiwaScrewTheory.MInertiaMode, 'jsl')==1)
                Mt = zeros(n_joints);
                for i = 1:n_joints
                    Hsli0 = trvP2tform(LiMas(1:3,i));
                    Ii = LiMas(4:6,i);
                    mi = LiMas(7,i);        
                    Mi = [mi*eye(3),zeros(3);zeros(3),[Ii(1) 0 0; 0 Ii(2) 0; 0 0 Ii(3)]];
                    JslT = IiwaScrewTheory.GeoJacobianL(TwMag, Hsli0,i);
                    ImL = JslT'*Mi*JslT;
                    Mt = Mt+ImL;
                end
            elseif (strcmp(IiwaScrewTheory.MInertiaMode, 'aij')==1)
                Mt = zeros(n_joints);
                for i = 1:n_joints
                    for j = 1:n_joints
                        k = max(i,j);
                        for l = k:n_joints
                            Hsli0 = IiwaScrewTheory.trvP2tform(LiMas(1:3,l));
                            Ii = LiMas(4:6,l);
                            mi = LiMas(7,l);
                            MISij = Twist(:,i)'*IiwaScrewTheory.Aij2adjoint(l,i,TwMag)';
                            MISij = MISij*IiwaScrewTheory.LinkInertiaS(Hsli0,Ii,mi);
                            MISij = MISij*IiwaScrewTheory.Aij2adjoint(l,j,TwMag)*Twist(:,j);
                            Mt(i,j) = Mt(i,j) + MISij;
                        end
                    end
                end
            else
                disp ('Wrong definition for IiwaScrewTheory.MInertiaMode');
            end
        end
        function Ctdt = CCoriolis(q, qdot)
            %aij
            n = IiwaRobot.n_joints;
            TwMag = [IiwaRobot.Twist; q];
            LiMas = IiwaRobot.LiMas;
            Ctdt = zeros(n);
            for i = 1:n
                for j = 1:n
                    for k = 1:n
                        Cosij = IiwaScrewTheory.Christoffel(TwMag,LiMas,i,j,k);
                        Cosij = Cosij+IiwaScrewTheory.Christoffel(TwMag,LiMas,i,k,j);
                        Cosij = Cosij-IiwaScrewTheory.Christoffel(TwMag,LiMas,k,j,i);
                        Cosij = Cosij*qdot(k);
                        Ctdt(i,j)= Ctdt(i,j) + Cosij;
                    end
                    Ctdt(i,j) = 1/2 * Ctdt(i,j);
                end
            end
        end
        function Nt = NPotential(q)
            LiMas = IiwaRobot.LiMas;
            n_joints = IiwaRobot.n_joints;
            Twist = IiwaRobot.Twist;
            TwMag = [Twist; q];
            if (strcmp(IiwaScrewTheory.NPotentialMode, 'difsym')==1)
            	syms t1 t2 t3 t4 t5 t6 t7; % maximum for a robot with 7 DOF
                ThetaSymmax = [t1 t2 t3 t4 t5 t6 t7];
                ThSym = ThetaSymmax(:,1:n_joints);
                TwMagSym = [TwMag(1:6,1:n_joints); ThSym];
                NtSym = ThSym';
                VtSym = zeros(3,1);
                for i = 1:n_joints
                    Hsli0 = IiwaScrewTheory.trvP2tform(LiMas(1:3,i));
                    Hslit = IiwaScrewTheory.ForwardKinematicsPOE(TwMagSym(:,1:i))*Hsli0;
                    VtSym = VtSym + diag(LiMas(7,i)*(IiwaRobot.PoAcc))*Hslit(1:3,4);
                end
                for i = 1:n_joints
                    NtSym(i) = diff(VtSym(1),ThSym(i));
                    NtSym(i) = NtSym(i) + diff(VtSym(2),ThSym(i));
                    NtSym(i) = NtSym(i) + diff(VtSym(3),ThSym(i));
                end
                NtSym = simplify(NtSym);
                Nt = - double(subs(NtSym, ThSym, TwMag(7,1:n_joints)));
            elseif (strcmp(IiwaScrewTheory.NPotentialMode, 'aij')==1)
                Eg = [IiwaRobot.PoAcc; 0; 0; 0];
                Nt = zeros(n_joints,1);
                for i = 1:n_joints
                    for l = i:n_joints
                        Hsli0 = IiwaScrewTheory.trvP2tform(LiMas(1:3,l));
                        Ii = LiMas(4:6,l);
                        mi = LiMas(7,l);
                        Nti = TwMag(1:6,i)'*IiwaScrewTheory.Aij2adjoint(l,i,TwMag)';
                        Nti = Nti*IiwaScrewTheory.LinkInertiaS(Hsli0,Ii,mi);
                        Nti = Nti*IiwaScrewTheory.Aij2adjoint(l,1,TwMag)*Eg;
                        Nt(i) = Nt(i) - Nti;
                    end
                end
            elseif (strcmp(IiwaScrewTheory.NPotentialMode, 'gwrench')==1)
                forcetype = 'tra';
                MagnG = norm(IiwaRobot.PoAcc);
                AxisG = IiwaRobot.PoAcc/MagnG;
                Wrench = zeros(6,n_joints);
                for i = 1:n_joints
                    Hsli0 = IiwaScrewTheory.trvP2tform(LiMas(1:3,i));
                    Hslit = IiwaScrewTheory.ForwardKinematicsPOE(TwMag(:,1:i))*Hsli0;
                    mi = LiMas(7,i);
                    Wrench(:,i) = mi*MagnG*(IiwaScrewTheory.link2wrench(AxisG, Hslit(1:3,4), forcetype));
                end
                Nt = zeros(n_joints,1);
                for i = 1:n_joints  
                    JstS = zeros(6,n_joints);
                    JstS(:,1:i) = IiwaScrewTheory.GeoJacobianS(TwMag(:,1:i));
                    Ntnew = JstS'*Wrench(:,i);
                    Nt = Nt - Ntnew;
                end
            else
                disp ('Wrong definition for IiwaScrewTheory.NPotential');
            end
        end
        function JslT = GeoJacobianL(TwMag, Hsli0, Li)
            % q: Screw for the joint screw movement
            % Hsli0: Pose of the link frame L associated to the COM of that 
            % link at the reference (home) configuration or the manipulator.
            % Li: Id of that link
            % JslL = (Ad(Hsl0)^-1)*[Ai1*E1 ... Aii*Ei 0 ... 0] (6xn)
            % where Aij = Aij2Adjoint and En are the Twists.
            JslT = zeros(6,IiwaRobot.n_joints);
            AdHsli0 = IiwaScrewTheory.tform2adjoint(Hsli0);
            for j = 1:IiwaRobot.n_joints
                JslT(:,j) = AdHsli0\(IiwaScrewTheory.Aij2adjoint(Li,j,TwMag)*IiwaRobot.Twist(:,j));
            end
        end
        function Ad = Aij2adjoint(i,j,TwMag)
            AZ = zeros(6);
            AI = eye(6);
            if i<j
                Ad = AZ;
            elseif i==j
                Ad = AI;    
            else
                PoE = IiwaScrewTheory.expScrew(TwMag(:,j+1));
                for k = j+2:i
                    PoE = PoE*IiwaScrewTheory.expScrew(TwMag(:,k)); 
                end 
                Ad = IiwaScrewTheory.tform2adjoint(PoE)\AI;
            end
        end
        function ImS = LinkInertiaS(Hsli0,Ii,mi)
            AI = eye(6);
            Im = [mi*eye(3),zeros(3);zeros(3),[Ii(1) 0 0; 0 Ii(2) 0; 0 0 Ii(3)]];
            AdHsli0 = IiwaScrewTheory.tform2adjoint(Hsli0)\AI; % Ad(Hsli0)^-1
            ImS = AdHsli0'*Im*AdHsli0;
        end
        function dMt = Christoffel(TwMag,LiMas,i,j,k)
            n = size(TwMag,2);    
            m = max(i,j);
            dMt = 0;
            for l = m:n
                Hsli0 = IiwaScrewTheory.trvP2tform(LiMas(1:3,l));
                Ii = LiMas(4:6,l);
                mi = LiMas(7,l);
                dMt1 = IiwaScrewTheory.twistbracket(IiwaScrewTheory.Aij2adjoint(k,i,TwMag)*TwMag(1:6,i),TwMag(1:6,k))';
                dMt1 = dMt1 * IiwaScrewTheory.Aij2adjoint(l,k,TwMag)'*IiwaScrewTheory.LinkInertiaS(Hsli0,Ii,mi);
                dMt1 = dMt1 * IiwaScrewTheory.Aij2adjoint(l,j,TwMag)*TwMag(1:6,j);
                dMt2 = TwMag(1:6,i)'*IiwaScrewTheory.Aij2adjoint(l,i,TwMag)';
                dMt2 = dMt2 *IiwaScrewTheory.LinkInertiaS(Hsli0,Ii,mi)*IiwaScrewTheory.Aij2adjoint(l,k,TwMag);
                dMt2 = dMt2 *IiwaScrewTheory.twistbracket(IiwaScrewTheory.Aij2adjoint(k,j,TwMag)*TwMag(1:6,j),TwMag(1:6,k));
                dMt = dMt + dMt1 + dMt2;
            end
        end
        function fo = link2wrench(ForceAxis, ForcePoint, forceType)
            if forceType == "rot"
               fo = [0; 0; 0; ForceAxis];
            elseif forceType == "tra"
               fo = [ForceAxis; -cross(ForceAxis,ForcePoint)];
            else
               fo = [0; 0 ; 0: 0; 0; 0];
            end
        end
        function xi = twistbracket(x1,x2)
            E1 = IiwaScrewTheory.twist2tform(x1);
            E2 = IiwaScrewTheory.twist2tform(x2);
            xi = IiwaScrewTheory.tform2twist(E1*E2-E2*E1);
        end
    end
end

