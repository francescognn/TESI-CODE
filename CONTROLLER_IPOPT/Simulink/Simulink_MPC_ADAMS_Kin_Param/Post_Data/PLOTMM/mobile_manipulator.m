classdef mobile_manipulator
    properties
        a
        d
        alpha
        
        x_ur5_base
        y_ur5_base
        z_ur5_base
        phi_ur5_base
        
        T_ur5_mobilebase
        base_length
        base_breadth
        base_height
        
        base_center_height
        wheel_offset
        wheel_radius
        wheel_length
        wheel_base
        wheel_track
        
        corner_sphere_rad
        center_sphere_rad
        
        DOFs
        mb_DOFs
        %         sphere_radii
    end
    
    methods (Access = private)
        function T = DH(self, a, alpha, d, theta)
            T = [[cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)];
                [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)];
                [0, sin(alpha), cos(alpha), d];
                [0, 0, 0, 1]];
            
            %             T(abs(T) < 0.0001) = 0;
        end
    end
    methods
        function self = mobile_manipulator(x_disp, y_disp, z_disp, phi_disp)
            %             self.x_ur5_base = 0.21;
            %             self.y_ur5_base = 0;
            %             self.z_ur5_base = 0.68;
            %             self.phi_ur5_base = pi/2;
            self.x_ur5_base = x_disp;
            self.y_ur5_base = y_disp;
            self.z_ur5_base = z_disp;
            self.phi_ur5_base = phi_disp;
            self.a = [0, -0.425, -0.39225, 0, 0, 0];
            self.d = [0.089159, 0, 0, 0.10915, 0.09465, 0.0823];
            self.alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];
            self.T_ur5_mobilebase = ROTZ(self.phi_ur5_base);
            self.T_ur5_mobilebase(4,:) = [0,0,0];
            self.T_ur5_mobilebase(:,4) = [self.x_ur5_base; self.y_ur5_base; self.z_ur5_base; 1];
            self.base_length = 0.76;
            self.base_breadth = 0.46;
            self.base_height = 0.55;
            self.wheel_radius = 0.2;
            self.wheel_offset = 0.2;
            self.base_center_height = self.wheel_radius + self.wheel_offset;
            self.wheel_length = 0.15;
            self.wheel_track = 0.71;
            self.wheel_base = 0.44;
            self.center_sphere_rad = self.base_length/8;
            %             self.corner_sphere_rad = sqrt(3*(self.base_length/2)^2) - self.center_sphere_rad;
            self.corner_sphere_rad = 0.05;
            self.mb_DOFs = 3;
            self.DOFs = 9;
            
        end
        %         function T_gripper_ur5 = T_gripper_ur5_func(self, T_gripper_world)
        %             T_gripper_ur5 = self.T_ur5_world\T_gripper_world;
        %
        %
        %         end
        
        function T = forward_kinematics(self, dofs)
            T = {};
            Twmb = ROTZ(dofs(3));
            Twmb(4,:) = [0,0,0];
            Twmb(:,4) = [dofs(1); dofs(2); 0; 1];
            Tmb0 = self.T_ur5_mobilebase;
            Tw0 = Twmb*Tmb0;
            T01 = self.DH(self.a(1), self.alpha(1), self.d(1), dofs(4));
            T12 = self.DH(self.a(2), self.alpha(2), self.d(2), dofs(5));
            T23 = self.DH(self.a(3), self.alpha(3), self.d(3), dofs(6));
            T34 = self.DH(self.a(4), self.alpha(4), self.d(4), dofs(7));
            T45 = self.DH(self.a(5), self.alpha(5), self.d(5), dofs(8));
            T56 = self.DH(self.a(6), self.alpha(6), self.d(6), dofs(9));
            T{end+1} = Tw0*T01;
            T{end+1} = T{end}*T12;
            T{end+1} = T{end}*T23;
            T{end+1} = T{end}*T34;
            T{end+1} = T{end}*T45;
            T{end+1} = T{end}*T56;
        end
        
        function V = forward_kinematics_pose(self, dofs)
            T = self.forward_kinematics(dofs);
            V = get_xyzabc_from_T(T{end});
            
        end
        
        
        function T = forward_kinematics_joints(self,  base_pose, targetJoints, i )
            
            T = {};
            Twmb = ROTZ(base_pose(3));
            Twmb(4,:) = [0,0,0];
            Twmb(:,4) = [base_pose(1:2)'; 0; 1];
            Tmb0 = self.T_ur5_mobilebase;
            Tw0 = Twmb*Tmb0;
            T01 = self.DH(self.a(1), self.alpha(1), self.d(1), targetJoints(1));
            T12 = self.DH(self.a(2), self.alpha(2), self.d(2), targetJoints(2));
            T23 = self.DH(self.a(3), self.alpha(3), self.d(3), targetJoints(3));
            T34 = self.DH(self.a(4), self.alpha(4), self.d(4), targetJoints(4));
            T45 = self.DH(self.a(5), self.alpha(5), self.d(5), targetJoints(5));
            T56 = self.DH(self.a(6), self.alpha(6), self.d(6), targetJoints(6));
            T{end+1} = Tw0*T01;
            T{end+1} = T{end}*T12;
            T{end+1} = T{end}*T23;
            T{end+1} = T{end}*T34;
            T{end+1} = T{end}*T45;
            T{end+1} = T{end}*T56;
            
            T = T{i};
        end
        
        %         function [ik_sols] = inverse_kinematics(self, T_ee, T_base)
        %             T_ee_ur5 = inv(self.T_ur5_mobilebase)*inv(T_base)*T_ee;
        %             ik_sols = UR5Kinematics.inverse_kinematics(T_ee_ur5, -pi);
        %         end
        
        
        
        function [thetas] = inv_kin_ur5_base_pose_known(self, T_ee_world, T_base_world)
            
            T_ee_ur5 = inv(self.T_ur5_mobilebase)*inv(T_base_world)*T_ee_world;
            gd = T_ee_ur5;
            %             thetas = zeros(9, 8);
            theta = zeros(6, 8);
            
            % Calculating theta(1)
            p05 = gd * [0, 0, -self.d(6), 1]'  - [0, 0, 0, 1]';
            psi = atan2(p05(2), p05(1));
            temp = self.d(4) / sqrt(p05(2)*p05(2) + p05(1)*p05(1));
            phi = acos(self.d(4) / sqrt(p05(2)*p05(2) + p05(1)*p05(1)));
            theta(1, 1:4) = pi/2 + psi + phi;
            theta(1, 5:8) = pi/2 + psi - phi;
            if ~real(phi)
                %                 disp('theta 1 imaginary')
                thetas = [];
                return;
            end
            %             theta = real(theta);
            %             disp('1');
            %             theta
            
            % Calculating theta(5)
            cols = [1, 5];
            for i=1:length(cols)
                c = cols(i);
                T10 = inv(self.DH(self.a(1), self.alpha(1), self.d(1), theta(1,c)));
                T16 = T10 * gd;
                p16z = T16(3,4);
                t5 = acos((p16z-self.d(4))/self.d(6));
                if ~real(t5)
                    %                     disp('theta 5 imaginary')
                    thetas = [];
                    return;
                end
                theta(5, c:c+1) = t5;
                theta(5, c+2:c+3) = -t5;
            end
            %             theta = real(theta);
            
            %             disp('2');
            %             theta
            
            % Calculating theta(6)
            cols = [1, 3, 5, 7];
            skip_ids = [];
            for i=1:length(cols)
                c = cols(i);
                T01 = self.DH(self.a(1), self.alpha(1), self.d(1), theta(1,c));
                T61 = inv(gd) * T01;
                T61zy = T61(2, 3);
                T61zx = T61(1, 3);
                t5 = theta(5, c);
                %                 if isreal(T61)
                theta(6, c:c+1) = atan2(-T61zy/sin(t5), T61zx/sin(t5));
                %                 else
                % %                     theta(6, c:c+1) = -99;
                %                     skip_ids = [skip_ids; c];
                %                 end
            end
            %             theta = real(theta);
            
            % Calculating theta(3)
            cols = [1, 3, 5, 7];
            for i=1:length(cols)
                c = cols(i);
                if size(skip_ids,1) > 0
                    find_idx = find(c, skip_ids);
                    if size(find_idx,1) > 0
                        continue;
                    end
                end
                T10 = inv(self.DH(self.a(1), self.alpha(1), self.d(1), theta(1,c)));
                T65 = inv(self.DH(self.a(6), self.alpha(6), self.d(6), theta(6,c)));
                T54 = inv(self.DH(self.a(5), self.alpha(5), self.d(5), theta(5,c)));
                T14 = T10 * gd * T65 * T54;
                p13 = T14 * [0, -self.d(4), 0, 1]' - [0,0,0,1]';
                p13norm2 = norm(p13) ^ 2;
                %                 (p13norm2-self.a(2)*self.a(2)-self.a(3)*self.a(3))/(2*self.a(2)*self.a(3));
                t3p = acos((p13norm2-self.a(2)*self.a(2)-self.a(3)*self.a(3))/(2*self.a(2)*self.a(3)));
                if ~real(t3p)
                    %                     disp('theta 3 imaginary')
                    thetas = [];
                    return;
                end
                theta(3, c) = t3p;
                theta(3, c+1) = -t3p;
            end
            theta = real(theta);
            
            % Calculating theta(2) and theta(4)
            cols = [1, 2, 3, 4, 5, 6, 7, 8];
            for i=1:length(cols)
                c = cols(i);
                T10 = inv(self.DH(self.a(1), self.alpha(1), self.d(1), theta(1,c)));
                T65 = inv(self.DH(self.a(6), self.alpha(6), self.d(6), theta(6,c)));
                T54 = inv(self.DH(self.a(5), self.alpha(5), self.d(5), theta(5,c)));
                T14 = T10 * gd * T65 * T54;
                p13 = T14 * [0, -self.d(4), 0, 1]' - [0,0,0,1]';
                p13norm = norm(p13);
                theta(2, c) = -atan2(p13(2), -p13(1))+asin(self.a(3)*sin(theta(3,c))/p13norm);
                T32 = inv(self.DH(self.a(3), self.alpha(3), self.d(3), theta(3,c)));
                T21 = inv(self.DH(self.a(2), self.alpha(2), self.d(2), theta(2,c)));
                T34 = T32 * T21 * T14;
                theta(4, c) = atan2(T34(2,1), T34(1,1));
            end
            %             theta = real(theta);
            
            % Put theta at good range
            for i=1:6
                for j=1:8
                    if theta(i,j) > pi
                        theta(i,j) = theta(i,j) - 2*pi;
                    elseif theta(i,j) < -pi
                        theta(i,j) = theta(i,j) + 2*pi;
                    end
                end
            end
            %             theta = 180/pi*theta;
            
            thetas = theta;
        end
        %
        %
        
        
        
        
        function J = analytical_jacobian(self, dofs)
            base_pose = dofs(1:3);
            joints_angles = dofs(4:9);
            phi = base_pose(3);
            theta1 = joints_angles(1);
            theta2 = joints_angles(2);
            theta3 = joints_angles(3);
            theta4 = joints_angles(4);
            theta5 = joints_angles(5);
            theta6 = joints_angles(6);
            
            t2 = sin(phi);
            t3 = cos(phi);
            t4 = cos(theta1);
            t7 = t3.*6.123233995736766e-17;
            t5 = t2-t7;
            t6 = sin(theta2);
            t8 = t4.*t5.*6.123233995736766e-17;
            t9 = sin(theta1);
            t10 = t2.*6.123233995736766e-17;
            t11 = t3+t10;
            t12 = t9.*t11.*6.123233995736766e-17;
            t13 = t8+t12;
            t14 = cos(theta2);
            t15 = t5.*t9;
            t18 = t4.*t11;
            t16 = t15-t18;
            t17 = cos(theta3);
            t19 = t6.*t16;
            t25 = t13.*t14;
            t20 = t19-t25;
            t21 = sin(theta3);
            t22 = t6.*t13;
            t23 = t14.*t16;
            t24 = t22+t23;
            t26 = cos(theta4);
            t27 = t17.*t20;
            t28 = sin(theta4);
            t29 = t17.*t24;
            t30 = t21.*(t22+t23);
            t31 = t27+t30;
            t34 = t20.*t21;
            t32 = t29-t34;
            t33 = sin(theta5);
            t35 = t26.*t32;
            t36 = t20.*t21.*3.9225e-1;
            t37 = t26.*t31.*9.465000000000001e-2;
            t38 = cos(theta5);
            t39 = t4.*t5;
            t40 = t26.*t31.*6.123233995736766e-17;
            t41 = t28.*t32.*6.123233995736766e-17;
            t42 = t9.*t11;
            t43 = t39+t40+t41+t42;
            t44 = t5.*t9.*6.123233995736766e-17;
            t46 = t4.*t11.*6.123233995736766e-17;
            t45 = t44-t46;
            t47 = t39+t42;
            t48 = t6.*t45;
            t53 = t14.*t47;
            t49 = t48-t53;
            t50 = t14.*t45;
            t51 = t6.*t47;
            t52 = t50+t51;
            t54 = t17.*t49;
            t55 = t21.*t52;
            t56 = t54+t55;
            t57 = t17.*t52;
            t59 = t21.*t49;
            t58 = t57-t59;
            t60 = t26.*t58;
            t61 = t60-t28.*t56;
            t63 = t14.*(t39+t42);
            t64 = t48-t63;
            t62 = t21.*t64.*3.9225e-1;
            t65 = t17.*t64;
            t69 = t21.*t64;
            t66 = t57-t69;
            t67 = t55+t65;
            t68 = t26.*t67.*6.123233995736766e-17;
            t70 = t28.*t66.*6.123233995736766e-17;
            t71 = t68+t70;
            t73 = t21.*(t48-t63);
            t74 = t57-t73;
            t72 = t28.*t74.*9.465000000000001e-2;
            t75 = t26.*t67;
            t76 = t28.*t74;
            t77 = t75+t76;
            t78 = t28.*t67.*6.123233995736766e-17;
            t79 = t14.*(t39+t42).*(1.7e1./4.0e1);
            t80 = t4.*t11.*1.0915e-1;
            t81 = t28.*(t55+t65).*9.465000000000001e-2;
            t118 = t26.*t74.*6.123233995736766e-17;
            t82 = t15-t18+t78-t118;
            t83 = t26.*t31;
            t84 = t28.*t32;
            t85 = t83+t84;
            t87 = t6.*(t15-t18);
            t88 = t25-t87;
            t86 = t17.*t88.*3.9225e-1;
            t89 = t21.*t88;
            t90 = t29+t89;
            t92 = t17.*t88;
            t91 = t30-t92;
            t93 = t26.*t90.*6.123233995736766e-17;
            t160 = t28.*t91.*6.123233995736766e-17;
            t161 = t93-t160;
            t94 = t38.*t161.*8.23e-2;
            t95 = t28.*t91.*9.465000000000001e-2;
            t96 = t14.*t21;
            t97 = t6.*t17;
            t98 = t96+t97;
            t99 = t6.*t21;
            t101 = t14.*t17;
            t100 = t99-t101;
            t102 = t6.*t21.*3.9225e-1;
            t103 = t26.*t100;
            t104 = t28.*t98;
            t221 = t103+t104;
            t105 = t33.*t221.*8.23e-2;
            t106 = t26.*t98.*6.123233995736766e-17;
            t109 = t28.*t100.*6.123233995736766e-17;
            t107 = t106-t109;
            t108 = t26.*t98.*9.465000000000001e-2;
            t110 = t26.*t90;
            t119 = t28.*t91;
            t111 = t110-t119;
            t112 = t26.*t91;
            t113 = t28.*t90;
            t114 = t26.*t91.*6.123233995736766e-17;
            t115 = t28.*t90.*6.123233995736766e-17;
            t116 = t39+t42+t114+t115;
            t117 = sin(theta6);
            t120 = cos(theta6);
            t121 = t33.*t111.*6.123233995736766e-17;
            t122 = t38.*t116.*6.123233995736766e-17;
            t123 = -t8-t12+t112+t113+t121+t122;
            t124 = t38.*(t75+t76);
            t125 = t124-t33.*t82;
            t126 = t120.*t125;
            t127 = t33.*t116;
            t138 = t38.*t111;
            t128 = t127-t138;
            t130 = t117.*t123;
            t131 = t33.*(t75+t76).*6.123233995736766e-17;
            t132 = t26.*t74;
            t133 = t28.*(t55+t65);
            t134 = t26.*(t57-t73).*6.123233995736766e-17;
            t140 = t15-t18+t78-t134;
            t135 = t38.*t140.*6.123233995736766e-17;
            t136 = -t44+t46+t131-t132+t133+t135;
            t137 = t117.*t136;
            t139 = t120.*t128;
            t142 = real(-t130);
            t146 = real(-t139);
            t129 = t142+t146-imag(t126)+imag(t137);
            t143 = t33.*t140;
            t144 = t124-t143;
            t145 = t120.*t144;
            t148 = imag(-t130);
            t152 = imag(-t139);
            t153 = real(t145);
            t141 = t148+t152+t153-real(t137);
            t149 = t38.*(t15-t18+t78-t134).*6.123233995736766e-17;
            t150 = -t44+t46+t131-t132+t133+t149;
            t151 = t117.*t150;
            t156 = imag(t151);
            t157 = imag(t145);
            t147 = t142+t146+t156-t157;
            t155 = real(t151);
            t154 = t148+t152+t153-t155;
            t158 = t147.^2;
            t159 = t154.^2;
            t162 = t112+t113;
            t163 = t132-t133;
            t164 = t28.*t74.*6.123233995736766e-17;
            t165 = t68+t164;
            t166 = t28.*(t29+t89);
            t167 = t112+t166;
            t168 = t38.*t161.*6.123233995736766e-17;
            t169 = t33.*t163.*6.123233995736766e-17;
            t170 = t38.*t165.*6.123233995736766e-17;
            t171 = t75+t76+t169+t170;
            t172 = 1.0./t154.^2;
            t173 = t158+t159;
            t174 = 1.0./t173;
            t175 = t33.*t161;
            t176 = t33.*t165;
            t179 = t38.*t163;
            t177 = t176-t179;
            t178 = t120.*t177;
            t180 = imag(t178);
            t181 = t38.*t167;
            t182 = t175+t181;
            t183 = t120.*t182;
            t190 = t117.*t171;
            t184 = imag(-t190);
            t188 = t33.*t167.*6.123233995736766e-17;
            t185 = t110-t119+t168-t188;
            t186 = 1.0./t154;
            t187 = real(t178);
            t191 = t117.*t185;
            t189 = imag(-t191);
            t192 = imag(t183);
            t193 = real(t183);
            t194 = real(-t191);
            t195 = t180-t184-t193+t194;
            t196 = t186.*t195;
            t197 = real(-t190);
            t198 = t187-t189+t192-t197;
            t199 = t147.*t172.*t198;
            t200 = t196+t199;
            t201 = t33.*t111;
            t202 = t38.*t116;
            t203 = t201+t202;
            t204 = t33.*t77;
            t205 = t38.*t140;
            t206 = t204+t205;
            t207 = t33.*t116.*6.123233995736766e-17;
            t208 = t207-t38.*t111.*6.123233995736766e-17;
            t209 = t38.*t77.*6.123233995736766e-17;
            t210 = t209-t33.*t140.*6.123233995736766e-17;
            t211 = t117.*t210;
            t212 = t120.*t150;
            t213 = t117.*t144;
            t214 = t26.*t98;
            t218 = t28.*t100;
            t215 = t214-t218;
            t216 = t26.*t100.*6.123233995736766e-17;
            t217 = t28.*t98.*6.123233995736766e-17;
            t219 = t216+t217-6.123233995736766e-17;
            t222 = t33.*t215.*6.123233995736766e-17;
            t223 = t26.*(t99-t101);
            t224 = t38.*t215;
            t225 = t33.*t219;
            t226 = t224-t225;
            t227 = t120.*t226;
            t220 = -t227+t117.*(t104+t222+t223+t38.*t219.*6.123233995736766e-17+3.749399456654644e-33);
            t236 = t38.*(t216+t217-6.123233995736766e-17).*6.123233995736766e-17;
            t237 = t104+t222+t223+t236+3.749399456654644e-33;
            t238 = t117.*t237;
            t228 = t227-t238;
            t229 = t38.*t221;
            t230 = t33.*t107;
            t231 = t229+t230;
            t232 = t120.*t231;
            t233 = t38.*t107.*6.123233995736766e-17;
            t240 = t33.*t221.*6.123233995736766e-17;
            t234 = t214-t218+t233-t240;
            t235 = t117.*t234;
            t239 = t227-t238;
            t241 = t227-t238;
            t242 = t227-t238;
            t243 = t117.*t226;
            t244 = t38.*t219;
            t245 = t120.*t237;
            t246 = t33.*t215;
            t247 = t38.*t107;
            t248 = t120.*t234;
            t249 = real(-t103);
            t250 = real(t104);
            t251 = t250.*6.123233995736766e-17;
            t252 = imag(t243);
            t253 = real(-t244);
            t254 = imag(t245);
            t255 = real(t246);
            t257 = t249.*6.123233995736766e-17;
            t256 = t251+t252+t253+t254-t255-t257+2.295845021658468e-49;
            t258 = imag(-t103);
            t259 = t258.*6.123233995736766e-17;
            t260 = imag(t104);
            t261 = imag(-t244);
            t262 = real(t243);
            t263 = real(t245);
            t264 = imag(t246);
            t284 = t260.*6.123233995736766e-17;
            t265 = t259-t261+t262+t263+t264-t284;
            t266 = t256.^2;
            t267 = imag(t247);
            t276 = t117.*t231;
            t268 = real(-t276);
            t269 = imag(t214);
            t270 = real(t248);
            t271 = imag(-t218);
            t283 = t33.*t221;
            t272 = imag(-t283);
            t289 = t269.*6.123233995736766e-17;
            t290 = t271.*6.123233995736766e-17;
            t273 = t267+t268+t270+t272-t289-t290;
            t274 = 1.0./t256;
            t275 = t273.*t274;
            t277 = real(t247);
            t278 = imag(t248);
            t279 = real(t214);
            t280 = t279.*6.123233995736766e-17;
            t281 = real(-t218);
            t282 = t281.*6.123233995736766e-17;
            t285 = 1.0./t256.^2;
            t286 = t265.^2;
            t287 = t266+t286;
            t288 = 1.0./t287;
            t291 = imag(-t276);
            t292 = real(-t283);
            t293 = -t277+t278+t280+t282+t291-t292;
            t294 = t275-t265.*t285.*t293;
            t295 = t244+t246;
            t296 = t33.*t219.*6.123233995736766e-17;
            t298 = t38.*t215.*6.123233995736766e-17;
            t297 = t296-t298;
            t299 = t117.*t295;
            J = reshape([1.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,t2.*(-2.1e1./1.0e2)+t36+t37-t4.*t5.*1.0915e-1-t6.*t13.*(1.7e1./4.0e1)-t9.*t11.*1.0915e-1-t14.*t16.*(1.7e1./4.0e1)-t17.*t24.*3.9225e-1+t28.*t32.*9.465000000000001e-2-t38.*t43.*8.23e-2-t33.*(t35-t28.*(t27+t21.*t24)).*8.23e-2,t3.*(2.1e1./1.0e2)+t79+t80+t81-t5.*t9.*1.0915e-1-t6.*t45.*(1.7e1./4.0e1)-t21.*t52.*3.9225e-1-t17.*t64.*3.9225e-1-t26.*t74.*9.465000000000001e-2-t33.*t77.*8.23e-2-t38.*t82.*8.23e-2,0.0,(t159.*(t129.^2.*1.0./(-real(t117.*(-t44+t46+t28.*t67-t26.*t74+t33.*t77.*6.123233995736766e-17+t38.*t82.*6.123233995736766e-17))+imag(-t117.*t123)+imag(-t120.*t128)+real(t126)).^2+1.0))./(t158+t141.^2),0.0,0.0,t36+t37-t4.*t5.*1.0915e-1-t6.*t13.*(1.7e1./4.0e1)-t9.*t11.*1.0915e-1-t14.*t16.*(1.7e1./4.0e1)-t17.*t24.*3.9225e-1-t38.*t43.*8.23e-2+t28.*(t29-t34).*9.465000000000001e-2-t33.*(t35-t28.*t31).*8.23e-2,t79+t80+t81-t5.*t9.*1.0915e-1-t6.*t45.*(1.7e1./4.0e1)-t21.*t52.*3.9225e-1-t17.*t64.*3.9225e-1-t26.*t74.*9.465000000000001e-2-t33.*t77.*8.23e-2-t38.*t82.*8.23e-2,0.0,t159.*t174.*(t158.*t172+1.0),0.0,0.0,t62+t26.*(t54+t55).*9.465000000000001e-2-t38.*(t26.*t56.*6.123233995736766e-17+t28.*t58.*6.123233995736766e-17).*8.23e-2-t6.*t47.*(1.7e1./4.0e1)-t14.*t45.*(1.7e1./4.0e1)-t17.*t52.*3.9225e-1-t33.*t61.*8.23e-2+t28.*(t57-t21.*(t48-t14.*(t39+t42))).*9.465000000000001e-2,t86+t14.*(t8+t12).*(1.7e1./4.0e1)+t38.*(t26.*t32.*6.123233995736766e-17-t28.*t31.*6.123233995736766e-17).*8.23e-2-t6.*t16.*(1.7e1./4.0e1)-t21.*t24.*3.9225e-1-t26.*t32.*9.465000000000001e-2+t28.*t31.*9.465000000000001e-2-t33.*t85.*8.23e-2,t14.*(-1.7e1./4.0e1)+t102+t105+t108-t14.*t17.*3.9225e-1-t28.*t100.*9.465000000000001e-2-t38.*t107.*8.23e-2,-t159.*t174.*(t186.*(t180-t184-real(t120.*(t175+t38.*t162))+real(-t117.*(t110-t119+t168-t33.*t162.*6.123233995736766e-17)))+t147.*t172.*(t187-t189+t192-real(-t117.*t171))),1.0./sqrt(-t220.^2+1.0).*(t232+t235),-t266.*t288.*(t275-t265.*t285.*(-t277+t278+t280+t282+imag(-t117.*t231)-real(-t33.*t221))),t62+t72-t17.*t52.*3.9225e-1+t26.*t67.*9.465000000000001e-2-t33.*t61.*8.23e-2-t38.*t71.*8.23e-2,t86+t94+t95-t21.*t24.*3.9225e-1-t26.*t90.*9.465000000000001e-2-t33.*t85.*8.23e-2,t102+t105+t108-t14.*t17.*3.9225e-1-t28.*t100.*9.465000000000001e-2-t38.*t107.*8.23e-2,-t159.*t174.*t200,1.0./sqrt(-t228.^2+1.0).*(t232+t235),-t266.*t288.*t294,t72+t26.*(t55+t65).*9.465000000000001e-2-t38.*t71.*8.23e-2-t33.*(t26.*t66-t28.*t67).*8.23e-2,t94+t95-t26.*t90.*9.465000000000001e-2-t33.*t162.*8.23e-2,t105+t108-t28.*t100.*9.465000000000001e-2-t38.*t107.*8.23e-2,-t159.*t174.*t200,1.0./sqrt(-t239.^2+1.0).*(t232+t235),-t266.*t288.*t294,t38.*t77.*(-8.23e-2)+t33.*(t15-t18+t78-t26.*(t57-t73).*6.123233995736766e-17).*8.23e-2,t33.*t116.*(-8.23e-2)+t38.*t111.*8.23e-2,t33.*t219.*8.23e-2-t38.*t215.*8.23e-2,t159.*t174.*(t186.*(imag(-t120.*t206)-real(-t120.*t203)+real(-t117.*t208)-imag(t211))+t147.*t172.*(imag(-t120.*t203)-imag(-t117.*t208)+real(-t120.*t206)-real(t211))),-1.0./sqrt(-t241.^2+1.0).*(t117.*t297-t120.*t295),-t266.*t288.*(t274.*(imag(-t225)+real(-t120.*t297)+imag(t224)-real(t299))+t265.*t285.*(real(-t225)-imag(-t120.*t297)+imag(t299)+real(t224))),0.0,0.0,0.0,-t159.*t174.*(t186.*(real(-t120.*t123)-real(-t117.*t128)+imag(t212)+imag(t213))+t147.*t172.*(-imag(-t120.*t123)+imag(-t117.*t128)+real(t212)+real(t213))),1.0./sqrt(-t242.^2+1.0).*(t243+t245),-t266.*t288.*(t274.*(real(t227)-real(t238))-t265.*t285.*(imag(t227)-imag(t238)))],[6,9]);
            
        end
        
        
        
        
        function [v_theta, dofs] = IK_jacobian_fmincon(self, delta_t, pose1, pose2, base_pose, joints1)
            
            v = (pose2-pose1)/delta_t;
            v(4:6) = angdiff(pose1(4:6), pose2(4:6)) / delta_t;
            v_X = v';
            J = self.analytical_jacobian2([base_pose, joints1]);
            function f = obj_function(d_theta)
                f = (d_theta' * (J' * J) * d_theta - 2 * d_theta' * J' * v_X + v_X' * v_X);
            end
            ub = [1, 1,1,1,1,1,1] * pi;
            ub = [5,5, ub]';
            lb = -ub;
            
            A = []; b = []; Aeq = []; beq = [];
            function [c, ceq] = nonlconst(d_theta)
                ceq = (d_theta(1))*sin(base_pose(3)+delta_t*d_theta(3)) - (d_theta(2))*cos(base_pose(3)+delta_t*d_theta(3));
                c = 0;
            end
            q0 = [base_pose, joints1]';
            v_theta = fmincon(@obj_function, q0, A, b, Aeq, beq, lb, ub, @nonlconst);
            dofs = zeros(1,9);
            dofs(3:9) = wrapToPi([base_pose(3), joints1] + delta_t*v_theta(3:9)');
            dofs(1:2) = base_pose(1:2) + delta_t*v_theta(1:2)';
        end
        
        
        function [v_theta, output_dofs] = IK_fk_fmincon(self, delta_t, desired_pose, base_pose, joints1, in_CONFIG)
            function f = obj_function(q)
                lambda_self_coll = 1/50;
                lambda_ext_coll = 1/10;
                lambda_base = 1/50;
                self_coll = 0;
                ext_coll = 0;
                [arm_centers, r_sphere] = self.get_arm_sphere_centers(q, in_CONFIG);
                [mb_centers, mb_sphere_radii] = self.get_mb_sphere_centers(q);
                x0 = 3;
                for aid = 3:length(arm_centers)
                    %                   ext_coll = ext_coll + log(abs(arm_centers(aid, 1) - x0));
                    ext_coll = ext_coll + exp(10*(arm_centers(aid, 1) - x0));
                    for mid = 1:length(mb_centers)
                        ext_coll = ext_coll +  exp(10*(mb_centers(mid, 1) - x0));
                        self_coll = self_coll + exp(10*(- norm(arm_centers(aid, :) - mb_centers(mid, :)) + (r_sphere + mb_sphere_radii(mid))));
                        %                       ext_coll = ext_coll -  log(abs(mb_centers(mid, 1) - x0));
                        %                       self_coll = self_coll - log(abs(- norm(arm_centers(aid, :) - mb_centers(mid, :)) + (r_sphere + mb_sphere_radii(mid))));
                    end
                end
                ee_pose = self.forward_kinematics_pose(q);
                f = 25*(norm(ee_pose(1:3) - desired_pose(1:3)) ) + 5*sum(abs(angdiff(ee_pose(4:6), desired_pose(4:6))/(pi))) + lambda_self_coll*self_coll ...
                    + lambda_ext_coll*ext_coll + lambda_base*norm(base_pose(1:2) - q(1:2));
            end
            
            ub = [1, 1,1,1,1,1,1] * pi;
            ub = [5,5, ub]';
            lb = -ub;
            
            A = []; b = []; Aeq = []; beq = [];
            function [c, ceq] = nonlconst(q)
                ceq = (base_pose(1)-q(1))*sin(q(3)) - (base_pose(2)-q(2))*cos(q(3));
                %                 ceq = [];
                
                c = [];
                %                 [arm_centers, r_sphere] = self.get_arm_sphere_centers(q, in_CONFIG);
                %                 [mb_centers, mb_sphere_radii] = self.get_mb_sphere_centers(q);
                %                 for aid = 3:length(arm_centers)
                %                     for mid = 1:length(mb_centers)
                %                         c = [c; - norm(arm_centers(aid, :) - mb_centers(mid, :)) + (r_sphere + mb_sphere_radii(mid))];
                %                     end
                %                 end
                
            end
            q0 = [base_pose, joints1];
            options = optimoptions('fmincon','MaxIterations',10000, 'OptimalityTolerance', 1.0000e-8);
            v_theta = fmincon(@obj_function,q0,A,b,Aeq,beq,lb,ub, @nonlconst, options);
            global obj_fn_ar
            
            obj_fn_ar = [obj_fn_ar; obj_function(v_theta)];
            output_dofs = zeros(1,9);
            output_dofs(3:9) = wrapToPi([base_pose(3), joints1] + delta_t * v_theta(3:9));
            output_dofs(1:2) = base_pose(1:2) + delta_t*v_theta(1:2);
        end
        
        
        function [centers, r_sphere] = get_arm_sphere_centers(self, in_mm_pose, in_CONFIG)
            smallest_dist = 60/1000;
            r_sphere = smallest_dist;
            center_id = 1;
            robot_links_temp = in_CONFIG.ROBOT_LINKs;
            for jid = 1:6
                transf_mat = self.forward_kinematics_joints(in_mm_pose(1:3), in_mm_pose(4:end), jid);
                robot_pts = robot_links_temp{jid};
                robot_pts = transf_mat*robot_pts;
                if size(robot_pts,2) > 1
                    for rpid = 1:size(robot_pts,2)-1
                        dist = compute_distance(robot_pts(1:3,rpid)', robot_pts(1:3, rpid+1)');
                        delta_seg = round(dist/(smallest_dist));
                        %                 r_sphere = dist*0.5/(delta_seg);
                        deltas = [robot_pts(1:3, rpid+1)' - robot_pts(1:3,rpid)']./delta_seg;
                        for did =1:delta_seg
                            centers(center_id,:) = robot_pts(1:3,rpid)'+ deltas.*did;
                            center_id = center_id + 1;
                        end
                        robot_links_temp{jid} = robot_pts;
                    end
                end
            end
        end
        
        function [centers, sphere_radii] = get_mb_sphere_centers(self, in_mm_pose)
            centers = [];
            
            %            centers(1,:) = [in_mm_pose(1:2), self.base_height/2 - (self.base_length - self.base_height)/2];
            centers(1,:) = ROTZ(in_mm_pose(3))*[self.base_length/2 - self.corner_sphere_rad; -(self.base_breadth/2 - self.corner_sphere_rad); self.base_height- self.corner_sphere_rad];
            centers(2,:) = ROTZ(in_mm_pose(3))*[self.base_length/2 - self.corner_sphere_rad; self.base_breadth/2 - self.corner_sphere_rad; self.base_height- self.corner_sphere_rad];
            centers(3,:) = ROTZ(in_mm_pose(3))*[self.base_length/2 - self.corner_sphere_rad; 0; self.base_height- self.corner_sphere_rad];
            centers(4,:) = ROTZ(in_mm_pose(3))*[self.base_length/4 ; -(self.base_breadth/2 - self.corner_sphere_rad); self.base_height- self.corner_sphere_rad];
            centers(5,:) = ROTZ(in_mm_pose(3))*[self.base_length/4 ; self.base_breadth/2 - self.corner_sphere_rad; self.base_height- self.corner_sphere_rad];
            centers(6,:) = ROTZ(in_mm_pose(3))*[self.base_length/4 ; 0; self.base_height- self.corner_sphere_rad];
            
            centers(7,:) = ROTZ(in_mm_pose(3))*[0; -(self.base_breadth/2 - self.corner_sphere_rad); self.base_height- self.corner_sphere_rad];
            centers(8,:) = ROTZ(in_mm_pose(3))*[0; self.base_breadth/2 - self.corner_sphere_rad; self.base_height- self.corner_sphere_rad];
            centers(9,:) = ROTZ(in_mm_pose(3))*[0; 0; self.base_height- self.corner_sphere_rad];
            
            centers(10,:) = ROTZ(in_mm_pose(3))*[-(self.base_length/2 - self.corner_sphere_rad); -(self.base_breadth/2 - self.corner_sphere_rad); self.base_height- self.corner_sphere_rad];
            centers(11,:) = ROTZ(in_mm_pose(3))*[-(self.base_length/2 - self.corner_sphere_rad); self.base_breadth/2 - self.corner_sphere_rad; self.base_height- self.corner_sphere_rad];
            centers(12,:) = ROTZ(in_mm_pose(3))*[-(self.base_length/2 - self.corner_sphere_rad); 0; self.base_height- self.corner_sphere_rad];
            centers(13,:) = ROTZ(in_mm_pose(3))*[-self.base_length/4 ; -(self.base_breadth/2 - self.corner_sphere_rad); self.base_height- self.corner_sphere_rad];
            centers(14,:) = ROTZ(in_mm_pose(3))*[-self.base_length/4 ; self.base_breadth/2 - self.corner_sphere_rad; self.base_height- self.corner_sphere_rad];
            centers(15,:) = ROTZ(in_mm_pose(3))*[-self.base_length/4 ; 0; self.base_height- self.corner_sphere_rad];
            
            centers(16,:) = ROTZ(in_mm_pose(3))*[(self.base_length/2 - self.center_sphere_rad); self.center_sphere_rad; self.base_height- self.center_sphere_rad];
            centers(17,:) = ROTZ(in_mm_pose(3))*[(self.base_length/2 - 3*self.center_sphere_rad); self.center_sphere_rad; self.base_height- self.center_sphere_rad];
            centers(18,:) = ROTZ(in_mm_pose(3))*[(self.base_length/2 - self.center_sphere_rad); -self.center_sphere_rad; self.base_height- self.center_sphere_rad];
            centers(19,:) = ROTZ(in_mm_pose(3))*[(self.base_length/2 - 3*self.center_sphere_rad); -self.center_sphere_rad; self.base_height- self.center_sphere_rad];
            
            centers(20,:) = ROTZ(in_mm_pose(3))*[-(self.base_length/2 - self.center_sphere_rad); self.center_sphere_rad; self.base_height- self.center_sphere_rad];
            centers(21,:) = ROTZ(in_mm_pose(3))*[-(self.base_length/2 - 3*self.center_sphere_rad); self.center_sphere_rad; self.base_height- self.center_sphere_rad];
            centers(22,:) = ROTZ(in_mm_pose(3))*[-(self.base_length/2 - self.center_sphere_rad); -self.center_sphere_rad; self.base_height- self.center_sphere_rad];
            centers(23,:) = ROTZ(in_mm_pose(3))*[-(self.base_length/2 - 3*self.center_sphere_rad); -self.center_sphere_rad; self.base_height- self.center_sphere_rad];
            
            centers = centers + [in_mm_pose(1:2), 0];
            sphere_radii = [self.corner_sphere_rad*ones(1,length(centers)-8), self.center_sphere_rad*ones(1,8)];
            
            
        end
        
        
        function visualize_mm(self, in_mm_pose, f)
            
            hold on
            axis equal
            DrawCuboid([self.base_length, self.base_breadth, self.base_height]',[in_mm_pose(1:2), self.base_center_height]', [in_mm_pose(3), 0, 0]' ,[1, 0, 0], 0.8)
            r = self.wheel_radius; n = 100; h = self.wheel_length;
            [X,Y,Z] = cylinder(r,n);
            P1 = [X(1,:); Y(1,:); Z(1,:)-h*Z(2,:)/2];
            P2 = [X(2,:); Y(2,:); h*Z(2,:)/2];
            
            
            P11r = ROTZ(in_mm_pose(3))*(ROTX(pi/2)*P1  + [self.wheel_base/2; self.wheel_track/2; self.wheel_offset] ) + [in_mm_pose(1:2)';0];
            X11 = P11r(1,:); Y11 = P11r(2,:); Z11 = P11r(3,:);
            P12r = ROTZ(in_mm_pose(3))*(ROTX(pi/2)*P2  + [self.wheel_base/2; self.wheel_track/2; self.wheel_offset] ) + [in_mm_pose(1:2)';0];
            X12 = P12r(1,:); Y12 = P12r(2,:); Z12 = P12r(3,:);
            X1r = [X11;X12]; Y1r = [Y11;Y12]; Z1r = [Z11;Z12];
            surf(X1r,Y1r,Z1r,'facecolor','k','LineStyle','none');
            hold on
            fill3(P11r(1,:),P11r(2,:),P11r(3,:),'k')
            fill3(P12r(1,:),P12r(2,:),P12r(3,:),'k')
            
            P11r = ROTZ(in_mm_pose(3))*(ROTX(pi/2)*P1  + [self.wheel_base/2; -self.wheel_track/2; self.wheel_offset]) + [in_mm_pose(1:2)';0];
            X11 = P11r(1,:); Y11 = P11r(2,:); Z11 = P11r(3,:);
            P12r = ROTZ(in_mm_pose(3))*(ROTX(pi/2)*P2  + [self.wheel_base/2; -self.wheel_track/2; self.wheel_offset]) + [in_mm_pose(1:2)';0];
            X12 = P12r(1,:); Y12 = P12r(2,:); Z12 = P12r(3,:);
            X1r = [X11;X12]; Y1r = [Y11;Y12]; Z1r = [Z11;Z12];
            surf(X1r,Y1r,Z1r,'facecolor','k','LineStyle','none');
            hold on
            fill3(P11r(1,:),P11r(2,:),P11r(3,:),'k')
            fill3(P12r(1,:),P12r(2,:),P12r(3,:),'k')
            
            P11r = ROTZ(in_mm_pose(3))*(ROTX(pi/2)*P1  + [-self.wheel_base/2; self.wheel_track/2; self.wheel_offset]) + [in_mm_pose(1:2)';0];
            X11 = P11r(1,:); Y11 = P11r(2,:); Z11 = P11r(3,:);
            P12r = ROTZ(in_mm_pose(3))*(ROTX(pi/2)*P2  + [-self.wheel_base/2; self.wheel_track/2; self.wheel_offset]) + [in_mm_pose(1:2)';0];
            X12 = P12r(1,:); Y12 = P12r(2,:); Z12 = P12r(3,:);
            X1r = [X11;X12]; Y1r = [Y11;Y12]; Z1r = [Z11;Z12];
            surf(X1r,Y1r,Z1r,'facecolor','k','LineStyle','none');
            hold on
            fill3(P11r(1,:),P11r(2,:),P11r(3,:),'k')
            fill3(P12r(1,:),P12r(2,:),P12r(3,:),'k')
            
            P11r = ROTZ(in_mm_pose(3))*(ROTX(pi/2)*P1  + [-self.wheel_base/2; -self.wheel_track/2; self.wheel_offset]) + [in_mm_pose(1:2)';0];
            X11 = P11r(1,:); Y11 = P11r(2,:); Z11 = P11r(3,:);
            P12r = ROTZ(in_mm_pose(3))*(ROTX(pi/2)*P2  + [-self.wheel_base/2; -self.wheel_track/2; self.wheel_offset]) + [in_mm_pose(1:2)';0];
            X12 = P12r(1,:); Y12 = P12r(2,:); Z12 = P12r(3,:);
            X1r = [X11;X12]; Y1r = [Y11;Y12]; Z1r = [Z11;Z12];
            surf(X1r,Y1r,Z1r,'facecolor','k','LineStyle','none');
            hold on
            fill3(P11r(1,:),P11r(2,:),P11r(3,:),'k')
            fill3(P12r(1,:),P12r(2,:),P12r(3,:),'k')
            
             ur5_disp = UR5Display(f);

             ur5_disp.draw_configuration(in_mm_pose, self.T_ur5_mobilebase);
           
            
        end
        

        
        %         function [v_theta, next_base_pose, joints] = IK_jacobian_DLS(self, delta_t, pose1, pose2, base_pose, joints1)
        %             delta_pose = pose2-pose1;
        %             delta_pose(3) = angdiff(pose2(3),pose1(3));
        %             v = delta_pose/delta_t;
        %             v_X = v';
        %             J = self.analytical_jacobian(base_pose, joints1);
        %             C = eye(3);
        %             JJT = J * C * J';
        %             v_theta = J' * inv(JJT + self.lamda^2 * eye(size(JJT))) * v_X;
        %             joints = wrapTo2Pi([base_pose(3); joints1] + delta_t * v_theta(3:end)');
        %             next_base_pose = base_pose(1:2) + delta_t * v_theta(1:2);
        %         end
        
        
        
        
        
        
        
        
        
        
        
        
    end
end