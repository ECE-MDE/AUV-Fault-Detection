function [nu_dot, eta_dot] = Sim_690_Rev12( nu, delta, eta)
%==========================================================================
%     AUV 6-DOF dynamics

%     Revision 11: September 3, 2021

%     -  Fixed X_uudd_fin (was off by factor 0.5)        11 Sept 2021
%     -  Changed buoyancy from 1% -> 0.2%                23 Aug  2021
%     -  Fixed issue with M_w pure-heave terms            7 Aug  2021
%     -  Fixed Xuu and Xuudd, other fin stuff            15 Oct  2020
%     -  Implemented Fin Angles as Inputs                24 July 2020
%     -  Fixed yaw and pitch-axis coefficients           23 July 2020
%     -  Found a sign-error in M_uud (fixed pitch axis)  13 Dec  2019
%     -  Added-Coriolis effects on coeff. estimation     18 Sept 2019
%        fully accounted for, redundancies neutralized,         
%     -  Velocity control added                          29 July 2019
%     -  ACTUAL unique heave/pitch coefficients added,   19 July 2019
%     - "Fully-finned" hull coefficients implemented,    12 July 2019
%     -  Unique heave/pitch coefficients added,          12 July 2019
%     -  Small changes in Rigid-Body Coriolis Matrix,    12 July 2019
%     -  Final Cb-Cg separation value implemented,       12 July 2019
%     -  Original 690 Implementation by Dan Stilwell,       Dec  2018
%     -  Based on implementation by Collin Phillips,        July 2018


%% Define Inputs

u = nu(1);
v = nu(2);
w = nu(3);

p = nu(4);
q = nu(5);
r = nu(6);

x = eta(1);
y = eta(2);
z = eta(3);

phi = eta(4);   % roll
theta = eta(5); % pitch
psi = eta(6);   % yaw 


% the box's input vector is referred to as "delta"


Top_Fin_Angle = delta(1); 
Portside_Fin_Angle = delta(2);
Bottom_Fin_Angle = delta(3);
Starboard_Fin_Angle = delta(4);
des_vel = delta(5); % desired velocity

% Re-calculate delta_r, delta_e, and delta_roll for use in the rest of the code.
% This math-averaging works because all the rudder forces (except X) are linear
% with rudder angle:

delta_r = mean([-Top_Fin_Angle, Bottom_Fin_Angle]);
delta_e = mean([Portside_Fin_Angle, -Starboard_Fin_Angle]);
delta_roll = mean([Top_Fin_Angle, Portside_Fin_Angle, ...
    Bottom_Fin_Angle, Starboard_Fin_Angle]);

% Then lastly, in the Tau matrix, calculate the new X-forces with the
% individual fins, because they are non-linear with delta. 
% Also implement the new roll-moment in the Tau matrix.


% define rotations
xrot3 = @(o) [[1,    0   ,    0   ];    %3D rotation about the x-axis
              [0,  cos(o),  sin(o)];
              [0, -sin(o),  cos(o)]];

yrot3 = @(o) [[ cos(o),  0, -sin(o)];    %3D rotation about the y-axis
              [    0  ,  1,     0  ];
              [ sin(o),  0,  cos(o)]];
          
zrot3 = @(o) [[  cos(o),  sin(o), 0];    %3D rotation about the z-axis
              [ -sin(o),  cos(o), 0];
              [   0   ,   0    ,  1]];

%Define rotation angles from BRF to IRF
J1 = @(o,t,s) transpose(xrot3(o)*yrot3(t)*zrot3(s));
J2 = @(o,t,s) [[1, sin(o)*tan(t), cos(o)*tan(t)];
               [0, cos(o)       , -sin(o)      ];
               [0, sin(o)/cos(t), cos(o)/cos(t)]];
J = @(o,t,s) [[J1(o,t,s), zeros(3,3)];
              [zeros(3,3),  J2(o,t,s)]];
          
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Physical vehicle parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          
len    = 2.194;     %m (86.38 in)  ****
width  = 0.1753;    %m (6.90 in)  ****
height = width;     % for spheroid shape assumptions
m      = 47.556;    %mass in kg   ****

%center of gravity and buoyancy respectively
cb = [0, 0, 0]';       %m % CRAFT SPINS AROUND THIS POINT
%cg = [0, 0, 0.00].'; %m  **** % +Z IS DOWN
cg = [0, 0, 0.0083]'; %m  **** % +Z IS DOWN
%cg = [0, 0, 0.083]';


buoy  = (1  + 0.000)*m;  %bouyancy kg
%buoy  = (1  + 0.00)*m;  %bouyancy kg

%% Static terms not derived via CFD

x_B = cb(1);
y_B = cb(2);
z_B = cb(3);

x_G = cg(1);
y_G = cg(2);
z_G = cg(3);

rho_sw = 1000;      %kg/m^3 (density of 'seawater' <water for now>)
grav = 9.8066;  %m/s2 (acc. of gravity)

%definitions for geometric calculations
a = len/2;       
b = width/2; % craft radius

% Moments of inertia from elipsoid assumption (If you can't just use CAD)
  
% I_xx = 2/5*m*a^2;         %kg-m^2
% I_yy = 1/5*m*(b^2+a^2);   %kg-m^2
% I_zz = 1/5*m*(b^2+a^2);   %kg-m^2

% Moments of Inertia measured from CAD 
I_xx = 0.187;             %kg-m^2
I_yy = 14.933;            %kg-m^2
I_zz = 14.933;            %kg-m^2

I_xy = 0;
I_xz = 0;
I_yz = 0;
% I_xy = 0.0635297; 
% I_xz = 0.0000648;
% I_yz = -0.0000024;

I_yx = I_xy;
I_zx = I_xz;
I_zy = I_yz;


%% Coefficients derived via VPMM and Drag Testing (IMPORTABLE)

% ============= Fin Coefficients via Drag ===========
X_uuddr = -4.6937; %690, drag from fin deflection
X_uudde = X_uuddr; % if somehow asymmetric X-DRAG force. Extremely rare.
X_uudd = X_uuddr;
%X_uuddc = 0; % drag (in x) from canards deflection
X_uudd_fin = X_uudd/2; % X-force drag for EACH individual fin (2 for 690)

% THINGS TO MESS WITH TO SIMULATE FINS BEING BAD
% HOW EFFECTIVE RUDDER IS AT PRODUCING STARBOARD FORCE
Y_uud =6.5161; %9.0728;     %690 %5.7 will single handedly solve the time issue; the new uud coeffs are determined by Lakshmi after adding 15deg with higher mesh resolution and fin gap inclusion %6.5161;  %
% HOW EFFECTIVE RUDDER IS AT PRODUCING YAW MOMENT
N_uud =-6.9036; %-9.1827 % -6.9036; ;      %690

%K_uud_roll = -2*Y_uud*b; % Roll-moment per "delta_roll" radian
K_uud_roll = -0.4531;

% ========== Via Pure Surge VPMM  =========
X_udot = -13.047;    % 690, from pure-surge vpmm maneuver
X_uu =  -4.85;    % 690 (Also extracted in pure drag)


% ============== Via Pure Sway VPMM ==============

% Refined Coefficients with fins: %untouched from Taylor's determination
% since the issue is in yaw coefficients
% Y_uv = -34.5946; % 50-percent towing tank ONLY THIS PART UNCOMMENT ORIGINALLY
% Y_vv = -171.825;
% Y_vdot = -48.238;
% N_uv = -13.0885; 
% N_vv = 34.1192;
% N_vdot = 3.67588;

% Only-VPMM Coefficients (no towing-tank) with fins:
% Y_uv = -41.6059;
% Y_vv = -63.8578;
% Y_vdot = -48.2379;
% N_vv = 24.5598;  % ONLY THIS PART UNCOMMENT
% N_uv = -11.4956;
% N_vdot = 3.67491;

%oNLY vpmm
% Y_uv	=-36.5;
% Y_vv	=-72.5;
% Y_vdot	=-51.8;
% N_uv	=-26.1;
% N_vv	=3.7;
% N_vdot	=0.8;

%% 90% TOW TANK
Y_uv	=-28.03;
Y_vv	=-244.27;
Y_vdot	=-51.8;
N_uv	=-17.8962;
N_vv	=82.6146;
N_vdot	=0.8;


% ================  Via Pure Yaw VPMM  ===============

% % With fins: (both amplitudes 2cm and 4cm)
% N_rr = -46.5715;
% N_ur = -24.2459;
% N_rdot = -14.5316;
% Y_rr = 45.2878;
% Y_ur = 23.6598;
% Y_rdot = 2.6899;

% N_rr=-47.1122; %2,4,8cm
% N_ur=-26.5254;
% N_rdot=-13.6383;
% Y_rr=51.1689;
% Y_ur=25.9459;
% Y_rdot=2.0571;

%% 3 best 8,10,16
% N_rr=-32.2678;
% N_ur=-32.5724;
% N_rdot=-13.4604;
% Y_rr=22.8650;
% Y_ur=35.2440;
% Y_rdot=2.9235;
%% All 5
% N_rr = -35.7842;
% N_ur = -30.6572;
% N_rdot = -13.5114;
% Y_rr = 27.5814;
% Y_ur = 32.6763;
% Y_rdot = 2.91238;

% Corrected 8,10,16cm with the correct mesh
N_rr = -38.5351;
N_ur = -32.9786;
N_rdot = -13.8899;
Y_rr = 28.8384;
Y_ur = 36.3419;
Y_rdot = 3.2504;

% ============== Sideways coefficients ================

M_ww = -3.68259;
M_uw = 26.0741;
M_wdot = -0.776908;
Z_ww = -72.497;
Z_uw = -36.4678;
Z_wdot = -51.7901;


M_qq = -25.6784; % SIDEWAYS WITH FINS (+Nrr of the craft on its side)
M_uq = -22.3806; % SIDEWAYS WITH FINS
M_qdot = -14.5177; % SIDEWAYS WITH FINS
Z_qq = -11.824; % SIDEWAYS WITH FINS (-Yrr of the craft on its side)
Z_uq = -13.0932; % SIDEWAYS WITH FINS
Z_qdot = 0.67173; % SIDEWAYS WITH FINS



%% Nonzero terms that are annoying to measure

% ======== Roll axis TERMS REQUIRED FOR ROLL STABILITY ========
K_pp = 2*b^2*Y_vv; 
K_up = 2*b^2*Y_uv; 
K_pdot = 2*b^2*Y_vdot;
%K_up = -0.33238; % from vpmm
%K_pp = -0.018277; % from vpmm
%K_pdot = -0.29772; % from vpmm

% ========= unimportant terms =========
K_vv = 0.01; % currently estimated; would need vpmm with underfin for true value
K_vv = 0; 
K_uv = 0.01; % need to get this via ordinary pure-yaw vpmm with underfins
K_uv = 0.00;
K_vdot = 0; % need to get this via ordinary pure-yaw vpmm with underfins

K_rdot = 0; % maybe change later because asymmetric.
K_rr = 0; % would need to get this via ordinary pure-yaw vpmm with underfins
K_ur = 0; % would need to get this via ordinary pure-yaw vpmm with underfins

% Okay these three might are seriously unimportant:
Y_pdot = -K_vdot*b; 
Y_pp = -K_vv*b;
Y_up = -K_uv*b;

Y_pdot = 0; 
Y_pp = -0;
Y_up = -0;


%% All other terms


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ================== Terms that are definitely zero ===================
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% X_vdot = 0;
% X_rdot = 0;
% X_qdot = 0;
X_pdot = 0;

Y_wdot = 0;
Y_qdot = 0;
Y_udot = 0;

Z_udot = 0;
Z_pdot = 0;
Z_rdot = 0;
Z_vdot = 0;

K_wdot = 0;
K_qdot = 0;
K_ww = 0;
K_qq = 0;


M_udot = 0;
M_rdot = 0;

N_udot = 0; % something that could conceivably be non-zero, but is zero
N_pdot = 0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ======== Terms that are definitely symmetric to other terms =========
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ======= Surge axis =======
% X_wdot = X_vdot;

% ======= Heave axis =======
%Z_ww = Y_vv;   % if craft is symmetric
%Z_uw = Y_uv;   % if symmetric
%Z_wdot = Y_vdot; 
%Z_qq = -Y_rr;  % if symmetric
%Z_uq = -Y_ur;  % if symmetric
%Z_qdot = -Y_rdot;

% ======= Pitch axis ========
%M_uq = N_ur;  % if symmetric
%M_qq = N_rr;  % if symmetric
%M_qdot = N_rdot; % if symmetric
%M_ww = -N_vv;  % if symmetric
%M_uw = -N_uv;  % if symmetric
%M_wdot = -N_vdot; % if symmetric

%--------X Coefficients Added-------------

%% X coefficients
X_uv	=	-0.627072;
X_vv	=	-1.15448;
X_vdot	=	-3.21883;
X_rr	=	0.090838;
X_ur	=	-0.0212173;
X_rdot	=	-0.0141916;
%%Heave and Pitch		
X_uw	=	-0.590297;
X_ww	=	-1.48198;
X_wdot	=	-3.21851;
X_qq	=	-0.414574;
X_uq	=	0.0827942;
X_qdot	=   0.0329535;


% ======= Fin Terms =========
% THINGS TO MODIFY, DIVIDE BY 2, ETC
Z_uud = Y_uud;     % if symmetric control surfaces, (canards may violate)
M_uud =  -N_uud;      % according to our definition of d, if symmetric


%% Fin coefficients and thrust required via drag testing

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%    Rudder Inputs    %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

delta_re  = delta_r-0.178*atan2(-v,u) ;
delta_ee = delta_e ;

%%%%%%%%%%%%%%%    side-slip and angle of attack    %%%%%%%%%%%%%%%%
% UN-COMMENT BELOW IF CONSIDERING SIDE-SLIP ON RUDDER ANGLE OF ATTACK

% beta = atan2(-v,u);
% alpha = atan2(w,u);
% delta_re  = delta_r + beta; %effective rudder angle
% delta_ee = delta_e + alpha;



% Newtons, required thrust for the desired speed for 690
% simulate ineffective prop, divide X_uu to generate less trust (probably)
thrust = -X_uu*des_vel*abs(des_vel); % - X_uudde*u*abs(u)*(delta_ee^2) - X_uuddr*u*abs(u)*(delta_re^2); 

%% calculate the control input forces

% FOSSEN page 179 (eq 7.245)
Mrb = [  m       0       0       0        m*z_G    -m*y_G;...
         0       m       0      -m*z_G    0         m*x_G;...
         0       0       m       m*y_G   -m*x_G     0  ;...
         0      -m*z_G   m*y_G   I_xx    -I_xy     -I_xz;...
         m*z_G   0      -m*x_G  -I_yx     I_yy     -I_yz;...
        -m*y_G   m*x_G   0      -I_zx    -I_zy      I_zz ];

Ma = [  X_udot   X_vdot   X_wdot   X_pdot  X_qdot   X_rdot;...
        Y_udot   Y_vdot   Y_wdot   Y_pdot  Y_qdot   Y_rdot;...
        Z_udot   Z_vdot   Z_wdot   Z_pdot  Z_qdot   Z_rdot;...
        0        K_vdot   K_wdot   K_pdot  K_qdot   K_rdot;...
        M_udot   0        M_wdot   0       M_qdot   M_rdot;...
        N_udot   N_vdot   0        N_pdot  0        N_rdot];


% FOSSEN page 8, eq 1.2 # CONSTRUCTED 3/31 BASED ON FORMULAE
% (another instance on page 56, eq 3.60)
% this one identical to page 52, eq 3.41

Crb =  [  0                  0                   0                    m*(y_G*q + z_G*r)      -m*(x_G*q - w)          -m*(x_G*r + v) ;...
          0                  0                   0                   -m*(y_G*p + w)           m*(z_G*r + x_G*p)      -m*(y_G*r - u) ;...
          0                  0                   0                   -m*(z_G*p - v)          -m*(z_G*q + u)           m*(x_G*p + y_G*q) ;...
         -m*(y_G*q + z_G*r)  m*(y_G*p + 0)       m*(z_G*p - 0)        0                      (I_zz*r-I_xz*p-I_yz*q)  (I_yz*r+I_xy*p-I_yy*q) ;...
          m*(x_G*q - 0)     -m*(z_G*r + x_G*p)   m*(z_G*q + 0)       (I_yz*q+I_xz*p-I_zz*r)   0                      (I_xx*p-I_xz*r-I_xy*q) ;...
          m*(x_G*r + 0)      m*(y_G*r - 0)      -m*(x_G*p + y_G*q)   (I_yy*q-I_yz*r-I_xy*p)  (I_xz*r+I_xy*q-I_xx*p)   0];

%        u                               v                             w                             p                           q                               r  
Ca = -1*[0                               0                             0                             0                           Z_udot*u+0*Z_wdot*w+0*Z_qdot*q -0*Y_vdot*v-0*Y_rdot*r-Y_pdot*p ;...
         0                               0                             0                            -Z_wdot*w-Z_qdot*q-Z_udot*u  0                               0*X_udot*u+X_wdot*w+X_qdot*q ;...
         0                               0                             0                             Y_vdot*v+Y_rdot*r+Y_pdot*p -0*X_udot*u-X_wdot*w-X_qdot*q    0                          ;... 
         0                               Z_udot*u+Z_wdot*w+Z_qdot*q   -Y_vdot*v-Y_rdot*r-Y_pdot*p    0                           N_vdot*v+N_pdot*p+N_rdot*r     -M_qdot*q-M_wdot*w-M_udot*u ;...
        -Z_udot*u-0*Z_wdot*w-0*Z_qdot*q  0                             0*X_udot*u+X_wdot*w+X_qdot*q -N_rdot*r-N_vdot*v-N_pdot*p  0                               K_vdot*v+K_pdot*p+K_rdot*r ;...
         0*Y_vdot*v+Y_pdot*p+0*Y_rdot*r  0*X_udot*u-X_wdot*w-X_qdot*q  0                             M_udot*u+M_wdot*w+M_qdot*q -K_vdot*v-K_pdot*p-K_rdot*r      0                          ];
     
     
     
% D = -[X_uu*abs(u)  0                    0                   0                         0                   0                   ;...
%       0            Y_vv*abs(v)+Y_uv*u   0                   0                         0                   Y_rr*abs(r)+Y_ur*u  ;...
%       0            0                    Z_ww*abs(w)+Z_uw*u  0                         Z_qq*abs(q)+Z_uq*u  0                   ;...
%       0            K_vv*abs(v)+K_uv*u   0                   K_pp*abs(p)+K_up*u        0                   0                   ;...
%       0            0                    M_ww*abs(w)+M_uw*u  0                         M_qq*abs(q)+M_uq*u  0                   ;...
%       0            N_vv*abs(v)+N_uv*u   0                   0                         0                   N_rr*abs(r)+N_ur*u ];

D = -[X_uu*abs(u)  X_vv*abs(v)+X_uv*u   X_ww*abs(w)+X_uw*u  0                         X_qq*abs(q)+X_uq*u  X_rr*abs(r)+X_ur*u  ;...
      0            Y_vv*abs(v)+Y_uv*u   0                   0                         0                   Y_rr*abs(r)+Y_ur*u  ;...
      0            0                    Z_ww*abs(w)+Z_uw*u  0                         Z_qq*abs(q)+Z_uq*u  0                   ;...
      0            K_vv*abs(v)+K_uv*u   0                   K_pp*abs(p)+K_up*u        0                   0                   ;...
      0            0                    M_ww*abs(w)+M_uw*u  0                         M_qq*abs(q)+M_uq*u  0                   ;...
      0            N_vv*abs(v)+N_uv*u   0                   0                         0                   N_rr*abs(r)+N_ur*u ];
       

G = -grav.*[ (m-buoy)*sin(theta); 
    -(m-buoy)*cos(theta)*sin(phi); 
    -(m-buoy)*cos(theta)*cos(phi);
    -(y_G*m - y_B*buoy)*cos(theta)*cos(phi) + (z_G*m - z_B*buoy)*cos(theta)*sin(phi) ;
     ((z_G*m - z_B*buoy)*sin(theta)         + (x_G*m - x_B*buoy)*cos(theta)*cos(phi) ) ;
     (x_G*m - x_B*buoy)*cos(theta)*sin(phi) - (y_G*m - y_B*buoy)*sin(theta)         ];
 

tau = [thrust + X_uudd_fin*u*abs(u)*(sum([Top_Fin_Angle, Portside_Fin_Angle, ...
       Bottom_Fin_Angle, Starboard_Fin_Angle].^2))                     ; ... % + X_uudd_canards*u*abs(u)*(delta_canards^2)    
       Y_uud*u*abs(u)*delta_re                                         ; ...
       Z_uud*u*abs(u)*delta_ee                                         ; ... % + Z_uud_canards*u*abs(u)*delta_canards
       K_uud_roll*u*abs(u)*delta_roll                                  ; ...
       M_uud*u*abs(u)*delta_ee                                         ; ... % + M_uud_canards*u*abs(u)*delta_canards
       N_uud*u*abs(u)*delta_re                                         ];

  
%% End Matrix Definition

% total mass matrix
M = Mrb - Ma;

nu_dot = M\(Ca*nu - Crb*nu - D*nu + G + tau); % update law for nu (velocities)

%nu_dot(1) = 0; % lock the surge vel u 
% === locking pitch axis stuff =====
%  nu_dot(5) = 0; % lock the pitch
%  nu_dot(3) = 0; % lock the heave w
%  nu_dot(4) = 0; % lock the roll p

% %nu_dot(1) = 0; % lock the surge vel u 
% % === locking yaw axis stuff =====
% nu_dot(6) = 0; % lock the yaw
% nu_dot(2) = 0; % lock the sway w
% nu_dot(4) = 0; % lock the roll p

eta_dot = J(phi, theta, psi)*nu; % update law for eta (world positions)


end


