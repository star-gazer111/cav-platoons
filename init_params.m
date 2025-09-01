function P = init_params()
% ---- core sim ----
P.dt        = 0.05;
P.horizon1  = 20.0;
P.horizon2  = 30.0;

% ---- road / lanes ----
P.laneWidth = 4.0;
P.lanes.exp1 = [0 1 2 3]*P.laneWidth;
P.lanes.exp2 = [0 1]*P.laneWidth;

% ---- platoon ----
P.N        = 5;
P.v_des    = 25.0;
P.hdwy_des = 18.0;        % tighter headway so merging matters

% ---- geometry & dynamics ----
P.len=4.5; P.wid=1.8; P.r_eff=1.0;
P.m=1500; P.Iz=2250; P.lf=1.2; P.lr=1.6; P.Cf=65000; P.Cr=65000; 
P.mu=0.9; P.h_cg=0.55; P.Tmax=4000; P.Rw=0.31;
P.v_min=0; P.v_max=40; P.th_min=-0.5; P.th_max=0.5;
P.steer_min=-0.5; P.steer_max=0.5; P.ax_min=-5; P.ax_max=3;
P.rdot_min=-0.5; P.rdot_max=0.5;

% ---- IDM/TTM (placeholders) ----
P.idm_T=1.2; P.idm_s0=2.0; P.idm_a=1.2; P.idm_b=2.0;

% ---- undetermined constants (will be calibrated) ----
P.G=0.5; P.zeta=1.2; P.K=5.0; P.eps=1.0; P.rho=0.2; P.omega1=1.0; P.omega2=1.0;
P.road = struct('xi',1,'xi0',1,'mu',1,'mu0',1,'varrho',0,'varrho0',0,'tau',0,'tau0',0, ...
                'g1',0,'g2',0,'g3',0,'g4',0);

% ---- Experiment 1 ----
P.exp1.OV.vx0   = 22.0;
P.exp1.OV.alat  = +0.5;                % POSITIVE => moves from lane-1 upward across lanes
P.exp1.delays   = [0, 0.025, 0.05, 0.075, 0.10];

% ---- Experiment 2 (unchanged) ----
P.exp2.OV1.vx0 = 23.0;
P.exp2.OV2.vx0 = 24.0;
P.exp2.case1 = struct('ov','OV1','a1',-2.0,'t1',2.0,'a2',1.0,'t2',4.0);
P.exp2.case2 = struct('ov','OV2','a1',-1.0,'t1',3.0,'a2',1.5,'t2',2.0);

% ... keep your file as-is, but ensure these exist:
P.exp1.dx0   = 6.5;     % initial guess: OV is ~6.5 m ahead of leader
P.exp1.OV.alat = 0.5;   % initial lateral accel magnitude (m/s^2)
end
