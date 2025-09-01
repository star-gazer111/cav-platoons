function res = simulate_exp2(P)
% Multiple obstacles + emergency stop-and-go (per Fig. 11b, Fig. 14–15)

% initial platoon on lane-2
y_lane2 = 1*P.laneWidth;
X0 = zeros(P.N,6); % [Y vy X vx theta r]
for i=1:P.N
    X0(i,:) = [y_lane2, 0, -(i-1)*P.hdwy_des, P.v_des, 0, 0];
end

% scenario base (steady OVs)
scenario = make_exp2_scenario(P);

% run base (no emergencies)
out_base = run_closed_loop(P, X0, scenario, 0.0, P.horizon2);

% emergency case 1
sc1 = scenario;
sc1 = apply_stopgo(sc1, P, P.exp2.case1);
out_c1 = run_closed_loop(P, X0, sc1, 0.0, P.horizon2);

% emergency case 2
sc2 = scenario;
sc2 = apply_stopgo(sc2, P, P.exp2.case2);
out_c2 = run_closed_loop(P, X0, sc2, 0.0, P.horizon2);

res.base = out_base;
res.case1= out_c1;
res.case2= out_c2;

end

function scenario = make_exp2_scenario(P)
scenario.type = 'exp2';
% OV1 lane-2 (same lane), slower 23 m/s
OV1.pos = [1*P.laneWidth, 60];
OV1.v   = [0, P.exp2.OV1.vx0];
OV1.a   = [0, 0];
OV1.type= 'car'; OV1.size=[P.len,P.wid]; OV1.mass=1500;

% OV2 lane-1 (left), 24 m/s in front of OV1
OV2.pos = [0*P.laneWidth, 90];
OV2.v   = [0, P.exp2.OV2.vx0];
OV2.a   = [0, 0];
OV2.type= 'car'; OV2.size=[P.len,P.wid]; OV2.mass=1500;

scenario.obstacles = [OV1, OV2];
end

function sc = apply_stopgo(sc, P, caseSpec)
% Apply decel/accel profile to the specified OV
names = { 'OV1','OV2' };
idx = find(strcmp(caseSpec.ov, names));
if idx==1, sel = 1; else, sel = 2; end

prof = struct('a',[caseSpec.a1, caseSpec.a2], 't',[caseSpec.t1, caseSpec.t2]);
sc.obstacles(sel).profile = prof;
end
