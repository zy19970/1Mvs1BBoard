function results = run_lifting_temperature_mismatch()
%RUN_LIFTING_TEMPERATURE_MISMATCH
% Demonstrate temperature-induced control mismatch for marine deployment/recovery.
%
% Core question:
%   A fixed hoisting/deployment controller tuned for a warm South-China-Sea-like
%   operating condition works acceptably at 25 degC. If the SAME controller and
%   SAME external sea disturbance are used while only temperature-dependent plant
%   parameters change, how does deployment/recovery performance degrade?
%
% IMPORTANT:
%   - No low-temperature compensation or gain scheduling is used.
%   - Controller gains are identical for every temperature.
%   - Ship motion / external disturbance records are identical for every case.
%   - Temperature only changes the controlled plant: winch lag/gain/dead-zone,
%     friction, rope stiffness/damping and effective swing damping.
%
% The simulated operation is one complete cycle:
%   hold -> deployment -> hold -> recovery -> hold
%
% Outputs:
%   Lifting_Temperature_Mismatch_Operation.png
%   Lifting_Temperature_Mismatch_Sweep.png
%   Lifting_Temperature_Mismatch_Parameters.png
%
% This is an engineering reduced-order model for necessity demonstration and
% control-mechanism study. Temperature sensitivities must ultimately be replaced
% by identified low-temperature test data for equipment-level prediction.

clc;
close all;
rng(20260918, 'twister');

%% 1. Global simulation settings
p.dt = 0.01;                 % s
p.Tsim = 360;                % s
p.t = 0:p.dt:p.Tsim;
p.fs = 1/p.dt;
p.N = numel(p.t);
p.g = 9.81;

% Payload / cable
p.payloadMass = 2200;        % kg
p.ropeK0 = 1.45e5;           % N/m, equivalent axial stiffness at 25 degC
p.ropeC0 = 1.10e4;           % N*s/m, equivalent axial damping
p.swingZeta0 = 0.045;        % equivalent pendulum damping at 25 degC
p.boomHeight = 5.5;          % m, suspension point lever arm for roll excitation

% Warm-condition reference temperature
p.Tref = 25;                 % degC

% Fixed controller: tuned ONCE at the warm reference condition
p.Kp = 0.55;                 % 1/s
p.Ki = 0.08;                 % 1/s^2
p.Kd = 0.18;                 % dimensionless velocity feedback
p.heaveFF = 1.00;            % fixed active-heave feedforward coefficient
p.integralLimit = 3.0;       % m*s
p.maxRopeSpeed = 0.55;       % m/s
p.maxRopeAccel = 0.35;       % m/s^2

% Warm-condition winch actuator
p.winchTau0 = 0.20;          % s
p.winchGain0 = 1.00;         % normalized
p.deadZone0 = 0.002;         % m/s equivalent command dead zone
p.frictionAccel0 = 0.003;    % m/s^2 equivalent friction term

% Temperature sensitivity surrogate.
% These coefficients intentionally belong to the PLANT, not the controller.
p.tauTempCoeff = 0.030;       % lag increase per degC below 25
p.gainTempCoeff = 0.0065;     % drive authority loss per degC below 25
p.deadTempCoeff = 0.0012;     % dead-zone growth (m/s)/degC
p.frictionTempCoeff = 0.0005;% friction acceleration growth per degC
p.ropeKTempCoeff = 0.0015;    % rope stiffness relative change per degC
p.ropeCTempCoeff = 0.0040;    % rope damping relative change per degC
p.swingDampTempCoeff = 0.014; % effective swing damping degradation per degC

% Temperature sweep
temps = [25 10 0 -10 -20 -30 -40];

%% 2. Build one common ship-motion disturbance record
% Exactly the same exogenous sea excitation is reused at every temperature.
env = build_common_ship_motion(p);

%% 3. Build one common deployment + recovery reference
ref = build_operation_reference(p);

%% 4. Simulate all temperatures with exactly the same controller
cases = cell(size(temps));
for i = 1:numel(temps)
    cases{i} = simulate_temperature_case(p, env, ref, temps(i));
end

%% 5. Collect temperature-sweep metrics
M.temperature_C = temps;
M.depthRms_m = zeros(size(temps));
M.depthPeak_m = zeros(size(temps));
M.swingRms_deg = zeros(size(temps));
M.swingPeak_deg = zeros(size(temps));
M.tensionCV_pct = zeros(size(temps));
M.tensionRms_kN = zeros(size(temps));
M.winchTau_s = zeros(size(temps));
M.winchGain = zeros(size(temps));
M.deadZone_mps = zeros(size(temps));
M.ropeStiffnessRatio = zeros(size(temps));
M.ropeDampingRatio = zeros(size(temps));
M.swingDampingRatio = zeros(size(temps));

for i = 1:numel(temps)
    c = cases{i};
    M.depthRms_m(i) = c.metrics.depthRms_m;
    M.depthPeak_m(i) = c.metrics.depthPeak_m;
    M.swingRms_deg(i) = c.metrics.swingRms_deg;
    M.swingPeak_deg(i) = c.metrics.swingPeak_deg;
    M.tensionCV_pct(i) = c.metrics.tensionCV_pct;
    M.tensionRms_kN(i) = c.metrics.tensionRms_kN;
    M.winchTau_s(i) = c.params.winchTau;
    M.winchGain(i) = c.params.winchGain;
    M.deadZone_mps(i) = c.params.deadZone;
    M.ropeStiffnessRatio(i) = c.params.ropeK/p.ropeK0;
    M.ropeDampingRatio(i) = c.params.ropeC/p.ropeC0;
    M.swingDampingRatio(i) = c.params.swingZeta/p.swingZeta0;
end

% Composite mismatch index: 1.0 means the 25 degC warm reference.
% This is a simulation summary metric, not an equipment qualification limit.
M.mismatchIndex = 0.40*(M.depthRms_m/M.depthRms_m(1)) ...
                + 0.35*(M.swingRms_deg/M.swingRms_deg(1)) ...
                + 0.25*(M.tensionCV_pct/M.tensionCV_pct(1));

%% 6. Plot results
fileOperation = 'Lifting_Temperature_Mismatch_Operation.png';
fileSweep = 'Lifting_Temperature_Mismatch_Sweep.png';
fileParameters = 'Lifting_Temperature_Mismatch_Parameters.png';

plot_operation_comparison(p, env, ref, cases, temps, fileOperation);
plot_temperature_sweep(M, fileSweep);
plot_parameter_drift(M, p, fileParameters);

%% 7. Console summary
fprintf('\n======================================================================\n');
fprintf('Fixed lifting controller under temperature-dependent plant mismatch\n');
fprintf('Same controller + same sea disturbance; temperature is the variable.\n');
fprintf('----------------------------------------------------------------------\n');
fprintf(' Temp[C] | depth RMS[m] | swing RMS[deg] | swing peak[deg] | tension CV[%%] | mismatch\n');
fprintf('----------------------------------------------------------------------\n');
for i = 1:numel(temps)
    fprintf('%8.1f | %12.3f | %14.3f | %15.3f | %13.3f | %8.3f\n', ...
        temps(i), M.depthRms_m(i), M.swingRms_deg(i), M.swingPeak_deg(i), ...
        M.tensionCV_pct(i), M.mismatchIndex(i));
end
fprintf('----------------------------------------------------------------------\n');
fprintf('Figure 1: %s\n', fileOperation);
fprintf('Figure 2: %s\n', fileSweep);
fprintf('Figure 3: %s\n', fileParameters);
fprintf('======================================================================\n\n');

%% 8. Return all results
results.meta.description = ['Temperature-induced mismatch of a fixed marine ' ...
    'deployment/recovery controller; no low-temperature compensation.'];
results.meta.controller = 'Identical fixed controller for every temperature';
results.meta.environment = 'Identical ship-motion disturbance for every temperature';
results.meta.note = ['Reduced-order necessity-demonstration model. Replace ' ...
    'temperature sensitivities with identified low-temperature test data.'];
results.parameters = p;
results.environment = env;
results.reference = ref;
results.temperatures_C = temps;
results.cases = cases;
results.metrics = M;
results.outputFiles.operation = fileOperation;
results.outputFiles.sweep = fileSweep;
results.outputFiles.parameters = fileParameters;

end

%% ========================================================================
function env = build_common_ship_motion(p)
% Build a single nonstationary marine disturbance record.
% zSusp: vertical suspension-point motion, positive downward.
% xSusp: horizontal suspension-point motion caused mainly by ship roll.

t = p.t;
N = p.N;
dt = p.dt;

% Heave: irregular multi-frequency motion with slowly varying envelope.
heaveFreq = [0.085 0.110 0.145 0.180 0.220];
heaveAmp  = [0.220 0.180 0.120 0.080 0.050];
heavePhase = 2*pi*rand(1,numel(heaveFreq));
heaveEnvelope = 1 + 0.18*colored_noise(N, dt, 25);
heaveEnvelope = min(1.50, max(0.55, heaveEnvelope));

heave = zeros(1,N);
for k = 1:numel(heaveFreq)
    heave = heave + heaveAmp(k)*sin(2*pi*heaveFreq(k)*t + heavePhase(k));
end
heave = heave.*heaveEnvelope;

% Roll: another independent nonstationary record.
rollFreq = [0.070 0.095 0.130 0.170];
rollAmpDeg = [1.60 1.20 0.80 0.50];
rollPhase = 2*pi*rand(1,numel(rollFreq));
rollEnvelope = 1 + 0.20*colored_noise(N, dt, 30);
rollEnvelope = min(1.60, max(0.50, rollEnvelope));

roll = zeros(1,N);
for k = 1:numel(rollFreq)
    roll = roll + deg2rad(rollAmpDeg(k))* ...
        sin(2*pi*rollFreq(k)*t + rollPhase(k));
end
roll = roll.*rollEnvelope;

env.heave = heave;
env.roll = roll;
env.zSusp = heave;
env.xSusp = p.boomHeight*sin(roll);

env.zSuspVel = numerical_derivative(env.zSusp, dt);
env.zSuspAcc = numerical_derivative(env.zSuspVel, dt);
env.xSuspVel = numerical_derivative(env.xSusp, dt);
env.xSuspAcc = numerical_derivative(env.xSuspVel, dt);

end

%% ========================================================================
function ref = build_operation_reference(p)
% Full operation:
% 0-30 s    : initial hold
% 30-150 s  : deployment by 18 m
% 150-190 s : lower-position hold
% 190-310 s : recovery by 18 m
% 310-360 s : final hold

t = p.t;
z0 = 10.0;
travel = 18.0;

ref.depth = z0*ones(size(t));
ref.velocity = zeros(size(t));

% Deployment
idx = (t >= 30 & t <= 150);
s = (t(idx)-30)/120;
ref.depth(idx) = z0 + travel*smoothstep5(s);
ref.velocity(idx) = travel/120*dsmoothstep5(s);
ref.depth(t > 150 & t < 190) = z0 + travel;

% Recovery
idx = (t >= 190 & t <= 310);
s = (t(idx)-190)/120;
ref.depth(idx) = z0 + travel*(1-smoothstep5(s));
ref.velocity(idx) = -travel/120*dsmoothstep5(s);
ref.depth(t > 310) = z0;

ref.stageTimes = [30 150 190 310];

end

%% ========================================================================
function out = simulate_temperature_case(p, env, ref, T)
% Same controller for every case. Only plant parameters depend on temperature.

dt = p.dt;
N = p.N;
g = p.g;
m = p.payloadMass;

dT = max(0, p.Tref - T);

% Temperature-dependent plant
ropeK = p.ropeK0*(1 + p.ropeKTempCoeff*dT);
ropeC = p.ropeC0*(1 + p.ropeCTempCoeff*dT);
swingZeta = p.swingZeta0/(1 + p.swingDampTempCoeff*dT);

winchTau = p.winchTau0*(1 + p.tauTempCoeff*dT);
winchGain = max(0.52, p.winchGain0 - p.gainTempCoeff*dT);
deadZone = p.deadZone0 + p.deadTempCoeff*dT;
frictionAccel = p.frictionAccel0 + p.frictionTempCoeff*dT;

% States
L = zeros(1,N);               % unstretched paid-out rope length
vWinch = zeros(1,N);
theta = zeros(1,N);           % pendulum angle
omega = zeros(1,N);
delta = zeros(1,N);           % rope elastic extension
deltaVel = zeros(1,N);

zLoad = zeros(1,N);
xLoad = zeros(1,N);
tension = zeros(1,N);
uCmd = zeros(1,N);
depthError = zeros(1,N);

% Initial equilibrium
theta(1) = deg2rad(0.5);
delta(1) = m*g/ropeK;
L(1) = (ref.depth(1)-env.zSusp(1))/cos(theta(1)) - delta(1);
L(1) = max(4.0, L(1));
integralError = 0;

for k = 1:N-1
    r = max(2.0, L(k)+delta(k));

    % Current payload position
    zLoad(k) = env.zSusp(k) + r*cos(theta(k));
    xLoad(k) = env.xSusp(k) + r*sin(theta(k));

    % Rope tension
    tension(k) = max(0, ropeK*delta(k) + ropeC*deltaVel(k));

    % Payload vertical velocity
    zLoadVel = env.zSuspVel(k) ...
        + (vWinch(k)+deltaVel(k))*cos(theta(k)) ...
        - r*sin(theta(k))*omega(k);

    % ---------------------------------------------------------------
    % FIXED warm-condition controller: identical at every temperature.
    % ---------------------------------------------------------------
    e = ref.depth(k) - zLoad(k);
    ev = ref.velocity(k) - zLoadVel;
    integralError = integralError + e*dt;
    integralError = min(p.integralLimit, max(-p.integralLimit, integralError));

    u = ref.velocity(k) ...
        - p.heaveFF*env.zSuspVel(k) ...
        + p.Kp*e + p.Ki*integralError + p.Kd*ev;

    u = min(p.maxRopeSpeed, max(-p.maxRopeSpeed, u));
    uCmd(k) = u;

    % Temperature-dependent drive dead-zone and authority loss
    if abs(u) <= deadZone
        drive = 0;
    else
        drive = sign(u)*(abs(u)-deadZone);
    end
    vTarget = winchGain*drive;

    % Temperature-dependent winch lag and friction
    vAcc = (vTarget-vWinch(k))/winchTau ...
        - frictionAccel*tanh(vWinch(k)/0.01);
    vAcc = min(p.maxRopeAccel, max(-p.maxRopeAccel, vAcc));

    vWinch(k+1) = vWinch(k) + dt*vAcc;
    vWinch(k+1) = min(p.maxRopeSpeed, max(-p.maxRopeSpeed, vWinch(k+1)));
    L(k+1) = max(4.0, L(k) + dt*vWinch(k+1));

    % Pendulum tangential dynamics with moving suspension point
    erx = sin(theta(k));
    erz = cos(theta(k));
    etx = cos(theta(k));
    etz = -sin(theta(k));

    aSupportT = env.xSuspAcc(k)*etx + env.zSuspAcc(k)*etz;
    rDot = vWinch(k) + deltaVel(k);
    wnSwing = sqrt(g/r);

    thetaAcc = (-g*sin(theta(k)) - aSupportT ...
        - 2*rDot*omega(k))/r ...
        - 2*swingZeta*wnSwing*omega(k);

    omega(k+1) = omega(k) + dt*thetaAcc;
    theta(k+1) = theta(k) + dt*omega(k+1);

    % Rope radial dynamics
    aSupportR = env.xSuspAcc(k)*erx + env.zSuspAcc(k)*erz;

    deltaAcc = g*cos(theta(k)) - aSupportR - vAcc ...
        + r*omega(k)^2 ...
        - (ropeK*delta(k) + ropeC*deltaVel(k))/m;

    deltaVel(k+1) = deltaVel(k) + dt*deltaAcc;
    delta(k+1) = delta(k) + dt*deltaVel(k+1);

    depthError(k) = e;
end

% Final sample
r = max(2.0, L(end)+delta(end));
zLoad(end) = env.zSusp(end) + r*cos(theta(end));
xLoad(end) = env.xSusp(end) + r*sin(theta(end));
tension(end) = max(0, ropeK*delta(end) + ropeC*deltaVel(end));
depthError(end) = ref.depth(end)-zLoad(end);
uCmd(end) = uCmd(end-1);

% Metrics over the relevant operation window
idxOp = (p.t >= 20 & p.t <= 330);
Tmean = mean(tension(idxOp));
Tfluct = tension(idxOp)-Tmean;

metrics.depthRms_m = rms_local(depthError(idxOp));
metrics.depthPeak_m = max(abs(depthError(idxOp)));
metrics.swingRms_deg = rad2deg(rms_local(theta(idxOp)));
metrics.swingPeak_deg = rad2deg(max(abs(theta(idxOp))));
metrics.tensionCV_pct = 100*std(tension(idxOp))/max(Tmean,eps);
metrics.tensionRms_kN = rms_local(Tfluct)/1000;
metrics.meanTension_kN = Tmean/1000;

out.temperature_C = T;
out.params.ropeK = ropeK;
out.params.ropeC = ropeC;
out.params.swingZeta = swingZeta;
out.params.winchTau = winchTau;
out.params.winchGain = winchGain;
out.params.deadZone = deadZone;
out.params.frictionAccel = frictionAccel;
out.metrics = metrics;

out.ropeLength = L;
out.winchSpeed = vWinch;
out.commandSpeed = uCmd;
out.swingAngle = theta;
out.ropeExtension = delta;
out.payloadDepth = zLoad;
out.payloadHorizontal = xLoad;
out.tension = tension;
out.depthError = depthError;

end

%% ========================================================================
function plot_operation_comparison(p, env, ref, cases, temps, outputFile)
% Selected temperatures show how the SAME strategy progressively departs from
% the warm reference behavior.

C = ieee_colors();
selectedT = [25 0 -20 -40];
selectedIdx = zeros(size(selectedT));
for j = 1:numel(selectedT)
    [~, selectedIdx(j)] = min(abs(temps-selectedT(j)));
end
styles = {'-','--','-.',':'};
colors = [C.gray; C.blue; C.gold; C.red];

fig = ieee_figure(17.8, 14.0);

ax = subplot(2,2,1);
plot(p.t, ref.depth, 'k-', 'LineWidth',1.15); hold on;
for j = 1:numel(selectedIdx)
    plot(p.t, cases{selectedIdx(j)}.payloadDepth, styles{j}, ...
        'Color', colors(j,:), 'LineWidth',1.0);
end
xlabel('时间 (s)'); ylabel('吊载深度 (m)');
title('(a) 装备布放与回收轨迹');
legend('参考轨迹','25 ^{\circ}C','0 ^{\circ}C','-20 ^{\circ}C','-40 ^{\circ}C', ...
    'Location','best');
ieee_axes(ax);

ax = subplot(2,2,2);
for j = 1:numel(selectedIdx)
    plot(p.t, cases{selectedIdx(j)}.depthError, styles{j}, ...
        'Color', colors(j,:), 'LineWidth',1.0); hold on;
end
xlabel('时间 (s)'); ylabel('深度跟踪误差 (m)');
title('(b) 同一控制策略下的深度跟踪误差');
legend('25 ^{\circ}C','0 ^{\circ}C','-20 ^{\circ}C','-40 ^{\circ}C', ...
    'Location','best');
ieee_axes(ax);

ax = subplot(2,2,3);
for j = 1:numel(selectedIdx)
    plot(p.t, rad2deg(cases{selectedIdx(j)}.swingAngle), styles{j}, ...
        'Color', colors(j,:), 'LineWidth',1.0); hold on;
end
xlabel('时间 (s)'); ylabel('吊载摆角 (deg)');
title('(c) 吊载摆动响应');
legend('25 ^{\circ}C','0 ^{\circ}C','-20 ^{\circ}C','-40 ^{\circ}C', ...
    'Location','best');
ieee_axes(ax);

ax = subplot(2,2,4);
for j = 1:numel(selectedIdx)
    c = cases{selectedIdx(j)};
    idxOp = (p.t >= 20 & p.t <= 330);
    meanT = mean(c.tension(idxOp));
    plot(p.t, (c.tension-meanT)/1000, styles{j}, ...
        'Color', colors(j,:), 'LineWidth',0.95); hold on;
end
xlabel('时间 (s)'); ylabel('缆绳张力波动 (kN)');
title('(d) 缆绳张力波动');
legend('25 ^{\circ}C','0 ^{\circ}C','-20 ^{\circ}C','-40 ^{\circ}C', ...
    'Location','best');
ieee_axes(ax);

ieee_export(fig, outputFile);
end

%% ========================================================================
function plot_temperature_sweep(M, outputFile)
% Temperature is plotted warm -> cold from left to right to make the
% progressive mismatch visually obvious.

C = ieee_colors();
T = M.temperature_C;

fig = ieee_figure(17.8, 13.8);

ax = subplot(2,2,1);
plot(T, M.depthRms_m, '-o', 'Color',C.blue, 'MarkerFaceColor','w', ...
    'LineWidth',1.25, 'MarkerSize',4.5);
set(gca,'XDir','reverse');
xlabel('环境温度 (^{\circ}C)'); ylabel('深度误差 RMS (m)');
title('(a) 深度跟踪性能随温度降低逐渐恶化');
ieee_axes(ax);

ax = subplot(2,2,2);
plot(T, M.swingRms_deg, '-o', 'Color',C.blue, 'MarkerFaceColor','w', ...
    'LineWidth',1.25, 'MarkerSize',4.5); hold on;
plot(T, M.swingPeak_deg, '--s', 'Color',C.red, 'MarkerFaceColor','w', ...
    'LineWidth',1.15, 'MarkerSize',4.2);
set(gca,'XDir','reverse');
xlabel('环境温度 (^{\circ}C)'); ylabel('吊载摆角 (deg)');
title('(b) 吊载摆动随温度降低增大');
legend('均方根 RMS','峰值','Location','best');
ieee_axes(ax);

ax = subplot(2,2,3);
plot(T, M.tensionCV_pct, '-o', 'Color',C.gold, 'MarkerFaceColor','w', ...
    'LineWidth',1.25, 'MarkerSize',4.5);
set(gca,'XDir','reverse');
xlabel('环境温度 (^{\circ}C)'); ylabel('张力变异系数 (%)');
title('(c) 缆绳张力波动随温度降低增大');
ieee_axes(ax);

ax = subplot(2,2,4);
plot(T, M.mismatchIndex, '-o', 'Color',C.red, 'MarkerFaceColor','w', ...
    'LineWidth',1.35, 'MarkerSize',4.8); hold on;
yline(1.0,'--','Color',C.gray,'LineWidth',0.8);
set(gca,'XDir','reverse');
xlabel('环境温度 (^{\circ}C)'); ylabel('归一化控制失配指数');
title('(d) 固定控制策略的温度失配程度');
legend('控制失配指数','25 ^{\circ}C 基准','Location','best');
ieee_axes(ax);

ieee_export(fig, outputFile);
end

%% ========================================================================
function plot_parameter_drift(M, p, outputFile)
% Explain WHY the same controller becomes increasingly mismatched.

C = ieee_colors();
T = M.temperature_C;

fig = ieee_figure(17.8, 13.8);

ax = subplot(2,2,1);
plot(T, M.winchTau_s/p.winchTau0, '-o', 'Color',C.red, ...
    'MarkerFaceColor','w','LineWidth',1.25,'MarkerSize',4.5);
set(gca,'XDir','reverse');
xlabel('环境温度 (^{\circ}C)'); ylabel('归一化时间常数');
title('(a) 绞车响应随温度降低变慢');
ieee_axes(ax);

ax = subplot(2,2,2);
plot(T, M.winchGain/p.winchGain0, '-o', 'Color',C.blue, ...
    'MarkerFaceColor','w','LineWidth',1.25,'MarkerSize',4.5);
set(gca,'XDir','reverse');
xlabel('环境温度 (^{\circ}C)'); ylabel('归一化驱动增益');
title('(b) 有效驱动能力随温度降低下降');
ieee_axes(ax);

ax = subplot(2,2,3);
plot(T, M.deadZone_mps, '-o', 'Color',C.gold, ...
    'MarkerFaceColor','w','LineWidth',1.25,'MarkerSize',4.5);
set(gca,'XDir','reverse');
xlabel('环境温度 (^{\circ}C)'); ylabel('等效死区 (m/s)');
title('(c) 摩擦与死区随温度降低增大');
ieee_axes(ax);

ax = subplot(2,2,4);
plot(T, M.ropeStiffnessRatio, '-o', 'Color',C.blue, ...
    'MarkerFaceColor','w','LineWidth',1.15,'MarkerSize',4.2); hold on;
plot(T, M.ropeDampingRatio, '--s', 'Color',C.gold, ...
    'MarkerFaceColor','w','LineWidth',1.15,'MarkerSize',4.2);
plot(T, M.swingDampingRatio, '-.^', 'Color',C.red, ...
    'MarkerFaceColor','w','LineWidth',1.15,'MarkerSize',4.2);
set(gca,'XDir','reverse');
xlabel('环境温度 (^{\circ}C)'); ylabel('归一化参数');
title('(d) 缆绳与吊摆动力学参数漂移');
legend('缆绳轴向刚度','缆绳轴向阻尼','吊摆等效阻尼','Location','best');
ieee_axes(ax);

ieee_export(fig, outputFile);
end

%% ========================================================================
function y = smoothstep5(s)
y = 10*s.^3 - 15*s.^4 + 6*s.^5;
end

%% ========================================================================
function y = dsmoothstep5(s)
y = 30*s.^2 - 60*s.^3 + 30*s.^4;
end

%% ========================================================================
function y = colored_noise(N, dt, tau)
w = randn(1,N);
y = zeros(1,N);
a = dt/(tau+dt);
for k = 2:N
    y(k) = y(k-1) + a*(w(k)-y(k-1));
end
y = y-mean(y);
sy = std(y);
if sy > eps
    y = y/sy;
end
end

%% ========================================================================
function dx = numerical_derivative(x, dt)
dx = zeros(size(x));
dx(2:end-1) = (x(3:end)-x(1:end-2))/(2*dt);
dx(1) = (x(2)-x(1))/dt;
dx(end) = (x(end)-x(end-1))/dt;
end

%% ========================================================================
function val = rms_local(x)
val = sqrt(mean(x.^2));
end

%% ========================================================================
function C = ieee_colors()
C.blue = [0.0000 0.4470 0.7410];
C.red  = [0.8500 0.3250 0.0980];
C.gold = [0.9290 0.6940 0.1250];
C.gray = [0.35 0.35 0.35];
end

%% ========================================================================
function fig = ieee_figure(widthCm, heightCm)
fig = figure('Color','w','Units','centimeters', ...
    'Position',[2 2 widthCm heightCm],'PaperPositionMode','auto');
end

%% ========================================================================
function ieee_axes(ax)
set(ax,'FontName','Times New Roman','FontSize',8.5, ...
    'LineWidth',0.75,'TickDir','out','TickLength',[0.015 0.015], ...
    'Box','on','XGrid','on','YGrid','on','GridAlpha',0.12, ...
    'MinorGridAlpha',0.06,'Layer','top');
set(get(ax,'XLabel'),'FontName','SimSun','FontSize',9);
set(get(ax,'YLabel'),'FontName','SimSun','FontSize',9);
set(get(ax,'Title'),'FontName','SimSun','FontSize',9,'FontWeight','normal');
lgd = findobj(ax.Parent,'Type','Legend');
if ~isempty(lgd)
    set(lgd,'FontName','SimSun','FontSize',7.5,'Box','off');
end
end

%% ========================================================================
function ieee_export(fig, outputFile)
set(fig,'InvertHardcopy','off');
print(fig, outputFile, '-dpng', '-r600');
end
