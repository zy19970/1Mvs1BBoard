function results = run_1Mvs1B_SeaState4()
%RUN_1MVS1B_SEASTATE4 Compare 10,000-t and 100-t ship motions in Sea State 4.
%
% Usage:
%   results = run_1Mvs1B_SeaState4();
%
% This is a reduced-order, physically interpretable seakeeping simulation.
% It is intended for engineering comparison and presentation, not for class
% approval or replacement of vessel-specific model tests/CFD/RAO data.
%
% The model contains only ONE public entry function. All helper functions are
% local functions in this file.
%
% Simulated rotational DOFs:
%   1) roll  (横摇)
%   2) pitch (纵摇)
%   3) yaw   (艏摇/偏航)
%
% Environmental model:
%   - Sea State 4 representative combined significant wave height ~1.9 m
%   - JONSWAP oblique wind sea: Hs = 1.7 m, Tp = 7.5 s, heading = +35 deg
%   - long-period swell:       Hs = 0.8 m, Tp = 14.0 s, heading = -20 deg
%   - multi-harmonic low-frequency tide
%   - transient surge packets
%   - second-order wave-drift surrogate from low-pass-filtered eta^2
%
% Vessel model:
%   I*qdd + C*qd + K*q = M_env(t)
% where I is the effective rotational inertia, C is linear damping and K is
% the effective restoring/directional-stability stiffness. Wave forcing is
% generated from finite-hull spatial filtering, so the 120 m vessel does not
% simply follow every short wave component while the 25 m vessel responds
% much more strongly.
%
% Important assumption:
%   Both vessels are treated at zero/very-low forward speed to isolate the
%   hull-size/inertia effect. For a named real vessel, replace the dimensions,
%   GM, damping and RAO/hydrodynamic coefficients with measured/model-test data.
%
% Output:
%   - one figure with roll/pitch/yaw comparison curves
%   - PNG: SeaState4_10000t_vs_100t_3DOF.png
%   - returned struct containing time histories, environment and model data
%
% Reproducible seed: 20260916

clc;
close all;
rng(20260916, 'twister');

%% 1. Global simulation settings
g = 9.81;                    % m/s^2
rho = 1025;                  % kg/m^3, seawater (stored for reference)
dt = 0.10;                   % s
Tsim = 900;                  % s, 15 min
t = 0:dt:Tsim;

% Sea State 4 representative combination.
% Independent wind-sea + swell variances combine approximately by RSS:
% Hs_total ~= sqrt(1.7^2 + 0.8^2) = 1.88 m.
windSea = make_jonswap_component(1.70, 7.50, 3.3, 35.0, ...
    0.025, 0.65, 320, g);
swell = make_jonswap_component(0.80, 14.0, 5.0, -20.0, ...
    0.015, 0.35, 220, g);
sea = combine_components(windSea, swell);

% Realized free-surface elevation at the vessel reference point.
eta = synthesize_scalar(sea.A, sea.omega, sea.phase, t);
HsRealized = 4.0 * std(eta);

%% 2. Representative vessel parameters
large = make_ship('一万吨级船舶', 10000, 120.0, 20.0, 6.5, 0.95, ...
    0.10, 8.2, 0.16, 48.0, 0.28, [0.35 0.22 1.30], g);
small = make_ship('一百吨级船舶',   100,  25.0,  6.0, 1.8, 0.75, ...
    0.08, 3.9, 0.13, 18.0, 0.22, [1.40 0.80 5.00], g);

%% 3. Complex low-frequency tide / surge / second-order drift
envLF = make_low_frequency_environment(t, eta, dt);

%% 4. Build equivalent environmental attitude excitation
qLarge = build_equivalent_excitation(large, sea, t, envLF);
qSmall = build_equivalent_excitation(small, sea, t, envLF);

%% 5. Solve physical 3-DOF reduced-order equations
motionLarge.roll  = simulate_rotational_dof(qLarge.roll,  large.roll,  dt);
motionLarge.pitch = simulate_rotational_dof(qLarge.pitch, large.pitch, dt);
motionLarge.yaw   = simulate_rotational_dof(qLarge.yaw,   large.yaw,   dt);

motionSmall.roll  = simulate_rotational_dof(qSmall.roll,  small.roll,  dt);
motionSmall.pitch = simulate_rotational_dof(qSmall.pitch, small.pitch, dt);
motionSmall.yaw   = simulate_rotational_dof(qSmall.yaw,   small.yaw,   dt);

% Convert radians to degrees for plotting/reporting.
rad2deg_ = 180/pi;
largeDeg.roll  = motionLarge.roll  * rad2deg_;
largeDeg.pitch = motionLarge.pitch * rad2deg_;
largeDeg.yaw   = motionLarge.yaw   * rad2deg_;
smallDeg.roll  = motionSmall.roll  * rad2deg_;
smallDeg.pitch = motionSmall.pitch * rad2deg_;
smallDeg.yaw   = motionSmall.yaw   * rad2deg_;

%% 6. Plot one figure with all three rotational DOFs
fig = figure('Color', 'w', 'Position', [80 60 1320 900]);
largeColor = [0.10 0.32 0.68];
smallColor = [0.90 0.32 0.08];

ax1 = subplot(3,1,1);
plot(t, largeDeg.roll, 'LineWidth', 1.35, 'Color', largeColor); hold on;
plot(t, smallDeg.roll, 'LineWidth', 1.10, 'Color', smallColor);
grid on; box on;
ylabel('横摇 / deg');
title('横摇 Roll');
legend(large.name, small.name, 'Location', 'best');

ax2 = subplot(3,1,2);
plot(t, largeDeg.pitch, 'LineWidth', 1.35, 'Color', largeColor); hold on;
plot(t, smallDeg.pitch, 'LineWidth', 1.10, 'Color', smallColor);
grid on; box on;
ylabel('纵摇 / deg');
title('纵摇 Pitch');

ax3 = subplot(3,1,3);
plot(t, largeDeg.yaw, 'LineWidth', 1.35, 'Color', largeColor); hold on;
plot(t, smallDeg.yaw, 'LineWidth', 1.10, 'Color', smallColor);
grid on; box on;
ylabel('艏摇 / deg');
xlabel('时间 / s');
title('艏摇 / 偏航 Yaw');

linkaxes([ax1 ax2 ax3], 'x');
xlim([0 Tsim]);

sgtitle(sprintf(['四级海况：一万吨级船舶 vs 一百吨级船舶 3-DOF运动对比\n' ...
    '斜向风浪 + 长周期涌浪 + 潮汐低频项 + 瞬态浪涌 + 二阶漂移；实现 H_s = %.2f m'], ...
    HsRealized), 'FontWeight', 'bold');

set(fig, 'PaperPositionMode', 'auto');
print(fig, 'SeaState4_10000t_vs_100t_3DOF.png', '-dpng', '-r220');

%% 7. Summary metrics
fprintf('\n==============================================================\n');
fprintf('Sea State 4 representative simulation\n');
fprintf('Realized combined Hs = %.3f m\n', HsRealized);
fprintf('Wind sea: Hs 1.70 m, Tp 7.50 s, direction +35 deg\n');
fprintf('Swell:    Hs 0.80 m, Tp 14.0 s, direction -20 deg\n');
fprintf('Simulation duration = %.0f s, dt = %.2f s\n', Tsim, dt);
fprintf('--------------------------------------------------------------\n');
print_metric_line('横摇 Roll ', largeDeg.roll,  smallDeg.roll);
print_metric_line('纵摇 Pitch', largeDeg.pitch, smallDeg.pitch);
print_metric_line('艏摇 Yaw  ', largeDeg.yaw,   smallDeg.yaw);
fprintf('--------------------------------------------------------------\n');
fprintf('Large ship roll natural period  = %.2f s\n', large.roll.Tn);
fprintf('Small ship roll natural period  = %.2f s\n', small.roll.Tn);
fprintf('Output figure: SeaState4_10000t_vs_100t_3DOF.png\n');
fprintf('==============================================================\n\n');

%% 8. Return complete result struct
results.meta.description = 'Sea State 4 reduced-order 3-DOF ship motion comparison';
results.meta.seed = 20260916;
results.meta.dt = dt;
results.meta.duration = Tsim;
results.meta.rho = rho;
results.meta.g = g;
results.meta.note = ['Representative engineering simulation, not vessel-specific ' ...
    'certification/model-test data.'];

results.environment.t = t;
results.environment.eta = eta;
results.environment.HsRealized = HsRealized;
results.environment.windSea = windSea;
results.environment.swell = swell;
results.environment.tideIndex = envLF.tide;
results.environment.surgeIndex = envLF.surge;
results.environment.driftIndex = envLF.drift;

results.large.ship = large;
results.large.roll_deg = largeDeg.roll;
results.large.pitch_deg = largeDeg.pitch;
results.large.yaw_deg = largeDeg.yaw;
results.large.excitation = qLarge;

results.small.ship = small;
results.small.roll_deg = smallDeg.roll;
results.small.pitch_deg = smallDeg.pitch;
results.small.yaw_deg = smallDeg.yaw;
results.small.excitation = qSmall;

end

%% ========================================================================
function comp = make_jonswap_component(Hs, Tp, gamma, betaDeg, fmin, fmax, Nf, g)
% Create a JONSWAP spectral component and scale exactly to target variance.

f = linspace(fmin, fmax, Nf);
df = f(2) - f(1);
fp = 1 / Tp;

sigma = 0.07 * ones(size(f));
sigma(f > fp) = 0.09;
r = exp(-0.5 * ((f - fp) ./ (sigma * fp)).^2);

S = (g^2 / (2*pi)^4) .* f.^(-5) .* ...
    exp(-1.25 * (fp ./ f).^4) .* gamma.^r;

m0 = trapz(f, S);
targetM0 = (Hs / 4)^2;
S = S * (targetM0 / m0);

A = sqrt(2 * S * df);
phase = 2*pi*rand(size(f));
omega = 2*pi*f;
k = omega.^2 / g;  % deep-water dispersion

comp.Hs = Hs;
comp.Tp = Tp;
comp.gamma = gamma;
comp.betaDeg = betaDeg;
comp.beta = betaDeg*pi/180;
comp.f = f;
comp.df = df;
comp.S = S;
comp.A = A;
comp.phase = phase;
comp.omega = omega;
comp.k = k;
end

%% ========================================================================
function sea = combine_components(a, b)
% Concatenate statistically independent wind-sea and swell components.

sea.A = [a.A, b.A];
sea.phase = [a.phase, b.phase];
sea.omega = [a.omega, b.omega];
sea.k = [a.k, b.k];
sea.beta = [a.beta*ones(size(a.A)), b.beta*ones(size(b.A))];
sea.f = [a.f, b.f];
end

%% ========================================================================
function ship = make_ship(name, displacement_t, L, B, draft, GM, ...
    zetaRoll, Tpitch, zetaPitch, Tyaw, zetaYaw, lfAmpDeg, g)
% Create representative ship inertial/restoring/damping parameters.
%
% displacement_t : metric tonnes
% L, B, draft    : m
% GM             : transverse metacentric height, m
% lfAmpDeg       : [roll pitch yaw] low-frequency equivalent-angle scales

m = displacement_t * 1000;

% Effective rotational inertias including simple added-inertia multipliers.
Iroll  = 1.15 * m * (B^2 + (2*draft)^2) / 12;
Ipitch = 1.10 * m * (L^2 + (2*draft)^2) / 12;
Iyaw   = 1.08 * m * (L^2 + B^2) / 12;

% Roll natural period from radius of gyration and GM.
kx = 0.38 * B;
Troll = 2*pi*kx / sqrt(g*GM);

roll  = make_dof(Iroll,  Troll,  zetaRoll);
pitch = make_dof(Ipitch, Tpitch, zetaPitch);
yaw   = make_dof(Iyaw,   Tyaw,   zetaYaw);

ship.name = name;
ship.displacement_t = displacement_t;
ship.mass_kg = m;
ship.L = L;
ship.B = B;
ship.draft = draft;
ship.GM = GM;
ship.roll = roll;
ship.pitch = pitch;
ship.yaw = yaw;
ship.lowFrequencyAmplitudeDeg = lfAmpDeg;
end

%% ========================================================================
function dof = make_dof(I, Tn, zeta)
% Convert natural period + damping ratio into linear physical coefficients.

wn = 2*pi / Tn;
K = I * wn^2;
C = 2*zeta*I*wn;

dof.I = I;
dof.Tn = Tn;
dof.wn = wn;
dof.zeta = zeta;
dof.K = K;
dof.C = C;
end

%% ========================================================================
function env = make_low_frequency_environment(t, eta, dt)
% Complex low-frequency environment:
%   tide  : several long periods to avoid a single artificial sine wave
%   surge : transient wave packets / long-period surge groups
%   drift : low-pass-filtered eta^2 as a second-order wave-drift surrogate

% Multi-harmonic tide/current-direction variation.
tide = sin(2*pi*t/420 + 0.30) + ...
       0.55*sin(2*pi*t/690 + 1.10) + ...
       0.35*sin(2*pi*t/250 + 2.00);
tide = normalize_std(tide);

% Two transient surge groups plus a weak persistent long-period component.
env1 = exp(-0.5*((t - 300)/70).^2);
env2 = exp(-0.5*((t - 650)/95).^2);
surge = 0.80*env1.*sin(2*pi*t/48 + 0.80) + ...
        0.65*env2.*sin(2*pi*t/36 + 2.20) + ...
        0.20*sin(2*pi*t/95 + 0.50);
surge = normalize_std(surge);

% Second-order drift surrogate: remove DC from eta^2 and retain LF content.
rawDrift = eta.^2 - mean(eta.^2);
tau = 22.0;                         % s low-pass time constant
alpha = dt / (tau + dt);
drift = zeros(size(rawDrift));
for k = 2:numel(rawDrift)
    drift(k) = drift(k-1) + alpha*(rawDrift(k) - drift(k-1));
end
drift = normalize_std(drift);

env.tide = tide;
env.surge = surge;
env.drift = drift;
end

%% ========================================================================
function q = build_equivalent_excitation(ship, sea, t, envLF)
% Convert the directional irregular wave field into equivalent angular
% excitation. Finite-hull sinc filtering is used to represent spatial wave
% averaging over hull length/beam.

beta = sea.beta;
k = sea.k;
A = sea.A;

% Spatial averaging factors. custom_sinc(x) = sin(x)/x.
FL = custom_sinc(0.5 .* k .* ship.L .* cos(beta));
FB = custom_sinc(0.5 .* k .* ship.B .* sin(beta));

% Wave-slope based equivalent equilibrium angles (radians).
rollAmp  = A .* k .* sin(beta) .* FB;
pitchAmp = A .* k .* cos(beta) .* FL;

% Yaw has no ordinary hydrostatic restoring stiffness. Here the yaw input is
% a reduced-order oblique-wave differential-moment proxy; the ship.yaw.K
% coefficient is therefore an EFFECTIVE directional-stability stiffness.
yawAmp = 0.30 .* A .* k .* sin(2*beta) .* FL .* FB;

qWaveRoll  = synthesize_scalar(rollAmp,  sea.omega, sea.phase, t);
qWavePitch = synthesize_scalar(pitchAmp, sea.omega, sea.phase, t);
qWaveYaw   = synthesize_scalar_sine(yawAmp, sea.omega, sea.phase, t);

% Vessel-size-dependent low-frequency equivalent moment amplitudes.
lf = ship.lowFrequencyAmplitudeDeg * pi/180;

q.roll = qWaveRoll + lf(1) .* ...
    (0.35*envLF.tide + 0.55*envLF.surge + 0.10*envLF.drift);
q.pitch = qWavePitch + lf(2) .* ...
    (0.10*envLF.tide + 0.75*envLF.surge + 0.15*envLF.drift);
q.yaw = qWaveYaw + lf(3) .* ...
    (0.65*envLF.tide + 0.15*envLF.surge + 0.20*envLF.drift);
end

%% ========================================================================
function x = simulate_rotational_dof(qEq, dof, dt)
% Solve I*xdd + C*xd + K*x = K*qEq(t)
% using exact zero-order-hold discretization of the linear state equation.

I = dof.I;
C = dof.C;
K = dof.K;

Ac = [0, 1; -K/I, -C/I];
Bc = [0; 1/I];
Ad = expm(Ac*dt);
Bd = Ac \ ((Ad - eye(2))*Bc);

moment = K .* qEq;
state = zeros(2, numel(qEq));
for k = 1:(numel(qEq)-1)
    state(:,k+1) = Ad*state(:,k) + Bd*moment(k);
end

x = state(1,:);
end

%% ========================================================================
function y = synthesize_scalar(amplitude, omega, phase, t)
% Spectral synthesis using cosine components.

arg = omega(:)*t + phase(:);
y = sum(amplitude(:) .* cos(arg), 1);
end

%% ========================================================================
function y = synthesize_scalar_sine(amplitude, omega, phase, t)
% Spectral synthesis using sine components (quadrature for yaw proxy).

arg = omega(:)*t + phase(:);
y = sum(amplitude(:) .* sin(arg), 1);
end

%% ========================================================================
function y = custom_sinc(x)
% Toolbox-free sin(x)/x.

y = ones(size(x));
idx = abs(x) > 1e-12;
y(idx) = sin(x(idx)) ./ x(idx);
end

%% ========================================================================
function y = normalize_std(x)
% Zero-mean, unit-standard-deviation normalization.

y = x - mean(x);
s = std(y);
if s > eps
    y = y / s;
end
end

%% ========================================================================
function print_metric_line(labelText, largeDeg, smallDeg)
% Print RMS and absolute peak values for rapid engineering comparison.

rmsLarge = sqrt(mean(largeDeg.^2));
rmsSmall = sqrt(mean(smallDeg.^2));
peakLarge = max(abs(largeDeg));
peakSmall = max(abs(smallDeg));

fprintf('%s | RMS: large %6.3f deg, small %6.3f deg | ', ...
    labelText, rmsLarge, rmsSmall);
fprintf('peak: large %6.3f deg, small %6.3f deg\n', peakLarge, peakSmall);
end
