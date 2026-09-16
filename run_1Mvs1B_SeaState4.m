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
% The model contains ONE public entry function. All helper functions are
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
% Every run generates EXACTLY TWO PNG figures:
%   1) SeaState4_10000t_vs_100t_Academic.png
%      Academic/technical view: local raw histories + full-duration moving RMS
%      + RMS / peak / 95% absolute-response metrics.
%   2) SeaState4_10000t_vs_100t_Brief.png
%      Presentation view: smooth full-duration moving-RMS curves with direct
%      large-vs-small response ratios for rapid visual comparison.
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

%% 6. Calculate summary metrics once and generate TWO figures
metrics = calculate_metrics(largeDeg, smallDeg);

academicFile = 'SeaState4_10000t_vs_100t_Academic.png';
briefFile = 'SeaState4_10000t_vs_100t_Brief.png';

plot_academic_figure(t, largeDeg, smallDeg, large, small, ...
    metrics, HsRealized, Tsim, dt, academicFile);
plot_brief_figure(t, largeDeg, smallDeg, large, small, ...
    metrics, HsRealized, Tsim, dt, briefFile);

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
fprintf('Figure 1 (academic): %s\n', academicFile);
fprintf('Figure 2 (brief):    %s\n', briefFile);
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
results.meta.outputFiles.academic = academicFile;
results.meta.outputFiles.brief = briefFile;

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

results.metrics = metrics;

end

%% ========================================================================
function plot_academic_figure(t, largeDeg, smallDeg, large, small, ...
    metrics, HsRealized, Tsim, dt, outputFile)
% Technical view:
% Left column  = 0-180 s raw time histories for actual motion detail.
% Right column = 30 s moving RMS across the complete 900 s simulation.
% Each raw panel reports global RMS, absolute peak and 95% |response|.

largeColor = [0.10 0.32 0.68];
smallColor = [0.90 0.32 0.08];
zoomEnd = min(180, Tsim);
idx = t <= zoomEnd;
winSec = 30;
nwin = max(5, round(winSec/dt));

rollRmsLarge  = moving_rms(largeDeg.roll,  nwin);
rollRmsSmall  = moving_rms(smallDeg.roll,  nwin);
pitchRmsLarge = moving_rms(largeDeg.pitch, nwin);
pitchRmsSmall = moving_rms(smallDeg.pitch, nwin);
yawRmsLarge   = moving_rms(largeDeg.yaw,   nwin);
yawRmsSmall   = moving_rms(smallDeg.yaw,   nwin);

fig = figure('Color', 'w', 'Position', [60 35 1500 930]);

% ---------------- Roll ----------------
subplot(3,2,1);
plot(t(idx), largeDeg.roll(idx), 'LineWidth', 1.35, 'Color', largeColor); hold on;
plot(t(idx), smallDeg.roll(idx), 'LineWidth', 1.00, 'Color', smallColor);
grid on; box on;
xlim([0 zoomEnd]);
ylim(expand_ylim([largeDeg.roll(idx), smallDeg.roll(idx)], 0.08));
ylabel('横摇 / deg');
title('横摇 Roll：局部原始时程');
legend(large.name, small.name, 'Location', 'best');
text(0.985, 0.04, metric_text(metrics.roll), 'Units', 'normalized', ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom', ...
    'BackgroundColor', 'w', 'EdgeColor', [0.75 0.75 0.75], 'Margin', 4, ...
    'FontSize', 8.5);

subplot(3,2,2);
plot(t, rollRmsLarge, 'LineWidth', 1.9, 'Color', largeColor); hold on;
plot(t, rollRmsSmall, 'LineWidth', 1.9, 'Color', smallColor);
grid on; box on;
xlim([0 Tsim]);
ylim([0, positive_upper([rollRmsLarge, rollRmsSmall])]);
ylabel('滑动 RMS / deg');
title(sprintf('横摇 Roll：全时程滑动 RMS（%d s窗口）', winSec));

% ---------------- Pitch ----------------
subplot(3,2,3);
plot(t(idx), largeDeg.pitch(idx), 'LineWidth', 1.35, 'Color', largeColor); hold on;
plot(t(idx), smallDeg.pitch(idx), 'LineWidth', 1.00, 'Color', smallColor);
grid on; box on;
xlim([0 zoomEnd]);
ylim(expand_ylim([largeDeg.pitch(idx), smallDeg.pitch(idx)], 0.08));
ylabel('纵摇 / deg');
title('纵摇 Pitch：局部原始时程');
text(0.985, 0.04, metric_text(metrics.pitch), 'Units', 'normalized', ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom', ...
    'BackgroundColor', 'w', 'EdgeColor', [0.75 0.75 0.75], 'Margin', 4, ...
    'FontSize', 8.5);

subplot(3,2,4);
plot(t, pitchRmsLarge, 'LineWidth', 1.9, 'Color', largeColor); hold on;
plot(t, pitchRmsSmall, 'LineWidth', 1.9, 'Color', smallColor);
grid on; box on;
xlim([0 Tsim]);
ylim([0, positive_upper([pitchRmsLarge, pitchRmsSmall])]);
ylabel('滑动 RMS / deg');
title(sprintf('纵摇 Pitch：全时程滑动 RMS（%d s窗口）', winSec));

% ---------------- Yaw ----------------
subplot(3,2,5);
plot(t(idx), largeDeg.yaw(idx), 'LineWidth', 1.35, 'Color', largeColor); hold on;
plot(t(idx), smallDeg.yaw(idx), 'LineWidth', 1.00, 'Color', smallColor);
grid on; box on;
xlim([0 zoomEnd]);
ylim(expand_ylim([largeDeg.yaw(idx), smallDeg.yaw(idx)], 0.08));
ylabel('艏摇 / deg');
xlabel('时间 / s');
title('艏摇 / 偏航 Yaw：局部原始时程');
text(0.985, 0.04, metric_text(metrics.yaw), 'Units', 'normalized', ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom', ...
    'BackgroundColor', 'w', 'EdgeColor', [0.75 0.75 0.75], 'Margin', 4, ...
    'FontSize', 8.5);

subplot(3,2,6);
plot(t, yawRmsLarge, 'LineWidth', 1.9, 'Color', largeColor); hold on;
plot(t, yawRmsSmall, 'LineWidth', 1.9, 'Color', smallColor);
grid on; box on;
xlim([0 Tsim]);
ylim([0, positive_upper([yawRmsLarge, yawRmsSmall])]);
ylabel('滑动 RMS / deg');
xlabel('时间 / s');
title(sprintf('艏摇 / 偏航 Yaw：全时程滑动 RMS（%d s窗口）', winSec));

sgtitle(sprintf(['四级海况 3-DOF 技术对比：一万吨级船舶 vs 一百吨级船舶\n' ...
    '左列为局部原始时程，右列为全时程滑动RMS；H_s = %.2f m'], ...
    HsRealized), 'FontWeight', 'bold');

set(fig, 'PaperPositionMode', 'auto');
print(fig, outputFile, '-dpng', '-r220');
end

%% ========================================================================
function plot_brief_figure(t, largeDeg, smallDeg, large, small, ...
    metrics, HsRealized, Tsim, dt, outputFile)
% Presentation view:
% Only smooth full-duration motion-intensity curves are shown. This avoids
% the raw high-frequency trace becoming visually dense over 900 seconds.
% A 45 s moving RMS is used to emphasize the engineering conclusion.

largeColor = [0.10 0.32 0.68];
smallColor = [0.90 0.32 0.08];
winSec = 45;
nwin = max(5, round(winSec/dt));

rollLarge  = moving_rms(largeDeg.roll, nwin);
rollSmall  = moving_rms(smallDeg.roll, nwin);
pitchLarge = moving_rms(largeDeg.pitch, nwin);
pitchSmall = moving_rms(smallDeg.pitch, nwin);
yawLarge   = moving_rms(largeDeg.yaw, nwin);
yawSmall   = moving_rms(smallDeg.yaw, nwin);

fig = figure('Color', 'w', 'Position', [80 45 1420 900]);

subplot(3,1,1);
plot(t, rollLarge, 'LineWidth', 2.15, 'Color', largeColor); hold on;
plot(t, rollSmall, 'LineWidth', 2.15, 'Color', smallColor);
grid on; box on;
xlim([0 Tsim]);
ylim([0, positive_upper([rollLarge, rollSmall])]);
ylabel('横摇 RMS / deg');
title('横摇 Roll');
legend(large.name, small.name, 'Location', 'best');
text(0.985, 0.92, ratio_text(metrics.roll), 'Units', 'normalized', ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
    'FontWeight', 'bold', 'FontSize', 11, 'BackgroundColor', 'w', ...
    'EdgeColor', [0.70 0.70 0.70], 'Margin', 5);

subplot(3,1,2);
plot(t, pitchLarge, 'LineWidth', 2.15, 'Color', largeColor); hold on;
plot(t, pitchSmall, 'LineWidth', 2.15, 'Color', smallColor);
grid on; box on;
xlim([0 Tsim]);
ylim([0, positive_upper([pitchLarge, pitchSmall])]);
ylabel('纵摇 RMS / deg');
title('纵摇 Pitch');
text(0.985, 0.92, ratio_text(metrics.pitch), 'Units', 'normalized', ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
    'FontWeight', 'bold', 'FontSize', 11, 'BackgroundColor', 'w', ...
    'EdgeColor', [0.70 0.70 0.70], 'Margin', 5);

subplot(3,1,3);
plot(t, yawLarge, 'LineWidth', 2.15, 'Color', largeColor); hold on;
plot(t, yawSmall, 'LineWidth', 2.15, 'Color', smallColor);
grid on; box on;
xlim([0 Tsim]);
ylim([0, positive_upper([yawLarge, yawSmall])]);
ylabel('艏摇 RMS / deg');
xlabel('时间 / s');
title('艏摇 / 偏航 Yaw');
text(0.985, 0.92, ratio_text(metrics.yaw), 'Units', 'normalized', ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
    'FontWeight', 'bold', 'FontSize', 11, 'BackgroundColor', 'w', ...
    'EdgeColor', [0.70 0.70 0.70], 'Margin', 5);

sgtitle(sprintf(['四级海况下船舶运动强度对比（%d s滑动RMS）\n' ...
    '一万吨级船舶 vs 一百吨级船舶；H_s = %.2f m'], ...
    winSec, HsRealized), 'FontWeight', 'bold');

set(fig, 'PaperPositionMode', 'auto');
print(fig, outputFile, '-dpng', '-r220');
end

%% ========================================================================
function metrics = calculate_metrics(largeDeg, smallDeg)
% Global comparison metrics for each DOF.
metrics.roll  = metric_pair(largeDeg.roll,  smallDeg.roll);
metrics.pitch = metric_pair(largeDeg.pitch, smallDeg.pitch);
metrics.yaw   = metric_pair(largeDeg.yaw,   smallDeg.yaw);
end

%% ========================================================================
function out = metric_pair(largeSignal, smallSignal)
out.largeRms = sqrt(mean(largeSignal.^2));
out.smallRms = sqrt(mean(smallSignal.^2));
out.largePeak = max(abs(largeSignal));
out.smallPeak = max(abs(smallSignal));
out.largeP95 = percentile_abs(largeSignal, 0.95);
out.smallP95 = percentile_abs(smallSignal, 0.95);
out.rmsRatio = out.smallRms / max(out.largeRms, eps);
out.peakRatio = out.smallPeak / max(out.largePeak, eps);
end

%% ========================================================================
function txt = metric_text(m)
txt = sprintf(['全时程统计\n' ...
    '1万吨：RMS %.2f°, Peak %.2f°, 95%%|响应| %.2f°\n' ...
    '100吨：RMS %.2f°, Peak %.2f°, 95%%|响应| %.2f°'], ...
    m.largeRms, m.largePeak, m.largeP95, ...
    m.smallRms, m.smallPeak, m.smallP95);
end

%% ========================================================================
function txt = ratio_text(m)
txt = sprintf('100吨 / 1万吨：RMS %.1f×   峰值 %.1f×', ...
    m.rmsRatio, m.peakRatio);
end

%% ========================================================================
function y = moving_rms(x, nwin)
% Toolbox-free moving RMS using convolution. Output length equals input.
window = ones(1, nwin) / nwin;
y = sqrt(conv(x.^2, window, 'same'));
end

%% ========================================================================
function p = percentile_abs(x, probability)
% Toolbox-free percentile of absolute response.
v = sort(abs(x(:)));
idx = max(1, min(numel(v), ceil(probability*numel(v))));
p = v(idx);
end

%% ========================================================================
function yl = expand_ylim(x, padRatio)
xmin = min(x);
xmax = max(x);
span = xmax - xmin;
if span < 1e-9
    span = max(1, abs(xmax));
end
pad = padRatio * span;
yl = [xmin - pad, xmax + pad];
end

%% ========================================================================
function ymax = positive_upper(x)
ymax = max(x);
if ymax < 1e-9
    ymax = 1;
else
    ymax = 1.08*ymax;
end
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