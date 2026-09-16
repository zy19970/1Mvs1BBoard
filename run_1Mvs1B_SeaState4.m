function results = run_1Mvs1B_SeaState4()
%RUN_1MVS1B_SEASTATE4 Compare 10,000-t and 100-t ship motions in Sea State 4.
%
% Usage:
%   results = run_1Mvs1B_SeaState4();
%
% One public entry function. All helper functions are local to this file.
%
% The environmental forcing is intentionally NONSTATIONARY and IRREGULAR.
% It avoids an unrealistically neat, almost periodic response by using:
%   - four partially overlapping JONSWAP sea partitions (multi-modal sea)
%   - stratified-random frequency bins rather than perfectly even bins
%   - random directional spreading for every spectral component
%   - independent slowly varying wave-energy envelopes / wave groups
%   - realistic slow tidal-current variation over a 15 min record
%   - random transient long-wave/surge packets
%   - low-frequency second-order drift from eta^2
%
% Sea-state target:
%   nominal combined Hs ~= 1.85 m, within Sea State 4 (moderate sea).
%
% Simulated rotational DOFs:
%   1) roll  (横摇)
%   2) pitch (纵摇)
%   3) yaw   (艏摇/偏航)
%
% Vessel model:
%   I*qdd + C*qd + K*q = M_env(t)
%
% The large and small vessels use representative dimensions, not data from a
% named ship. Replace dimensions / GM / damping / RAO data if vessel-specific
% predictions are required.
%
% Every run generates exactly TWO PNG figures:
%   1) SeaState4_10000t_vs_100t_Academic.png
%   2) SeaState4_10000t_vs_100t_Brief.png
%
% Reproducible random seed: 20260916

clc;
close all;
rng(20260916, 'twister');

%% 1. Simulation settings
g = 9.81;
rho = 1025;
dt = 0.10;
Tsim = 900;
t = 0:dt:Tsim;

%% 2. Nonstationary multi-modal Sea State 4
% Instead of one narrow wind-sea peak plus one fixed swell peak, the sea is
% represented by several overlapping partitions. Their energies combine in
% quadrature; each partition has independent phase, frequency jitter,
% directional spreading and slow wave-group modulation.
parts = cell(1,4);
parts{1} = make_jonswap_partition(1.25,  6.2, 2.0,  20, 32, ...
    0.035, 0.72, 260,  70, 0.30, g, 'wind-sea A');
parts{2} = make_jonswap_partition(1.05,  8.7, 2.4,  50, 26, ...
    0.025, 0.55, 240, 105, 0.26, g, 'wind-sea B');
parts{3} = make_jonswap_partition(0.75, 13.5, 3.0, -25, 13, ...
    0.015, 0.32, 180, 170, 0.20, g, 'swell A');
parts{4} = make_jonswap_partition(0.45, 17.5, 2.2,  -5,  9, ...
    0.012, 0.24, 140, 230, 0.16, g, 'swell B');

sea = combine_partitions(parts, t, dt);
eta = synthesize_nonstationary(sea.A, sea, t, false);
HsRealized = 4.0 * std(eta);
HsNominal = sqrt(sum(cellfun(@(p) p.Hs^2, parts)));

%% 3. Representative vessels
large = make_ship('一万吨级船舶', 10000, 120.0, 20.0, 6.5, 0.95, ...
    0.12, 8.2, 0.18, 48.0, 0.30, [0.35 0.22 1.30], g);
small = make_ship('一百吨级船舶',   100,  25.0,  6.0, 1.8, 0.75, ...
    0.11, 3.9, 0.16, 18.0, 0.24, [1.40 0.80 5.00], g);

%% 4. Low-frequency tide / surge / second-order drift
envLF = make_low_frequency_environment(t, eta, dt);

%% 5. Equivalent environmental excitation
qLarge = build_equivalent_excitation(large, sea, t, envLF);
qSmall = build_equivalent_excitation(small, sea, t, envLF);

%% 6. Solve 3-DOF reduced-order equations
motionLarge.roll  = simulate_rotational_dof(qLarge.roll,  large.roll,  dt);
motionLarge.pitch = simulate_rotational_dof(qLarge.pitch, large.pitch, dt);
motionLarge.yaw   = simulate_rotational_dof(qLarge.yaw,   large.yaw,   dt);

motionSmall.roll  = simulate_rotational_dof(qSmall.roll,  small.roll,  dt);
motionSmall.pitch = simulate_rotational_dof(qSmall.pitch, small.pitch, dt);
motionSmall.yaw   = simulate_rotational_dof(qSmall.yaw,   small.yaw,   dt);

rad2deg_ = 180/pi;
largeDeg.roll  = motionLarge.roll  * rad2deg_;
largeDeg.pitch = motionLarge.pitch * rad2deg_;
largeDeg.yaw   = motionLarge.yaw   * rad2deg_;
smallDeg.roll  = motionSmall.roll  * rad2deg_;
smallDeg.pitch = motionSmall.pitch * rad2deg_;
smallDeg.yaw   = motionSmall.yaw   * rad2deg_;

%% 7. Metrics and TWO output figures
metrics = calculate_metrics(largeDeg, smallDeg);

academicFile = 'SeaState4_10000t_vs_100t_Academic.png';
briefFile = 'SeaState4_10000t_vs_100t_Brief.png';

plot_academic_figure(t, largeDeg, smallDeg, large, small, ...
    metrics, HsRealized, Tsim, dt, academicFile);
plot_brief_figure(t, largeDeg, smallDeg, large, small, ...
    metrics, HsRealized, Tsim, dt, briefFile);

%% 8. Console summary
fprintf('\n==============================================================\n');
fprintf('Sea State 4 nonstationary / directional simulation\n');
fprintf('Nominal combined Hs = %.3f m\n', HsNominal);
fprintf('Realized record Hs  = %.3f m\n', HsRealized);
fprintf('Simulation duration = %.0f s, dt = %.2f s\n', Tsim, dt);
fprintf('Sea partitions:\n');
for ip = 1:numel(parts)
    p = parts{ip};
    fprintf('  %-11s Hs %.2f m, Tp %.1f s, mean dir %+5.1f deg, spread %.1f deg\n', ...
        p.name, p.Hs, p.Tp, p.betaMeanDeg, p.betaSpreadDeg);
end
fprintf('--------------------------------------------------------------\n');
print_metric_line('横摇 Roll ', largeDeg.roll,  smallDeg.roll);
print_metric_line('纵摇 Pitch', largeDeg.pitch, smallDeg.pitch);
print_metric_line('艏摇 Yaw  ', largeDeg.yaw,   smallDeg.yaw);
fprintf('--------------------------------------------------------------\n');
fprintf('Figure 1 (academic): %s\n', academicFile);
fprintf('Figure 2 (brief):    %s\n', briefFile);
fprintf('==============================================================\n\n');

%% 9. Return result struct
results.meta.description = 'Nonstationary directional Sea State 4 reduced-order 3-DOF comparison';
results.meta.seed = 20260916;
results.meta.dt = dt;
results.meta.duration = Tsim;
results.meta.rho = rho;
results.meta.g = g;
results.meta.outputFiles.academic = academicFile;
results.meta.outputFiles.brief = briefFile;
results.meta.note = ['Representative engineering simulation; not vessel-specific ' ...
    'model-test, CFD or measured RAO data.'];

results.environment.t = t;
results.environment.eta = eta;
results.environment.HsNominal = HsNominal;
results.environment.HsRealized = HsRealized;
results.environment.partitions = parts;
results.environment.energyEnvelope = sea.energyEnvelope;
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
function p = make_jonswap_partition(Hs, Tp, gamma, betaMeanDeg, ...
    betaSpreadDeg, fmin, fmax, Nf, envTau, envDepth, g, name)
% One directional JONSWAP partition.
% Frequency samples are randomized inside narrow strata so the synthesized
% record does not inherit a perfectly repeating FFT-like frequency grid.

edges = linspace(fmin, fmax, Nf+1);
df = diff(edges);
f = edges(1:end-1) + rand(1,Nf).*df;
fp = 1/Tp;

sigma = 0.07*ones(size(f));
sigma(f > fp) = 0.09;
r = exp(-0.5*((f-fp)./(sigma*fp)).^2);
S = (g^2/(2*pi)^4).*f.^(-5).*exp(-1.25*(fp./f).^4).*gamma.^r;

m0 = sum(S.*df);
targetM0 = (Hs/4)^2;
S = S*(targetM0/m0);
A = sqrt(2*S.*df);

% Every spectral component gets an independent incident direction.
betaDeg = betaMeanDeg + betaSpreadDeg*randn(1,Nf);
maxSpread = 2.4*betaSpreadDeg;
betaDeg = min(betaMeanDeg + maxSpread, max(betaMeanDeg - maxSpread, betaDeg));
beta = betaDeg*pi/180;

p.name = name;
p.Hs = Hs;
p.Tp = Tp;
p.gamma = gamma;
p.betaMeanDeg = betaMeanDeg;
p.betaSpreadDeg = betaSpreadDeg;
p.f = f;
p.df = df;
p.S = S;
p.A = A;
p.phase = 2*pi*rand(1,Nf);
p.omega = 2*pi*f;
p.k = p.omega.^2/g;
p.beta = beta;
p.envTau = envTau;
p.envDepth = envDepth;
end

%% ========================================================================
function sea = combine_partitions(parts, t, dt)
% Concatenate all spectral components and create one independent slowly
% varying energy envelope for each partition.

nP = numel(parts);
sea.A = [];
sea.phase = [];
sea.omega = [];
sea.k = [];
sea.beta = [];
sea.group = [];
sea.energyEnvelope = zeros(nP, numel(t));

for ip = 1:nP
    p = parts{ip};
    n = numel(p.A);
    sea.A = [sea.A, p.A]; %#ok<AGROW>
    sea.phase = [sea.phase, p.phase]; %#ok<AGROW>
    sea.omega = [sea.omega, p.omega]; %#ok<AGROW>
    sea.k = [sea.k, p.k]; %#ok<AGROW>
    sea.beta = [sea.beta, p.beta]; %#ok<AGROW>
    sea.group = [sea.group, ip*ones(1,n)]; %#ok<AGROW>
    sea.energyEnvelope(ip,:) = make_energy_envelope(t, dt, p.envTau, p.envDepth);
end
sea.nGroups = nP;
end

%% ========================================================================
function env = make_energy_envelope(t, dt, tau, depth)
% Positive, slowly varying stochastic wave-energy envelope.
% It represents wave groups and evolving local sea-state intensity.

z1 = colored_noise(numel(t), dt, tau);
z2 = colored_noise(numel(t), dt, 0.45*tau);
raw = 0.72*z1 + 0.28*z2;

% Add several irregular local wave groups. Centers, widths and signs are
% random, avoiding a manually imposed periodic beat pattern.
nGroups = 5 + randi(3);
for k = 1:nGroups
    c = t(1) + rand*(t(end)-t(1));
    w = 25 + 80*rand;
    a = 0.35*randn;
    raw = raw + a*exp(-0.5*((t-c)/w).^2);
end

raw = normalize_std(raw);
env = exp(depth*raw);
env = min(1.65, max(0.50, env));
% Preserve approximate overall spectral variance after modulation.
env = env / sqrt(mean(env.^2));
end

%% ========================================================================
function seaSignal = synthesize_nonstationary(amplitude, sea, t, useSine)
% Chunked directional spectral synthesis with partition-specific envelopes.
% Chunking avoids allocating a huge [Nfrequency x Ntime] matrix.

seaSignal = zeros(size(t));
chunk = 700;
amplitude = amplitude(:).';

for ip = 1:sea.nGroups
    idx = find(sea.group == ip);
    a = amplitude(idx).';
    om = sea.omega(idx).';
    ph = sea.phase(idx).';

    for i0 = 1:chunk:numel(t)
        i1 = min(numel(t), i0+chunk-1);
        tt = t(i0:i1);
        arg = bsxfun(@plus, om*tt, ph);
        if useSine
            carrier = sum(bsxfun(@times, a, sin(arg)), 1);
        else
            carrier = sum(bsxfun(@times, a, cos(arg)), 1);
        end
        seaSignal(i0:i1) = seaSignal(i0:i1) + ...
            sea.energyEnvelope(ip,i0:i1).*carrier;
    end
end
end

%% ========================================================================
function ship = make_ship(name, displacement_t, L, B, draft, GM, ...
    zetaRoll, Tpitch, zetaPitch, Tyaw, zetaYaw, lfAmpDeg, g)

m = displacement_t*1000;
Iroll  = 1.15*m*(B^2 + (2*draft)^2)/12;
Ipitch = 1.10*m*(L^2 + (2*draft)^2)/12;
Iyaw   = 1.08*m*(L^2 + B^2)/12;

kx = 0.38*B;
Troll = 2*pi*kx/sqrt(g*GM);

ship.name = name;
ship.displacement_t = displacement_t;
ship.mass_kg = m;
ship.L = L;
ship.B = B;
ship.draft = draft;
ship.GM = GM;
ship.roll = make_dof(Iroll, Troll, zetaRoll);
ship.pitch = make_dof(Ipitch, Tpitch, zetaPitch);
ship.yaw = make_dof(Iyaw, Tyaw, zetaYaw);
ship.lowFrequencyAmplitudeDeg = lfAmpDeg;
end

%% ========================================================================
function dof = make_dof(I, Tn, zeta)
wn = 2*pi/Tn;
dof.I = I;
dof.Tn = Tn;
dof.wn = wn;
dof.zeta = zeta;
dof.K = I*wn^2;
dof.C = 2*zeta*I*wn;
end

%% ========================================================================
function env = make_low_frequency_environment(t, eta, dt)
% Low-frequency environment with physically separated time scales.
%
% IMPORTANT: astronomical tide itself has hour-scale periods. Over a 15 min
% record it should NOT oscillate every few minutes. Therefore the tide term
% below is a slow astronomical trend plus stochastic tidal-current meander.

T_M2 = 12.42*3600;
T_S2 = 12.00*3600;
T_K1 = 23.93*3600;
astro = sin(2*pi*t/T_M2 + 0.30) + ...
        0.42*sin(2*pi*t/T_S2 + 1.10) + ...
        0.24*sin(2*pi*t/T_K1 + 2.00);
astro = astro - mean(astro);
if std(astro) > eps
    astro = astro/std(astro);
end
currentMeander = 0.72*colored_noise(numel(t), dt, 210) + ...
                 0.28*colored_noise(numel(t), dt, 85);
tide = normalize_std(0.30*astro + 0.70*currentMeander);

% Random long-wave / surge packets: irregular center time, duration, period,
% amplitude and phase. No fixed 36/48/95 s comb is imposed.
surge = zeros(size(t));
nPackets = 8;
for k = 1:nPackets
    c = 50 + rand*(t(end)-100);
    width = 35 + 90*rand;
    period = 24 + 58*rand;
    amp = 0.55 + 0.75*rand;
    phase = 2*pi*rand;
    packet = exp(-0.5*((t-c)/width).^2) .* ...
        sin(2*pi*(t-c)/period + phase);
    surge = surge + amp*packet;
end
surge = surge + 0.30*colored_noise(numel(t), dt, 32);
surge = normalize_std(surge);

% Second-order drift surrogate from eta^2 plus a weak independent LF part.
rawDrift = eta.^2 - mean(eta.^2);
tau = 28.0;
alpha = dt/(tau+dt);
drift = zeros(size(rawDrift));
for k = 2:numel(rawDrift)
    drift(k) = drift(k-1) + alpha*(rawDrift(k)-drift(k-1));
end
drift = normalize_std(0.82*normalize_std(drift) + ...
    0.18*colored_noise(numel(t), dt, 95));

env.tide = tide;
env.surge = surge;
env.drift = drift;
end

%% ========================================================================
function q = build_equivalent_excitation(ship, sea, t, envLF)
% Directional wave-slope excitation with finite-hull spatial averaging.

beta = sea.beta;
k = sea.k;
A = sea.A;

FL = custom_sinc(0.5.*k.*ship.L.*cos(beta));
FB = custom_sinc(0.5.*k.*ship.B.*sin(beta));

rollAmp  = A.*k.*sin(beta).*FB;
pitchAmp = A.*k.*cos(beta).*FL;
yawAmp   = 0.30.*A.*k.*sin(2*beta).*FL.*FB;

qWaveRoll  = synthesize_nonstationary(rollAmp,  sea, t, false);
qWavePitch = synthesize_nonstationary(pitchAmp, sea, t, false);
qWaveYaw   = synthesize_nonstationary(yawAmp,   sea, t, true);

lf = ship.lowFrequencyAmplitudeDeg*pi/180;
q.roll = qWaveRoll + lf(1).*( ...
    0.22*envLF.tide + 0.58*envLF.surge + 0.20*envLF.drift);
q.pitch = qWavePitch + lf(2).*( ...
    0.08*envLF.tide + 0.70*envLF.surge + 0.22*envLF.drift);
q.yaw = qWaveYaw + lf(3).*( ...
    0.58*envLF.tide + 0.17*envLF.surge + 0.25*envLF.drift);
end

%% ========================================================================
function x = simulate_rotational_dof(qEq, dof, dt)
% Solve I*xdd + C*xd + K*x = K*qEq(t) with exact ZOH discretization.

I = dof.I;
C = dof.C;
K = dof.K;
Ac = [0, 1; -K/I, -C/I];
Bc = [0; 1/I];
Ad = expm(Ac*dt);
Bd = Ac\((Ad-eye(2))*Bc);

moment = K.*qEq;
state = zeros(2,numel(qEq));
for k = 1:numel(qEq)-1
    state(:,k+1) = Ad*state(:,k) + Bd*moment(k);
end
x = state(1,:);
end

%% ========================================================================
function x = colored_noise(n, dt, tau)
% Unit-standard-deviation Ornstein-Uhlenbeck-like colored noise.
a = exp(-dt/max(tau,dt));
b = sqrt(max(0,1-a^2));
x = zeros(1,n);
for k = 2:n
    x(k) = a*x(k-1) + b*randn;
end
x = normalize_std(x);
end

%% ========================================================================
function metrics = calculate_metrics(largeDeg, smallDeg)
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
out.largeP95 = percentile_abs(largeSignal,0.95);
out.smallP95 = percentile_abs(smallSignal,0.95);
out.rmsRatio = out.smallRms/max(out.largeRms,eps);
out.peakRatio = out.smallPeak/max(out.largePeak,eps);
end

%% ========================================================================
function plot_academic_figure(t, largeDeg, smallDeg, large, small, ...
    metrics, HsRealized, Tsim, dt, outputFile)
% Technical view: local raw time histories + full-duration moving RMS.

largeColor = [0.10 0.32 0.68];
smallColor = [0.90 0.32 0.08];
zoomEnd = min(180,Tsim);
idx = t <= zoomEnd;
winSec = 30;
nwin = max(5,round(winSec/dt));

rL = moving_rms(largeDeg.roll,nwin);   rS = moving_rms(smallDeg.roll,nwin);
pL = moving_rms(largeDeg.pitch,nwin);  pS = moving_rms(smallDeg.pitch,nwin);
yL = moving_rms(largeDeg.yaw,nwin);    yS = moving_rms(smallDeg.yaw,nwin);

fig = figure('Color','w','Position',[60 35 1500 930]);

subplot(3,2,1);
plot(t(idx),largeDeg.roll(idx),'LineWidth',1.35,'Color',largeColor); hold on;
plot(t(idx),smallDeg.roll(idx),'LineWidth',1.00,'Color',smallColor);
grid on; box on; xlim([0 zoomEnd]);
ylim(expand_ylim([largeDeg.roll(idx),smallDeg.roll(idx)],0.08));
ylabel('横摇 / deg'); title('横摇 Roll：局部原始时程');
legend(large.name,small.name,'Location','best');
add_metric_box(metrics.roll);

subplot(3,2,2);
plot(t,rL,'LineWidth',1.9,'Color',largeColor); hold on;
plot(t,rS,'LineWidth',1.9,'Color',smallColor);
grid on; box on; xlim([0 Tsim]); ylim([0 positive_upper([rL,rS])]);
ylabel('滑动 RMS / deg'); title(sprintf('横摇 Roll：全时程滑动 RMS（%d s窗口）',winSec));

subplot(3,2,3);
plot(t(idx),largeDeg.pitch(idx),'LineWidth',1.35,'Color',largeColor); hold on;
plot(t(idx),smallDeg.pitch(idx),'LineWidth',1.00,'Color',smallColor);
grid on; box on; xlim([0 zoomEnd]);
ylim(expand_ylim([largeDeg.pitch(idx),smallDeg.pitch(idx)],0.08));
ylabel('纵摇 / deg'); title('纵摇 Pitch：局部原始时程'); add_metric_box(metrics.pitch);

subplot(3,2,4);
plot(t,pL,'LineWidth',1.9,'Color',largeColor); hold on;
plot(t,pS,'LineWidth',1.9,'Color',smallColor);
grid on; box on; xlim([0 Tsim]); ylim([0 positive_upper([pL,pS])]);
ylabel('滑动 RMS / deg'); title(sprintf('纵摇 Pitch：全时程滑动 RMS（%d s窗口）',winSec));

subplot(3,2,5);
plot(t(idx),largeDeg.yaw(idx),'LineWidth',1.35,'Color',largeColor); hold on;
plot(t(idx),smallDeg.yaw(idx),'LineWidth',1.00,'Color',smallColor);
grid on; box on; xlim([0 zoomEnd]);
ylim(expand_ylim([largeDeg.yaw(idx),smallDeg.yaw(idx)],0.08));
ylabel('艏摇 / deg'); xlabel('时间 / s');
title('艏摇 / 偏航 Yaw：局部原始时程'); add_metric_box(metrics.yaw);

subplot(3,2,6);
plot(t,yL,'LineWidth',1.9,'Color',largeColor); hold on;
plot(t,yS,'LineWidth',1.9,'Color',smallColor);
grid on; box on; xlim([0 Tsim]); ylim([0 positive_upper([yL,yS])]);
ylabel('滑动 RMS / deg'); xlabel('时间 / s');
title(sprintf('艏摇 / 偏航 Yaw：全时程滑动 RMS（%d s窗口）',winSec));

sgtitle(sprintf(['四级海况 3-DOF 技术对比：一万吨级船舶 vs 一百吨级船舶\n' ...
    '多模态方向扩散随机海浪 + 非平稳波群 + 潮流缓变 + 随机浪涌 + 二阶漂移；H_s = %.2f m'], ...
    HsRealized),'FontWeight','bold');
set(fig,'PaperPositionMode','auto');
print(fig,outputFile,'-dpng','-r220');
end

%% ========================================================================
function plot_brief_figure(t, largeDeg, smallDeg, large, small, ...
    metrics, HsRealized, Tsim, dt, outputFile)
% Presentation view: smooth full-duration motion-intensity curves.

largeColor = [0.10 0.32 0.68];
smallColor = [0.90 0.32 0.08];
winSec = 45;
nwin = max(5,round(winSec/dt));

rL = moving_rms(largeDeg.roll,nwin);   rS = moving_rms(smallDeg.roll,nwin);
pL = moving_rms(largeDeg.pitch,nwin);  pS = moving_rms(smallDeg.pitch,nwin);
yL = moving_rms(largeDeg.yaw,nwin);    yS = moving_rms(smallDeg.yaw,nwin);

fig = figure('Color','w','Position',[80 45 1420 900]);

subplot(3,1,1);
plot(t,rL,'LineWidth',2.15,'Color',largeColor); hold on;
plot(t,rS,'LineWidth',2.15,'Color',smallColor);
grid on; box on; xlim([0 Tsim]); ylim([0 positive_upper([rL,rS])]);
ylabel('横摇 RMS / deg'); title('横摇 Roll');
legend(large.name,small.name,'Location','best'); add_ratio_box(metrics.roll);

subplot(3,1,2);
plot(t,pL,'LineWidth',2.15,'Color',largeColor); hold on;
plot(t,pS,'LineWidth',2.15,'Color',smallColor);
grid on; box on; xlim([0 Tsim]); ylim([0 positive_upper([pL,pS])]);
ylabel('纵摇 RMS / deg'); title('纵摇 Pitch'); add_ratio_box(metrics.pitch);

subplot(3,1,3);
plot(t,yL,'LineWidth',2.15,'Color',largeColor); hold on;
plot(t,yS,'LineWidth',2.15,'Color',smallColor);
grid on; box on; xlim([0 Tsim]); ylim([0 positive_upper([yL,yS])]);
ylabel('艏摇 RMS / deg'); xlabel('时间 / s');
title('艏摇 / 偏航 Yaw'); add_ratio_box(metrics.yaw);

sgtitle(sprintf(['四级海况下船舶运动强度对比（%d s滑动RMS）\n' ...
    '非平稳方向扩散随机海况；一万吨级船舶 vs 一百吨级船舶；H_s = %.2f m'], ...
    winSec,HsRealized),'FontWeight','bold');
set(fig,'PaperPositionMode','auto');
print(fig,outputFile,'-dpng','-r220');
end

%% ========================================================================
function add_metric_box(m)
text(0.985,0.04,metric_text(m),'Units','normalized', ...
    'HorizontalAlignment','right','VerticalAlignment','bottom', ...
    'BackgroundColor','w','EdgeColor',[0.75 0.75 0.75], ...
    'Margin',4,'FontSize',8.5);
end

%% ========================================================================
function add_ratio_box(m)
text(0.985,0.92,ratio_text(m),'Units','normalized', ...
    'HorizontalAlignment','right','VerticalAlignment','top', ...
    'FontWeight','bold','FontSize',11,'BackgroundColor','w', ...
    'EdgeColor',[0.70 0.70 0.70],'Margin',5);
end

%% ========================================================================
function txt = metric_text(m)
txt = sprintf(['全时程统计\n' ...
    '1万吨：RMS %.2f°, Peak %.2f°, 95%%|响应| %.2f°\n' ...
    '100吨：RMS %.2f°, Peak %.2f°, 95%%|响应| %.2f°'], ...
    m.largeRms,m.largePeak,m.largeP95,m.smallRms,m.smallPeak,m.smallP95);
end

%% ========================================================================
function txt = ratio_text(m)
txt = sprintf('100吨 / 1万吨：RMS %.1f×   峰值 %.1f×',m.rmsRatio,m.peakRatio);
end

%% ========================================================================
function y = moving_rms(x,nwin)
window = ones(1,nwin)/nwin;
y = sqrt(conv(x.^2,window,'same'));
end

%% ========================================================================
function p = percentile_abs(x,probability)
v = sort(abs(x(:)));
idx = max(1,min(numel(v),ceil(probability*numel(v))));
p = v(idx);
end

%% ========================================================================
function yl = expand_ylim(x,padRatio)
xmin = min(x); xmax = max(x); span = xmax-xmin;
if span < 1e-9, span = max(1,abs(xmax)); end
pad = padRatio*span;
yl = [xmin-pad,xmax+pad];
end

%% ========================================================================
function ymax = positive_upper(x)
ymax = max(x);
if ymax < 1e-9, ymax = 1; else, ymax = 1.08*ymax; end
end

%% ========================================================================
function y = custom_sinc(x)
y = ones(size(x));
idx = abs(x) > 1e-12;
y(idx) = sin(x(idx))./x(idx);
end

%% ========================================================================
function y = normalize_std(x)
y = x-mean(x);
s = std(y);
if s > eps, y = y/s; end
end

%% ========================================================================
function print_metric_line(labelText, largeDeg, smallDeg)
rmsLarge = sqrt(mean(largeDeg.^2));
rmsSmall = sqrt(mean(smallDeg.^2));
peakLarge = max(abs(largeDeg));
peakSmall = max(abs(smallDeg));
fprintf('%s | RMS: large %6.3f deg, small %6.3f deg | ', ...
    labelText,rmsLarge,rmsSmall);
fprintf('peak: large %6.3f deg, small %6.3f deg\n',peakLarge,peakSmall);
end
