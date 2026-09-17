function results = run_lowtemp_deck_multiphysics_control()
%RUN_LOWTEMP_DECK_MULTIPHYSICS_CONTROL
% Low-temperature deck-equipment flow-solid-thermal coupled disturbance
% simulation with optimized dual-stage microdynamic compensation control.
%
% Usage:
%   results = run_lowtemp_deck_multiphysics_control();
%
% One public entry function; all other functions are local to this file.
%
% Model scope
%   Fluid: nonstationary gusts, direction meander, turbulence, VIV, spray/slam.
%   Solid: rigid-base vibration, flexible deck mode, icing-added mass,
%          actuator reaction, coarse compensation stage and fine micro-stage.
%   Thermal: ambient cooling, forced convection, spray heat exchange,
%            structural/actuator thermal nodes, thermal contraction, heater.
%   Low-T degradation: stiffness/damping/friction drift and actuator lag drift.
%   Sensing/control: delay, noise, prediction, thermal feedforward, flow/spray
%                    feedforward, temperature-adaptive model compensation,
%                    filtered feedback, disturbance observer, actuator lead
%                    compensation, saturation/rate limits, fine-stage cleanup.
%
% Performance optimization relative to the previous version
%   1) Output-error feedback rather than relative-stage error only.
%   2) Acceleration-aided delay prediction for support motion.
%   3) Wider but filtered disturbance feedforward.
%   4) Low-bandwidth disturbance observer for unmeasured residual force.
%   5) Partial inverse-lag lead compensation for the coarse actuator.
%   6) Dedicated fine micro-stage to suppress residual 5--30 Hz motion.
%   7) Explicit high-frequency roll-off to avoid noise injection.
%
% Every run generates three IEEE-style 600-dpi figures:
%   LowTempDeck_Multiphysics_Fields.png
%   LowTempDeck_Control_Response.png
%   LowTempDeck_Control_Performance.png
%
% This is an engineering reduced-order model for control-mechanism study and
% presentation. It does not replace equipment-specific CFD/FEA, modal tests,
% environmental qualification, or identified plant/actuator models.
%
% Reproducible seed: 20260917

clc;
close all;
rng(20260917, 'twister');

%% 1. Simulation and plant parameters
p.dt = 0.005;                 % s, 200 Hz
p.Tsim = 240;                 % s
p.t = 0:p.dt:p.Tsim;
p.fs = 1/p.dt;
p.N = numel(p.t);

% Coarse precision compensation stage
p.me = 160;                   % kg
p.fa0 = 18.0;                 % Hz
p.za0 = 0.08;
p.ka0 = (2*pi*p.fa0)^2*p.me;
p.ca0 = 2*p.za0*sqrt(p.me*p.ka0);

% Local flexible deck mode
p.md0 = 850;                  % kg
p.fd0 = 5.8;                  % Hz
p.zd0 = 0.045;
p.kd0 = (2*pi*p.fd0)^2*p.md0;
p.cd0 = 2*p.zd0*sqrt(p.md0*p.kd0);

% Low-temperature sensitivity
p.Tcal = -18;                 % degC
p.EtempCoeff = 7.0e-4;
p.deckDampColdCoeff = 5.0e-3;
p.stageDampColdCoeff = 6.0e-3;
p.frictionColdCoeff = 3.5e-2;
p.Fc0 = 18;                   % N
p.frictionVelocity = 4e-5;    % m/s

% Thermal deformation
p.alphaEff = 7.5e-6;          % 1/K
p.Lthermal = 0.42;            % m

% Coarse actuator
p.Fmax = 4200;                % N
p.forceRateMax = 8.0e4;       % N/s
p.tauAct0 = 0.018;            % s
p.tauColdCoeff = 0.028;
p.reactionRatio = 0.03;

% Optimized coarse controller
p.Kp = 8.0e5;                 % N/m
p.Kd = 8.0e3;                 % N/(m/s)
p.Ki = 5.0e4;                 % N/(m*s)
p.intLimit = 4.0e-4;          % m*s
p.feedbackFilterTau = 0.015;  % s, suppress sensor/high-frequency injection
p.commandFilterTau = 0.008;   % s
p.actuatorLeadFraction = 0.50;% partial inverse-lag compensation
p.flowFF = 0.85;              % predictable flow-load feedforward fraction
p.sprayFF = 0.55;             % spray-pressure feedforward fraction

% Disturbance observer (low-bandwidth residual force channel)
p.obsK = 4.0e5;               % N/m
p.obsD = 2.0e3;               % N/(m/s)
p.obsTau = 0.080;             % s
p.obsLimit = 600;              % N

% Measurement / computation latency
p.sensorDelay = 0.010;        % s
p.delaySteps = max(1, round(p.sensorDelay/p.dt));

% Fine micro-stage: fast, small-stroke cleanup actuator
p.fineStroke = 300e-6;        % m, +/-300 um
p.fineTau0 = 0.004;           % s
p.fineColdCoeff = 0.012;
p.fineGain = 0.95;
p.fineCommandTau = 0.003;     % s, explicit HF roll-off

% Thermal nodes
p.Cstruct = 2.60e4;           % J/K
p.Cact = 1.20e4;              % J/K
p.Astruct = 1.40;             % m^2
p.Aact = 0.65;                % m^2
p.Tsea = -1.5;                % degC

% Icing surrogate
p.iceRate = 0.028;            % kg/s per unit spray index
p.iceAreaCoeff = 0.03;
p.iceSheddingCoeff = 3.0e-4;  % vibration/wind-assisted shedding surrogate

%% 2. Common stochastic multiphysics environment
env = build_environment(p);

%% 3. Baseline and optimized controlled cases
passive = simulate_case(p, env, false);
controlled = simulate_case(p, env, true);

%% 4. Metrics
metrics = calculate_metrics(p, passive, controlled);

%% 5. IEEE-style figures
fileFields = 'LowTempDeck_Multiphysics_Fields.png';
fileResponse = 'LowTempDeck_Control_Response.png';
filePerformance = 'LowTempDeck_Control_Performance.png';

plot_multiphysics_fields(p, env, controlled, fileFields);
plot_control_response(p, env, passive, controlled, metrics, fileResponse);
plot_control_performance(p, passive, controlled, metrics, filePerformance);

%% 6. Console summary
fprintf('\n======================================================================\n');
fprintf('Low-temperature deck equipment multiphysics compensation simulation\n');
fprintf('Ambient temperature: %.2f to %.2f degC\n', min(env.Tair), max(env.Tair));
fprintf('Wind speed:          %.2f to %.2f m/s\n', min(env.wind), max(env.wind));
fprintf('Final ice mass:      %.3f kg\n', controlled.iceMass(end));
fprintf('----------------------------------------------------------------------\n');
fprintf('Passive RMS:         %8.3f um\n', metrics.passiveRms_um);
fprintf('Controlled RMS:      %8.3f um\n', metrics.controlledRms_um);
fprintf('RMS reduction:       %8.3f x  (%6.2f dB)\n', ...
    metrics.rmsRatio, metrics.attenuation_dB);
fprintf('Passive peak:        %8.3f um\n', metrics.passivePeak_um);
fprintf('Controlled peak:     %8.3f um\n', metrics.controlledPeak_um);
fprintf('Controlled 95%%:      %8.3f um\n', metrics.controlledP95_um);
fprintf('Coarse force RMS:    %8.3f N\n', metrics.controlRms_N);
fprintf('Coarse force peak:   %8.3f N\n', metrics.controlPeak_N);
fprintf('Fine-stage RMS:      %8.3f um\n', metrics.fineRms_um);
fprintf('Fine-stage peak:     %8.3f um\n', metrics.finePeak_um);
fprintf('Coarse saturation:   %8.4f %%\n', 100*metrics.saturationFraction);
fprintf('Fine stroke limit:   %8.4f %%\n', 100*metrics.fineLimitFraction);
fprintf('----------------------------------------------------------------------\n');
fprintf('Figure 1: %s\n', fileFields);
fprintf('Figure 2: %s\n', fileResponse);
fprintf('Figure 3: %s\n', filePerformance);
fprintf('======================================================================\n\n');

%% 7. Return complete result struct
results.meta.description = ['Optimized dual-stage flow-solid-thermal coupled ' ...
    'microdynamic compensation simulation for low-temperature deck equipment'];
results.meta.seed = 20260917;
results.meta.dt = p.dt;
results.meta.duration = p.Tsim;
results.meta.note = ['Engineering reduced-order model; replace surrogate plant ' ...
    'parameters with equipment-specific identified/CFD/FEA/test data for prediction.'];
results.meta.outputFiles.fields = fileFields;
results.meta.outputFiles.response = fileResponse;
results.meta.outputFiles.performance = filePerformance;
results.parameters = p;
results.environment = env;
results.passive = passive;
results.controlled = controlled;
results.metrics = metrics;
end

%% ========================================================================
function env = build_environment(p)
% Nonstationary low-temperature marine deck environment.
t = p.t;
dt = p.dt;
N = p.N;

% Ambient temperature: colored fluctuation + cold-front passage.
env.Tair = -30 + 2.0*colored_noise(N, dt, 35) ...
    - 5.0*exp(-0.5*((t-135)/28).^2) + sin(2*pi*t/180);

% Nonstationary wind and direction meander.
env.wind = 13 + 2.8*colored_noise(N, dt, 5) ...
    + 7.0*exp(-0.5*((t-55)/12).^2) ...
    + 9.0*exp(-0.5*((t-145)/18).^2) ...
    + 6.0*exp(-0.5*((t-205)/10).^2);
env.wind = min(28, max(4, env.wind));
env.windDirDeg = 35 + 18*colored_noise(N, dt, 25);
env.windDir = env.windDirDeg*pi/180;

% Cold-air density correction.
env.rhoAir = 1.225*293.15./(env.Tair + 273.15);
env.rhoAir = min(1.45, max(1.18, env.rhoAir));

% Irregular rigid-base deck motion.
env.base = make_irregular_base_motion(p);
env.baseVel = numerical_derivative(env.base, dt);
env.baseAcc = numerical_derivative(env.baseVel, dt);

% Payload flow load: drag fluctuation + VIV + broadband turbulence.
CdPayload = 1.15;
Apayload = 0.75;
Dchar = 0.65;
St = 0.20;
Ux = env.wind.*cos(env.windDir);
FdragRaw = 0.5.*env.rhoAir.*CdPayload.*Apayload.*Ux.*abs(Ux);
FdragMean = lowpass_signal(FdragRaw, 20, dt);
env.FdragPayload = FdragRaw - FdragMean;
fVortex = St*abs(Ux)/Dchar;
phaseVortex = cumsum(2*pi*fVortex*dt);
env.FvortexPayload = 0.22*abs(FdragRaw).*sin(phaseVortex + 0.40);
env.FturbPayload = 18*colored_noise(N, dt, 0.18);
env.Fpayload = env.FdragPayload + env.FvortexPayload + env.FturbPayload;

% Deck/support flow load.
CdDeck = 1.10;
Adeck = 3.20;
FdeckRaw = 0.5.*env.rhoAir.*CdDeck.*Adeck.*Ux.*abs(Ux);
FdeckMean = lowpass_signal(FdeckRaw, 25, dt);
env.FdragDeck = FdeckRaw - FdeckMean;
env.FvortexDeck = 0.10*abs(FdeckRaw).*sin(0.73*phaseVortex + 1.10);
env.FturbDeck = 60*colored_noise(N, dt, 0.28);
env.Fdeck = env.FdragDeck + env.FvortexDeck + env.FturbDeck;

% Random spray/slam events.
[env.sprayIndex, env.Fspray, env.sprayEventCount] = make_spray_impacts(p);

% Sensor channels. Common records ensure fair passive/controlled comparison.
env.noise.x = 0.25e-6*colored_noise(N, dt, 0.010);
env.noise.xVel = 4.0e-6*colored_noise(N, dt, 0.020);
env.noise.support = 0.60e-6*colored_noise(N, dt, 0.020);
env.noise.supportVel = 5.0e-6*colored_noise(N, dt, 0.030);
env.noise.supportAcc = 1.5e-3*colored_noise(N, dt, 0.015);
env.noise.temp = 0.12*colored_noise(N, dt, 0.30);
env.noise.flow = 0.04*colored_noise(N, dt, 0.12);
env.noise.spray = 0.05*colored_noise(N, dt, 0.08);
end

%% ========================================================================
function y = make_irregular_base_motion(p)
% Multi-band nonstationary deck micro-motion with non-commensurate frequencies.
t = p.t;
N = p.N;
dt = p.dt;
y = zeros(1,N);
cfg = {
    [0.18 0.90], 45e-6, 28, 25;
    [2.00 5.00], 12e-6, 18, 12;
    [7.00 14.0],  6e-6, 16,  7
    };
for ib = 1:size(cfg,1)
    band = cfg{ib,1};
    ampScale = cfg{ib,2};
    nComp = cfg{ib,3};
    tau = cfg{ib,4};
    freq = band(1) + (band(2)-band(1))*rand(1,nComp);
    phase = 2*pi*rand(1,nComp);
    weight = rand(1,nComp);
    weight = weight/sqrt(sum(weight.^2));
    carrier = zeros(1,N);
    for k = 1:nComp
        carrier = carrier + weight(k)*sin(2*pi*freq(k)*t + phase(k));
    end
    carrier = normalize_std(carrier);
    envelope = max(0.35, 1 + 0.25*colored_noise(N, dt, tau));
    y = y + ampScale*envelope.*carrier;
end
y = y + 60e-6*exp(-0.5*((t-80)/7).^2).* ...
    sin(2*pi*0.55*(t-80) + 2*pi*rand);
y = y + 75e-6*exp(-0.5*((t-168)/10).^2).* ...
    sin(2*pi*0.42*(t-168) + 2*pi*rand);
end

%% ========================================================================
function [sprayIndex, Fspray, count] = make_spray_impacts(p)
% Poisson-like random spray/impact pulses.
dt = p.dt;
N = p.N;
sprayIndex = zeros(1,N);
Fspray = zeros(1,N);
meanInterval = 6.5;
count = 0;
k = 1;
while k <= N
    if rand < dt/meanInterval
        count = count + 1;
        magnitude = exp(0.45*randn);
        signImpact = 2*(rand > 0.5) - 1;
        tau = 0.12 + 0.25*rand;
        duration = max(2, round(2.5*tau/dt));
        k2 = min(N, k+duration-1);
        ii = k:k2;
        decay = exp(-(ii-k)*dt/tau);
        sprayIndex(ii) = sprayIndex(ii) + magnitude*decay;
        Famp = 120 + 160*rand;
        Fspray(ii) = Fspray(ii) + signImpact*Famp*magnitude.*decay;
        k = k + max(1, round(0.05/dt));
    end
    k = k + 1;
end
end

%% ========================================================================
function out = simulate_case(p, env, enableControl)
% Coupled flow-solid-thermal simulation with dual-stage compensation.
% z      : local flexible-deck displacement relative to rigid base
% x      : coarse-stage displacement relative to local deck
% xFine  : fine-stage micro displacement relative to coarse stage
% y      : absolute precision-point error = base + z + x + xFine + thermal

N = p.N;
dt = p.dt;
x = zeros(1,N);
xVel = zeros(1,N);
z = zeros(1,N);
zVel = zeros(1,N);
zAccHist = zeros(1,N);
xFine = zeros(1,N);
xFineCmd = zeros(1,N);
y = zeros(1,N);

Tstruct = zeros(1,N);
Tact = zeros(1,N);
Tstruct(1) = p.Tcal;
Tact(1) = -12;
thermalDef = zeros(1,N);
iceMass = zeros(1,N);

uAct = zeros(1,N);
uCmd = zeros(1,N);
residualForce = zeros(1,N);
heaterPower = zeros(1,N);
stageStiffnessRatio = ones(1,N);
deckNaturalFreq = zeros(1,N);
actuatorTau = zeros(1,N);
fineTau = zeros(1,N);
frictionScale = zeros(1,N);

% Controller/filter states.
integralError = 0;
TstructEst = p.Tcal;
dObserver = 0;
eFilt = 0;
eVelFilt = 0;
uDesiredFilt = 0;
uDesiredPrev = 0;
fineCmdFilt = 0;

for k = 1:N-1
    % Temperature-dependent plant parameters.
    thermalDef(k) = p.alphaEff*p.Lthermal*(Tstruct(k)-p.Tcal);
    Escale = 1 + p.EtempCoeff*(20-Tstruct(k));
    Escale = min(1.08, max(0.96, Escale));
    ka = p.ka0*Escale;
    kd = p.kd0*Escale;
    ca = p.ca0*(1 + p.stageDampColdCoeff*max(0, -Tstruct(k)-5));
    cd = p.cd0*(1 + p.deckDampColdCoeff*max(0, -Tstruct(k)-5));
    md = p.md0 + iceMass(k);

    stageStiffnessRatio(k) = ka/p.ka0;
    deckNaturalFreq(k) = sqrt(kd/md)/(2*pi);
    actuatorTau(k) = p.tauAct0*(1 + p.tauColdCoeff*max(0, -Tact(k)-10));
    fineTau(k) = p.fineTau0*(1 + p.fineColdCoeff*max(0, -Tact(k)-15));
    frictionScale(k) = p.Fc0*(1 + p.frictionColdCoeff*max(0, -Tact(k)-5));

    areaScale = 1 + p.iceAreaCoeff*min(iceMass(k), 3.0);
    Fpayload = areaScale*env.Fpayload(k) + 0.25*env.Fspray(k);
    Fdeck = areaScale*env.Fdeck(k) + 0.80*env.Fspray(k);

    if enableControl
        j = max(1, k-p.delaySteps);
        latency = (k-j)*dt;

        % Delayed/noisy sensor channels with second-order support prediction.
        xMeas = x(j) + env.noise.x(j) + 0.08e-6*(Tact(j)-p.Tcal);
        xVelMeas = xVel(j) + env.noise.xVel(j);
        supportMeas = env.base(j) + z(j) + env.noise.support(j);
        supportVelMeas = env.baseVel(j) + zVel(j) + env.noise.supportVel(j);
        supportAccMeas = env.baseAcc(j) + zAccHist(j) + env.noise.supportAcc(j);

        xPred = xMeas + xVelMeas*latency;
        xVelPred = xVelMeas;
        supportPred = supportMeas + supportVelMeas*latency ...
            + 0.5*supportAccMeas*latency^2;
        supportVelPred = supportVelMeas + supportAccMeas*latency;

        % Temperature estimation and thermal feedforward.
        TstructEst = TstructEst + dt/(0.40+dt)* ...
            ((Tstruct(j)+env.noise.temp(j))-TstructEst);
        thermalEst = p.alphaEff*p.Lthermal*(TstructEst-p.Tcal);

        % Coarse-stage reference cancels support + thermal drift.
        xRef = -(supportPred + thermalEst);
        xRefVel = -supportVelPred;

        Eest = 1 + p.EtempCoeff*(20-TstructEst);
        Eest = min(1.08, max(0.96, Eest));
        kaEst = p.ka0*Eest;
        caEst = p.ca0*(1 + p.stageDampColdCoeff*max(0, -TstructEst-5));

        % Predictable disturbance feedforward.
        FflowEst = p.flowFF*(env.FdragPayload(j)+env.FvortexPayload(j));
        FflowEst = FflowEst*(1 + env.noise.flow(j));
        FsprayEst = p.sprayFF*env.Fspray(j)*(1 + env.noise.spray(j));

        % Absolute output-error feedback. Low-pass states provide HF roll-off.
        yPredCoarse = supportPred + xPred + thermalEst;
        yVelPredCoarse = supportVelPred + xVelPred;
        e = -yPredCoarse;
        eVel = -yVelPredCoarse;
        af = dt/(p.feedbackFilterTau+dt);
        eFilt = eFilt + af*(e-eFilt);
        eVelFilt = eVelFilt + af*(eVel-eVelFilt);
        integralError = integralError + eFilt*dt;
        integralError = min(p.intLimit, max(-p.intLimit, integralError));

        % Model-based feedforward + residual disturbance observer.
        uFF = kaEst*xRef + caEst*xRefVel - FflowEst - FsprayEst;
        dTarget = p.obsK*eFilt + p.obsD*eVelFilt;
        dObserver = dObserver + dt/(p.obsTau+dt)*(dTarget-dObserver);
        dObserver = min(p.obsLimit, max(-p.obsLimit, dObserver));
        residualForce(k) = dObserver;

        uDesired = uFF + p.Kp*eFilt + p.Kd*eVelFilt ...
            + p.Ki*integralError + dObserver;

        % Command prefilter followed by limited inverse-lag lead compensation.
        ac = dt/(p.commandFilterTau+dt);
        uDesiredFilt = uDesiredFilt + ac*(uDesired-uDesiredFilt);
        duDesired = (uDesiredFilt-uDesiredPrev)/dt;
        uDesiredPrev = uDesiredFilt;
        uLead = uDesiredFilt + p.actuatorLeadFraction*actuatorTau(k)*duDesired;
        uCmd(k) = min(p.Fmax, max(-p.Fmax, uLead));

        % Fine-stage cleanup uses the predicted residual left by coarse motion.
        fineTarget = -p.fineGain*yPredCoarse;
        fineTarget = min(p.fineStroke, max(-p.fineStroke, fineTarget));
        afine = dt/(p.fineCommandTau+dt);
        fineCmdFilt = fineCmdFilt + afine*(fineTarget-fineCmdFilt);
        xFineCmd(k) = fineCmdFilt;
    else
        uCmd(k) = 0;
        residualForce(k) = 0;
        xFineCmd(k) = 0;
    end

    % Coarse actuator lag/rate/saturation.
    du = (uCmd(k)-uAct(k))/actuatorTau(k);
    du = min(p.forceRateMax, max(-p.forceRateMax, du));
    uAct(k+1) = uAct(k) + dt*du;
    uAct(k+1) = min(p.Fmax, max(-p.Fmax, uAct(k+1)));

    % Fine actuator: inner-loop closed position servo, first-order equivalent.
    xFine(k+1) = xFine(k) + dt/(fineTau(k)+dt)*(xFineCmd(k)-xFine(k));
    xFine(k+1) = min(p.fineStroke, max(-p.fineStroke, xFine(k+1)));

    % Mechanical dynamics.
    Fc = frictionScale(k);
    Ffriction = Fc*tanh(xVel(k)/p.frictionVelocity);
    zAcc = (Fdeck - cd*zVel(k) - kd*z(k) ...
        - p.reactionRatio*uAct(k) - md*env.baseAcc(k))/md;
    zAccHist(k) = zAcc;
    supportAcc = env.baseAcc(k) + zAcc;
    xAcc = (uAct(k) + Fpayload - ca*xVel(k) - ka*x(k) ...
        - Ffriction - p.me*supportAcc)/p.me;

    zVel(k+1) = zVel(k) + dt*zAcc;
    z(k+1) = z(k) + dt*zVel(k+1);
    xVel(k+1) = xVel(k) + dt*xAcc;
    x(k+1) = x(k) + dt*xVel(k+1);

    % Icing growth with wind/vibration-assisted shedding.
    freezeFactor = min(1, max(0, (-env.Tair(k)-2)/25));
    grow = p.iceRate*env.sprayIndex(k)*freezeFactor;
    shearIndex = min(2, abs(Fdeck)/400 + abs(zAcc)/2.0);
    shed = p.iceSheddingCoeff*iceMass(k)*shearIndex;
    iceMass(k+1) = max(0, iceMass(k) + dt*(grow-shed));

    % Thermal dynamics.
    hStruct = 7 + 1.25*env.wind(k);
    hAct = 6 + 0.90*env.wind(k);
    Qspray = 18*env.sprayIndex(k)*(p.Tsea-Tstruct(k));
    TstructDot = (hStruct*p.Astruct*(env.Tair(k)-Tstruct(k)) ...
        + 25 + Qspray)/p.Cstruct;

    if Tact(k) < -17
        heaterPower(k) = 190;
    elseif Tact(k) < -14
        heaterPower(k) = 60;
    else
        heaterPower(k) = 0;
    end
    Qmotion = 0.06*abs(uAct(k)*xVel(k));
    TactDot = (hAct*p.Aact*(env.Tair(k)-Tact(k)) + 35 ...
        + heaterPower(k) + Qmotion + 0.08*(Tstruct(k)-Tact(k)))/p.Cact;
    Tstruct(k+1) = Tstruct(k) + dt*TstructDot;
    Tact(k+1) = Tact(k) + dt*TactDot;

    y(k) = env.base(k) + z(k) + x(k) + xFine(k) + thermalDef(k);
end

% Final samples.
thermalDef(end) = p.alphaEff*p.Lthermal*(Tstruct(end)-p.Tcal);
y(end) = env.base(end) + z(end) + x(end) + xFine(end) + thermalDef(end);
stageStiffnessRatio(end) = stageStiffnessRatio(end-1);
deckNaturalFreq(end) = deckNaturalFreq(end-1);
actuatorTau(end) = actuatorTau(end-1);
fineTau(end) = fineTau(end-1);
frictionScale(end) = frictionScale(end-1);
heaterPower(end) = heaterPower(end-1);
uCmd(end) = uCmd(end-1);
residualForce(end) = residualForce(end-1);
xFineCmd(end) = xFineCmd(end-1);

out.controlEnabled = enableControl;
out.x = x;
out.xVel = xVel;
out.fineDisplacement = xFine;
out.fineCommand = xFineCmd;
out.deckFlex = z;
out.deckFlexVel = zVel;
out.output = y;
out.Tstruct = Tstruct;
out.Tactuator = Tact;
out.thermalDeformation = thermalDef;
out.iceMass = iceMass;
out.uCommand = uCmd;
out.uActuator = uAct;
out.residualCompensation = residualForce;
out.heaterPower = heaterPower;
out.stageStiffnessRatio = stageStiffnessRatio;
out.deckNaturalFreq = deckNaturalFreq;
out.actuatorTau = actuatorTau;
out.fineTau = fineTau;
out.frictionScale = frictionScale;
end

%% ========================================================================
function metrics = calculate_metrics(p, passive, controlled)
passive_um = passive.output*1e6;
controlled_um = controlled.output*1e6;
metrics.passiveRms_um = rms_local(passive_um);
metrics.controlledRms_um = rms_local(controlled_um);
metrics.passivePeak_um = max(abs(passive_um));
metrics.controlledPeak_um = max(abs(controlled_um));
metrics.passiveP95_um = percentile_abs(passive_um, 0.95);
metrics.controlledP95_um = percentile_abs(controlled_um, 0.95);
metrics.rmsRatio = metrics.passiveRms_um/max(metrics.controlledRms_um, eps);
metrics.attenuation_dB = 20*log10(metrics.rmsRatio);
metrics.controlRms_N = rms_local(controlled.uActuator);
metrics.controlPeak_N = max(abs(controlled.uActuator));
metrics.fineRms_um = rms_local(controlled.fineDisplacement*1e6);
metrics.finePeak_um = max(abs(controlled.fineDisplacement))*1e6;
metrics.saturationFraction = mean(abs(controlled.uActuator) >= 0.98*p.Fmax);
metrics.fineLimitFraction = mean(abs(controlled.fineDisplacement) >= 0.98*p.fineStroke);
metrics.controlWorkProxy_J = sum(abs(controlled.uActuator.*controlled.xVel))*p.dt;
end

%% ========================================================================
function plot_multiphysics_fields(p, env, controlled, outputFile)
% IEEE-like compact multipanel figure: short titles, Times New Roman, 600 dpi.
t = p.t;
C = ieee_colors();
fig = ieee_figure(17.8, 17.0);

ax = subplot(3,2,1);
plot(t, env.Tair, '-', 'Color', C.gray, 'LineWidth',0.9); hold on;
plot(t, controlled.Tstruct, '-', 'Color', C.blue, 'LineWidth',1.1);
plot(t, controlled.Tactuator, '--', 'Color', C.red, 'LineWidth',1.1);
ylabel('Temperature (^{\circ}C)'); title('(a) Thermal field');
legend('Ambient','Structure','Actuator','Location','best'); ieee_axes(ax);

ax = subplot(3,2,2);
yyaxis left;
plot(t, env.wind, '-', 'Color', C.blue, 'LineWidth',1.0);
ylabel('Wind speed (m/s)');
yyaxis right;
plot(t, env.windDirDeg, '-', 'Color', C.red, 'LineWidth',0.9);
ylabel('Direction (deg)'); title('(b) Nonstationary wind field');
ieee_axes(ax);

ax = subplot(3,2,3);
plot(t, env.Fpayload, '-', 'Color', C.blue, 'LineWidth',0.85); hold on;
plot(t, env.Fspray, '-', 'Color', C.red, 'LineWidth',0.80);
ylabel('Disturbance force (N)'); title('(c) Fluid and spray loads');
legend('Aerodynamic/VIV','Spray/slam','Location','best'); ieee_axes(ax);

ax = subplot(3,2,4);
plot(t, env.base*1e6, '-', 'Color', C.blue, 'LineWidth',0.85); hold on;
plot(t, controlled.deckFlex*1e6, '-', 'Color', C.red, 'LineWidth',0.85);
ylabel('Displacement (\mum)'); title('(d) Structural response');
legend('Rigid-base input','Local deck flex','Location','best'); ieee_axes(ax);

ax = subplot(3,2,5);
yyaxis left;
plot(t, controlled.thermalDeformation*1e6, '-', 'Color', C.blue, 'LineWidth',1.0);
ylabel('Thermal drift (\mum)');
yyaxis right;
plot(t, controlled.iceMass, '-', 'Color', C.red, 'LineWidth',1.0);
ylabel('Ice mass (kg)'); xlabel('Time (s)'); title('(e) Thermal drift and icing');
ieee_axes(ax);

ax = subplot(3,2,6);
plot(t, controlled.stageStiffnessRatio, '-', 'Color', C.blue, 'LineWidth',1.0); hold on;
plot(t, controlled.deckNaturalFreq/p.fd0, '--', 'Color', C.red, 'LineWidth',1.0);
plot(t, controlled.actuatorTau/p.tauAct0, '-.', 'Color', C.gold, 'LineWidth',1.0);
xlabel('Time (s)'); ylabel('Normalized parameter'); title('(f) Low-temperature parameter drift');
legend('Stage stiffness','Deck modal freq.','Coarse lag','Location','best'); ieee_axes(ax);

ieee_export(fig, outputFile);
end

%% ========================================================================
function plot_control_response(p, env, passive, controlled, metrics, outputFile)
t = p.t;
C = ieee_colors();
passive_um = passive.output*1e6;
controlled_um = controlled.output*1e6;
winSec = 2.0;
nwin = max(5, round(winSec/p.dt));
rmsPassive = moving_rms(passive_um, nwin);
rmsControlled = moving_rms(controlled_um, nwin);

[~, iPeakWind] = max(env.wind);
tCenter = t(iPeakWind);
t1 = max(0, tCenter-8);
t2 = min(p.Tsim, tCenter+12);
idxZoom = (t >= t1 & t <= t2);

fig = ieee_figure(17.8, 13.5);
ax = subplot(2,2,1);
plot(t, passive_um, '-', 'Color', C.gray, 'LineWidth',0.75); hold on;
plot(t, controlled_um, '-', 'Color', C.blue, 'LineWidth',0.95);
xlabel('Time (s)'); ylabel('Position error (\mum)'); title('(a) Full-duration response');
legend('Passive','Compensated','Location','best'); ieee_axes(ax);

ax = subplot(2,2,2);
plot(t(idxZoom), passive_um(idxZoom), '-', 'Color', C.gray, 'LineWidth',0.85); hold on;
plot(t(idxZoom), controlled_um(idxZoom), '-', 'Color', C.blue, 'LineWidth',1.00);
xlabel('Time (s)'); ylabel('Position error (\mum)'); title('(b) Response near strongest gust');
legend('Passive','Compensated','Location','best'); ieee_axes(ax);

ax = subplot(2,2,3);
plot(t, rmsPassive, '-', 'Color', C.gray, 'LineWidth',1.0); hold on;
plot(t, rmsControlled, '-', 'Color', C.blue, 'LineWidth',1.1);
xlabel('Time (s)'); ylabel('Moving RMS (\mum)'); title('(c) Local motion intensity');
legend('Passive','Compensated','Location','best'); ieee_axes(ax);
text(0.98,0.92,sprintf('RMS: %.1f \rightarrow %.1f \mum\nReduction: %.1f dB', ...
    metrics.passiveRms_um, metrics.controlledRms_um, metrics.attenuation_dB), ...
    'Units','normalized','HorizontalAlignment','right','VerticalAlignment','top', ...
    'FontName','Times New Roman','FontSize',8,'BackgroundColor','w');

ax = subplot(2,2,4);
plot(t, 100*controlled.uActuator/p.Fmax, '-', 'Color', C.blue, 'LineWidth',0.9); hold on;
plot(t, 100*controlled.fineDisplacement/p.fineStroke, '-', 'Color', C.red, 'LineWidth',0.9);
xlabel('Time (s)'); ylabel('Actuator utilization (%)'); title('(d) Coarse/fine actuator utilization');
legend('Coarse force','Fine stroke','Location','best'); ieee_axes(ax);

ieee_export(fig, outputFile);
end

%% ========================================================================
function plot_control_performance(p, passive, controlled, metrics, outputFile)
C = ieee_colors();
[f, asdPassive] = one_sided_asd(passive.output, p.fs);
[~, asdControlled] = one_sided_asd(controlled.output, p.fs);

bands = [0.10 2.0; 2.0 8.0; 8.0 30.0];
bandAtt = zeros(1,3);
for ib = 1:3
    rp = band_rms_from_asd(f, asdPassive, bands(ib,1), bands(ib,2));
    rc = band_rms_from_asd(f, asdControlled, bands(ib,1), bands(ib,2));
    bandAtt(ib) = 20*log10(max(rp,eps)/max(rc,eps));
end

fig = ieee_figure(17.8, 13.5);
ax = subplot(2,2,1);
semilogy(f, asdPassive*1e6, '-', 'Color', C.gray, 'LineWidth',0.85); hold on;
semilogy(f, asdControlled*1e6, '-', 'Color', C.blue, 'LineWidth',1.00);
xlim([0.05 30]); xlabel('Frequency (Hz)'); ylabel('ASD (\mum/\surdHz)');
title('(a) Displacement spectrum'); legend('Passive','Compensated','Location','best'); ieee_axes(ax);

ax = subplot(2,2,2);
B = [metrics.passiveRms_um, metrics.controlledRms_um; ...
     metrics.passivePeak_um, metrics.controlledPeak_um; ...
     metrics.passiveP95_um, metrics.controlledP95_um];
hb = bar(B, 'grouped');
set(hb(1),'FaceColor',C.gray,'EdgeColor','none');
set(hb(2),'FaceColor',C.blue,'EdgeColor','none');
set(gca,'XTickLabel',{'RMS','Peak','95%'});
ylabel('Position error (\mum)'); title('(b) Time-domain metrics');
legend('Passive','Compensated','Location','best'); ieee_axes(ax);

ax = subplot(2,2,3);
plot(p.t, controlled.Tactuator, '-', 'Color', C.blue, 'LineWidth',0.95); hold on;
plot(p.t, 1000*controlled.actuatorTau, '--', 'Color', C.red, 'LineWidth',0.95);
plot(p.t, controlled.frictionScale, '-.', 'Color', C.gold, 'LineWidth',0.95);
xlabel('Time (s)'); ylabel('State value'); title('(c) Low-temperature actuator states');
legend('Temperature (^{\circ}C)','Coarse lag (ms)','Friction scale (N)','Location','best'); ieee_axes(ax);

ax = subplot(2,2,4);
hb = bar(1:3, bandAtt, 0.62);
set(hb,'FaceColor',C.blue,'EdgeColor','none');
set(gca,'XTick',1:3,'XTickLabel',{'0.1-2 Hz','2-8 Hz','8-30 Hz'});
ylabel('Attenuation (dB)'); title('(d) Band-wise disturbance attenuation');
yline(0,'-','Color',[0.25 0.25 0.25],'LineWidth',0.7); ieee_axes(ax);

ieee_export(fig, outputFile);
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
    'Position',[2 2 widthCm heightCm], 'PaperPositionMode','auto');
end

%% ========================================================================
function ieee_axes(ax)
set(ax,'FontName','Times New Roman','FontSize',8.5, ...
    'LineWidth',0.75,'TickDir','out','TickLength',[0.015 0.015], ...
    'Box','on','XGrid','on','YGrid','on','GridAlpha',0.12, ...
    'MinorGridAlpha',0.06,'Layer','top');
set(get(ax,'XLabel'),'FontName','Times New Roman','FontSize',9);
set(get(ax,'YLabel'),'FontName','Times New Roman','FontSize',9);
set(get(ax,'Title'),'FontName','Times New Roman','FontSize',9,'FontWeight','normal');
lgd = findobj(ax.Parent,'Type','Legend');
if ~isempty(lgd)
    set(lgd,'FontName','Times New Roman','FontSize',7.5,'Box','off');
end
end

%% ========================================================================
function ieee_export(fig, outputFile)
set(fig,'InvertHardcopy','off');
print(fig, outputFile, '-dpng', '-r600');
end

%% ========================================================================
function y = colored_noise(N, dt, tau)
w = randn(1,N);
y = zeros(1,N);
a = dt/(tau+dt);
for k = 2:N
    y(k) = y(k-1) + a*(w(k)-y(k-1));
end
y = normalize_std(y);
end

%% ========================================================================
function y = lowpass_signal(x, tau, dt)
y = zeros(size(x));
y(1) = x(1);
a = dt/(tau+dt);
for k = 2:numel(x)
    y(k) = y(k-1) + a*(x(k)-y(k-1));
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
function y = normalize_std(x)
y = x - mean(x);
s = std(y);
if s > eps
    y = y/s;
end
end

%% ========================================================================
function y = moving_rms(x, nwin)
window = ones(1,nwin)/nwin;
y = sqrt(conv(x.^2, window, 'same'));
end

%% ========================================================================
function val = rms_local(x)
val = sqrt(mean(x.^2));
end

%% ========================================================================
function pval = percentile_abs(x, probability)
v = sort(abs(x(:)));
idx = max(1, min(numel(v), ceil(probability*numel(v))));
pval = v(idx);
end

%% ========================================================================
function [f, asd] = one_sided_asd(x, fs)
% Toolbox-free one-sided ASD using a Hann window.
x = x(:).';
x = x - mean(x);
N = numel(x);
if N < 4
    f = 0;
    asd = 0;
    return;
end
w = 0.5 - 0.5*cos(2*pi*(0:N-1)/(N-1));
Nfft = 2^nextpow2(N);
X = fft(x.*w, Nfft);
P2 = abs(X).^2/(fs*sum(w.^2));
P1 = P2(1:Nfft/2+1);
if numel(P1) > 2
    P1(2:end-1) = 2*P1(2:end-1);
end
f = fs*(0:(Nfft/2))/Nfft;
asd = sqrt(P1);
end

%% ========================================================================
function r = band_rms_from_asd(f, asd, f1, f2)
idx = (f >= f1 & f <= f2);
if nnz(idx) < 2
    r = 0;
else
    r = sqrt(trapz(f(idx), asd(idx).^2));
end
end
