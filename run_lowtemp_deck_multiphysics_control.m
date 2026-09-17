function results = run_lowtemp_deck_multiphysics_control()
%RUN_LOWTEMP_DECK_MULTIPHYSICS_CONTROL
% 耐低温甲板设备在流-固-热多场扰动下的补偿微动态控制仿真。
%
% Usage:
%   results = run_lowtemp_deck_multiphysics_control();
%
% 说明：
%   这是一个用于方案论证、控制机理分析和汇报展示的降阶多物理场模型，
%   不是替代 CFD-FEA 联合仿真、环境试验或具体设备实测辨识的高保真模型。
%   文件只有一个对外入口函数，其余均为本文件局部函数。
%
% 主要考虑因素：
%   [流场]
%   - 低温空气密度变化
%   - 非平稳阵风、风向摆动、湍流
%   - 准定常气动力
%   - 随风速变化的涡激振动（Strouhal 关系）
%   - 海水喷溅/浪击随机脉冲
%
%   [固体/结构]
%   - 船体/甲板微振动基座输入（低频波浪 + 中频结构 + 高频机械）
%   - 甲板一阶柔性模态
%   - 精密补偿执行级相对运动
%   - 执行器反力对甲板柔性模态的弱耦合
%   - 结冰增质量导致的模态漂移
%
%   [热场/低温]
%   - 环境冷却、风冷增强、海水喷溅热交换
%   - 结构与执行器双热节点
%   - 执行器内部发热与低温加热器
%   - 低温导致的刚度、阻尼、摩擦、执行器时间常数变化
%   - 结构热收缩引起的微米级零位漂移
%   - 传感器温漂
%
%   [控制/测量]
%   - 无补偿（passive）与多场补偿控制对比
%   - 甲板位置/速度前馈
%   - 热变形前馈
%   - 流场扰动力前馈
%   - 温度自适应刚度/阻尼参数
%   - 低带宽残余扰动力补偿
%   - 10 ms 传感/计算延迟 + 一阶预测补偿
%   - 测量噪声
%   - 执行器一阶迟滞、饱和、力变化率限制
%
% 每次运行自动生成 3 张 PNG：
%   1) LowTempDeck_Multiphysics_Fields.png
%   2) LowTempDeck_Control_Response.png
%   3) LowTempDeck_Control_Performance.png
%
% 可复现实验随机种子：20260917

clc;
close all;
rng(20260917, 'twister');

%% 1. Simulation settings
p.dt = 0.005;                 % s, 200 Hz control/simulation rate
p.Tsim = 240;                 % s
p.t = 0:p.dt:p.Tsim;
p.fs = 1/p.dt;
p.N = numel(p.t);

% Precision compensation stage
p.me = 160;                   % kg, equivalent moving mass
p.fa0 = 18.0;                 % Hz, nominal local actuator-stage mode
p.za0 = 0.08;                 % nominal damping ratio
p.ka0 = (2*pi*p.fa0)^2*p.me;
p.ca0 = 2*p.za0*sqrt(p.me*p.ka0);

% Local flexible deck mode
p.md0 = 850;                  % kg, equivalent modal mass
p.fd0 = 5.8;                  % Hz
p.zd0 = 0.045;
p.kd0 = (2*pi*p.fd0)^2*p.md0;
p.cd0 = 2*p.zd0*sqrt(p.md0*p.kd0);

% Low-temperature material / mechanism sensitivity
p.Tcal = -18;                 % degC, calibration temperature
p.EtempCoeff = 7.0e-4;        % stiffness ratio change per degC below 20C
p.deckDampColdCoeff = 5.0e-3;
p.stageDampColdCoeff = 6.0e-3;
p.frictionColdCoeff = 3.5e-2;
p.Fc0 = 18;                   % N, nominal Coulomb friction scale
p.frictionVelocity = 4e-5;    % m/s, tanh regularization velocity

% Thermal deformation
p.alphaEff = 7.5e-6;          % 1/K, effective constrained expansion coeff.
p.Lthermal = 0.42;            % m, effective thermal deformation length

% Actuator dynamics and limits
p.Fmax = 4200;                % N
p.forceRateMax = 8.0e4;       % N/s
p.tauAct0 = 0.018;            % s, nominal force-loop time constant
p.tauColdCoeff = 0.028;       % time constant degradation per degC below -10C
p.reactionRatio = 0.03;       % actuator reaction coupled into deck mode

% Controller: moderate gains because a 10 ms delay is explicitly modeled
p.Kp = 2.0e5;                 % N/m
p.Kd = 3.0e3;                 % N/(m/s)
p.Ki = 3.0e4;                 % N/(m*s)
p.intLimit = 1.0e-3;          % m*s
p.residualGain = 1.0e5;       % N/m, slow residual-force correction target
p.residualTau = 0.20;         % s
p.residualLimit = 250;        % N
p.sensorDelay = 0.010;        % s
p.delaySteps = max(1, round(p.sensorDelay/p.dt));

% Thermal nodes
p.Cstruct = 2.60e4;           % J/K
p.Cact = 1.20e4;              % J/K
p.Astruct = 1.40;             % m^2
p.Aact = 0.65;                % m^2
p.Tsea = -1.5;                % degC, cold seawater/spray temperature

% Icing surrogate
p.iceRate = 0.028;            % kg/s per unit spray index at strong freezing
p.iceAreaCoeff = 0.03;        % flow-area increase per kg ice (limited effect)

%% 2. Build common multiphysics environment
% The same stochastic environment is used for passive and compensated cases.
env = build_environment(p);

%% 3. Simulate baseline and compensated cases
passive = simulate_case(p, env, false);
controlled = simulate_case(p, env, true);

%% 4. Metrics
metrics = calculate_metrics(p, passive, controlled);

%% 5. Plot simulation results
fileFields = 'LowTempDeck_Multiphysics_Fields.png';
fileResponse = 'LowTempDeck_Control_Response.png';
filePerformance = 'LowTempDeck_Control_Performance.png';

plot_multiphysics_fields(p, env, controlled, fileFields);
plot_control_response(p, env, passive, controlled, metrics, fileResponse);
plot_control_performance(p, passive, controlled, metrics, filePerformance);

%% 6. Console report
fprintf('\n======================================================================\n');
fprintf('Low-temperature deck equipment: flow-solid-thermal microdynamic control\n');
fprintf('Ambient temperature range: %.2f to %.2f degC\n', ...
    min(env.Tair), max(env.Tair));
fprintf('Wind speed range: %.2f to %.2f m/s\n', min(env.wind), max(env.wind));
fprintf('Final ice mass (controlled case): %.3f kg\n', controlled.iceMass(end));
fprintf('----------------------------------------------------------------------\n');
fprintf('Passive RMS displacement:     %8.3f um\n', metrics.passiveRms_um);
fprintf('Controlled RMS displacement:  %8.3f um\n', metrics.controlledRms_um);
fprintf('RMS attenuation ratio:        %8.3f x\n', metrics.rmsRatio);
fprintf('RMS attenuation:              %8.3f dB\n', metrics.attenuation_dB);
fprintf('Passive abs. peak:            %8.3f um\n', metrics.passivePeak_um);
fprintf('Controlled abs. peak:         %8.3f um\n', metrics.controlledPeak_um);
fprintf('Controlled 95%% |response|:    %8.3f um\n', metrics.controlledP95_um);
fprintf('Control-force RMS:            %8.3f N\n', metrics.controlRms_N);
fprintf('Control-force abs. peak:      %8.3f N\n', metrics.controlPeak_N);
fprintf('Actuator saturation fraction: %8.4f %%\n', 100*metrics.saturationFraction);
fprintf('----------------------------------------------------------------------\n');
fprintf('Figure 1: %s\n', fileFields);
fprintf('Figure 2: %s\n', fileResponse);
fprintf('Figure 3: %s\n', filePerformance);
fprintf('======================================================================\n\n');

%% 7. Return complete data struct
results.meta.description = ['Reduced-order flow-solid-thermal coupled simulation ' ...
    'for low-temperature deck-equipment microdynamic compensation control'];
results.meta.seed = 20260917;
results.meta.dt = p.dt;
results.meta.duration = p.Tsim;
results.meta.note = ['Engineering surrogate model for mechanism/control study; ' ...
    'not a replacement for equipment-specific CFD/FEA/test identification.'];
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
% Common nonstationary low-temperature marine deck environment.

t = p.t;
dt = p.dt;
N = p.N;

% ---------- Ambient temperature: cold background + cold-front passage ----------
Tnoise = colored_noise(N, dt, 35);
env.Tair = -30 + 2.0*Tnoise ...
    - 5.0*exp(-0.5*((t-135)/28).^2) ...
    + 1.0*sin(2*pi*t/180);

% ---------- Nonstationary wind and slowly varying direction ----------
env.wind = 13 + 2.8*colored_noise(N, dt, 5);
env.wind = env.wind ...
    + 7.0*exp(-0.5*((t-55)/12).^2) ...
    + 9.0*exp(-0.5*((t-145)/18).^2) ...
    + 6.0*exp(-0.5*((t-205)/10).^2);
env.wind = min(28, max(4, env.wind));

env.windDirDeg = 35 + 18*colored_noise(N, dt, 25);
env.windDir = env.windDirDeg*pi/180;

% Cold-air density correction at approximately constant atmospheric pressure.
env.rhoAir = 1.225*293.15./(env.Tair + 273.15);
env.rhoAir = min(1.45, max(1.18, env.rhoAir));

% ---------- Irregular deck/base micro-vibration ----------
% Three bands: wave-induced / structural / machinery. Frequencies are random,
% and every band has its own nonstationary amplitude envelope.
env.base = make_irregular_base_motion(p);
env.baseVel = numerical_derivative(env.base, dt);
env.baseAcc = numerical_derivative(env.baseVel, dt);

% ---------- Flow loads ----------
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

% Larger flow load acting on the local deck/equipment support structure.
CdDeck = 1.10;
Adeck = 3.20;
FdeckRaw = 0.5.*env.rhoAir.*CdDeck.*Adeck.*Ux.*abs(Ux);
FdeckMean = lowpass_signal(FdeckRaw, 25, dt);
env.FdragDeck = FdeckRaw - FdeckMean;
env.FvortexDeck = 0.10*abs(FdeckRaw).*sin(0.73*phaseVortex + 1.10);
env.FturbDeck = 60*colored_noise(N, dt, 0.28);
env.Fdeck = env.FdragDeck + env.FvortexDeck + env.FturbDeck;

% ---------- Random sea-spray / wave-impact impulses ----------
[env.sprayIndex, env.Fspray, env.sprayEventCount] = make_spray_impacts(p);

% ---------- Sensor noises, generated once for fair comparison ----------
env.noise.x = 0.25e-6*colored_noise(N, dt, 0.010);          % m
env.noise.xVel = 4.0e-6*colored_noise(N, dt, 0.020);       % m/s
env.noise.support = 0.60e-6*colored_noise(N, dt, 0.020);    % m
env.noise.supportVel = 5.0e-6*colored_noise(N, dt, 0.030);  % m/s
env.noise.temp = 0.12*colored_noise(N, dt, 0.30);           % degC
env.noise.flow = 0.04*colored_noise(N, dt, 0.12);           % relative scale
end

%% ========================================================================
function y = make_irregular_base_motion(p)
% Irregular deck micro-motion with multiple non-commensurate spectral bands.

t = p.t;
N = p.N;
dt = p.dt;
y = zeros(1,N);

% [fmin fmax], RMS-like scale, number of components, envelope time scale
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

    envelope = 1 + 0.25*colored_noise(N, dt, tau);
    envelope = max(0.35, envelope);
    y = y + ampScale*envelope.*carrier;
end

% Two isolated wave/slam groups to make the base record explicitly nonstationary.
y = y + 60e-6*exp(-0.5*((t-80)/7).^2).* ...
    sin(2*pi*0.55*(t-80) + 2*pi*rand);
y = y + 75e-6*exp(-0.5*((t-168)/10).^2).* ...
    sin(2*pi*0.42*(t-168) + 2*pi*rand);
end

%% ========================================================================
function [sprayIndex, Fspray, count] = make_spray_impacts(p)
% Poisson-like random spray/impact events with random magnitude and decay.

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
% Coupled reduced-order flow-solid-thermal simulation.
%
% Coordinates:
%   z : local flexible-deck deflection relative to rigid base motion
%   x : active precision-stage displacement relative to local deck
%   y : precision-point absolute microdynamic error
%       y = base + z + x + thermal_deformation

N = p.N;
dt = p.dt;

x = zeros(1,N);
xVel = zeros(1,N);
z = zeros(1,N);
zVel = zeros(1,N);
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
frictionScale = zeros(1,N);

% Control observer/filter states
integralError = 0;
TstructEst = p.Tcal;
dResidual = 0;

for k = 1:N-1
    % -------- Current low-temperature material / actuator parameters --------
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
    frictionScale(k) = p.Fc0*(1 + p.frictionColdCoeff*max(0, -Tact(k)-5));

    % Ice slightly increases projected flow area.
    areaScale = 1 + p.iceAreaCoeff*min(iceMass(k), 3.0);
    Fpayload = areaScale*env.Fpayload(k) + 0.25*env.Fspray(k);
    Fdeck = areaScale*env.Fdeck(k) + 0.80*env.Fspray(k);

    % -------- Controller --------
    if enableControl
        j = max(1, k-p.delaySteps);
        latency = (k-j)*dt;

        % Encoder + deck sensors include delay/noise. A first-order state
        % prediction compensates the known 10 ms signal-chain latency.
        xMeasDelayed = x(j) + env.noise.x(j) ...
            + 0.08e-6*(Tact(j)-p.Tcal);
        xVelMeasDelayed = xVel(j) + env.noise.xVel(j);
        xPred = xMeasDelayed + xVelMeasDelayed*latency;
        xVelPred = xVelMeasDelayed;

        supportDelayed = env.base(j) + z(j) + env.noise.support(j);
        supportVelDelayed = env.baseVel(j) + zVel(j) + env.noise.supportVel(j);
        supportPred = supportDelayed + supportVelDelayed*latency;
        supportVelPred = supportVelDelayed;

        % Temperature sensor/filter and thermal-strain compensation.
        TstructEst = TstructEst + dt/(0.45+dt)* ...
            ((Tstruct(j)+env.noise.temp(j))-TstructEst);
        thermalEst = p.alphaEff*p.Lthermal*(TstructEst-p.Tcal);

        % Desired relative-stage motion cancels support motion + thermal drift.
        xRef = -(supportPred + thermalEst);
        xRefVel = -supportVelPred;

        % Temperature-adaptive model used by feedforward compensation.
        Eest = 1 + p.EtempCoeff*(20-TstructEst);
        Eest = min(1.08, max(0.96, Eest));
        kaEst = p.ka0*Eest;
        caEst = p.ca0*(1 + p.stageDampColdCoeff*max(0, -TstructEst-5));

        % Flow feedforward: only the predictable drag/VIV portion is used;
        % turbulence and spray impacts remain unmeasured disturbances.
        FflowEst = 0.65*(env.FdragPayload(j)+env.FvortexPayload(j));
        FflowEst = FflowEst*(1 + env.noise.flow(j));

        % Composite control: model feedforward + tracking feedback + slow
        % residual-force correction. Moderate gains preserve delay robustness.
        e = xRef - xPred;
        eVel = xRefVel - xVelPred;
        integralError = integralError + e*dt;
        integralError = min(p.intLimit, max(-p.intLimit, integralError));

        uFF = kaEst*xRef + caEst*xRefVel - FflowEst;

        dTarget = p.residualGain*e;
        dResidual = dResidual + dt/(p.residualTau+dt)*(dTarget-dResidual);
        dResidual = min(p.residualLimit, max(-p.residualLimit, dResidual));
        residualForce(k) = dResidual;

        uRequested = uFF + p.Kp*e + p.Kd*eVel ...
            + p.Ki*integralError + dResidual;
        uCmd(k) = min(p.Fmax, max(-p.Fmax, uRequested));
    else
        uCmd(k) = 0;
        residualForce(k) = 0;
    end

    % -------- Low-temperature actuator lag, saturation and slew-rate limit --------
    du = (uCmd(k)-uAct(k))/actuatorTau(k);
    du = min(p.forceRateMax, max(-p.forceRateMax, du));
    uAct(k+1) = uAct(k) + dt*du;
    uAct(k+1) = min(p.Fmax, max(-p.Fmax, uAct(k+1)));

    % -------- Structural / mechanical dynamics --------
    Fc = frictionScale(k);
    Ffriction = Fc*tanh(xVel(k)/p.frictionVelocity);

    zAcc = (Fdeck ...
        - cd*zVel(k) - kd*z(k) ...
        - p.reactionRatio*uAct(k) ...
        - md*env.baseAcc(k))/md;

    supportAcc = env.baseAcc(k) + zAcc;
    xAcc = (uAct(k) + Fpayload ...
        - ca*xVel(k) - ka*x(k) - Ffriction ...
        - p.me*supportAcc)/p.me;

    % Semi-implicit Euler is robust for this small, stiff coupled system.
    zVel(k+1) = zVel(k) + dt*zAcc;
    z(k+1) = z(k) + dt*zVel(k+1);
    xVel(k+1) = xVel(k) + dt*xAcc;
    x(k+1) = x(k) + dt*xVel(k+1);

    % -------- Icing --------
    freezeFactor = (-env.Tair(k)-2)/25;
    freezeFactor = min(1, max(0, freezeFactor));
    dmIce = p.iceRate*env.sprayIndex(k)*freezeFactor;
    if Tstruct(k) > 0
        dmIce = dmIce - 2.0e-4*Tstruct(k)*iceMass(k);
    end
    iceMass(k+1) = max(0, iceMass(k) + dt*dmIce);

    % -------- Thermal dynamics --------
    hStruct = 7 + 1.25*env.wind(k);
    hAct = 6 + 0.90*env.wind(k);

    % Structure: convection + electronics + spray/seawater heat exchange.
    QstructInternal = 25;
    Qspray = 18*env.sprayIndex(k)*(p.Tsea-Tstruct(k));
    TstructDot = (hStruct*p.Astruct*(env.Tair(k)-Tstruct(k)) ...
        + QstructInternal + Qspray)/p.Cstruct;

    % Actuator: convection + standby heat + thermostat heater + mechanical loss.
    if Tact(k) < -17
        heaterPower(k) = 190;
    elseif Tact(k) < -14
        heaterPower(k) = 60;
    else
        heaterPower(k) = 0;
    end
    Qmotion = 0.06*abs(uAct(k)*xVel(k));
    QactInternal = 35 + heaterPower(k) + Qmotion;
    TactDot = (hAct*p.Aact*(env.Tair(k)-Tact(k)) ...
        + QactInternal + 0.08*(Tstruct(k)-Tact(k)))/p.Cact;

    Tstruct(k+1) = Tstruct(k) + dt*TstructDot;
    Tact(k+1) = Tact(k) + dt*TactDot;

    % Output precision-point error at current step.
    y(k) = env.base(k) + z(k) + x(k) + thermalDef(k);
end

% Final samples
thermalDef(end) = p.alphaEff*p.Lthermal*(Tstruct(end)-p.Tcal);
y(end) = env.base(end) + z(end) + x(end) + thermalDef(end);
stageStiffnessRatio(end) = stageStiffnessRatio(end-1);
deckNaturalFreq(end) = deckNaturalFreq(end-1);
actuatorTau(end) = actuatorTau(end-1);
frictionScale(end) = frictionScale(end-1);
heaterPower(end) = heaterPower(end-1);
uCmd(end) = uCmd(end-1);
residualForce(end) = residualForce(end-1);

out.controlEnabled = enableControl;
out.x = x;
out.xVel = xVel;
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
metrics.saturationFraction = mean(abs(controlled.uActuator) >= 0.98*p.Fmax);
metrics.controlWorkProxy_J = sum(abs(controlled.uActuator.*controlled.xVel))*p.dt;
end

%% ========================================================================
function plot_multiphysics_fields(p, env, controlled, outputFile)
t = p.t;
fig = figure('Color','w','Position',[60 45 1500 920]);

subplot(3,2,1);
plot(t, env.Tair, 'LineWidth',1.2); hold on;
plot(t, controlled.Tstruct, 'LineWidth',1.4);
plot(t, controlled.Tactuator, 'LineWidth',1.4);
grid on; box on;
ylabel('温度 / ^\circC');
title('低温热环境与设备温度');
legend('环境','结构','执行器','Location','best');

subplot(3,2,2);
plot(t, env.wind, 'LineWidth',1.2); hold on;
plot(t, env.windDirDeg, 'LineWidth',1.0);
grid on; box on;
ylabel('风速 / m/s；风向 / deg');
title('非平稳风场：阵风 + 风向摆动');
legend('风速','相对风向','Location','best');

subplot(3,2,3);
plot(t, env.Fpayload, 'LineWidth',1.0); hold on;
plot(t, env.Fspray, 'LineWidth',0.9);
grid on; box on;
ylabel('扰动力 / N');
title('流场扰动：风载/涡激/湍流 + 喷溅冲击');
legend('设备流场合力','喷溅冲击','Location','best');

subplot(3,2,4);
plot(t, env.base*1e6, 'LineWidth',1.0); hold on;
plot(t, controlled.deckFlex*1e6, 'LineWidth',1.0);
grid on; box on;
ylabel('位移 / \mum');
title('固体场：基座微振动与甲板柔性响应');
legend('刚性甲板输入','局部柔性挠度','Location','best');

subplot(3,2,5);
plot(t, controlled.thermalDeformation*1e6, 'LineWidth',1.3); hold on;
plot(t, controlled.iceMass, 'LineWidth',1.3);
grid on; box on;
xlabel('时间 / s');
ylabel('热变形 / \mum；结冰质量 / kg');
title('热收缩与结冰累积');
legend('热变形','结冰质量','Location','best');

subplot(3,2,6);
plot(t, controlled.stageStiffnessRatio, 'LineWidth',1.3); hold on;
plot(t, controlled.deckNaturalFreq/p.fd0, 'LineWidth',1.3);
plot(t, controlled.actuatorTau/p.tauAct0, 'LineWidth',1.3);
grid on; box on;
xlabel('时间 / s');
ylabel('相对名义值');
title('低温引起的参数漂移');
legend('执行级刚度','甲板固有频率','执行器时间常数','Location','best');

sgtitle(['耐低温甲板设备：流-固-热多场扰动及参数演化' newline ...
    '随机风/涡激/喷溅 + 甲板柔性 + 热收缩 + 结冰 + 低温执行器退化'], ...
    'FontWeight','bold');
set(fig,'PaperPositionMode','auto');
print(fig, outputFile, '-dpng', '-r220');
end

%% ========================================================================
function plot_control_response(p, env, passive, controlled, metrics, outputFile)
t = p.t;
passive_um = passive.output*1e6;
controlled_um = controlled.output*1e6;

winSec = 2.0;
nwin = max(5, round(winSec/p.dt));
rmsPassive = moving_rms(passive_um, nwin);
rmsControlled = moving_rms(controlled_um, nwin);

% Choose a window around the strongest gust for detailed inspection.
[~, iPeakWind] = max(env.wind);
tCenter = t(iPeakWind);
t1 = max(0, tCenter-8);
t2 = min(p.Tsim, tCenter+12);
idxZoom = (t >= t1 & t <= t2);

fig = figure('Color','w','Position',[65 40 1500 920]);

subplot(2,2,1);
plot(t, passive_um, 'LineWidth',0.85); hold on;
plot(t, controlled_um, 'LineWidth',1.15);
grid on; box on;
xlabel('时间 / s');
ylabel('精密点误差 / \mum');
title('全时程微动态响应');
legend('无补偿','多场补偿控制','Location','best');

subplot(2,2,2);
plot(t(idxZoom), passive_um(idxZoom), 'LineWidth',1.0); hold on;
plot(t(idxZoom), controlled_um(idxZoom), 'LineWidth',1.25);
grid on; box on;
xlabel('时间 / s');
ylabel('精密点误差 / \mum');
title(sprintf('强阵风附近局部响应（%.1f–%.1f s）', t1, t2));
legend('无补偿','多场补偿控制','Location','best');

subplot(2,2,3);
plot(t, rmsPassive, 'LineWidth',1.5); hold on;
plot(t, rmsControlled, 'LineWidth',1.7);
grid on; box on;
xlabel('时间 / s');
ylabel('滑动 RMS / \mum');
title(sprintf('局部运动强度（%.1f s 滑动 RMS）', winSec));
legend('无补偿','多场补偿控制','Location','best');

subplot(2,2,4);
plot(t, controlled.uActuator, 'LineWidth',1.0); hold on;
plot(t, controlled.residualCompensation, 'LineWidth',1.0);
grid on; box on;
xlabel('时间 / s');
ylabel('力 / N');
title('控制力与低带宽残差补偿');
legend('执行器实际力','残差补偿力','Location','best');

sgtitle(sprintf(['低温流-固-热多场扰动下的补偿微动态控制结果\n' ...
    'RMS：%.1f \mum \rightarrow %.1f \mum；衰减 %.1f dB；峰值：%.1f \mum \rightarrow %.1f \mum'], ...
    metrics.passiveRms_um, metrics.controlledRms_um, metrics.attenuation_dB, ...
    metrics.passivePeak_um, metrics.controlledPeak_um), 'FontWeight','bold');

set(fig,'PaperPositionMode','auto');
print(fig, outputFile, '-dpng', '-r220');
end

%% ========================================================================
function plot_control_performance(p, passive, controlled, metrics, outputFile)
[f1, asdPassive] = one_sided_asd(passive.output, p.fs);
[f2, asdControlled] = one_sided_asd(controlled.output, p.fs);

fig = figure('Color','w','Position',[80 45 1450 880]);

subplot(2,2,1);
semilogy(f1, asdPassive*1e6, 'LineWidth',1.1); hold on;
semilogy(f2, asdControlled*1e6, 'LineWidth',1.3);
grid on; box on;
xlim([0.05 30]);
xlabel('频率 / Hz');
ylabel('ASD / (\mum/\surdHz)');
title('频域响应：多频段扰动抑制');
legend('无补偿','多场补偿控制','Location','best');

subplot(2,2,2);
bar([metrics.passiveRms_um, metrics.controlledRms_um; ...
     metrics.passivePeak_um, metrics.controlledPeak_um; ...
     metrics.passiveP95_um, metrics.controlledP95_um]);
grid on; box on;
set(gca,'XTickLabel',{'RMS','绝对峰值','95%|响应|'});
ylabel('位移 / \mum');
title('关键微动态指标');
legend('无补偿','多场补偿控制','Location','best');

subplot(2,2,3);
plot(p.t, controlled.Tactuator, 'LineWidth',1.2); hold on;
plot(p.t, 1000*controlled.actuatorTau, 'LineWidth',1.2);
plot(p.t, controlled.frictionScale, 'LineWidth',1.2);
grid on; box on;
xlabel('时间 / s');
ylabel('温度 / ^\circC；\tau / ms；摩擦尺度 / N');
title('低温执行机构状态');
legend('执行器温度','力环时间常数(ms)','摩擦力尺度','Location','best');

subplot(2,2,4);
axis off;
text(0.04,0.90,sprintf('RMS 衰减倍数：%.2f×',metrics.rmsRatio), ...
    'FontSize',12,'FontWeight','bold');
text(0.04,0.77,sprintf('RMS 衰减：%.2f dB',metrics.attenuation_dB), ...
    'FontSize',12);
text(0.04,0.64,sprintf('控制后 RMS：%.2f \mum',metrics.controlledRms_um), ...
    'FontSize',12);
text(0.04,0.51,sprintf('控制后峰值：%.2f \mum',metrics.controlledPeak_um), ...
    'FontSize',12);
text(0.04,0.38,sprintf('控制力 RMS：%.1f N',metrics.controlRms_N), ...
    'FontSize',12);
text(0.04,0.25,sprintf('控制力峰值：%.1f N',metrics.controlPeak_N), ...
    'FontSize',12);
text(0.04,0.12,sprintf('饱和占比：%.3f %%',100*metrics.saturationFraction), ...
    'FontSize',12);
title('综合性能指标');

sgtitle('耐低温甲板设备补偿微动态控制：频域与性能评估', ...
    'FontWeight','bold');
set(fig,'PaperPositionMode','auto');
print(fig, outputFile, '-dpng', '-r220');
end

%% ========================================================================
function y = colored_noise(N, dt, tau)
% First-order filtered white noise, normalized to zero mean / unit std.
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
% Toolbox-free one-sided amplitude spectral density using a Hann window.
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
