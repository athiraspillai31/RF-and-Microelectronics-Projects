%% ========================================================================
%  Data Converters - Complex Task 1: SAR ADC
%  Section 4 - Evaluation and Analysis  (v2 - auto signal logging)
%  ------------------------------------------------------------------------
%  This version does NOT assume any particular block/signal names in your
%  model. Instead it:
%    1) Programmatically turns ON logging for EVERY signal line in the
%       model (no manual "Log Selected Signal" clicking needed).
%    2) After each sim() run, inspects simOut.logsout, prints every
%       available signal name it found, and tries to auto-pick the
%       "digital output code" and "analog input" signals by keyword.
%    3) If auto-pick fails, it stops and shows you the list so you can
%       tell me the right names -> I hardcode them for you.
%
%  Requires: MATLAB + Simulink, model file Task1_SARADC.slx on the path.
% ========================================================================

clear; clc; close all;

%% ------------------------------------------------------------------
%  SECTION 0: General parameters
%  ------------------------------------------------------------------
nbit    = 8;
clkper  = 10e-9;
Tconv   = 9*clkper;
LSB     = 1/2^nbit;

modelName = 'Task1_SARADC';      % <-- your .slx name, no extension

fprintf('=== SAR ADC Evaluation Script (v2) ===\n');
fprintf('nbit = %d, clkper = %.3g s, Tconv = %.3g s, LSB = %.6f\n\n', ...
         nbit, clkper, Tconv, LSB);

runSim = exist(strcat(modelName,'.slx'), 'file') == 4 || ...
         exist(strcat(modelName,'.mdl'), 'file') == 4;

if ~runSim
    error(['Model "%s.slx" not found on the MATLAB path.\n' ...
           'Put run_evaluation_v2.m in the SAME FOLDER as %s.slx, ' ...
           'or add that folder to the path (addpath).'], modelName, modelName);
end

load_system(modelName);

% --- Enable logging on EVERY signal line in the model automatically ---
set_param(modelName, 'SignalLogging', 'on', ...
                      'SignalLoggingName', 'logsout', ...
                      'SaveFormat', 'Dataset');
allLines = find_system(modelName, 'FindAll', 'on', 'type', 'line');
nLogged = 0;
for i = 1:numel(allLines)
    try
        set_param(allLines(i), 'DataLogging', 'on');
        nLogged = nLogged + 1;
    catch
        % some lines (e.g. inside library blocks) can't be tagged; skip
    end
end
fprintf('Enabled logging on %d/%d signal lines in "%s".\n\n', ...
         nLogged, numel(allLines), modelName);


%% ==========================================================================
%  4.1  INPUT RANGE FOR MAX QUANTIZATION ERROR <= LSB/2   (analytical only)
%  ==========================================================================
Vin_min = 0;
Vin_max = 1 - LSB/2;

fprintf('--- 4.1 Input range for max quant. error <= LSB/2 ---\n');
fprintf('LSB           = %.6f (normalized, Vref = 1)\n', LSB);
fprintf('Vin_min       = %.6f\n', Vin_min);
fprintf('Vin_max       = %.6f  (= 1 - LSB/2)\n\n', Vin_max);


%% ==========================================================================
%  4.2  COEFFICIENT SETS + SLEW RATE  (analytical)
%  ==========================================================================
coeff_ideal = 2.^(-nbit:1:-1);

msb_err_pct = 0.01;
coeff_msb_plus     = coeff_ideal; coeff_msb_plus(1)  = coeff_ideal(1)*(1+msb_err_pct);
coeff_msb_minus    = coeff_ideal; coeff_msb_minus(1) = coeff_ideal(1)*(1-msb_err_pct);

lsb_err_pct = 0.10;
coeff_lsb_plus     = coeff_ideal; coeff_lsb_plus(end)  = coeff_ideal(end)*(1+lsb_err_pct);
coeff_lsb_minus    = coeff_ideal; coeff_lsb_minus(end) = coeff_ideal(end)*(1-lsb_err_pct);

SlewRate_max = LSB / Tconv;
slope_max    = SlewRate_max;
T_ramp       = 2^nbit * Tconv;

fprintf('--- 4.2 Max ramp slew rate (no missing codes) ---\n');
fprintf('slope_max = LSB/Tconv = %.6g (normalized units/s)\n', SlewRate_max);
fprintf('Recommended ramp duration T_ramp = %d * Tconv = %.4g s\n\n', 2^nbit, T_ramp);

% Push everything the model might need into base workspace
assignin('base','coeff',      coeff_ideal);
assignin('base','clkper',     clkper);
assignin('base','slope_max',  slope_max);
assignin('base','T_ramp',     T_ramp);


%% --- Run once with the RAMP scenario active in your model, inspect logsout ---
fprintf('--- Running simulation (ramp scenario) to discover signal names ---\n');
simOut = sim(modelName, 'StopTime', num2str(T_ramp));

if ~isprop(simOut,'logsout') && ~ismember('logsout', simOut.who)
    error(['No "logsout" found after simulation. This means the model has\n' ...
           'no loggable signal lines reachable from the top level (e.g. all\n' ...
           'signals are inside masked/library subsystems that block logging).\n' ...
           'Open the model, right-click the digital output wire directly and\n' ...
           'the analog input wire, choose "Log Selected Signal" manually on\n' ...
           'each, then re-run this script.']);
end

logsout = simOut.get('logsout');
names = logsout.getElementNames;
fprintf('\nFound %d logged signals:\n', numel(names));
for i = 1:numel(names)
    fprintf('  [%d] %s\n', i, names{i});
end
fprintf('\n');

% --- Auto-pick candidates by keyword ---
codeSig = pick_signal(logsout, {'code','out','digital','sa','q'});
vinSig  = pick_signal(logsout, {'vin','in','analog','ramp'});

if isempty(codeSig)
    fprintf(2, ['\n*** Could not auto-detect the DIGITAL OUTPUT CODE signal.\n' ...
                'Look at the list above and tell me the exact name (e.g. "%s")\n' ...
                'so I can hardcode it. ***\n\n'], names{1});
else
    fprintf('Auto-picked digital output signal: "%s"\n', codeSig.Name);
end
if isempty(vinSig)
    fprintf(2, ['\n*** Could not auto-detect the ANALOG INPUT signal.\n' ...
                'Look at the list above and tell me the exact name. ***\n\n']);
else
    fprintf('Auto-picked analog input signal:   "%s"\n\n', vinSig.Name);
end

%% --- If both found, proceed with INL analysis across coefficient sets ---
coeffSets = {coeff_ideal, coeff_msb_plus, coeff_msb_minus, coeff_lsb_plus, coeff_lsb_minus};
setNames  = {'Ideal','MSB+1%','MSB-1%','LSB+10%','LSB-10%'};
INL_results = cell(size(coeffSets));

if ~isempty(codeSig)
    for i = 1:numel(coeffSets)
        assignin('base','coeff', coeffSets{i});
        simOut_i = sim(modelName, 'StopTime', num2str(T_ramp));
        logsout_i = simOut_i.get('logsout');
        sigEl = logsout_i.getElement(codeSig.Name);
        codes = double(sigEl.Values.Data(:));
        t     = sigEl.Values.Time(:);

        vin_ideal  = min(t/T_ramp, 1);
        code_ideal = min(floor(vin_ideal*2^nbit), 2^nbit-1);
        INL = codes - code_ideal;
        INL_results{i} = struct('t',t,'codes',codes,'INL',INL);

        missing = setdiff(0:2^nbit-1, unique(round(codes)));
        fprintf('[%s] missing codes: %s\n', setNames{i}, mat2str(missing));
    end

    figure('Name','INL Comparison'); hold on; grid on;
    for i = 1:numel(INL_results)
        if ~isempty(INL_results{i})
            stairs(INL_results{i}.codes, INL_results{i}.INL, 'DisplayName', setNames{i});
        end
    end
    xlabel('Digital Code'); ylabel('INL [LSB]');
    title('INL: Ideal vs +-1% MSB error vs 10% LSB error'); legend show;
end


%% ==========================================================================
%  4.3 / 4.4 / 4.5  SINE + SNR   (only runs if you confirm signal names)
%  ==========================================================================
vin_dc  = 0.5; vin_amp = 0.5;
fs   = 1/(clkper*9);
Nfft = 2^12;
Mcycles  = 61;
f_signal = Mcycles*fs/Nfft;
Tsim = Nfft*clkper*9;
n = nbit; SNR_theory = 6.02*n + 1.78;

fprintf('--- 4.3 Coherent sampling params ---\n');
fprintf('f_signal = %.6g Hz, Nfft = %d, Mcycles = %d, Tsim = %.6g s\n', ...
         f_signal, Nfft, Mcycles, Tsim);
fprintf('--- 4.4 Theoretical ideal SNR = %.2f dB ---\n\n', SNR_theory);

assignin('base','vin_dc',   vin_dc);
assignin('base','vin_amp',  vin_amp);
assignin('base','f_signal', f_signal);

if ~isempty(codeSig)
    coeffTest = {coeff_ideal, coeff_msb_plus, coeff_msb_minus, coeff_lsb_plus, coeff_lsb_minus};
    labelTest = {'Ideal','MSB+1%','MSB-1%','LSB+10%','LSB-10%'};
    fprintf('--- 4.4/4.5 SNR results ---\n');
    for i = 1:numel(coeffTest)
        assignin('base','coeff', coeffTest{i});
        simOut_i = sim(modelName, 'StopTime', num2str(Tsim));
        logsout_i = simOut_i.get('logsout');
        sigEl = logsout_i.getElement(codeSig.Name);
        code = double(sigEl.Values.Data(:));
        x = code; if max(code) > 1.5, x = code/2^nbit; end
        SNR_i = compute_snr_fft(x, Nfft, Mcycles);
        fprintf('  %-10s : SNR = %.2f dB (theory ideal: %.2f dB, Delta = %.2f dB)\n', ...
                 labelTest{i}, SNR_i, SNR_theory, SNR_i - SNR_theory);
    end
else
    fprintf(2, ['4.3-4.5 skipped: digital output signal not auto-detected.\n' ...
                'Tell me the correct signal name from the list above.\n']);
end


%% ==========================================================================
%  LOCAL FUNCTIONS
%  ==========================================================================
function sig = pick_signal(logsout, keywords)
% Search logsout element names for the first that contains any keyword
% (case-insensitive). Returns the Simulink.SimulationData.Signal object,
% or [] if none match.
    sig = [];
    names = logsout.getElementNames;
    for k = 1:numel(keywords)
        for i = 1:numel(names)
            if contains(lower(names{i}), lower(keywords{k}))
                sig = logsout.getElement(names{i});
                return;
            end
        end
    end
end

function SNR_dB = compute_snr_fft(x, Nfft, Msignal)
    x = x(1:Nfft);
    x = x - mean(x);
    X = fft(x, Nfft);
    Pxx = abs(X(1:Nfft/2+1)).^2;
    sigBin = Msignal + 1;
    signalPower = Pxx(sigBin);
    noisePower  = sum(Pxx) - Pxx(1) - signalPower;
    SNR_dB = 10*log10(signalPower/noisePower);
end
