clc; clear; close all;
%% wifi_tx.m — Transmitter only (hardware, fully inline)
%  Run this on the session connected to the TX radio (SDR_B).
%  Generates the HE-SU burst and streams it in a continuous DMA loop until
%  Start this BEFORE wifi_rx.m.
%  you press Ctrl+C or close the session. 

%% ── Config ─────────────────────────────────────────────────────────────
wifi_config;          % runs the config script, populates CFG in this workspace

%% ── PHY config (inline) ────────────────────────────────────────────────
cfgPHY = wlanHESUConfig('ChannelBandwidth', CFG.phy.BW, ...
                        'MCS',              CFG.phy.mcs, ...
                        'APEPLength',       CFG.phy.APEPLength);
Fs_Hz = wlanSampleRate(cfgPHY);
Fc_Hz = wlanChannelFrequency(CFG.phy.channelNum, CFG.phy.freqGHz);

%% ── Waveform generation (inline) ───────────────────────────────────────
% Fixed seed so the waveform is reproducible run to run.
rng(42);
bits = randi([0 1], CFG.phy.APEPLength*8, 1);

try
    txsig = wlanWaveformGenerator(bits, cfgPHY, ...
        'NumPackets',              CFG.burst.numPackets, ...
        'ScramblerInitialization', CFG.phy.scrInit, ...
        'IdleTime',                CFG.burst.idleTime_us*1e-6);
catch
    onePkt = wlanWaveformGenerator(bits, cfgPHY, ...
                'NumPackets', 1, 'ScramblerInitialization', CFG.phy.scrInit);
    gap = zeros(round(CFG.burst.idleTime_us*1e-6*Fs_Hz), 1);
    txsig = [];
    for k = 1:CFG.burst.numPackets
        txsig = [txsig; onePkt]; %#ok<AGROW>
        if k < CFG.burst.numPackets, txsig = [txsig; gap]; end %#ok<AGROW>
    end
end
txsig = txsig / rms(txsig);   % normalize RMS

%% ── Banner ─────────────────────────────────────────────────────────────
fprintf('\n');
fprintf('+--------------------------------------------+\n');
fprintf('|         Wi-Fi CSI Sensing  --  TX          |\n');
fprintf('+--------------------------------------------+\n');
fprintf('|  SDR     : %s  |\n', CFG.hw.txID);
fprintf('|  Fc      : %.4f GHz                     |\n', Fc_Hz/1e9);
fprintf('|  BW      : %-6s                          |\n', CFG.phy.BW);
fprintf('|  Fs      : %.3f MHz                     |\n', Fs_Hz/1e6);
fprintf('|  Gain    : %d dB                           |\n', CFG.hw.txGain_dB);
fprintf('|  Burst   : %d pkts x %d bytes             |\n', ...
    CFG.burst.numPackets, CFG.phy.APEPLength);
fprintf('|  Waveform: %.3f k samples                |\n', numel(txsig)/1e3);
fprintf('+--------------------------------------------+\n');

%% ── Hardware setup ─────────────────────────────────────────────────────
tx = sdrtx('Pluto', ...
    'RadioID',            CFG.hw.txID, ...
    'CenterFrequency',    Fc_Hz, ...
    'BasebandSampleRate', Fs_Hz, ...
    'Gain',               CFG.hw.txGain_dB);

%% ── Transmit loop ──────────────────────────────────────────────────────
fprintf('\n  Transmitting continuously — press Ctrl+C to stop.\n\n');

transmitRepeat(tx, txsig);   % DMA loop: runs until release() or Ctrl+C

% transmitRepeat keeps streaming in the background as long as the tx object
% lives in the workspace. Keep this session open while wifi_rx.m runs.
% To stop and free the radio, run:  release(tx)