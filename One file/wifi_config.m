%% wifi_config.m — shared parameters (plain script; populates CFG)
%  Both wifi_tx.m and wifi_rx.m run this via `wifi_config;` at startup, so
%  CFG lands directly in their workspace. Edit a value here once and both
%  scripts pick it up. No function wrapper — just variable assignments.

CFG = struct();

%% --- PHY ----------------------------------------------------------------
% Tone count depends on bandwidth:
%   CBW20 -> 242 active HE-LTF tones
%   CBW40 -> 484 active HE-LTF tones
% freqGHz is the BAND (2.4 / 5 / 6) passed to wlanChannelFrequency, NOT the
% literal RF centre. For a custom centre, set CenterFrequency on the radio.
CFG.phy.standard   = "HE";
CFG.phy.BW         = 'CBW40';
CFG.phy.mcs        = 4;
CFG.phy.APEPLength = 100;       % bytes — short packets = higher packet rate
CFG.phy.scrInit    = 93;
CFG.phy.freqGHz    = 2.4;       % band: 2.4, 5 or 6
CFG.phy.channelNum = 5;

%% --- Burst --------------------------------------------------------------
CFG.burst.numPackets  = 1000;
CFG.burst.idleTime_us = 0;

%% --- Hardware IDs -------------------------------------------------------
CFG.hw.rxID = 'sn: 1044730a1997001610002a0036067c6324';  % SDR_B (TX)
CFG.hw.txID = 'sn: 10447318ac0f001606000f00677f17a2c5';  % SDR_A (RX)

%% --- TX hardware -------------------------------------------------------
CFG.hw.txGain_dB = 0;

%% --- RX hardware -------------------------------------------------------
CFG.hw.rxGainMode = 'AGC Fast Attack';
CFG.hw.rxGain_dB  = 50;

%% --- RX framing / timing -----------------------------------------------
CFG.run.SamplesPerFrame   = 65536;
CFG.run.captureDuration_s = 300;
CFG.run.UI_UPDATE_EVERY   = 5;
CFG.run.preallocRows      = 5000;   % CSI matrix preallocation (grows if exceeded)

%% --- RX processing toggles ----------------------------------------------
CFG.proc.applyCFO  = true;
CFG.proc.applyCPE  = true;
CFG.proc.applySFO  = true;
CFG.proc.applyNorm = true;
CFG.proc.normAlpha = 0.02;

%% --- Visualisation (RX only) -------------------------------------------
CFG.viz.enablePlot   = true;   % false = pure collection, max throughput
CFG.viz.historyCols  = 500;
CFG.viz.wfCols       = 300;
CFG.viz.numFFTPoints = 2048;

%% --- CLI indicator (RX only) -------------------------------------------
CFG.cli.printEvery_s = 2;
CFG.cli.useSpinner   = true;