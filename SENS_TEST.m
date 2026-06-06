clc; clear; close all;

%% ===================== USER CONFIG =====================================
CFG = struct();

% --- Run mode
CFG.useHardware        = true;

% *** PLOT TOGGLE — set false for pure CSI collection (much faster) ***
CFG.viz.enablePlot     = true;

% --- PHY / Packet
% NOTE: tone count depends on bandwidth.
%   CBW20 -> 242 active HE tones  
%   CBW40 -> 484 active HE tones
% The pipeline uses numTones dynamically, so either works.
CFG.phy.standard       = "HE";
CFG.phy.BW             = 'CBW40';
CFG.phy.mcs            = 1;
CFG.phy.APEPLength     = 100;
CFG.phy.scrInit        = 93;
CFG.phy.freqGHz        = 2.4;
CFG.phy.channelNum     = 5;

% --- Burst / generator
CFG.burst.numPackets   = 500;
CFG.burst.idleTime_us  = 10;

% --- Live processing toggles
CFG.proc.applyCFO      = true;
CFG.proc.applyCPE      = true;   % set false for cleanest target Doppler (see note)
CFG.proc.applySFO      = true;   % set false for cleanest target delay  (see note)
CFG.proc.applyNorm     = true;
CFG.proc.normAlpha     = 0.02;

% --- Timing / framing
CFG.run.captureDuration_s = 20;
CFG.run.SamplesPerFrame   = 65536;
CFG.run.UI_UPDATE_EVERY   = 5;
CFG.run.preallocRows      = 5000;    % CSI matrix preallocation (grows if exceeded)

% --- Waterfall / DD display (only used when plotting)
CFG.viz.historyCols   = 500;
CFG.viz.wfCols        = 300;
CFG.viz.numFFTPoints  = 2048;

% --- CLI progress indicator
CFG.cli.printEvery_s  = 2;     % print a status line every N seconds
CFG.cli.useSpinner    = false;  % animated spinner between status lines

% --- Offline synthetic channel (only used when useHardware = false)
CFG.sim.SNR_dB       = 20;
CFG.sim.CFO_Hz       = 0;
CFG.sim.loopReps     = 6;       % buffer length in bursts; large enough that the
                                % 500-packet DD window stays inside one pass
CFG.sim.chunkSamples = 250000;  % simulator working block size; lower if RAM is tight
CFG.sim.roundTrip    = true;    % true: monostatic (tau=2R/c, fD=2V*Fc/c)
CFG.sim.includeLOS   = true;    % static reference path (keep ON for validation)
CFG.sim.losGain      = 1.0;

% Targets: range (m), radial velocity (m/s, + = approaching), gain (reflectivity)
CFG.sim.targets(1).range_m = 6;   CFG.sim.targets(1).velocity_mps =  +8;  CFG.sim.targets(1).gain = 0.5;
CFG.sim.targets(2).range_m = 15;  CFG.sim.targets(2).velocity_mps = -12;  CFG.sim.targets(2).gain = 0.3;
% add as many as you like:
% CFG.sim.targets(3).range_m = 25;  CFG.sim.targets(3).velocity_mps = +4;  CFG.sim.targets(3).gain = 0.2;

% --- Hardware (Pluto SDR)
CFG.hw.txID       = 'sn: 1044730a1997001610002a0036067c6324';
CFG.hw.rxID       = 'sn: 10447318ac0f001606000f00677f17a2c5';
CFG.hw.rxGainMode = 'AGC Fast Attack';
CFG.hw.rxGain_dB  = 50;
CFG.hw.txGain_dB  = 0;

%% ===================== DERIVED PHY / OBJECTS ===========================
[cfgPHY, ~, Fs_Hz] = makePHYConfig(CFG.phy);

ofdmInfo     = wlanHEOFDMInfo('HE-LTF', cfgPHY);
activeFFTIdx = ofdmInfo.ActiveFFTIndices;
Nfft         = ofdmInfo.FFTLength;
dcBin        = Nfft/2 + 1;
scIdx        = activeFFTIdx - dcBin;
scIdx(scIdx==0) = [];
numTones     = numel(scIdx);

reorder = [find(scIdx<0).'  find(scIdx>0).'];

SamplesPerFrame = CFG.run.SamplesPerFrame;
numFFTPoints    = CFG.viz.numFFTPoints;
Fc_Hz           = wlanChannelFrequency(CFG.phy.channelNum, CFG.phy.freqGHz);

%% ===================== TX WAVEFORM =====================================
rng(42);
bits  = randi([0 1], CFG.phy.APEPLength*8, 1);
txsig = generateHESUBurst(bits, cfgPHY, CFG.burst.numPackets, CFG.phy.scrInit, ...
                          CFG.burst.idleTime_us, Fs_Hz);
txsig = txsig / rms(txsig);

% Per-packet stride in samples: used to walk through every packet in a frame
% AND as the TRUE slow-time PRI for the Doppler axis / expectation printout.
pktStride = round(numel(txsig)/CFG.burst.numPackets) ...
          + round(CFG.burst.idleTime_us*1e-6*Fs_Hz);

%% ===================== I/O ABSTRACTION =================================
if CFG.useHardware
    tx = sdrtx('Pluto', ...
        'RadioID',            CFG.hw.txID, ...
        'CenterFrequency',    Fc_Hz, ...
        'BasebandSampleRate', Fs_Hz, ...
        'Gain',               CFG.hw.txGain_dB);

    rx = sdrrx('Pluto', ...
        'RadioID',            CFG.hw.rxID, ...
        'CenterFrequency',    Fc_Hz, ...
        'BasebandSampleRate', Fs_Hz, ...
        'GainSource',         CFG.hw.rxGainMode, ...
        'Gain',               CFG.hw.rxGain_dB, ...
        'SamplesPerFrame',    SamplesPerFrame, ...
        'OutputDataType',     'double');

    transmitRepeat(tx, txsig);
    pause(0.3);
    rxRead = @() rx();
else
    % ---- Controllable synthetic channel (offline validation) ----
    simSig = buildSyntheticRxStream(txsig, CFG.sim, Fs_Hz, Fc_Hz);
    reportSyntheticDD(CFG.sim, Fs_Hz, Fc_Hz, pktStride, CFG.viz.historyCols);
    simRead_next('init', simSig, SamplesPerFrame);
    rxRead   = @() simRead_next();
end

%% ===================== BUFFERS =========================================
indHE       = wlanFieldIndices(cfgPHY);
historyCols = CFG.viz.historyCols;
wfCols      = CFG.viz.wfCols;

% Output matrix layout (rows grow by one per DETECTED packet):
%   col 1            : complex(timestamp_s, 0)   — real part = elapsed seconds
%   cols 2..(1+T)    : complex CSI per subcarrier (T = numTones, e.g. 484 @ CBW40)
% Preallocated, then trimmed to packetCount at the end.
CSI_with_timestamps = complex(zeros(CFG.run.preallocRows, 1 + numTones));

if CFG.viz.enablePlot
    csiBuffer   = nan(numTones, historyCols, 'single');
    wf          = nan(numFFTPoints, wfCols, 'single'); wfWrite = 0;
    fAxisMHz    = linspace(-Fs_Hz/2, Fs_Hz/2, numFFTPoints)/1e6;
    magBaseline = ones(1, numTones, 'single');
    [hSurfWF, hSurfCSI, hSurfDD] = makeFigure(fAxisMHz, numFFTPoints, wfCols, ...
                                              csiBuffer, scIdx);
end

%% ===================== CLI INDICATOR SETUP =============================
spinChars       = {'|','/','-','\'};
spinIdx         = 1;
lastPrintTime   = -inf;   % force immediate first print
lastSpinTime    = 0;
SPIN_INTERVAL_s = 0.15;

fprintf('\n');
fprintf('+------------------------------------------+\n');
fprintf('|      Wi-Fi CSI Sensing -- STARTING       |\n');
fprintf('|  BW: %-6s  Tones: %3d  Fc: %.3f GHz |\n', ...
    CFG.phy.BW, numTones, Fc_Hz/1e9);
fprintf('|  Duration: %4d s      Plot: %-3s         |\n', ...
    CFG.run.captureDuration_s, mat2str(CFG.viz.enablePlot));
fprintf('+------------------------------------------+\n\n');

%% ===================== MAIN LOOP =======================================
haveFirstPacket = false;
firstPktTime    = NaN;
packetCount     = 0;          % also the row index into CSI_with_timestamps
frameCount      = 0;
elapsed_s       = 0;
UI_UPDATE_EVERY = CFG.run.UI_UPDATE_EVERY;
t0 = tic;

while true

    elapsed_wall = toc(t0);

    % --- Exit conditions
    if elapsed_wall >= CFG.run.captureDuration_s
        if CFG.viz.enablePlot
            pushUI(hSurfWF, hSurfCSI, hSurfDD, wf, wfWrite, wfCols, csiBuffer, reorder);
        end
        printStatus('DONE   ', elapsed_wall, packetCount, numTones, '');
        fprintf('\n');
        break;
    end
    if CFG.viz.enablePlot && ~ishandle(ancestor(hSurfWF,'figure'))
        printStatus('STOPPED', elapsed_wall, packetCount, numTones, '');
        fprintf('\n');
        break;
    end

    % --- CLI: periodic full status line (overwrites spinner line)
    if (elapsed_wall - lastPrintTime) >= CFG.cli.printEvery_s
        printStatus('RUNNING', elapsed_wall, packetCount, numTones, '');
        lastPrintTime = elapsed_wall;
        spinIdx = 1;
    end

    % --- CLI: spinner between status lines
    if CFG.cli.useSpinner && (elapsed_wall - lastSpinTime) >= SPIN_INTERVAL_s
        fprintf('\r  %s  capturing...', spinChars{spinIdx});
        spinIdx      = mod(spinIdx, numel(spinChars)) + 1;
        lastSpinTime = elapsed_wall;
    end

    % --- Fetch frame
    buf = rxRead();
    if isempty(buf)
        if CFG.viz.enablePlot, drawnow; end
        continue;
    end

    % 1) Waterfall (only when plotting)
    if CFG.viz.enablePlot
        Nslice  = min(numel(buf), SamplesPerFrame);
        spec    = 20*log10(abs(fftshift(fft(buf(1:Nslice), numFFTPoints))) + 1e-7);
        wfWrite = mod(wfWrite, wfCols) + 1;
        wf(:, wfWrite) = single(spec);
        wfIdx   = [wfWrite+1:wfCols, 1:wfWrite];
    end

    % 2) Detect & process EVERY packet in this frame (uniform slow-time)
    searchStart = 1;
    while true
        if (numel(buf) - searchStart + 1) < pktStride, break; end

        off = wlanPacketDetect(buf(searchStart:end), cfgPHY.ChannelBandwidth);
        if isempty(off), break; end
        pktStart = searchStart + off;            % absolute, 1-based

        iH1 = pktStart + indHE.HELTF(1) - 1;
        jH2 = pktStart + indHE.HELTF(2) - 1;
        if jH2 > numel(buf), break; end

        % CFO correction (operates on a local copy; only the HELTF slice is read)
        bufP = buf;
        if CFG.proc.applyCFO && isfield(indHE,'LSTF') && isfield(indHE,'LLTF')
            bufP = applyCFOFromLegacy(bufP, pktStart, indHE, cfgPHY, Fs_Hz, iH1, jH2);
        end

        % HE-LTF -> CSI
        heltfsym = wlanHEDemodulate(bufP(iH1:jH2), "HE-LTF", cfgPHY);
        chEstHE  = wlanHELTFChannelEstimate(heltfsym, cfgPHY);
        if ndims(chEstHE) == 3, h = chEstHE(:,1,1); else, h = chEstHE(:); end
        h = h(:);
        if numel(h) ~= numTones, h = padOrTrim(h, numTones); end

        % Per-packet corrections
        h_corr = postCSI(h, scIdx, CFG.proc.applyCPE, CFG.proc.applySFO);

        % Timestamp
        now_s = toc(t0);
        if ~haveFirstPacket, firstPktTime = now_s; haveFirstPacket = true; end
        elapsed_s = now_s - firstPktTime;

        % Store [timestamp | numTones complex CSI] (grow prealloc if needed)
        packetCount = packetCount + 1;
        if packetCount > size(CSI_with_timestamps, 1)
            CSI_with_timestamps = [CSI_with_timestamps; ...
                complex(zeros(CFG.run.preallocRows, 1 + numTones))]; %#ok<AGROW>
        end
        CSI_with_timestamps(packetCount, :) = ...
            [complex(double(elapsed_s), 0),  double(h_corr(:).')];

        % Rolling CSI image buffer (viz only)
        if CFG.viz.enablePlot
            h_mag = abs(h_corr).';
            if CFG.proc.applyNorm
                magBaseline = (1-CFG.proc.normAlpha)*magBaseline + CFG.proc.normAlpha*single(h_mag);
                normMag = h_mag ./ max(magBaseline, 1e-6);
            else
                normMag = h_mag / max(rms(h_mag), eps);
            end
            colIndex = mod(packetCount - 1, size(csiBuffer,2)) + 1;
            csiBuffer(:, colIndex) = single(normMag(:));
        end

        searchStart = pktStart + pktStride;      % skip to the next packet
    end

    % 3) UI update ONCE per frame
    frameCount = frameCount + 1;
    if CFG.viz.enablePlot && mod(frameCount, UI_UPDATE_EVERY) == 0
        set(hSurfWF,  'ZData', wf(:, wfIdx));
        set(hSurfCSI, 'ZData', csiBuffer(reorder,:));

        if packetCount > 0
            Kdop          = CFG.viz.historyCols;
            CSI_corrected = CSI_with_timestamps(1:packetCount, 2:end);   % valid rows only
            X  = double(CSI_corrected(:, reorder));
            Nt = size(X,1);
            if Nt >= Kdop, X = X(end-Kdop+1:end, :);
            else,          X = [zeros(Kdop-Nt, size(X,2)); X];
            end
            winTime = hann(Kdop);
            winFreq = hann(size(X,2)).';
            X       = X .* (winTime * winFreq);
            Ndelay  = size(X,2) * 4;
            Ndop    = Kdop * 4;
            D   = ifftshift(ifft(X, Ndelay, 2), 2);
            Z   = fftshift(fft(D, Ndop, 1), 1);
            Zdd = abs(Z.').^2;
            set(hSurfDD, 'ZData', single(Zdd));
        end
        drawnow limitrate;
    end

    % --- Hard stop by packet-elapsed time
    if haveFirstPacket && elapsed_s >= CFG.run.captureDuration_s
        if CFG.viz.enablePlot
            pushUI(hSurfWF, hSurfCSI, hSurfDD, wf, wfWrite, wfCols, csiBuffer, reorder);
        end
        printStatus('DONE   ', elapsed_s, packetCount, numTones, '');
        fprintf('\n');
        break;
    end
end

%% ===================== SHUTDOWN / SUMMARY ==============================
if CFG.useHardware
    release(tx);
    release(rx);
end

% Trim preallocated rows down to what was actually captured
CSI_with_timestamps = CSI_with_timestamps(1:packetCount, :);

Npkts = size(CSI_with_timestamps, 1);
fprintf('+--------------------------------------------------+\n');
fprintf('|   CAPTURE SUMMARY : CSI_with_timestamps          |\n');
fprintf('+--------------------------------------------------+\n');
fprintf('|  Packets collected  : %-6d                     |\n', Npkts);
fprintf('|  Subcarriers/packet : %-6d                     |\n', numTones);
fprintf('|  Matrix size        : [%6d x %4d]            |\n', Npkts, 1+numTones);
fprintf('+--------------------------------------------------+\n');

%% ===================== POST-RUN INSPECTION =============================
% Run these lines in the Command Window after capture:
%
%   t = real(CSI_with_timestamps(:,1));
%   H = CSI_with_timestamps(:, 2:end);      % [Npackets x numTones] complex
%
%   figure; imagesc(t, 1:size(H,2), abs(H).');
%   xlabel('Time (s)'); ylabel('Subcarrier'); title('|CSI|'); colorbar;
%
%   % Save for AI pipeline:
%   save('csi_dataset.mat', 'CSI_with_timestamps', 'scIdx');

%% ===================== LOCAL FUNCTIONS =================================

function rx = buildSyntheticRxStream(txsig, sim, Fs_Hz, Fc_Hz)
% Memory-lean synthetic multipath radar channel.
% Same physics as a continuous delay+Doppler channel, but avoids full-size
% repeated delayed copies. The stored offline stream is complex single;
% simRead_next casts each returned frame to double for WLAN receiver functions.

    c  = 299792458;                          % speed of light [m/s]
    kR = 2;
    if ~sim.roundTrip
        kR = 1;
    end

    if ~isfield(sim, 'loopReps') || isempty(sim.loopReps)
        sim.loopReps = 1;
    end
    reps = max(1, round(sim.loopReps));

    if ~isfield(sim, 'chunkSamples') || isempty(sim.chunkSamples)
        sim.chunkSamples = 2^20;
    end
    chunkSamples = max(1024, round(sim.chunkSamples));

    tx  = single(complex(txsig(:)));          % keep simulator storage small
    Ltx = numel(tx);
    N   = Ltx * reps;
    rx  = complex(zeros(N, 1, 'single'));

    % Static LOS/clutter. Build by indexing instead of repmat(...).
    if sim.includeLOS
        losGain = single(sim.losGain);
        for r = 1:reps
            i1 = (r-1)*Ltx + 1;
            i2 = r*Ltx;
            rx(i1:i2) = rx(i1:i2) + losGain * tx;
        end
    end

    % One burst time index. Avoid a full N-sample n vector.
    nBurst = single((0:Ltx-1).');

    for p = 1:numel(sim.targets)
        tg     = sim.targets(p);
        tauSmp = kR * tg.range_m      / c * Fs_Hz;  % delay [samples]
        fD     = kR * tg.velocity_mps * Fc_Hz / c;  % Doppler [Hz]
        wD     = single(2*pi*fD/Fs_Hz);
        gain   = single(tg.gain);

        txDelay = fracDelay(tx, tauSmp);

        % cos/sin keeps the vector complex single (exp() can promote to double).
        phBurst      = wD * nBurst;
        burstDoppler = complex(cos(phBurst), sin(phBurst));
        targetBurst  = gain * (txDelay .* burstDoppler);

        % Repeat the delayed/Doppler burst with CONTINUOUS phase across reps,
        % so the global Doppler phase is exp(1j*wD*n) end to end.
        for r = 1:reps
            i1 = (r-1)*Ltx + 1;
            i2 = r*Ltx;
            ph0    = wD * single(i1-1);
            phase0 = complex(cos(ph0), sin(ph0));
            rx(i1:i2) = rx(i1:i2) + phase0 * targetBurst;
        end

        clear txDelay burstDoppler targetBurst phBurst
    end

    % Optional receiver CFO, also without a full N-sample time vector.
    if sim.CFO_Hz ~= 0
        wCFO    = single(2*pi*sim.CFO_Hz/Fs_Hz);
        phBurst = wCFO * nBurst;
        burstCFO = complex(cos(phBurst), sin(phBurst));
        for r = 1:reps
            i1 = (r-1)*Ltx + 1;
            i2 = r*Ltx;
            ph0    = wCFO * single(i1-1);
            phase0 = complex(cos(ph0), sin(ph0));
            rx(i1:i2) = rx(i1:i2) .* (phase0 * burstCFO);
        end
    end

    % Add measured complex AWGN in chunks (avoids a full-length noise vector).
    rx = addMeasuredAWGN(rx, sim.SNR_dB, chunkSamples);
end

function y = fracDelay(x, d)
% Delay x by d >= 0 samples using windowed-sinc fractional interpolation.
% Preserves the class of x and avoids concatenating large vectors.

    x = x(:);
    if d <= 0
        y = x;
        return;
    end

    nInt = floor(d);
    frac = d - nInt;

    if frac > 0
        L  = 8;
        m  = (-L:L).';
        sv = m - frac;

        h  = ones(size(sv));
        nz = (sv ~= 0);
        h(nz) = sin(pi*sv(nz)) ./ (pi*sv(nz));

        M = 2*L + 1;
        w = 0.5 - 0.5*cos(2*pi*(0:M-1).'/(M-1));

        h = h .* w;
        h = h / sum(h);
        h = cast(h, class(real(x)));          % keep single input as single
        x = conv(x, h, 'same');               % same length as x
    end

    y = zeros(size(x), 'like', x);
    if nInt < numel(x)
        y((nInt+1):end) = x(1:(end-nInt));
    end
end

function y = addMeasuredAWGN(x, SNR_dB, chunkSamples)
% Add complex AWGN at measured SNR without allocating a full-length noise vector.

    y = x;
    N = numel(y);

    pSig = 0;
    for i1 = 1:chunkSamples:N
        i2 = min(i1 + chunkSamples - 1, N);
        seg = y(i1:i2);
        pSig = pSig + double(sum(abs(seg).^2));
    end
    pSig = pSig / max(N, 1);

    pNoise = pSig / (10^(SNR_dB/10));
    sigma  = sqrt(pNoise/2);

    useSingle = isa(real(y), 'single');

    for i1 = 1:chunkSamples:N
        i2 = min(i1 + chunkSamples - 1, N);
        M  = i2 - i1 + 1;
        if useSingle
            noise = single(sigma) * complex(randn(M,1,'single'), randn(M,1,'single'));
        else
            noise = sigma * complex(randn(M,1), randn(M,1));
        end
        y(i1:i2) = y(i1:i2) + noise;
    end
end

function reportSyntheticDD(sim, Fs_Hz, Fc_Hz, pktLen, Kdop)
% Print expected delay/Doppler per target + resolution/ambiguity limits.
% pktLen is the TRUE slow-time PRI in samples (= per-packet stride), so these
% numbers match what the delay-Doppler map actually shows.
    c  = 299792458;
    kR = 2;
    if ~sim.roundTrip
        kR = 1;
    end

    PRI  = pktLen / Fs_Hz;                     % packet repetition interval [s]
    vMax = (1/(2*PRI))    * c / (kR*Fc_Hz);    % unambiguous radial velocity [m/s]
    vRes = (1/(Kdop*PRI)) * c / (kR*Fc_Hz);    % velocity resolution [m/s]
    rRes = c / (2*Fs_Hz);                      % approximate range resolution [m]

    fprintf('\n--- Synthetic channel expectation ---\n');
    fprintf('PRI=%.3f ms | v_unamb=+/-%.1f m/s | v_res~%.2f m/s | r_res~%.2f m\n', ...
            PRI*1e3, vMax, vRes, rRes);

    for p = 1:numel(sim.targets)
        tg  = sim.targets(p);
        tau = kR * tg.range_m / c;
        fD  = kR * tg.velocity_mps * Fc_Hz / c;

        fprintf('  T%d: R=%5.1f m  V=%+6.1f m/s  ->  tau=%5.1f ns  fD=%+7.1f Hz\n', ...
                p, tg.range_m, tg.velocity_mps, tau*1e9, fD);

        if abs(tg.velocity_mps) > vMax
            fprintf('       !! |V| > v_unamb -> aliases in Doppler\n');
        end
        if abs(tg.velocity_mps) < vRes
            fprintf('       !! |V| < v_res -> may merge with the zero-Doppler line\n');
        end
    end
    fprintf('-------------------------------------\n\n');
end

function printStatus(state, elapsed, pkts, tones, ~)
% Overwrite current line with a compact status update
    rate = pkts / max(elapsed, 0.001);
    fprintf('\r  [%s]  t=%6.1fs  pkts=%-7d  %.0f pkt/s  tones=%d    ', ...
        state, elapsed, pkts, rate, tones);
end

function [cfg, Fc_Hz, Fs_Hz] = makePHYConfig(p)
    switch upper(string(p.standard))
        case "HE"
            cfg = wlanHESUConfig('ChannelBandwidth', p.BW, ...
                                  'MCS',              p.mcs, ...
                                  'APEPLength',       p.APEPLength);
        otherwise
            error('Unsupported PHY standard: %s', p.standard);
    end
    Fc_Hz = wlanChannelFrequency(p.channelNum, p.freqGHz);
    Fs_Hz = wlanSampleRate(cfg);
end

function sig = generateHESUBurst(bits, cfg, numPkts, scrInit, idleTime_us, Fs_Hz)
    try
        sig = wlanWaveformGenerator(bits, cfg, ...
            'NumPackets', numPkts, ...
            'ScramblerInitialization', scrInit, ...
            'IdleTime', idleTime_us*1e-6);
    catch
        onePkt = wlanWaveformGenerator(bits, cfg, ...
                   'NumPackets', 1, 'ScramblerInitialization', scrInit);
        gap = zeros(round(idleTime_us*1e-6*Fs_Hz),1);
        sig = [];
        for k = 1:numPkts
            sig = [sig; onePkt]; %#ok<AGROW>
            if k < numPkts, sig = [sig; gap]; end %#ok<AGROW>
        end
    end
end

function buf = applyCFOFromLegacy(buf, pktStart, indHE, cfg, Fs_Hz, iH1, jH2)
    iLS = pktStart + indHE.LSTF(1) - 1; jLS = pktStart + indHE.LSTF(2) - 1;
    iLT = pktStart + indHE.LLTF(1) - 1; jLT = pktStart + indHE.LLTF(2) - 1;
    cCFO = 0; fCFO = 0;
    if iLS >= 1 && jLS <= numel(buf) && exist('wlanCoarseCFOEstimate','file')==2
        cCFO = wlanCoarseCFOEstimate(buf(iLS:jLS), cfg.ChannelBandwidth);
    end
    if iLT >= 1 && jLT <= numel(buf) && exist('wlanFineCFOEstimate','file')==2
        nFine   = single(0:(jLT - iLT)).';
        wFine   = single(2*pi*cCFO/Fs_Hz);
        rxLLTFc = buf(iLT:jLT) .* exp(-1j*wFine.*nFine);
        fCFO    = wlanFineCFOEstimate(rxLLTFc, cfg.ChannelBandwidth);
    end
    totCFO = cCFO + fCFO;
    if abs(totCFO) > 0
        nCFO = single(0:(jH2 - iH1)).';
        wCFO = single(2*pi*totCFO/Fs_Hz);
        buf(iH1:jH2) = buf(iH1:jH2) .* exp(-1j*wCFO.*nCFO);
    end
end

function hCorr = postCSI(hIn, subcarrierIdxCentered, applyCPE, applySFO)
    h = hIn(:).';
    if applyCPE
        phi = angle(mean(h));
        h   = h .* exp(-1j*phi);
    end
    if applySFO
        xsc   = subcarrierIdxCentered(:);
        ph    = unwrap(angle(h)).';
        P     = polyfit(xsc, ph, 1);
        phHat = polyval(P, xsc);
        h     = h .* exp(-1j*phHat.');
    end
    hCorr = h(:);
end

function v = padOrTrim(v, N)
    v = v(:);
    if numel(v) > N,     v = v(1:N);
    elseif numel(v) < N, v = [v; zeros(N-numel(v), 'like', v)];
    end
end

function [hSurfWF, hSurfCSI, hSurfDD] = makeFigure(fAxisMHz, numFFTPoints, wfCols, csiBuffer, scIdx)
    hFig = figure('Name','Wi-Fi: CSI | Delay-Doppler | Waterfall','NumberTitle','off');
    set(hFig, 'Position', [60 60 1500 560], 'Color','w');
    tiledlayout(1,3,'TileSpacing','compact','Padding','compact');

    nexttile(1);
    [xCSI, yCSI] = meshgrid(1:size(csiBuffer,2), scIdx);
    hSurfCSI = surf(xCSI, yCSI, nan(size(csiBuffer),'single'), ...
        'EdgeColor','none','FaceColor','interp');
    shading interp; view(2); axis tight;
    xlabel('Packet Index'); ylabel('Subcarrier (DC-centered)');
    title('CSI |H| (Corrected)'); cb1 = colorbar; ylabel(cb1,'|H|');

    nexttile(2);
    hSurfDD = surf(nan(2,2),'EdgeColor','none','FaceColor','interp');
    shading interp; view(2); axis tight;
    xlabel('Doppler bins'); ylabel('Delay bins');
    title('Delay-Doppler'); cbDD = colorbar; ylabel(cbDD,'magnitude');

    nexttile(3);
    [xWF, yWF] = meshgrid(1:wfCols, fAxisMHz);
    hSurfWF = surf(xWF, yWF, nan(numFFTPoints, wfCols,'single'), ...
        'EdgeColor','none','FaceColor','interp');
    shading interp; view(2); axis tight;
    xlabel('Time Steps'); ylabel('Frequency (MHz)');
    title('RF Waterfall (raw)'); cb2 = colorbar; ylabel(cb2,'dB');
    drawnow;
end

function pushUI(hSurfWF, hSurfCSI, ~, wf, wfWrite, wfCols, csiBuffer, reorder)
    set(hSurfWF,  'ZData', wf(:, [wfWrite+1:wfCols, 1:wfWrite]));
    set(hSurfCSI, 'ZData', csiBuffer(reorder,:));
    drawnow;
end

function y = simRead_next(mode, sig, spf)
    persistent SIM
    if nargin >= 1 && (ischar(mode) || isstring(mode))
        SIM.sig = sig; SIM.spf = spf; SIM.idx = 1;
        y = []; return;
    end
    if isempty(SIM) || ~isfield(SIM,'sig') || isempty(SIM.sig)
        y = []; return;
    end
    idx = SIM.idx; spf = SIM.spf; s = SIM.sig;
    if idx+spf-1 <= numel(s)
        y = s(idx:idx+spf-1); SIM.idx = idx + spf;
    else
        tail = s(idx:end); need = spf - numel(tail);
        y = [tail; s(1:need)]; SIM.idx = need + 1;
    end

    % The offline synthetic stream is stored as complex single to save RAM.
    % Cast only this returned frame to double for WLAN receiver calls.
    if isa(y, 'single')
        y = double(y);
    end
end