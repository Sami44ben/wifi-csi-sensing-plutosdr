% clc; clear; close all;
%% wifi_rx.m — Receiver + CSI extraction (hardware, fully inline)
%  Run this AFTER wifi_tx.m is transmitting on the TX radio.
%  Produces CSI_with_timestamps: [Npackets x (1 + numTones)] complex.

%% ── Interactive startup prompts ─────────────────────────────────────────
fprintf('\n+--------------------------------------------+\n');
fprintf('|         Wi-Fi CSI Sensing  --  RX          |\n');
fprintf('+--------------------------------------------+\n\n');

durInput = input('  Capture duration (seconds) [default 30]: ', 's');
if isempty(strtrim(durInput))
    captureDuration_s = 30;
else
    captureDuration_s = str2double(durInput);
    if isnan(captureDuration_s) || captureDuration_s <= 0
        warning('Invalid input — using default 30 s.');
        captureDuration_s = 30;
    end
end

plotInput = input('  Enable live plot? (y/n) [default n]: ', 's');
if isempty(strtrim(plotInput))
    enablePlot = false;
else
    enablePlot = strcmpi(strtrim(plotInput), 'y');
end

fprintf('\n  Starting with: duration = %g s,  plot = %s\n\n', ...
    captureDuration_s, mat2str(enablePlot));

%% ── Config ─────────────────────────────────────────────────────────────
wifi_config;          % runs the config script; populates CFG in this workspace
CFG.run.captureDuration_s = captureDuration_s;   % override with prompt values
CFG.viz.enablePlot        = enablePlot;

%% ── PHY config (inline) ────────────────────────────────────────────────
cfgPHY = wlanHESUConfig('ChannelBandwidth', CFG.phy.BW, ...
                        'MCS',              CFG.phy.mcs, ...
                        'APEPLength',       CFG.phy.APEPLength);
Fs_Hz = wlanSampleRate(cfgPHY);
Fc_Hz = wlanChannelFrequency(CFG.phy.channelNum, CFG.phy.freqGHz);

%% ── OFDM layout ─────────────────────────────────────────────────────────
ofdmInfo     = wlanHEOFDMInfo('HE-LTF', cfgPHY);
activeFFTIdx = ofdmInfo.ActiveFFTIndices;
Nfft         = ofdmInfo.FFTLength;
dcBin        = Nfft/2 + 1;
scIdx        = activeFFTIdx - dcBin;
scIdx(scIdx==0) = [];
numTones     = numel(scIdx);                 % 242 @ CBW20, 484 @ CBW40

reorder = [find(scIdx<0).'  find(scIdx>0).'];

SamplesPerFrame = CFG.run.SamplesPerFrame;
numFFTPoints    = CFG.viz.numFFTPoints;

% Per-packet stride in samples (one reference packet), used to walk through
% every packet in a captured frame AND as the slow-time PRI for Doppler.
rng(42);
refBits   = randi([0 1], CFG.phy.APEPLength*8, 1);
refPkt    = wlanWaveformGenerator(refBits, cfgPHY, 'NumPackets', 1, ...
                                  'ScramblerInitialization', CFG.phy.scrInit);
pktStride = numel(refPkt) + round(CFG.burst.idleTime_us*1e-6*Fs_Hz);

%% ── Hardware setup ──────────────────────────────────────────────────────
rx = sdrrx('Pluto', ...
    'RadioID',            CFG.hw.rxID, ...
    'CenterFrequency',    Fc_Hz, ...
    'BasebandSampleRate', Fs_Hz, ...
    'GainSource',         CFG.hw.rxGainMode, ...
    'Gain',               CFG.hw.rxGain_dB, ...
    'SamplesPerFrame',    SamplesPerFrame, ...
    'OutputDataType',     'double');

%% ── Buffers ─────────────────────────────────────────────────────────────
indHE       = wlanFieldIndices(cfgPHY);
historyCols = CFG.viz.historyCols;
wfCols      = CFG.viz.wfCols;

% Output matrix layout (one row per detected packet):
%   col 1         : complex(timestamp_s, 0)  — real part = elapsed seconds
%   cols 2..(1+T) : complex CSI per subcarrier (T = numTones)
% Preallocated, then trimmed to packetCount at the end.
CSI_with_timestamps = complex(zeros(CFG.run.preallocRows, 1 + numTones));

%% ── Figure setup (inline, plot mode only) ──────────────────────────────
if CFG.viz.enablePlot
    csiBuffer   = nan(numTones, historyCols, 'single');
    wf          = nan(numFFTPoints, wfCols, 'single'); wfWrite = 0;
    fAxisMHz    = linspace(-Fs_Hz/2, Fs_Hz/2, numFFTPoints)/1e6;
    magBaseline = ones(1, numTones, 'single');

    hFig = figure('Name','Wi-Fi RX: CSI | DD | Waterfall','NumberTitle','off');
    set(hFig, 'Position', [60 60 1500 560], 'Color','w');
    tiledlayout(1,3,'TileSpacing','compact','Padding','compact');

    nexttile(1);
    [xCSI, yCSI] = meshgrid(1:historyCols, scIdx);
    hSurfCSI = surf(xCSI, yCSI, nan(numTones, historyCols,'single'), ...
        'EdgeColor','none','FaceColor','interp');
    shading interp; view(2); axis tight;
    xlabel('Packet Index'); ylabel('Subcarrier (DC-centered)');
    title('CSI |H| (Corrected)'); cbCSI = colorbar; ylabel(cbCSI,'|H|');

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
    title('RF Waterfall'); cbWF = colorbar; ylabel(cbWF,'dB');
    drawnow;
end

%% ── CLI banner ──────────────────────────────────────────────────────────
fprintf('+--------------------------------------------+\n');
fprintf('|  SDR      : %s |\n', CFG.hw.rxID);
fprintf('|  Fc       : %.4f GHz                     |\n', Fc_Hz/1e9);
fprintf('|  BW       : %-6s  Tones: %-4d           |\n', CFG.phy.BW, numTones);
fprintf('|  Fs       : %.3f MHz                     |\n', Fs_Hz/1e6);
fprintf('|  Duration : %g s                          |\n', CFG.run.captureDuration_s);
fprintf('|  Plot     : %-5s                          |\n', mat2str(CFG.viz.enablePlot));
fprintf('+--------------------------------------------+\n\n');

%% ── Main loop ───────────────────────────────────────────────────────────
spinChars       = {'|','/','-','\'};
spinIdx         = 1;
lastPrintTime   = -inf;
lastSpinTime    = 0;
SPIN_INTERVAL_s = 0.15;

haveFirstPacket = false;
firstPktTime    = NaN;
packetCount     = 0;            % also the row index into CSI_with_timestamps
frameCount      = 0;
elapsed_s       = 0;
UI_UPDATE_EVERY = CFG.run.UI_UPDATE_EVERY;
t0 = tic;

while true

    elapsed_wall = toc(t0);

    %% Exit: capture duration reached
    if elapsed_wall >= CFG.run.captureDuration_s
        if CFG.viz.enablePlot
            set(hSurfWF,  'ZData', wf(:, [wfWrite+1:wfCols, 1:wfWrite]));
            set(hSurfCSI, 'ZData', csiBuffer(reorder,:));
            drawnow;
        end
        fprintf('\r  [DONE   ]  t=%6.1fs  pkts=%-7d  %.0f pkt/s  tones=%d    ', ...
            elapsed_wall, packetCount, packetCount/max(elapsed_wall,0.001), numTones);
        fprintf('\n'); break;
    end

    %% Exit: figure closed
    if CFG.viz.enablePlot && ~ishandle(ancestor(hSurfWF,'figure'))
        fprintf('\r  [STOPPED]  t=%6.1fs  pkts=%-7d  %.0f pkt/s  tones=%d    ', ...
            elapsed_wall, packetCount, packetCount/max(elapsed_wall,0.001), numTones);
        fprintf('\n'); break;
    end

    %% CLI: periodic status line
    if (elapsed_wall - lastPrintTime) >= CFG.cli.printEvery_s
        fprintf('\r  [RUNNING]  t=%6.1fs  pkts=%-7d  %.0f pkt/s  tones=%d    ', ...
            elapsed_wall, packetCount, packetCount/max(elapsed_wall,0.001), numTones);
        lastPrintTime = elapsed_wall;
        spinIdx = 1;
    end

    %% CLI: spinner
    if CFG.cli.useSpinner && (elapsed_wall - lastSpinTime) >= SPIN_INTERVAL_s
        fprintf('\r  %s  capturing...', spinChars{spinIdx});
        spinIdx      = mod(spinIdx, numel(spinChars)) + 1;
        lastSpinTime = elapsed_wall;
    end

    %% Fetch frame
    buf = rx();
    if isempty(buf)
        if CFG.viz.enablePlot, drawnow; end
        continue;
    end

    %% 1) Waterfall (plot mode only)
    if CFG.viz.enablePlot
        Nslice  = min(numel(buf), SamplesPerFrame);
        spec    = 20*log10(abs(fftshift(fft(buf(1:Nslice), numFFTPoints))) + 1e-7);
        wfWrite = mod(wfWrite, wfCols) + 1;
        wf(:, wfWrite) = single(spec);
        wfIdx   = [wfWrite+1:wfCols, 1:wfWrite];
    end

    %% 2) Detect & process EVERY packet in this frame (uniform slow-time)
    searchStart = 1;
    while true
        if (numel(buf) - searchStart + 1) < pktStride, break; end

        off = wlanPacketDetect(buf(searchStart:end), cfgPHY.ChannelBandwidth);
        if isempty(off), break; end
        pktStart = searchStart + off;                 % absolute, 1-based

        iH1 = pktStart + indHE.HELTF(1) - 1;
        jH2 = pktStart + indHE.HELTF(2) - 1;
        if jH2 > numel(buf), break; end

        %% 3) CFO correction from legacy preamble (inline), on a local copy
        bufP = buf;
        if CFG.proc.applyCFO && isfield(indHE,'LSTF') && isfield(indHE,'LLTF')
            iLS = pktStart + indHE.LSTF(1) - 1; jLS = pktStart + indHE.LSTF(2) - 1;
            iLT = pktStart + indHE.LLTF(1) - 1; jLT = pktStart + indHE.LLTF(2) - 1;
            cCFO = 0; fCFO = 0;
            if iLS >= 1 && jLS <= numel(bufP) && exist('wlanCoarseCFOEstimate','file')==2
                cCFO = wlanCoarseCFOEstimate(bufP(iLS:jLS), cfgPHY.ChannelBandwidth);
            end
            if iLT >= 1 && jLT <= numel(bufP) && exist('wlanFineCFOEstimate','file')==2
                nFine   = single(0:(jLT-iLT)).';
                rxLLTFc = bufP(iLT:jLT) .* exp(-1j*single(2*pi*cCFO/Fs_Hz).*nFine);
                fCFO    = wlanFineCFOEstimate(rxLLTFc, cfgPHY.ChannelBandwidth);
            end
            totCFO = cCFO + fCFO;
            if abs(totCFO) > 0
                nCFO = single(0:(jH2-iH1)).';
                bufP(iH1:jH2) = bufP(iH1:jH2) .* exp(-1j*single(2*pi*totCFO/Fs_Hz).*nCFO);
            end
        end

        %% 4) HE-LTF -> raw CSI
        heltfsym = wlanHEDemodulate(bufP(iH1:jH2), "HE-LTF", cfgPHY);
        chEstHE  = wlanHELTFChannelEstimate(heltfsym, cfgPHY);
        if ndims(chEstHE) == 3, h = chEstHE(:,1,1); else, h = chEstHE(:); end
        h = h(:);
        if numel(h) > numTones
            h = h(1:numTones);
        elseif numel(h) < numTones
            h = [h; zeros(numTones-numel(h), 1, 'like', h)];
        end

        %% 5) Phase corrections (CPE + SFO), inline
        h = h(:).';
        if CFG.proc.applyCPE
            h = h .* exp(-1j*angle(mean(h)));
        end
        if CFG.proc.applySFO
            xsc   = scIdx(:);
            ph    = unwrap(angle(h)).';
            phHat = polyval(polyfit(xsc, ph, 1), xsc);
            h     = h .* exp(-1j*phHat.');
        end
        h_corr = h(:);

        %% 6) Timestamp
        now_s = toc(t0);
        if ~haveFirstPacket, firstPktTime = now_s; haveFirstPacket = true; end
        elapsed_s = now_s - firstPktTime;

        %% 7) Store [timestamp | h_corr...] (grow prealloc if needed)
        packetCount = packetCount + 1;
        if packetCount > size(CSI_with_timestamps, 1)
            CSI_with_timestamps = [CSI_with_timestamps; ...
                complex(zeros(CFG.run.preallocRows, 1 + numTones))]; %#ok<AGROW>
        end
        CSI_with_timestamps(packetCount, :) = ...
            [complex(double(elapsed_s), 0),  double(h_corr(:).')];

        %% 8) Rolling CSI image buffer (viz only)
        if CFG.viz.enablePlot
            h_mag = abs(h_corr).';
            if CFG.proc.applyNorm
                magBaseline = (1-CFG.proc.normAlpha)*magBaseline + ...
                               CFG.proc.normAlpha*single(h_mag);
                normMag = h_mag ./ max(magBaseline, 1e-6);
            else
                normMag = h_mag / max(rms(h_mag), eps);
            end
            colIndex = mod(packetCount-1, size(csiBuffer,2)) + 1;
            csiBuffer(:, colIndex) = single(normMag(:));
        end

        searchStart = pktStart + pktStride;            % skip to the next packet
    end

    %% 9) UI update ONCE per frame
    frameCount = frameCount + 1;
    if CFG.viz.enablePlot && mod(frameCount, UI_UPDATE_EVERY) == 0
        set(hSurfWF,  'ZData', wf(:, wfIdx));
        set(hSurfCSI, 'ZData', csiBuffer(reorder,:));

        if packetCount > 0
            Kdop          = CFG.viz.historyCols;
            CSI_corrected = CSI_with_timestamps(1:packetCount, 2:end);  % valid rows
            X  = double(CSI_corrected(:, reorder));
            Nt = size(X,1);
            if Nt >= Kdop, X = X(end-Kdop+1:end, :);
            else,          X = [zeros(Kdop-Nt, size(X,2)); X];
            end
            winTime = hann(Kdop);
            winFreq = hann(size(X,2)).';
            X       = X .* (winTime * winFreq);
            D   = ifftshift(ifft(X, size(X,2)*4, 2), 2);
            Z   = fftshift(fft(D,  Kdop*4,        1), 1);
            set(hSurfDD, 'ZData', single(abs(Z.').^2));
        end
        drawnow limitrate;
    end

    %% Hard stop by packet-elapsed time
    if haveFirstPacket && elapsed_s >= CFG.run.captureDuration_s
        if CFG.viz.enablePlot
            set(hSurfWF,  'ZData', wf(:, [wfWrite+1:wfCols, 1:wfWrite]));
            set(hSurfCSI, 'ZData', csiBuffer(reorder,:));
            drawnow;
        end
        fprintf('\r  [DONE   ]  t=%6.1fs  pkts=%-7d  %.0f pkt/s  tones=%d    ', ...
            elapsed_s, packetCount, packetCount/max(elapsed_s,0.001), numTones);
        fprintf('\n'); break;
    end
end

%% ── Shutdown ────────────────────────────────────────────────────────────
release(rx);

% Trim preallocated rows down to what was actually captured
CSI_with_timestamps = CSI_with_timestamps(1:packetCount, :);

Npkts = size(CSI_with_timestamps, 1);
fprintf('\n');
fprintf('+--------------------------------------------+\n');
fprintf('|   CAPTURE SUMMARY : CSI_with_timestamps    |\n');
fprintf('+--------------------------------------------+\n');
fprintf('|  Packets collected   : %-6d              |\n', Npkts);
fprintf('|  Subcarriers/packet  : %-6d  (%s)     |\n', numTones, CFG.phy.BW);
fprintf('|  Matrix size         : [%d x %d]       |\n', Npkts, 1+numTones);
fprintf('+--------------------------------------------+\n');

%% ── Post-run inspection (paste in Command Window) ───────────────────────
%   t = real(CSI_with_timestamps(:,1));
%   H = CSI_with_timestamps(:, 2:end);
%   figure; imagesc(t, 1:numTones, abs(H).');
%   xlabel('Time (s)'); ylabel('Subcarrier'); title('|CSI|'); colorbar;
%   save('csi_dataset.mat', 'CSI_with_timestamps', 'scIdx');