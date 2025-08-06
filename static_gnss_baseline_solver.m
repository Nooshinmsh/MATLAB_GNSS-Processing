%% GNSS Static Baseline Estimation using Single-Difference Pseudoranges
% This script estimates rover positions based on RINEX files using single-difference
% pseudorange observations between a base station and a rover receiver.
% Author: [Your Name]
% Date: [Today's Date]

%% USER INPUTS ----------------------------------------------------------
obsBaseFile = "203m0340.obs";    % Base station RINEX observation file
obsRovFile  = "_9390340.obs";    % Rover RINEX observation file
navFile     = "_9390340.nav";    % Navigation message file (shared)

% Base coordinates (from CSRS-PPP or known reference) - WGS84 (ITRF2020)
latBase = 10.833656825;   % Latitude in degrees
lonBase = -85.612487422;  % Longitude in degrees
hBase   = 301.224;        % Height in meters

%% SETUP ---------------------------------------------------------------
utmCrs = projcrs(32616); % UTM Zone 16N for projection
[baseECEF(1), baseECEF(2), baseECEF(3)] = geodetic2ecef(wgs84Ellipsoid, latBase, lonBase, hBase);

%% LOAD DATA -----------------------------------------------------------
obsBase = rinexread(obsBaseFile);
obsRov  = rinexread(obsRovFile);
nav     = rinexread(navFile);

gpsBase = obsBase.GPS;
gpsRov  = obsRov.GPS;
gpsNav  = nav.GPS;

% Find common epochs between base and rover
commonEpochs = intersect(gpsBase.Time, gpsRov.Time);
nEp = numel(commonEpochs);
fprintf("Processing %d common epochs...\n", nEp);

% Pre-allocate output arrays
lat = nan(nEp,1); lon = nan(nEp,1); h = nan(nEp,1);
east = nan(nEp,1); north = nan(nEp,1);

%% PROCESS EACH EPOCH --------------------------------------------------
for k = 1:nEp
    t = commonEpochs(k);
    
    % Extract observations at time t
    baseRows = gpsBase(gpsBase.Time == t, :);
    rovRows  = gpsRov(gpsRov.Time == t, :);

    % Use primary (C1C) or fallback (C5Q) pseudorange
    prBase = baseRows.C1C; if all(ismissing(prBase)), prBase = baseRows.C5Q; end
    prRov  = rovRows.C1C;  if all(ismissing(prRov)),  prRov  = rovRows.C5Q; end

    % Match PRNs (satellite IDs)
    prnBase = baseRows.SatelliteID;
    prnRov  = rovRows.SatelliteID;
    [commonPRNs, iB, iR] = intersect(prnBase, prnRov);

    prBase = prBase(iB);
    prRov  = prRov(iR);

    if numel(commonPRNs) < 4
        continue;  % Skip if fewer than 4 satellites in common
    end

    try
        % Get satellite positions
        satPosAll = gnssconstellation(t, gpsNav);
        [~, iSat] = ismember(commonPRNs, gpsNav.SatelliteID);
        satPos = satPosAll(iSat, :);

        % Single-difference pseudorange
        sdPr = prRov - prBase;  % Clock bias cancels

        % Linearized observation model A * dx = b
        A = zeros(numel(commonPRNs), 3);
        b = zeros(numel(commonPRNs), 1);

        for j = 1:numel(commonPRNs)
            rhoVec = satPos(j,:) - baseECEF;
            uVec = rhoVec / norm(rhoVec);
            A(j,:) = uVec;
            b(j) = sdPr(j);
        end

        % Estimate rover ECEF offset using least squares
        dx = A \ b;
        rovECEF = baseECEF + dx';

        % Convert to geographic coordinates and UTM
        [lat(k), lon(k), h(k)] = ecef2geodetic(wgs84Ellipsoid, rovECEF(1), rovECEF(2), rovECEF(3));
        [east(k), north(k)] = projfwd(utmCrs, lat(k), lon(k));
    catch
        continue;  % Skip this epoch on any error
    end
end

%% SAVE RESULTS --------------------------------------------------------
valid = ~isnan(lat);
T = table;
T.Time      = commonEpochs(valid);
T.Lat       = lat(valid);
T.Lon       = lon(valid);
T.Height_m  = h(valid);
T.UTM_E     = east(valid);
T.UTM_N     = north(valid);

writetable(T, "rover_baseline_positions.csv");
fprintf(" %d epochs written to rover_baseline_positions.csv\n", height(T));

