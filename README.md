# GNSS Static Baseline Estimation (MATLAB)

This MATLAB script estimates the position of a GNSS rover receiver relative to a fixed base station using single-difference pseudorange measurements. It processes RINEX observation and navigation files and outputs the estimated positions in both geodetic and UTM coordinates.

# Features

- Reads RINEX 3.x OBS and NAV files using MATLAB's `rinexread`
- Applies single-difference pseudorange method
- Converts positions to ECEF, Geodetic (WGS84), and UTM
- Outputs results to `rover_baseline_positions.csv`

# Requirements

- MATLAB R2021a or newer
- Mapping Toolbox (for coordinate transformations)

## How to Use

1. Update the following input filenames and base station coordinates in the script:
   ```matlab
   obsBaseFile = "203m0340.obs";
   obsRovFile  = "_9390340.obs";
   navFile     = "_9390340.nav";
