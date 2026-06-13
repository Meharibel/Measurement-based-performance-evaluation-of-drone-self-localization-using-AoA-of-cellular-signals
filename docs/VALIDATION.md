# Validation And Audit Notes

## Scope

The supplied paper, three MATLAB files, and `m30.mat` were reviewed before
cleanup. The original files are retained under `legacy/supplied_files/`.

No scientific constants were changed in the cleaned code.

## Data Structure

`m30.mat` contains one variable:

```text
m30: 19 x 5 x 100 double
```

The source CSV headers identify the columns as:

1. Mean angle
2. Power at 3.00 GHz
3. Power at 3.03 GHz
4. Power at 3.06 GHz
5. Relative angle

## Original Measurement-Code Output

Running the supplied `mainAngleSearch.m` with the supplied `m30.mat` produced:

| Series | Mean | Std. dev. | Median | Min | Max |
|---|---:|---:|---:|---:|---:|
| Original `foundA1` | 116.3866 deg | 0.8045 deg | 116.47 deg | 114.50 deg | 118.22 deg |
| Original `foundA2` | 90.7434 deg | 0.6862 deg | 90.79 deg | 89.16 deg | 92.36 deg |
| Absolute difference | 25.6432 deg | 0.4202 deg | 25.68 deg | 24.36 deg | 26.74 deg |

## Original Localization-Code Output

The supplied localization filename contains spaces and cannot be executed by
MATLAB with `run`. An identical audit copy was renamed to a valid MATLAB
filename without changing its contents.

One audit run produced:

| Estimator | AoA std. dev. | Mean | RMS | Median | 90th percentile |
|---|---:|---:|---:|---:|---:|
| Least squares | 6 deg | 49.2828 m | 60.1133 m | 42.7280 m | 91.4334 m |
| Auxiliary variable | 6 deg | 33.0752 m | 37.4999 m | 30.8549 m | 57.1905 m |
| Least squares | 3.6 deg | 29.6254 m | 35.7643 m | 25.2266 m | 56.0259 m |
| Auxiliary variable | 3.6 deg | 19.8344 m | 22.4929 m | 18.5530 m | 34.2213 m |

The original script does not set an RNG seed, so a different random-number
state can produce slightly different values.

## Items Preserved For Author Review

These points were documented rather than silently corrected:

1. `mainAngleSearch.m` uses MAT column 3 with `3e9` and MAT column 2 with
   `3.03e9`. The CSV headers indicate the opposite frequency association.
2. The paper's Figure 2 BS/frequency labels, CSV headers, and original script
   variable names are not mutually consistent.
3. The paper describes maximizing a gain-power correlation. The supplied
   measurement script minimizes squared error between normalized power and
   the antenna response.
4. The paper reports AoA standard deviations of `6.02` and `3.2` degrees.
   The supplied localization script uses `6` and `3.6` degrees.
5. The localization variable is named `ISD=250`, while base stations are
   placed at `2*ISD`, giving the 500 m adjacent spacing described in the paper.
6. The supplied localization script calculates both least-squares and
   auxiliary-variable estimates. The paper presents the least-squares result.
7. The supplied measurement script labels generated files as `outdoor`, but
   `m30.mat` is stored with chamber-measurement data.
8. The second `pdfcrop` command repeats the first output filename, and the
   script depends on external `pdfcrop` and Unix `rm` commands.
9. The original localization plot legend has duplicate and mismatched labels.

## Cleanup Changes

- Added valid MATLAB filenames.
- Added functions with documented inputs and outputs.
- Preallocated arrays without changing random-number calls or equations.
- Replaced platform-specific PDF crop/delete commands with `exportgraphics`.
- Added explicit, correct plot labels and 300 dpi PNG/vector PDF exports.
- Added summary CSV and MAT outputs.
- Kept the published CDF focused on the paper's least-squares hovering and
  stationary cases; no auxiliary-estimator curve is displayed.
- Added `publishRepositoryResults.m` so every figure displayed on GitHub is
  regenerated and exported directly by MATLAB without image post-processing.
