# MATLAB-Generated Results

Run `code/runReproduction.m` to create `results/generated/`.

Run `code/publishRepositoryResults.m` to reproduce the figures in `figures/`
and summary tables in `tables/`. The displayed Monte Carlo examples use
`rng(12345)` so that the same MATLAB run can be repeated. The model constants
remain the supplied values.

Every PNG in `figures/` is copied byte-for-byte from MATLAB's `exportgraphics`
output. No generative AI or external image editor is used.

The generated directory contains:

- AoA histograms as 300 dpi PNG and vector PDF files
- Localization trajectory plots
- Least-squares localization-error CDF for hovering and stationary cases
- CSV summary tables
- MAT result files
