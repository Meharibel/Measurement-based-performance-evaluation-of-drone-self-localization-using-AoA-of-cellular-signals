%RUNREPRODUCTION Run the measurement and localization workflows.

codeDirectory = fileparts(mfilename("fullpath"));
repositoryRoot = fullfile(codeDirectory, "..");
dataFile = fullfile(repositoryRoot, "data", "m30.mat");
outputDirectory = fullfile(repositoryRoot, "results", "generated");

if ~isfolder(outputDirectory)
    mkdir(outputDirectory);
end

fprintf("Estimating AoA from measurement data...\n");
aoaResults = estimateAoAFromMeasurements(dataFile, outputDirectory);

fprintf("Running Monte Carlo localization simulation...\n");
localizationResults = simulateDroneLocalization(outputDirectory);

fprintf("Results written to:\n  %s\n", outputDirectory);
