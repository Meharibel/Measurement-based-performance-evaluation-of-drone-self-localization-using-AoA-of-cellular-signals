function publishRepositoryResults()
%PUBLISHREPOSITORYRESULTS Regenerate the results displayed on GitHub.
%
% Every PNG placed in results/figures is exported directly by MATLAB using
% exportgraphics. No image editor or generative-image tool is used.

    codeDirectory = fileparts(mfilename("fullpath"));
    repositoryRoot = fullfile(codeDirectory, "..");
    dataFile = fullfile(repositoryRoot, "data", "m30.mat");
    generatedDirectory = fullfile(repositoryRoot, "results", "generated");
    figureDirectory = fullfile(repositoryRoot, "results", "figures");
    tableDirectory = fullfile(repositoryRoot, "results", "tables");

    if ~isfolder(generatedDirectory)
        mkdir(generatedDirectory);
    end
    if ~isfolder(figureDirectory)
        mkdir(figureDirectory);
    end
    if ~isfolder(tableDirectory)
        mkdir(tableDirectory);
    end

    previousRandomState = rng;
    randomStateCleanup = onCleanup(@() rng(previousRandomState));
    rng(12345);

    estimateAoAFromMeasurements(dataFile, generatedDirectory);
    simulateDroneLocalization(generatedDirectory);

    figureNames = [
        "aoa_column2_histogram.png"
        "aoa_column3_histogram.png"
        "aoa_difference_histogram.png"
        "localization_error_cdf.png"
        "localization_error_cdf_all_estimators.png"
        "localization_trajectory_sigma_3.6deg.png"
        "localization_trajectory_sigma_6deg.png"
    ];

    for figureIndex = 1:length(figureNames)
        copyfile( ...
            fullfile(generatedDirectory, figureNames(figureIndex)), ...
            fullfile(figureDirectory, figureNames(figureIndex)), "f");
    end

    tableNames = [
        "aoa_measurement_summary.csv"
        "localization_error_summary.csv"
    ];

    for tableIndex = 1:length(tableNames)
        copyfile( ...
            fullfile(generatedDirectory, tableNames(tableIndex)), ...
            fullfile(tableDirectory, tableNames(tableIndex)), "f");
    end

    fprintf("Published MATLAB-generated figures to:\n  %s\n", ...
        figureDirectory);
end
