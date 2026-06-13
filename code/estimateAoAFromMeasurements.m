function results = estimateAoAFromMeasurements(dataFile, outputDirectory)
%ESTIMATEAOAFROMMEASUREMENTS Estimate AoA from rotating-antenna measurements.
%
% results = estimateAoAFromMeasurements(dataFile, outputDirectory)
%
% The supplied search grid, frequency assignments, normalization, antenna
% model, and squared-error objective are preserved. Variable names identify
% MAT-file columns directly because the original BS labels conflict with the
% CSV headers and paper labels. See docs/VALIDATION.md.

    arguments
        dataFile (1, 1) string = defaultDataFile()
        outputDirectory (1, 1) string = defaultOutputDirectory()
    end

    settings.elementSpacingM = 0.05;
    settings.frequencyUsedForColumn3Hz = 3e9;
    settings.frequencyUsedForColumn2Hz = 3.03e9;
    settings.angleLowDeg = 0;
    settings.angleUpDeg = 179;
    settings.angleSearchDeg = 0:0.02:180;

    loadedData = load(dataFile, "m30");
    assert(isfield(loadedData, "m30"), ...
        "The MAT file must contain a variable named m30.");

    measurements = loadedData.m30;
    assert(ndims(measurements) == 3 && size(measurements, 2) >= 5, ...
        "m30 must be a three-dimensional array with at least five columns.");

    numberOfMeasurements = size(measurements, 3);
    estimatedAngleColumn3Deg = zeros(1, numberOfMeasurements);
    estimatedAngleColumn2Deg = zeros(1, numberOfMeasurements);

    for measurementIndex = 1:numberOfMeasurements
        currentMeasurement = measurements(:, :, measurementIndex);
        measuredAngleDeg = currentMeasurement(:, 5);

        normalizedPowerColumn2 = 10 .^ ( ...
            (currentMeasurement(:, 2) - max(currentMeasurement(:, 2))) / 10);
        normalizedPowerColumn3 = 10 .^ ( ...
            (currentMeasurement(:, 3) - max(currentMeasurement(:, 3))) / 10);

        column3Objective = @(candidateAngleDeg) sum(( ...
            normalizedPowerColumn3 - twoElemAntenna( ...
            measuredAngleDeg, candidateAngleDeg, ...
            settings.elementSpacingM, ...
            settings.frequencyUsedForColumn3Hz)) .^ 2);

        column2Objective = @(candidateAngleDeg) sum(( ...
            normalizedPowerColumn2 - twoElemAntenna( ...
            measuredAngleDeg, candidateAngleDeg, ...
            settings.elementSpacingM, ...
            settings.frequencyUsedForColumn2Hz)) .^ 2);

        column3Costs = column3Objective(settings.angleSearchDeg);
        column2Costs = column2Objective(settings.angleSearchDeg);

        [~, column3MinimumIndex] = min(column3Costs);
        [~, column2MinimumIndex] = min(column2Costs);

        estimatedAngleColumn3Deg(measurementIndex) = ...
            settings.angleSearchDeg(column3MinimumIndex);
        estimatedAngleColumn2Deg(measurementIndex) = ...
            settings.angleSearchDeg(column2MinimumIndex);
    end

    absoluteDifferenceDeg = abs( ...
        estimatedAngleColumn2Deg - estimatedAngleColumn3Deg);

    results.settings = settings;
    results.estimatedAngleColumn3Deg = estimatedAngleColumn3Deg;
    results.estimatedAngleColumn2Deg = estimatedAngleColumn2Deg;
    results.absoluteDifferenceDeg = absoluteDifferenceDeg;
    results.summary = createSummary( ...
        estimatedAngleColumn3Deg, ...
        estimatedAngleColumn2Deg, ...
        absoluteDifferenceDeg);

    if ~isfolder(outputDirectory)
        mkdir(outputDirectory);
    end

    writetable(results.summary, ...
        fullfile(outputDirectory, "aoa_measurement_summary.csv"));
    save(fullfile(outputDirectory, "aoa_measurement_results.mat"), ...
        "-struct", "results");

    createHistogram(estimatedAngleColumn3Deg, 10, ...
        "Estimated AoA from MAT column 3", ...
        fullfile(outputDirectory, "aoa_column3_histogram"));
    createHistogram(estimatedAngleColumn2Deg, 10, ...
        "Estimated AoA from MAT column 2", ...
        fullfile(outputDirectory, "aoa_column2_histogram"));
    createHistogram(absoluteDifferenceDeg, 15:0.5:40, ...
        "Absolute difference between estimated angles", ...
        fullfile(outputDirectory, "aoa_difference_histogram"));
end

function summary = createSummary(column3Deg, column2Deg, differenceDeg)
    labels = [
        "MAT column 3"
        "MAT column 2"
        "Absolute difference"
    ];

    values = [
        mean(column3Deg), std(column3Deg), median(column3Deg), ...
            min(column3Deg), max(column3Deg)
        mean(column2Deg), std(column2Deg), median(column2Deg), ...
            min(column2Deg), max(column2Deg)
        mean(differenceDeg), std(differenceDeg), median(differenceDeg), ...
            min(differenceDeg), max(differenceDeg)
    ];

    summary = table(labels, values(:, 1), values(:, 2), values(:, 3), ...
        values(:, 4), values(:, 5), ...
        VariableNames=["Series", "MeanDeg", "StdDeg", "MedianDeg", ...
        "MinDeg", "MaxDeg"]);
end

function createHistogram(values, binsOrCenters, plotTitle, outputBaseName)
    figureHandle = figure(Color="white", Position=[100 100 900 560]);

    [counts, centers] = hist(values, binsOrCenters); %#ok<HIST>
    bar(centers, counts, 1, ...
        FaceColor=[0.16 0.38 0.70], ...
        EdgeColor=[0.08 0.18 0.35], ...
        LineWidth=0.8);

    grid on;
    box on;
    xlabel("Estimated angle (degrees)");
    ylabel("Number of measurements");
    title(plotTitle);
    set(gca, FontName="Arial", FontSize=13, LineWidth=1);

    exportgraphics(figureHandle, outputBaseName + ".png", Resolution=300);
    exportgraphics(figureHandle, outputBaseName + ".pdf", ...
        ContentType="vector");
    close(figureHandle);
end

function path = defaultDataFile()
    codeDirectory = fileparts(mfilename("fullpath"));
    path = fullfile(codeDirectory, "..", "data", "m30.mat");
end

function path = defaultOutputDirectory()
    codeDirectory = fileparts(mfilename("fullpath"));
    path = fullfile(codeDirectory, "..", "results", "generated");
end
