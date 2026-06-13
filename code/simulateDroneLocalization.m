function results = simulateDroneLocalization(outputDirectory, rngSeed)
%SIMULATEDRONELOCALIZATION Run the supplied AoA localization simulation.
%
% results = simulateDroneLocalization(outputDirectory, rngSeed)
%
% rngSeed is optional. When it is NaN, the current MATLAB random-number
% state is used, matching the behavior of the supplied script.
%
% Numerical constants and estimator equations are preserved. The code has
% been organized, preallocated, and given publication-quality plots.

    arguments
        outputDirectory (1, 1) string = defaultOutputDirectory()
        rngSeed (1, 1) double = NaN
    end

    if ~isnan(rngSeed)
        rng(rngSeed);
    end

    settings.interSiteDistanceScaleM = 250;
    settings.positionPerturbationStdM = 1;
    settings.angleErrorStdDeg = [6 3.6];
    settings.numberOfRealizations = 100;

    interSiteDistanceScaleM = settings.interSiteDistanceScaleM;
    baseStationPositionsComplex = [ ...
        0, ...
        2 * interSiteDistanceScaleM * ...
        exp(1i * (1:6) * 60 / 180 * pi)];
    baseStationPositionsM = [
        real(baseStationPositionsComplex)
        imag(baseStationPositionsComplex)
    ];
    numberOfBaseStations = length(baseStationPositionsComplex);

    dronePositionsComplex = ( ...
        interSiteDistanceScaleM / 2 * (0:0.005:2)) .* ...
        exp(1i * (0:0.005:2) * 2 * pi) * ...
        exp(-1i * 0.333 * 2 * pi);
    dronePositionsM = [
        real(dronePositionsComplex)
        imag(dronePositionsComplex)
    ];
    dronePositionsM = dronePositionsM + ...
        settings.positionPerturbationStdM * randn(size(dronePositionsM));

    numberOfTimeSteps = length(dronePositionsM);
    angleErrorStdRad = settings.angleErrorStdDeg / 180 * pi;
    numberOfCases = length(angleErrorStdRad);
    numberOfRealizations = settings.numberOfRealizations;

    leastSquaresErrorsM = zeros( ...
        numberOfCases, numberOfTimeSteps * numberOfRealizations);
    auxiliaryErrorsM = zeros( ...
        numberOfCases, numberOfTimeSteps * numberOfRealizations);

    leastSquaresEstimatesM = zeros( ...
        2, numberOfTimeSteps, numberOfRealizations, numberOfCases);
    auxiliaryEstimatesM = zeros( ...
        2, numberOfTimeSteps, numberOfRealizations, numberOfCases);

    for caseIndex = 1:numberOfCases
        for realizationIndex = 1:numberOfRealizations
            angleMeasurementsRad = zeros( ...
                numberOfBaseStations, numberOfTimeSteps);
            tangentMeasurements = zeros( ...
                numberOfBaseStations, numberOfTimeSteps);
            sineMeasurements = zeros( ...
                numberOfBaseStations, numberOfTimeSteps);
            cosineMeasurements = zeros( ...
                numberOfBaseStations, numberOfTimeSteps);

            scaledBaseStationLocations = zeros(numberOfBaseStations, 1);
            auxiliaryRightHandSide = zeros(numberOfBaseStations, 1);
            leastSquaresEstimate = zeros(2, numberOfTimeSteps);
            auxiliaryVariableEstimate = zeros(3, numberOfTimeSteps);
            auxiliaryPositionEstimate = zeros(2, numberOfTimeSteps);
            auxiliaryAngleEstimateRad = zeros(numberOfTimeSteps, 1);

            for timeIndex = 1:numberOfTimeSteps
                for baseStationIndex = 1:numberOfBaseStations
                    angleMeasurementsRad(baseStationIndex, timeIndex) = ...
                        atan(( ...
                        dronePositionsM(2, timeIndex) - ...
                        baseStationPositionsM(2, baseStationIndex)) ./ ( ...
                        dronePositionsM(1, timeIndex) - ...
                        baseStationPositionsM(1, baseStationIndex))) + ...
                        randn * angleErrorStdRad(caseIndex);

                    tangentMeasurements(baseStationIndex, timeIndex) = ...
                        tan(angleMeasurementsRad( ...
                        baseStationIndex, timeIndex));
                    sineMeasurements(baseStationIndex, timeIndex) = ...
                        sin(angleMeasurementsRad( ...
                        baseStationIndex, timeIndex));
                    cosineMeasurements(baseStationIndex, timeIndex) = ...
                        cos(angleMeasurementsRad( ...
                        baseStationIndex, timeIndex));

                    scaledBaseStationLocations(baseStationIndex, 1) = ...
                        [-tangentMeasurements(baseStationIndex, timeIndex), 1] ...
                        * baseStationPositionsM(:, baseStationIndex);
                    auxiliaryRightHandSide(baseStationIndex, 1) = ...
                        [sineMeasurements(baseStationIndex, timeIndex), ...
                        -cosineMeasurements(baseStationIndex, timeIndex)] ...
                        * baseStationPositionsM(:, baseStationIndex);
                end

                auxiliaryMatrix = [
                    sineMeasurements(:, timeIndex), ...
                    -cosineMeasurements(:, timeIndex), ...
                    -baseStationPositionsM(1, :)' .* ...
                    cosineMeasurements(:, timeIndex) - ...
                    baseStationPositionsM(2, :)' .* ...
                    sineMeasurements(:, timeIndex)
                ];
                leastSquaresMatrix = [
                    -tangentMeasurements(:, timeIndex), ...
                    ones(size(tangentMeasurements(:, timeIndex)))
                ];

                leastSquaresEstimate(:, timeIndex) = ...
                    pinv(leastSquaresMatrix) * scaledBaseStationLocations;
                auxiliaryVariableEstimate(:, timeIndex) = ...
                    pinv(auxiliaryMatrix) * auxiliaryRightHandSide;

                auxiliaryPositionEstimate(1, timeIndex) = ( ...
                    auxiliaryVariableEstimate(1, timeIndex) - ...
                    auxiliaryVariableEstimate(3, timeIndex) * ...
                    auxiliaryVariableEstimate(2, timeIndex)) / ( ...
                    1 + auxiliaryVariableEstimate(3, timeIndex)^2);
                auxiliaryPositionEstimate(2, timeIndex) = ( ...
                    auxiliaryVariableEstimate(2, timeIndex) + ...
                    auxiliaryVariableEstimate(3, timeIndex) * ...
                    auxiliaryVariableEstimate(1, timeIndex)) / ( ...
                    1 + auxiliaryVariableEstimate(3, timeIndex)^2);
                auxiliaryAngleEstimateRad(timeIndex, 1) = ...
                    atan(auxiliaryVariableEstimate(3, timeIndex));

                errorIndex = timeIndex + ...
                    (realizationIndex - 1) * numberOfTimeSteps;
                leastSquaresErrorsM(caseIndex, errorIndex) = norm( ...
                    leastSquaresEstimate(:, timeIndex) - ...
                    dronePositionsM(:, timeIndex));
                auxiliaryErrorsM(caseIndex, errorIndex) = norm( ...
                    auxiliaryPositionEstimate(:, timeIndex) - ...
                    dronePositionsM(:, timeIndex));
            end

            leastSquaresEstimatesM(:, :, realizationIndex, caseIndex) = ...
                leastSquaresEstimate;
            auxiliaryEstimatesM(:, :, realizationIndex, caseIndex) = ...
                auxiliaryPositionEstimate;
        end
    end

    results.settings = settings;
    results.baseStationPositionsM = baseStationPositionsM;
    results.dronePositionsM = dronePositionsM;
    results.leastSquaresErrorsM = leastSquaresErrorsM;
    results.auxiliaryErrorsM = auxiliaryErrorsM;
    results.leastSquaresEstimatesM = leastSquaresEstimatesM;
    results.auxiliaryEstimatesM = auxiliaryEstimatesM;
    results.summary = createSummary( ...
        leastSquaresErrorsM, auxiliaryErrorsM, settings.angleErrorStdDeg);

    if ~isfolder(outputDirectory)
        mkdir(outputDirectory);
    end

    writetable(results.summary, ...
        fullfile(outputDirectory, "localization_error_summary.csv"));
    save(fullfile(outputDirectory, "localization_results.mat"), ...
        "-struct", "results");

    createTrajectoryPlots(results, outputDirectory);
    createPaperCdfPlot(results, outputDirectory);
end

function summary = createSummary(leastSquaresErrorsM, auxiliaryErrorsM, ...
    angleErrorStdDeg)

    estimator = strings(2 * length(angleErrorStdDeg), 1);
    angleStdDeg = zeros(2 * length(angleErrorStdDeg), 1);
    meanM = zeros(2 * length(angleErrorStdDeg), 1);
    rmsM = zeros(2 * length(angleErrorStdDeg), 1);
    medianM = zeros(2 * length(angleErrorStdDeg), 1);
    percentile90M = zeros(2 * length(angleErrorStdDeg), 1);

    row = 0;
    for caseIndex = 1:length(angleErrorStdDeg)
        for estimatorIndex = 1:2
            row = row + 1;
            angleStdDeg(row) = angleErrorStdDeg(caseIndex);

            if estimatorIndex == 1
                estimator(row) = "Least squares";
                errorsM = leastSquaresErrorsM(caseIndex, :);
            else
                estimator(row) = "Auxiliary variable";
                errorsM = auxiliaryErrorsM(caseIndex, :);
            end

            meanM(row) = mean(errorsM);
            rmsM(row) = sqrt(mean(errorsM .^ 2));
            medianM(row) = median(errorsM);
            percentile90M(row) = prctile(errorsM, 90);
        end
    end

    summary = table(estimator, angleStdDeg, meanM, rmsM, medianM, ...
        percentile90M, VariableNames=[ ...
        "Estimator", "AngleStdDeg", "MeanM", "RmsM", "MedianM", ...
        "Percentile90M"]);
end

function createTrajectoryPlots(results, outputDirectory)
    colors = [
        0.00 0.68 0.86
        0.15 0.45 0.82
    ];

    for caseIndex = 1:length(results.settings.angleErrorStdDeg)
        figureHandle = figure(Color="white", Position=[100 100 900 650]);
        estimates = results.leastSquaresEstimatesM(:, :, :, caseIndex);
        pointColor = 0.72 * [1 1 1] + 0.28 * colors(caseIndex, :);
        plot(reshape(estimates(1, :, :, :), 1, []), ...
            reshape(estimates(2, :, :, :), 1, []), ".", ...
            Color=pointColor, MarkerSize=4);
        hold on;
        plot(results.dronePositionsM(1, :), results.dronePositionsM(2, :), ...
            Color=[0.12 0.12 0.12], LineWidth=2);
        plot(results.baseStationPositionsM(1, :), ...
            results.baseStationPositionsM(2, :), "p", ...
            MarkerSize=12, MarkerFaceColor=[0.94 0.55 0.13], ...
            MarkerEdgeColor=[0.35 0.20 0.02], LineWidth=1);
        hold off;

        axis equal;
        xlim([-500 500]);
        ylim([-600 600]);
        grid on;
        box on;
        xlabel("x position (m)");
        ylabel("y position (m)");
        title(sprintf("Least-squares localization, AoA std. dev. %.1f deg", ...
            results.settings.angleErrorStdDeg(caseIndex)));
        legend("Estimated drone positions", "True drone trajectory", ...
            "Base stations", Location="northoutside", ...
            Orientation="horizontal");
        set(gca, FontName="Arial", FontSize=13, LineWidth=1);

        outputBaseName = fullfile(outputDirectory, sprintf( ...
            "localization_trajectory_sigma_%gdeg", ...
            results.settings.angleErrorStdDeg(caseIndex)));
        exportgraphics(figureHandle, outputBaseName + ".png", Resolution=300);
        exportgraphics(figureHandle, outputBaseName + ".pdf", ...
            ContentType="vector");
        close(figureHandle);
    end
end

function createPaperCdfPlot(results, outputDirectory)
    figureHandle = figure(Color="white", Position=[100 100 850 570]);
    colors = [
        0.00 0.45 0.74
        0.85 0.33 0.10
    ];

    hold on;
    for caseIndex = 1:length(results.settings.angleErrorStdDeg)
        [cdfValues, errorValuesM] = ecdf( ...
            results.leastSquaresErrorsM(caseIndex, :));
        plot(errorValuesM, cdfValues, LineWidth=2.2, ...
            Color=colors(caseIndex, :));
    end
    hold off;

    grid on;
    box on;
    xlim([0 100]);
    ylim([0 1]);
    xlabel("Absolute localization error (m)");
    ylabel("Cumulative probability");
    title("Least-squares drone localization error");
    legend([
        compose("Drone hovering (AoA std. dev. %.1f deg)", ...
            results.settings.angleErrorStdDeg(1))
        compose("Drone stationary (AoA std. dev. %.1f deg)", ...
            results.settings.angleErrorStdDeg(2))
    ], Location="southeast");
    set(gca, FontName="Arial", FontSize=13, LineWidth=1);

    outputBaseName = fullfile(outputDirectory, "localization_error_cdf");
    exportgraphics(figureHandle, outputBaseName + ".png", Resolution=300);
    exportgraphics(figureHandle, outputBaseName + ".pdf", ...
        ContentType="vector");
    close(figureHandle);
end

function path = defaultOutputDirectory()
    codeDirectory = fileparts(mfilename("fullpath"));
    path = fullfile(codeDirectory, "..", "results", "generated");
end
