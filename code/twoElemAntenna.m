function gainOut = twoElemAntenna(angleInDeg, rotationAngleDeg, ...
    elementSpacingM, frequencyHz)
%TWOELEMANTENNA Calculate the response of a two-element antenna.
%
% gainOut = twoElemAntenna(angleInDeg, rotationAngleDeg, ...
%     elementSpacingM, frequencyHz)
%
% Inputs
%   angleInDeg       Angles at which the response is evaluated, in degrees.
%   rotationAngleDeg Mechanical rotation angle, in degrees.
%   elementSpacingM  Physical distance between antenna elements, in meters.
%   frequencyHz      Carrier frequency, in hertz.
%
% Output
%   gainOut          Normalized complex antenna response.
%
% The numerical operations and constants are preserved from the supplied
% twoElemAntenna.m implementation.

    normalizedQuarterSpacing = 0.25;
    speedOfLightMps = 299792458;

    wavelengthM = speedOfLightMps ./ frequencyHz;
    spacingScale = elementSpacingM / (wavelengthM / 2);
    normalizedSpacing = normalizedQuarterSpacing * spacingScale;

    gainOut = 1 / 2 * ( ...
        exp(-1i * 2 * pi * (-normalizedSpacing * ...
        sind(angleInDeg - rotationAngleDeg))) + ...
        exp(-1i * 2 * pi * (normalizedSpacing * ...
        sind(angleInDeg - rotationAngleDeg))));

    gainOut = gainOut ./ max(gainOut);
end
