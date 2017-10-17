function [fitresult, gof] = createFit1(d, a)
%CREATEFIT1(D,A)
%  Create a fit.
%
%  Data for 'Fitted Curve' fit:
%      X Input : d
%      Y Output: a
%  Output:
%      fitresult : a fit object representing the fit.
%      gof : structure with goodness-of fit info.
%
%  See also FIT, CFIT, SFIT.

%  Auto-generated by MATLAB on 12-Oct-2017 12:52:12


%% Fit: 'Fitted Curve'.
[xData, yData] = prepareCurveData( d, a );

% Set up fittype and options.
ft = fittype( 'rat23' );
opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
opts.Display = 'Off';
opts.StartPoint = [0.51077156417211 0.817627708322262 0.794831416883453 0.644318130193692 0.378609382660268 0.811580458282477];

% Fit model to data.
[fitresult, gof] = fit( xData, yData, ft, opts );

% Plot fit with data.
h = plot( fitresult, xData, yData );
legend( h, 'Analog Output vs. Distance (cm)', 'Fitted Curve', 'Location', 'NorthEast' );
% Label axes
xlabel d
ylabel a
grid on

