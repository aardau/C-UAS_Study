clc;clear; close all
%% Result 1 Block
% Static-only and mobile-only performance benchmarks for 1-, 2-, and 3-defense configurations

% Static data
result1_static_labels = {'1 Static (origin)', '1 Static (N)', '1 Static (S)', '1 Static (E)', '1 Static (W)', ...
    '2 Static (N/S)', '2 Static (E/W)', '3 Static (Equilateral 2B/1T)', '3 Static (15deg offset CW)'};
result1_static_dsr    = [9.50, 20.80, 20.70, 19.20, 20.00, 40.50, 40.20, 66.27, 62.30];
result1_static_std    = [2.07, 6.53, 3.02, 2.89, 2.38, 7.02, 4.77, 4.03, 2.76];
result1_static_ci_low = [7.75, 18.32, 18.23, 16.80, 17.56, 37.44, 37.14, 62.76, 59.21];
result1_static_ci_up  = [11.49, 23.45, 23.35, 21.78, 22.62, 43.62, 43.31, 69.65, 65.31];

% Mobile data
result1_mobile_labels = {'1 Mobile (origin)', '1 Mobile (N)', '1 Mobile (S)', '1 Mobile (E)', '1 Mobile (W)', ...
    '2 Mobile (N/S)', '2 Mobile (E/W)', '3 Mobile (Equilateral 2B/1T)', '3 Mobile (30deg offset 1B/2T)'};
result1_mobile_dsr    = [87.90, 81.60, 81.20, 82.40, 85.10, 89.30, 83.60, 93.70, 93.30];
result1_mobile_std    = [2.20, 2.80, 1.80, 1.97, 1.48, 1.31, 2.54, 1.49, 1.04];
result1_mobile_ci_low = [85.72, 79.06, 78.64, 79.90, 82.74, 87.22, 81.16, 92.01, 91.57];
result1_mobile_ci_up  = [89.86, 83.96, 83.58, 84.71, 87.25, 91.15, 85.84, 95.13, 94.77];

% Plot(s) for result 1 data
figure;
hold on

hold off

%% Result 2 Block
% Mobile-only benchmark with new track/kill probabilities & defense speed

% Mobile data
result2_mobile_labels = { '1 Mobile (origin)', '1 Mobile (N)', '1 Mobile (S)', '1 Mobile (E)', '1 Mobile (W)', ...
    '2 Mobile (N/S)', '2 Mobile (E/W)', '3Mobile (Equilateral 2B/1T)', '3 SMobile (60deg offset 1B/2T)' };
result2_mobile_dsr = [41.00, 35.10, 36.40, 33.90, 34.40, 46.10, 48.00, 52.50, 53.40];
result2_mobile_std = [2.10, 4.61, 2.20, 3.49, 3.37, 2.31, 4.28, 5.17, 3.55];
result2_mobile_ci_low = [37.93, 32.14, 33.41, 30.97, 31.46, 42.98, 44.86, 49.35, 50.25];
result2_mobile_ci_up = [44.12, 38.15, 39.47, 36.93, 37.44, 49.25, 51.15, 55.63, 56.53];


% Plot(s) for result 2 data
figure;
hold on

hold off

%% Result 4 Block
% Study of mixed mobile & static defense types

% Mixed defenses -  4
result4_4eff_labels    = { '4 - Square/Diag', '4 - Square/SameSide', '4 - SD Tri, Center MD', ...
    '4, MD Tri, Center SD', '4 - All static', '4 - All mobile' };
result4_4eff_dsr       = [77.60, 75.20, 77.80, 58.60, 82.80, 40.60];
result4_4eff_std        = [2.75, 2.96, 3.42, 3.44, 3.04, 3.57];
result4_4eff_ci_low     = [74.89, 72.40, 75.09, 55.48, 80.32, 37.54];
result4_4eff_ci_up      = [80.15, 77.85, 80.34, 61.67, 85.09, 43.72];

% Mixed defenses -  5
result4_5eff_labels    = { '5 - Pentagon, SD Tri', '5 - Pentagon, MD Tri', 'Pentagon - 2 SD Bot', 'Pentagon - 2 MD Bot' };
result4_5eff_dsr        = [83.80, 76.60, 78.60, 80.40];
result4_5eff_std        = [1.76, 2.77, 3.50, 2.54];
result4_5eff_ci_low     = [81.37, 73.85, 75.93, 77.80];
result4_5eff_ci_up      = [86.03, 79.19, 81.10, 82.82];

% Mixed defenses -  6
result4_6eff_labels    = { 'Hexagon - on/off', 'Hexagon, L/R bias' };
result4_6eff_dsr        = [85.90, 82.20];
result4_6eff_std        = [1.98, 4.78];
result4_6eff_ci_low     = [83.59, 79.69];
result4_6eff_ci_up      = [88.00, 84.52];

% Plot(s) for result 4 data
figure;
hold on

hold off

%% Result 5 block
% Full distance study to see how it impacts test setsups of 3 MD or 3 SD defenses

% Distance values
result5_distance = 100:100:1000;

% Static data
result5_static_dsr    = [56.90, 55.70, 56.40, 61.00, 66.27, 68.10, 68.30, 67.90, 66.40, 67.40];
result5_static_std    = [3.87, 3.76, 2.69, 5.62, 4.03, 3.45, 2.83, 3.97, 2.46, 4.11];
result5_static_ci_low = [53.76, 52.56, 53.26, 57.90, 62.76, 65.11, 65.32, 64.91, 63.38, 64.40];
result5_static_ci_up  = [60.00, 58.81, 59.50, 64.04, 69.65, 70.98, 71.18, 70.79, 69.33, 70.30];

% Mobile data
result5_mobile_dsr    = [44.60, 47.70, 50.90, 52.70, 52.50, 57.90, 61.80, 57.80, 54.20, 54.50];
result5_mobile_std    = [3.86, 2.71, 4.07, 2.55, 5.17, 5.25, 4.51, 3.01, 3.86, 2.24];
result5_mobile_ci_low = [41.49, 44.56, 47.75, 49.55, 49.35, 54.77, 58.71, 54.67, 51.05, 51.35];
result5_mobile_ci_up  = [47.74, 50.85, 54.04, 55.83, 55.63, 60.98, 64.82, 60.88, 57.32, 57.62];

% Plot(s) for result 5 data
figure;
hold on;

% Plot defense success rate (DSR)
plot(result5_distance, result5_static_dsr, '-o', 'Color', [0.6350 0.0780 0.1840], 'LineWidth', 1.5);
plot(result5_distance, result5_mobile_dsr, '-s', 'Color', [0 0.4470 0.7410], 'LineWidth', 1.5);

% Plot error bars for DSR (standard deviation)
errorbar(result5_distance, result5_static_dsr, result5_static_std, 'o', 'LineStyle', 'none', ...
    'Color', [0.6350 0.0780 0.1840],'LineWidth',1.5);
errorbar(result5_distance, result5_mobile_dsr, result5_mobile_std, 's', 'LineStyle', 'none', ...
    'Color', [0 0.4470 0.7410],'LineWidth',1.5);

% Labels and legend
xlabel('Distance from Origin (units)');
ylabel('Defense Success Rate (%)');
title('Defense Success Rate (DSR) vs. Distance from Origin (Result Block 5)');
legend('Static DSR', 'Mobile DDSR', 'Static Std. Dev.', 'Mobile Std. Dev.', ...
    'Location', 'southoutside', 'Orientation', 'horizontal');
grid on;
ylim([40 75]);


%% Result 6 Block
% Combines results 4 and 5 to find the best defense setups

% Mixed defenses & optimal distance -  5
result6_pentagon_labels      = {'5 - Pentagon, SD Tri', '5 - Pentagon, SD Tri'};
result6_pentagon_SD_distance = [500, 700];
result6_pentagon_MD_distance = [500, 700];
result6_pentagon_dsr         = [83.80, 89.50];
result6_pentagon_std         = [1.76, 1.57];
result6_pentagon_ci_low      = [81.37, 87.43];
result6_pentagon_ci_up       = [86.03, 91.33];

% Mixed defenses & optimal distance -  6
result6_hexagon_labels      = {'Hexagon - on/off', 'Hexagon - on/off'};
result6_hexagon_SD_distance = [500, 700];
result6_hexagon_MD_distance = [500, 700];
result6_hexagon_dsr         = [85.90, 90.50];
result6_hexagon_std         = [1.98, 3.78];
result6_hexagon_ci_low      = [83.59, 88.51];
result6_hexagon_ci_up       = [88.00, 92.25];

% Plot(s) for result 6 data
figure;
hold on;
