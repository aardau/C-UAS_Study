clc;clear; close all
%% Result 1 Block
% Static-only and mobile-only performance benchmarks for 1-, 2-, and 3-defense configurations

% Static data
result1_static_labels = {'1 SD (Origin)', '1 SD (N)', '1 SD (S)', '1 SD (E)', '1 SD (W)', ...
    '2 SD (N/S)', '2 SD (E/W)', '3 SD (Tri.)', '3 SD (30° Offset)'};
result1_static_dsr    = [9.50, 20.80, 20.70, 19.20, 20.00, 40.50, 40.20, 66.27, 62.30];
result1_static_std    = [2.07, 6.53, 3.02, 2.89, 2.38, 7.02, 4.77, 4.03, 2.76];
result1_static_ci_low = [7.75, 18.32, 18.23, 16.80, 17.56, 37.44, 37.14, 62.76, 59.21];
result1_static_ci_up  = [11.49, 23.45, 23.35, 21.78, 22.62, 43.62, 43.31, 69.65, 65.31];

% Mobile data
result1_mobile_labels = {'1 MD (Origin)', '1 MD (N)', '1 MD (S)', '1 MD (E)', '1 MD (W)', ...
    '2 MD (N/S)', '2 MD (E/W)', '3 MD (Tri.)', '3 MD (30° Offset)'};
result1_mobile_dsr    = [87.90, 81.60, 81.20, 82.40, 85.10, 89.30, 83.60, 93.70, 93.30];
result1_mobile_std    = [2.20, 2.80, 1.80, 1.97, 1.48, 1.31, 2.54, 1.49, 1.04];
result1_mobile_ci_low = [85.72, 79.06, 78.64, 79.90, 82.74, 87.22, 81.16, 92.01, 91.57];
result1_mobile_ci_up  = [89.86, 83.96, 83.58, 84.71, 87.25, 91.15, 85.84, 95.13, 94.77];

% Plot for result 1 static data
figure(1);
hold on;

% Plot Static 1-effector data
errorbar((1:5), result1_static_dsr(1:5), result1_static_std(1:5), 'o', ...
    'Color', [0.6350 0.0780 0.1840], 'MarkerFaceColor', [0.6350 0.0780 0.1840], ...
    'DisplayName', '1 Static Defense');

% Plot Static 2-effector data
errorbar((6:7), result1_static_dsr(6:7), result1_static_std(6:7), 's', ...
    'Color', [0.6350 0.0780 0.1840], 'MarkerFaceColor', [0.6350 0.0780 0.1840], ...
    'DisplayName', '2 Static Defenses');

% Plot Static 3-effector data
errorbar((8:9), result1_static_dsr(8:9), result1_static_std(8:9), 'd', ...
    'Color', [0.6350 0.0780 0.1840], 'MarkerFaceColor', [0.6350 0.0780 0.1840], ...
    'DisplayName', '3 Static Defenses');

%% Result 2 Block
% Mobile-only benchmark with new track/kill probabilities & defense speed

% Mobile data
result2_mobile_labels = { '1 MD (Origin)', '1 MD (N)', '1 MD (S)', '1 MD (E)', '1 MD (W)', ...
    '2 MD (N/S)', '2 MD (E/W)', '3 MD (Tri.)', '3 MDe (30° Offset)' };
result2_mobile_dsr = [41.00, 35.10, 36.40, 33.90, 34.40, 46.10, 48.00, 52.50, 53.40];
result2_mobile_std = [2.10, 4.61, 2.20, 3.49, 3.37, 2.31, 4.28, 5.17, 3.55];
result2_mobile_ci_low = [37.93, 32.14, 33.41, 30.97, 31.46, 42.98, 44.86, 49.35, 50.25];
result2_mobile_ci_up = [44.12, 38.15, 39.47, 36.93, 37.44, 49.25, 51.15, 55.63, 56.53];


% Plot for result 2 mobile data
figure(1);
hold on;

% Plot Mobile 1-MD data (indices 1 to 5)
errorbar((1:5), result2_mobile_dsr(1:5), result2_mobile_std(1:5), 'o', ...
    'Color', [0 0.4470 0.7410], 'MarkerFaceColor', [0 0.4470 0.7410], ...
    'DisplayName', '1 Mobile Defense');

% Plot Mobile 2-MD data (indices 6 to 7)
errorbar((6:7), result2_mobile_dsr(6:7), result2_mobile_std(6:7), 's', ...
    'Color', [0 0.4470 0.7410], 'MarkerFaceColor', [0 0.4470 0.7410], ...
    'DisplayName', '2 Mobile Defenses');

% Plot Mobile 3-MD data (indices 8 to 9)
errorbar((8:9), result2_mobile_dsr(8:9), result2_mobile_std(8:9), 'd', ...
    'Color', [0 0.4470 0.7410], 'MarkerFaceColor', [0 0.4470 0.7410], ...
    'DisplayName', '3 Mobile Defenses');

% Make a label for combined results
result12_combined_labels = {'1 Defense (Origin)', '1 Defense (N)', '1 Defense (S)', '1 Defense (E)', '1 Defense (W)', ...
    '2 Defenses (N/S)', '2 Defenses (E/W)', '3 Defenses (Tri.)', '3 Defenses (30° Offset)'};

% Define x-axis labels using the combined results
set(gca, 'XTick', 1:9, 'XTickLabel', result12_combined_labels);
xtickangle(45);

% Add labels, title, and legend for combined results
xlabel('Defense Setup Type','fontweight','bold');
ylabel('Defense Success Rate (%)','fontweight','bold');
title('Defense Success Rate for Mobile- and Static-Only Defense Setups, 500 Units from Origin');
legend('Location', 'Northwest');
ylim([0 100]);
xlim([0.5, 9.5]);
grid on;
hold off;

%% Result 4 Block
% Study of mixed mobile & static defense types

% Mixed defenses -  4
result4_4eff_labels    = { 'Square (on/off)', 'Square (Bias)', 'SD Tri, Center MD', ...
    'MD Tri, Center SD'};
result4_4eff_dsr       = [77.60, 75.20, 77.80, 58.60];
result4_4eff_std        = [2.75, 2.96, 3.42, 3.44];
result4_4eff_ci_low     = [74.89, 72.40, 75.09, 55.48];
result4_4eff_ci_up      = [80.15, 77.85, 80.34, 61.67];

% Mixed defenses -  5
result4_5eff_labels    = { 'Pentagon (3 SD on/off)', 'Pentagon (3 MD on/off)', 'Pentagon (3 SD Bias)', 'Pentagon (3 MD Bias)' };
result4_5eff_dsr        = [83.80, 76.60, 78.60, 80.40];
result4_5eff_std        = [1.76, 2.77, 3.50, 2.54];
result4_5eff_ci_low     = [81.37, 73.85, 75.93, 77.80];
result4_5eff_ci_up      = [86.03, 79.19, 81.10, 82.82];

% Mixed defenses -  6
result4_6eff_labels    = { 'Hexagon (on/off)', 'Hexagon (Bias)' };
result4_6eff_dsr        = [85.90, 82.20];
result4_6eff_std        = [1.98, 4.78];
result4_6eff_ci_low     = [83.59, 79.69];
result4_6eff_ci_up      = [88.00, 84.52];


% Combine x-axis labels for all groups
result4_combined_labels = [result4_4eff_labels, result4_5eff_labels, result4_6eff_labels];

figure(2);
hold on;

% Plot 4-defense data
errorbar(1:4, result4_4eff_dsr, result4_4eff_std, 'o', 'LineStyle', 'none', ...
    'Color', [0.9290 0.6940 0.1250], 'MarkerFaceColor', [0.9290 0.6940 0.1250], ...
    'DisplayName', '4 Defenses');

% Plot 5-defense data
errorbar(5:8, result4_5eff_dsr, result4_5eff_std, 's', 'LineStyle', 'none', ...
    'Color', [0.4940 0.1840 0.5560], 'MarkerFaceColor', [0.4940 0.1840 0.5560], ...
    'DisplayName', '5 Defenses');

% Plot 6-defense data
errorbar(9:10, result4_6eff_dsr, result4_6eff_std, 'd', 'LineStyle', 'none', ...
    'Color', [0.4660 0.6740 0.1880], 'MarkerFaceColor', [0.4660 0.6740 0.1880], ...
    'DisplayName', '6 Defenses');

% Set x-axis labels and adjust margin
set(gca, 'XTick', 1:12, 'XTickLabel', result4_combined_labels);
xtickangle(45);
xlim([0.5, 10.5]);

% Add labels, title, and legend
xlabel('Defense Setup Type','fontweight','bold');
ylabel('Defense Success Rate (%)','fontweight','bold');
title('Defense Success Rate of Mixed-Defense Setups, 500 Units from Origin');
legend('Location', 'Southeast');
ylim([30 100])
grid on;
hold off;

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

figure(3);
hold on

% Plot(s) for result 5 data
% Plot Static DSR with error bars
errorbar(result5_distance, result5_static_dsr, result5_static_std, 'o', ...
    'Color', [0.6350 0.0780 0.1840], 'MarkerFaceColor', [0.6350 0.0780 0.1840], ...
    'DisplayName', '3 Static Defenses');

% Plot Mobile DSR with error bars
errorbar(result5_distance, result5_mobile_dsr, result5_mobile_std, 's', ...
    'Color', [0 0.4470 0.7410], 'MarkerFaceColor', [0 0.4470 0.7410], ...
    'DisplayName', '3 Mobile Defenses');

% Labels, title, and legend
xlabel('Distance from Origin (units)','fontweight','bold');
ylabel('Defense Success Rate (%)','fontweight','bold');
title('Defense Success Rate of 3-Defense Setups with Increasing Distance from Origin');
legend('Location', 'Northwest');
grid on;
ylim([30 100]);
xlim([50 1050]);
hold off;


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
figure(4);
hold on;

% Group 1: Result 4 5-defense data (4 points)
x1 = 1;
errorbar(x1, result4_5eff_dsr(1), result4_5eff_std(1), 's', 'LineStyle', 'none', ...
    'Color', [0.4940 0.1840 0.5560], 'MarkerFaceColor', [0.4940 0.1840 0.5560], ...
    'DisplayName', 'Pentagon (500)');

% Group 2: Result 6 5-defense data (2 points)
x2 = 2;
errorbar(x2, result6_pentagon_dsr(2), result6_pentagon_std(2), 's', 'LineStyle', 'none', ...
    'Color', [0.3010 0.7450 0.9330], 'MarkerFaceColor', [0.3010 0.7450 0.9330], ...
    'DisplayName', 'Optimal Pentagon (700)');

% Group 3: Result 4 6-defense data (2 points)
x3 = 3;
errorbar(x3, result4_6eff_dsr(1), result4_6eff_std(1), 'd', 'LineStyle', 'none', ...
    'Color', [0.4660 0.6740 0.1880], 'MarkerFaceColor', [0.4660 0.6740 0.1880], ...
    'DisplayName', 'Hexagon (500)');

% Group 4: Result 6 6-defense data (2 points)
x4 = 4;
errorbar(x4, result6_hexagon_dsr(2), result6_hexagon_std(2), 'd', 'LineStyle', 'none', ...
    'Color', [0.3010 0.7450 0.9330], 'MarkerFaceColor', [0.3010 0.7450 0.9330], ...
    'DisplayName', 'Optimal Hexagon (700)');

% Create custom x-axis tick labels to identify each group.
group_labels = { 'Pentagon (3 SD on/off)', ...
                 'Optimal Pentagon (3 SD on/off)', ...
                 'Hexagon (on/off)', ...
                 'Optimal Hexagon (on/off)'};
set(gca, 'XTick', 1:10, 'XTickLabel', group_labels);
xtickangle(45);

xlabel('Defense Setup Type','fontweight','bold');
ylabel('Defense Success Rate (%)','fontweight','bold');
title('Optimization of Mixed Defense Setups by Origin Distance');
legend('Location','Northwest');
grid on;
xlim([0.5 4.5]);
ylim([80 100])
hold off;