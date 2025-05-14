clear;
clc;
close all;

files_folder = "C:\Users\alvar\OneDrive\Desktop\My code\repositorios\TFM_Code\fast_marching-master\functions\Archivos";
main_folder = "C:\Users\alvar\OneDrive\Desktop\My code\repositorios\TFM_Code\fast_marching-master\functions\Ver_Resultados";

% Read memory log file
cd(files_folder)
fid = fopen('memory_log.txt', 'r');
if fid == -1
    error('Cannot open memory log file');
end

% Initialize arrays
checkpoints = {};
working_set = [];
private_usage = [];

% Parse file
while ~feof(fid)
    line = fgetl(fid);
    if contains(line, '=== Memory Checkpoint:')
        checkpoint = extractBetween(line, 'Checkpoint: ', ' ===');
        checkpoints{end+1} = char(checkpoint); % Convert to char array
    elseif contains(line, 'Working Set:') && ~contains(line, 'Peak')
        working_set(end+1) = sscanf(line, 'Working Set: %f');
    elseif contains(line, 'Private Usage:')
        private_usage(end+1) = sscanf(line, 'Private Usage: %f');
    end
end
fclose(fid);
cd(main_folder)

% Create figure
figure('Name', 'Memory Usage Analysis', 'Position', [100 100 1200 600]);

% Plot memory usage
plot(working_set, '-bo', 'LineWidth', 2, 'DisplayName', 'Working Set');
hold on;
plot(private_usage, '-ro', 'LineWidth', 2, 'DisplayName', 'Private Usage');
grid on;
title('Memory Usage Over Time');
ylabel('Memory (MB)');
legend('Location', 'northwest');
xticks(1:length(checkpoints));
xticklabels(checkpoints);
xtickangle(45);

% Add horizontal line at y=1 for reference
yline(1, '--k', 'Reference Line');

% Adjust layout
sgtitle('Fast Marching Memory Analysis');
set(gcf, 'Color', 'white');

% Print statistics
fprintf('\nMemory Usage Statistics:\n');
fprintf('Peak Working Set: %.2f MB\n', max(working_set));
fprintf('Peak Private Usage: %.2f MB\n', max(private_usage));
fprintf('Average Working Set: %.2f MB\n', mean(working_set));
fprintf('Average Private Usage: %.2f MB\n', mean(private_usage));
fprintf('Working Set / Private Usage Ratio: %.2f\n', mean(working_set./private_usage));
