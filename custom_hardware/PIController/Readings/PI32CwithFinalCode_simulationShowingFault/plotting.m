
%% =========================================================
% PANEL 3: ACTUATOR ON/OFF TIMELINE
% Same plotting method as the original live plot
% ==========================================================

ax3 = subplot(3,1,3);
hold(ax3,'on');
grid(ax3,'on');
box(ax3,'on');

% Read actual relay commands from CSV
% +1 = Heater ON
% -1 = Fan ON
%  0 = Both OFF

heaterY = double(relay == 1)*0.5 + 0.5;
fanY    = double(relay == -1)*0.5;

% Original area plotting method
hHeaterBand = area(ax3,t,heaterY,0.5);

set(hHeaterBand, ...
    'FaceColor',[0.85 0.15 0.15], ...
    'FaceAlpha',0.75, ...
    'EdgeColor',[0.5 0 0], ...
    'LineWidth',0.8);

hFanBand = area(ax3,t,fanY,0);

set(hFanBand, ...
    'FaceColor',[0.15 0.35 0.85], ...
    'FaceAlpha',0.75, ...
    'EdgeColor',[0 0 0.5], ...
    'LineWidth',0.8);

ylim(ax3,[0 1]);

yticks(ax3,[0.25 0.75]);

yticklabels(ax3,{'Fan','Heater'});

ylabel(ax3,'Actuator');

xlabel(ax3,'Time [s]');

title(ax3,'Actuator ON/OFF Timeline');

legend(ax3,[hHeaterBand hFanBand], ...
    {'Heater','Fan'}, ...
    'Location','northeast');

xlim(ax3,[0 600]);
