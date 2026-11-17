%% Build & Simulate 5V-to-12V BJT-MOSFET Heater Coil Test Model
% Requires: Simulink + Simscape + Simscape Electrical

modelName = 'heater_driver_test';

% Close and recreate model cleanly if it already exists
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
new_system(modelName);
open_system(modelName);

% Configure stiff solver required for switching circuits
set_param(modelName, 'Solver', 'ode23t', 'StopTime', '4');

%% 1. Add Blocks
% Control Input (Arduino 5V Pulse)
add_block('simulink/Sources/Pulse Generator', [modelName '/Arduino_5V_Pulse'], ...
    'Amplitude', '5', 'Period', '2', 'PulseWidth', '50', 'PhaseDelay', '0.5');
add_block('nesl_utility/Simulink-PS Converter', [modelName '/SL_to_PS']);
add_block('fl_lib/Electrical/Electrical Sources/Controlled Voltage Source', [modelName '/Arduino_Pin']);

% 5V-to-12V BJT Stage
add_block('fl_lib/Electrical/Electrical Elements/Resistor', [modelName '/R1_Base'], 'R', '2200');
add_block('ee_lib/Semiconductors & Converters/NPN Bipolar Transistor', [modelName '/Q1_NPN']);
add_block('fl_lib/Electrical/Electrical Elements/Resistor', [modelName '/R2_PullUp'], 'R', '2200');
add_block('fl_lib/Electrical/Electrical Sources/DC Voltage Source', [modelName '/V12_Gate'], 'v0', '12');

% MOSFET + Heater Coil Stage
add_block('fl_lib/Electrical/Electrical Elements/Resistor', [modelName '/Rg_Gate'], 'R', '10');
add_block('ee_lib/Semiconductors & Converters/N-Channel MOSFET', [modelName '/M1_MOSFET']);
add_block('fl_lib/Electrical/Electrical Elements/Resistor', [modelName '/R_Heater_Coil'], 'R', '10');
add_block('fl_lib/Electrical/Electrical Sources/DC Voltage Source', [modelName '/V12_Heater'], 'v0', '12');

% Ground & Solver
add_block('fl_lib/Electrical/Electrical Elements/Electrical Reference', [modelName '/GND']);
add_block('nesl_utility/Solver Configuration', [modelName '/Solver_Config']);

% Sensors & Scope
add_block('fl_lib/Electrical/Electrical Sensors/Voltage Sensor', [modelName '/V_Gate_Sensor']);
add_block('fl_lib/Electrical/Electrical Sensors/Current Sensor', [modelName '/I_Heater_Sensor']);
add_block('nesl_utility/PS-Simulink Converter', [modelName '/PS_to_SL_Gate']);
add_block('nesl_utility/PS-Simulink Converter', [modelName '/PS_to_SL_Current']);
add_block('simulink/Sinks/Scope', [modelName '/Test_Scope'], 'NumInputPorts', '3');

%% 2. Wire Connections (Physical Conserving & Signal Lines)
% Solver & Ground
add_line(modelName, 'Solver_Config/RConn1', 'GND/LConn1', 'autorouting', 'on');

% Arduino 5V Signal Path
add_line(modelName, 'Arduino_5V_Pulse/1', 'SL_to_PS/1', 'autorouting', 'on');
add_line(modelName, 'SL_to_PS/RConn1', 'Arduino_Pin/RConn1', 'autorouting', 'on');
add_line(modelName, 'Arduino_Pin/LConn1', 'R1_Base/LConn1', 'autorouting', 'on');
add_line(modelName, 'Arduino_Pin/RConn2', 'GND/LConn1', 'autorouting', 'on');

% BJT Base, Emitter, Collector
add_line(modelName, 'R1_Base/RConn1', 'Q1_NPN/LConn1', 'autorouting', 'on');       % Base
add_line(modelName, 'Q1_NPN/RConn2', 'GND/LConn1', 'autorouting', 'on');            % Emitter to GND
add_line(modelName, 'V12_Gate/LConn1', 'R2_PullUp/LConn1', 'autorouting', 'on');    % +12V to R2
add_line(modelName, 'V12_Gate/RConn1', 'GND/LConn1', 'autorouting', 'on');          % 12V GND
add_line(modelName, 'R2_PullUp/RConn1', 'Q1_NPN/RConn1', 'autorouting', 'on');      % R2 to Collector

% Collector -> MOSFET Gate
add_line(modelName, 'Q1_NPN/RConn1', 'Rg_Gate/LConn1', 'autorouting', 'on');
add_line(modelName, 'Rg_Gate/RConn1', 'M1_MOSFET/LConn1', 'autorouting', 'on');     % Gate
add_line(modelName, 'M1_MOSFET/RConn2', 'GND/LConn1', 'autorouting', 'on');         % Source to GND

% Heater Supply -> Current Sensor -> Heater Coil -> MOSFET Drain
add_line(modelName, 'V12_Heater/LConn1', 'I_Heater_Sensor/LConn1', 'autorouting', 'on');
add_line(modelName, 'V12_Heater/RConn1', 'GND/LConn1', 'autorouting', 'on');
add_line(modelName, 'I_Heater_Sensor/RConn2', 'R_Heater_Coil/LConn1', 'autorouting', 'on');
add_line(modelName, 'R_Heater_Coil/RConn1', 'M1_MOSFET/RConn1', 'autorouting', 'on'); % Drain

% Measure Gate Voltage & Heater Current -> Scope
add_line(modelName, 'M1_MOSFET/LConn1', 'V_Gate_Sensor/LConn1', 'autorouting', 'on');
add_line(modelName, 'V_Gate_Sensor/RConn2', 'GND/LConn1', 'autorouting', 'on');
add_line(modelName, 'V_Gate_Sensor/RConn1', 'PS_to_SL_Gate/LConn1', 'autorouting', 'on');
add_line(modelName, 'I_Heater_Sensor/RConn1', 'PS_to_SL_Current/LConn1', 'autorouting', 'on');

add_line(modelName, 'Arduino_5V_Pulse/1', 'Test_Scope/1', 'autorouting', 'on');
add_line(modelName, 'PS_to_SL_Gate/1', 'Test_Scope/2', 'autorouting', 'on');
add_line(modelName, 'PS_to_SL_Current/1', 'Test_Scope/3', 'autorouting', 'on');

%% 3. Auto-Layout, Save & Run
Simulink.BlockDiagram.arrangeSystem(modelName);
save_system(modelName);
sim(modelName);
open_system([modelName '/Test_Scope']);