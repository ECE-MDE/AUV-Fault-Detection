%==========================================================================
% Description: Parses and plots shared 690 data maneuvering data
% 
% Ver 1: 18 May 2021, Dan Stilwell
%
%==========================================================================

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all

DEGREES = 0;
RADIANS = 1;

ANGLES = DEGREES;   % plot angles in degrees or radians

cd 
% list of files: edit this list as needed
log_files = [   "share_pathfinder_dvl_node.velocity.dat", "DVL";
                   "share_depth_node.depth.dat", "depth";
                   "share_pid_attitude_control_node.pitch_pid.dat", "pitch control";
                   "share_pid_attitude_control_node.fins.dat", "flaps";
                   "share_pathfinder_dvl_node.height.dat", "altitude";
                   "share_inertial_nav_node.x.dat","Inertial Nav";
                   "share_pid_attitude_control_node.yaw_pid.dat","Yaw";
                   "share_ss_line_control_node.ss.dat", "line tracking";
                ];

saved_data_path = "C:\Users\stilw\My Drive\work\field_campaigns\2021.June.NRL\field_data\avl_log_plotter\SHARED\";


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% load tables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

number_of_log_files = size(log_files, 1);
    
for i = 1:number_of_log_files            

    data_filename = strcat(saved_data_path, log_files(i,1));
    opts = detectImportOptions(data_filename);

    % 1st row is description of data, used for plot title
    % 2nd row contains variable names
    % 3rd row containts units
      
    opts.DataLines =  [4 Inf];
    opts.VariableDescriptionsLine = 1;
    opts.VariableNamesLine= 2;
    opts.VariableUnitsLine = 3;
    opts.VariableNamingRule =  'preserve';
    
    p = readtable(data_filename,opts);
    
    % put all tables and related data in an array
    data_tables(i).table = p;                        % the table
    data_tables(i).name = log_files(i);    % file name of log file
    data_tables(i).figure_title = p.Properties.VariableDescriptions{1};  % figure title for plotting
    
end

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% plot variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

legend_strings = "";

 for i = 1:number_of_log_files   
    
    % plot time versus columns 2:n
    p = data_tables(i).table;
    
    figure;
    set(gcf,'name', data_tables(i).figure_title);
    set(gcf,'numbertitle','off');
    set(gca, 'LooseInset', get(gca,'TightInset'));
    set(gcf,'WindowStyle', 'docked')
    
    n_columns = size(p,2);
   
    for j=2:n_columns %this is the number of data rows not including time
         
        if (ANGLES == DEGREES)
        
            if ((data_tables(i).table.Properties.VariableUnits{j} == "rad") | ( data_tables(i).table.Properties.VariableUnits{j} == "rad/s" ) )
                
                p{:,j} = p{:,j}*180/pi;
                
                if (data_tables(i).table.Properties.VariableUnits{j} == "rad"), 
                
                    legend_strings(j-1) = sprintf('%s  (%s)', data_tables(i).table.Properties.VariableNames{j},"degrees");
                else
                    legend_strings(j-1) = sprintf('%s  (%s)', data_tables(i).table.Properties.VariableNames{j},"degrees/sec");
                end
            else
                legend_strings(j-1) = sprintf('%s  (%s)', data_tables(i).table.Properties.VariableNames{j},data_tables(i).table.Properties.VariableUnits{j}); 
            end
        end      
    end
     plot(p{:,1}, p{:,2:n_columns})
     legend(legend_strings);
     xlabel('time (secs)');
 end
 
 
 
 
