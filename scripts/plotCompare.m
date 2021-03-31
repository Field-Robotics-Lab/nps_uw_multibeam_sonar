clear;clc;close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nBeams = 512;
clims_base = [0 50];
start_dir = 1;
maxDataset = 5;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
freqMode = 'LF';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% caseSet = 'Background';
% caseSet = 'Horiz_Cylinder';
caseSet = 'Vert_Cylinder';
% caseSet = 'Plate';
% caseSet = 'Plate2';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
yPlotRange = 1;
xPlotRange = yPlotRange*cos(pi()/4);

fig = figure('position',[100,100,1100,450]);
set(gcf,'color','w');
for k=1:2
    % Construct casename
    if k == 1 % --------- Simulation
        if strcmp(caseSet,'Plate2')
            casename = ['Recordings/record_' freqMode '_Plate'];
        else
            if strcmp(caseSet,'Vert_Cylinder2')
            casename = ['Recordings/record_' freqMode '_Vert_Cylinder' ];
            else
            casename = ['Recordings/record_' freqMode '_' caseSet];
            end
        end
        subtitletext = 'Simulation';
    else % -------------- Dataset
        casename = ['Dataset/dataset_' freqMode '_' caseSet];
        %%%%%%%%%%%%%%%%%%%%%%%%%%% STRANGE %%%%%%%%%%%%%%%%%%%%%%%%%%
        nBeams = nBeams/2;
        %%%%%%%%%%%%%%%%%%%%%%%%%%% STRANGE %%%%%%%%%%%%%%%%%%%%%%%%%%
        subtitletext = 'Experiment';
    end
    % Set FOV according to frequency mode
    if strcmp(freqMode,'HF')
        FOV = 60/180*pi();
        titletext = [caseSet '@ 2.1 MHz'];
    elseif strcmp(freqMode,'LF')
        FOV = 130/180*pi();
        titletext = [caseSet '@ 1.2 MHz'];
    end
    titletextList = split(titletext,'_');
    
    % number of datasets for averaging
    all_files = dir(casename);
    all_dir = all_files([all_files(:).isdir]);
    num_dir = numel(all_dir)-2;
    if num_dir > start_dir+maxDataset
        end_dir = start_dir+maxDataset;
    else
        end_dir = num_dir;
    end
    
    % ---------- Read data ------------ %
    opts = delimitedTextImportOptions("NumVariables", 1);
    opts.DataLines = [1, Inf]; opts.VariableNames = "data"; opts.VariableTypes = "double";
    opts.Delimiter = ","; opts.ExtraColumnsRule = "ignore"; opts.EmptyLineRule = "read";
    % Read ranges
    filename = [casename '\' num2str(1) '\ranges'];
    ranges = readtable(filename, opts).data;
    % Read azimuth_angles
    filename = [casename '\' num2str(1) '\azimuth_angles'];
    azimuth_angles = readtable(filename, opts).data;
    % Read azimuth_beamwidth
    filename = [casename '\' num2str(1) '\azimuth_beamwidth'];
    azimuth_beamwidth = readtable(filename, opts).data;
    % Read elevation_beamwidth
    filename = [casename '\' num2str(1) '\elevation_beamwidth'];
    elevation_beamwidth = readtable(filename, opts).data;
    nn = 1;
    for n = start_dir:end_dir
        % Read intensities
        filename = [casename '\' num2str(n) '\intensities'];
        intensities_tot{nn} = readtable(filename, opts).data;
        nn = nn + 1;
    end
    for s = 1:length(intensities_tot{1})
        intensities(s) = 0;
        for n = 1:length(intensities_tot)
            intensities(s) = intensities(s)+intensities_tot{n}(s)/(end_dir-start_dir+1);
        end
    end
    
%     figure;set(gcf,'color','w');
%     for i=2:length(azimuth_angles)
%         increments(i) = (azimuth_angles(i)-azimuth_angles(i-1))/pi()*180;
%     end
%     bar(increments);
%     legend('azimuth angle spacing [deg]');
%     title(['azimuth beamwidth = ' num2str(azimuth_beamwidth/pi()*180)]);
%     clearvars increments

%     figure;set(gcf,'color','w');
%     for i=2:length(ranges)
%         increments(i) = (ranges(i)-ranges(i-1));
%     end
%     bar(increments);
%     legend('range spacing [m]');
%     clearvars increments

    clearvars Beams dist plotData
    plotSkips = 1;
    iIndex = 0;
    [Beams,dist] = ndgrid(1:length(1:plotSkips:nBeams), (100:length(intensities(:,1)))/1500);
    
    for i=1:nBeams
        iIndex = iIndex + 1;
        jIndex = 0;
        for j=1:length(ranges)
            jIndex = jIndex + 1;
            plotData(iIndex,jIndex) = intensities((j-1)*nBeams + i);
        end
    end
    
    range_vector = ranges';
    
    fl = nBeams / (2.0 * tan(FOV/2.0));
    sonarBeams = atan2( ((1:nBeams)-1) - 0.5 *(nBeams-1), fl);
    
    x = range_vector.*cos(azimuth_angles);
    y = range_vector.*sin(azimuth_angles);
    
    disp(['Range data length  :' num2str(length(ranges))])
    disp(['Range Resolution   :' num2str(ranges(10)-ranges(9))])
    disp(['Number of datasets :' num2str(end_dir-start_dir+1)])
    disp(['Start dir number   :' num2str(start_dir)])
    disp('')
    
    subplot(1,2,k)
    scatterPointSize = 8;
    scatter(y(:),x(:),scatterPointSize,20*log10(abs(plotData(:))),'filled')
    % scatter(y(:),x(:),scatterPointSize,abs(plotData(:)),'filled')
    % clims = clims_base + 20*log10(max(max(abs(plotData))));
    xlabel('X [m]'); ylabel('Y [m]')
    clims = clims_base; caxis(clims); colorbar
    h = colorbar; ylabel(h,'Echo Level')
    
    colormap(hot)
%     % set(gca,'Color','k')
%     ylim(1.02*[0 yPlotRange])
%     xlim(1.02*[-xPlotRange xPlotRange])
    axis equal; axis tight; grid on; set(gca,'FontSize',12)
    title(subtitletext);
end
sgtitle([titletextList{1} ' ' titletextList{2}]); set(gca,'FontSize',12)
