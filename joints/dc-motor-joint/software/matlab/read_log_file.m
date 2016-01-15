function D = read_log_file(filename,tauLP,tauHP)
% Parses info from log file to matlab array
% The log file can contain multiple logs but 
% each log needs to begin with the string:
% 'starting logging' and end with the string:
% 'logging completed!'
%
% Input:
% filename for the log file
% time constant for the low pass filter applied on the velocity (optional)
%
% Cell array containing all the different logs
% where each log is a stuct with the fields
% .time, .pos, .vel, .volt

if nargin < 2
    tauLP = 0;
end
if nargin < 3
    tauHP = 0;
end

function hdr = get_file_header(s)
    % Returns the construction of the header and thus the 
    % type of data constructed. Returns a 1xn vector where
    % n is the number of elements. The values in the vector
    % can have the following values and meanings:
    % 1  - Time
    % 2  - Position value
    % 3  - Velocity value
    % 10 - Voltage value for the previous position value
    % 11 - Current value vor the previous position value
    % 99 - Unknown
try
    
    SV = textscan(s,'%s','delimiter','|');
    hdr = [];
    for iin = 1:length(SV{1})
        if ~isempty(SV{1}{iin})
            sd = lower(deblank(SV{1}{iin}));
            
            if strncmp('time',sd,4)
                hdr(end+1) = 1;
            elseif strncmp('motor angle',sd,11)
                hdr(end+1) = 2;
            elseif strncmp('gyro value',sd,10)
                hdr(end+1) = 3;
            elseif strncmp('motor voltage',sd,13);
                hdr(end+1) = 10;
            elseif strncmp('motor current',sd,13);
                hdr(end+1) = 11;
            else
                fprintf('Unknown header field: %s\n',sd);
                hdr(end+1) = 99;
            end
        end % of isempty()
    end % of for
    
catch MEin
    disp('An error occured when building file header :(');
    disp(MEin);
    hdr = [];
end
end


fid = fopen(filename);
if (fid == -1)
    error('Cant open the file');
end

try
    D = {};
    i_number = 1;
    
    while 1
        prevline = ''; nameline = '';
        s = fgetl(fid);
        while(strncmp(s,'starting logging',15) ~= 1)
            if s == -1
                return;
            end
            if ~isempty(s) && s(1) == '|' && s(end) == '|'
                hdr = get_file_header(s);
                nameline = prevline;
            end
            if ~isempty(s)
                prevline = s;
            end
            s = fgetl(fid);
        end

        D{end+1} = struct();
        % Allocate struct fields
        n_alloc = 1e4;
        if any(hdr == 1)
            D{end}.time = zeros(n_alloc,1);
        end
        if any(hdr == 2 | hdr == 3)
            nelem = sum(hdr == 2 | hdr == 3);
            D{end}.pos = zeros(n_alloc,nelem);
            D{end}.vel = zeros(n_alloc,nelem);
        end
        if any(hdr == 10)
            nelem = sum(hdr == 10);
            D{end}.volt = zeros(n_alloc,nelem);
        end
        if any(hdr == 11)
            nelem = sum(hdr == 11);
            D{end}.current = zeros(n_alloc,nelem);
        end
        
        % Some warnings
        if sum(hdr == 1) > 1
            fprintf('Input header has multiple time fields\n');
        end
        if sum(hdr == 1) == 0
            fprintf('Input header has no time field\n');
        end
        

        % Goes through all the logging data
        i = 1;
        s = fgets(fid);
        while(strncmp(s,'logging completed',17) ~= 1)
            if (strncmp(s,'Error',5) ~= 1) % In case of transmission error
                r = sscanf(s,'%f,');
                if length(r) ~= length(hdr)
                    error('Header length (%d) is not equal to data length (%d)',length(hdr),length(r));
                end
                i_elem = 0; i_volt = 0; i_amp = 0;
                for i_hdr = 1:length(hdr)
                    if hdr(i_hdr) == 1
                        D{end}.time(i) = r(i_hdr);
                    elseif hdr(i_hdr) == 2
                        i_elem = i_elem + 1;
                        D{end}.pos(i,i_elem) = r(i_hdr);
                    elseif hdr(i_hdr) == 3
                        i_elem = i_elem + 1;
                        D{end}.vel(i,i_elem) = r(i_hdr);
                    elseif hdr(i_hdr) == 10
                        i_volt = i_volt + 1;
                        D{end}.volt(i,i_volt) = r(i_hdr);
                    elseif hdr(i_hdr) == 11
                        i_amp = i_amp + 1;
                        D{end}.current(i,i_amp) = r(i_hdr);
                    end
                end          

                i = i + 1;
            end % of if(!error)

            s = fgets(fid);
        end % of while(logging)
        
        % Removes excess
        if isfield(D{end},'time')
            D{end}.time = D{end}.time(1:(i-1),:);
        end
        if isfield(D{end},'pos')
            D{end}.pos = D{end}.pos(1:(i-1),:);
            D{end}.vel = D{end}.vel(1:(i-1),:);
        end
        if isfield(D{end},'volt')
            D{end}.volt = D{end}.volt(1:(i-1),:);
        end
        if isfield(D{end},'current')
            D{end}.current = D{end}.current(1:(i-1),:);
        end
        
        % Build up missing pos and vel fields by integrating or derivating
        % the other field        
        Ts = mean(diff(D{end}.time));
        
        for i_build = [1:sum(hdr == 2 | hdr == 3); hdr(hdr == 2 | hdr == 3)]
            if i_build(2) == 2
                % derivate pos to get vel
                if tauLP < Ts
                    D{end}.vel(:,i_build(1)) = [0; diff(D{end}.pos(:,i_build(1)))./diff(D{end}.time)];
                else
                    a = Ts/tauLP;
                    D{end}.vel(:,i_build(1)) = filter(a,[1 -(1-a)],[0; diff(D{end}.pos(:,i_build(1)))./diff(D{end}.time)]);        
                end
            elseif i_build(2) == 3
                % integrate vel to get pos
                if tauHP < Ts
                    D{end}.pos(:,i_build(1)) = filter([0 Ts],[1 -1],D{end}.vel(:,i_build(1)));
                else
                    %vel_temp = filter([1 -1],[1 (Ts-tauHP)/tauHP],D{end}.vel(:,i_build(1)));
                    a = tauHP/(tauHP + Ts);
                    vel_temp = filter([a -a],[1 -a],D{end}.vel(:,i_build(1)) );
                    D{end}.pos(:,i_build(1)) = filter([0 Ts],[1 -1],vel_temp);                    
                    %D{end}.pos(:,i_build(1)) = filter([0 Ts -Ts],[1 -2 ((Ts^2+tauHP)/tauHP)],D{end}.vel(:,i_build(1)));                    
                end
            end
        end
        
        
        
        
        % Add a name
%         nm = strsplit(filename,{'/','\'},'CollapseDelimiters',true);
%         nm = strsplit(nm{end},'.');
%         D{end}.name = sprintf('%s %d',strjoin(nm(1:end-1),'.'),i_number);
        D{end}.name = deblank(nameline);
        
        i_number = i_number + 1;
    end
catch ME
    disp('An error occured :(');
    disp(ME);
    D = 0;
end




fclose(fid);

end