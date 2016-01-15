function print_control_struct( Gdff, Gdfb, Id, cMode )
% Prints the control struct used by the software as defined by the input 
% feed-forward and feed-back discrete systems
if nargin < 3
    Id = 0;
end
if nargin < 4
    cMode = 0;
end

if Gdff.Ts ~= Gdfb.Ts
    error('Sample time for the systems is different');
end

if cMode == 0
  so = sprintf('control_params_struct posControlParams = \n{');
else
  so = sprintf('control_params_struct velControlParams = \n{');
end

so = strcat(so,sprintf('\n\t.cMode = %d,',cMode));

so = strcat(so,sprintf('\n\t.Fs = %d,',round(1/Gdff.Ts)));

[Nff,Dff] = tfdata(Gdff);
[Nfb,Dfb] = tfdata(Gdfb);
if Dff{1} ~= Dfb{1}
    error('The systems got different denominators');
end

% D
st = '';
for i = length(Nfb{1}):-1:1
    st = strcat(st,sprintf('%0.4f',Nfb{1}(i)));
    if i ~= 1
        st = strcat(st,', ');
    end
end

so = strcat(so,sprintf('\n\t.nd = %d, .d = {%s},',length(Nfb{1}),st));

% C
st = '';
for i = length(Nff{1}):-1:1
    st = strcat(st,sprintf('%0.4f',Nff{1}(i)));
    if i ~= 1
        st = strcat(st,', ');
    end
end

so = strcat(so,sprintf('\n\t.nc = %d, .c = {%s},',length(Nff{1}),st));

% F
st = '';
for i = length(Dfb{1}):-1:2
    st = strcat(st,sprintf('%0.4f',Dfb{1}(i)));
    if i ~= 2
        st = strcat(st,', ');
    end
end

so = strcat(so,sprintf('\n\t.nf = %d, .f = {%s},',length(Dfb{1})-1,st));

try 
    NUM = tfdata(Id);
    so = strcat(so,sprintf('\n\t.I = %0.4f',max(NUM{1})));
catch
    so = strcat(so,sprintf('\n\t.I = %0.4f',Id));
end

so = strcat(so,sprintf('\n};'));

%disp(so);
fprintf('%s\n',so);
end

