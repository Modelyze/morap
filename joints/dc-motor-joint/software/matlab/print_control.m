function print_control( cMode, umax, imax, K, n, R, Gdff, Gdfb, Ifd )
% prints control parameters for used in the implemented controller
[NUMfb, DEN] = tfdata(Gdfb); NUMff = tfdata(Gdff);
NUMfb = NUMfb{1}; NUMff = NUMff{1}; DEN = DEN{1}(2:end);
nd = length(NUMfb); nc = length(NUMff);  nf = length(DEN);
if nargin > 8
     NUMI = tfdata(Ifd);
     NUMI = NUMI{1};
     ni = length(NUMI);
end

fprintf('K = %0.4f; n = %0.4f; R = %0.4f;\n',K,n,R);
fprintf('cMode = %d; umax = %0.1f; imax = %0.1f;\n',cMode,umax,imax);

fprintf('Ts = 1/%0.2f;\n',1/Gdff.Ts);
st = sprintf('nd = %d; d = [',nd);
for i = nd:-1:1
    st = strcat(st,sprintf('%0.8f',NUMfb(i)));
    if i ~= 1, st = strcat(st,', '); end
end
st = strcat(st,'];');
disp(st);
st = sprintf('nc = %d; c = [',nc);
for i = nc:-1:1
    st = strcat(st,sprintf('%0.8f',NUMff(i)));
    if i ~= 1, st = strcat(st,', '); end
end
st = strcat(st,'];');
disp(st);
st = sprintf('nf = %d; f = [',nf);
for i = nf:-1:1
    st = strcat(st,sprintf('%0.8f',DEN(i)));
    if i ~= 1, st = strcat(st,', '); end
end
st = strcat(st,'];');
disp(st);
if nargin > 8 % Includes this part
%     st = sprintf('ni = %d; I = [',ni);
%     for i = ni:-1:1
%         st = strcat(st,sprintf('%0.8f',NUMI(i)));
%         if i ~= 1, st = strcat(st,', '); end
%     end
%     st = strcat(st,'];');
%     disp(st);
fprintf('Id = %0.8f;\n',max(NUMI));
else
%     fprintf('ni = 0; I = 0;\n');
fprintf('Id = 0;\n');
end


end