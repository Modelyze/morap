function wp = beam_model( t,w,J,c )
% Simple friction model for a free running beam
if length(c) == 1
    c = [0 c];
end

if w < 1e-2
    wp = 1/J*(-(c(2)*w));
else
    wp = 1/J*( -(c(2)*w)- c(1) );
end

end

