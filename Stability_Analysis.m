posture = 1;
fb_mult_P = 1.0;
fb_mult_D = fb_mult_P*0.1;
GTO_gain = 1;
delay_multiplier = 1;

Factor_of_Safety = 2;

[~, sys_u2u_fb] = Full_Model(posture,fb_mult_P,fb_mult_D,GTO_gain,delay_multiplier);

Gm = zeros(4); Pm = zeros(4); Wcg = zeros(4); Wcp = zeros(4);
for j = 1:4 % j'th input
    for i = 1:4 % i'th output
        [Gm(i,j),Pm(i,j),Wcg(i,j),Wcp(i,j)] = margin(sys_u2u_fb(i,j));
    end
end

% Correct PD gains with factor of safety applied.
fb_mult_P = min(min(Gm))/Factor_of_Safety;
fb_mult_D = fb_mult_P/10; % as suggested by Winters & Crago