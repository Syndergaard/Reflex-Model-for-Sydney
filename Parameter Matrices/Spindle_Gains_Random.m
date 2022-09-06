chalmers_stdv = mean([.21/.38,.12/.17,.19/.63,.15/.31]); % Mean of normalized standard deviations from Chalmers 1997

G = readmatrix("Spindle_Gains.csv");
figure(1); imagesc(G); colorbar

smallest_gain = min(abs(G(~~G))); % extracts the gain with the smallest nonzero absolute value

for i = 1:4
    for j = 1:4
        G(i,j) = G(i,j) + 2*chalmers_stdv*max(G(i,j),smallest_gain)*randn(1); 
    end
end

figure(2); imagesc(G); colorbar

writematrix(G,'Spindle_Gains_Randomized.csv');