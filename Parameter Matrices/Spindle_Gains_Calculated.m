% Attempt at approximating the Reflex Gain matrix.

% Agonistic muscles have positive gain.
% Antagonistic muscles have negative gain.
% Muscles which are neither agonists nor antagonists have 0 gain.

% Calculation is as follows:
    % Think of each column in the moment-arm matrix as a 7-dimensional vector.
    % The direction of the vector points to the amount it acts in each DOF.
    % Each vector is normalized to have length 1. The magnitude of each
    % muscle in the direction of every other muscle is computed using the
    % dot-product and is equal to the reflex gain between those two
    % muscles.

% Muscles are in the following order:
    % 1. Delt_Ant, 2. Delt_Lat, 3. Delt_Post, 4. Pec_Maj, 5. Bi_Long
    % 6. Bi_short, 7. Tri_Long, 8. Tri_Lat, 9. Bra, 10. Brd, 11. PT
    % 12. ECR, 13. ECU, 14. FCR, 15. FCU
    
muscle_names = {'ECR';'ECU';'FCR';'FCU'};

% function G2 = Calculated_Reflex_Gains()

    M = csvread('M1.csv');

    G1 = csvread('Spindle_Gains.csv');
    figure(1); clf;
    imagesc(G1)
    colorbar('southoutside')
    xticks(1:4); xticklabels(muscle_names); xtickangle(45);
    yticks(1:4); yticklabels(muscle_names);

    G2 = zeros(4);
    for i = 1:4
        for j = 1:4
            a = M(:,i)/norm(M(:,i));
            b = M(:,j)/norm(M(:,j));
            G2(i,j) = dot(a,b);
        end
    end
    figure(2); clf;
    imagesc(G2)
    colorbar('southoutside')
    
    xticks(1:4); xticklabels(muscle_names); xtickangle(45);
    yticks(1:4); yticklabels(muscle_names);

    writematrix(G2,'Spindle_Gains_Calculated.csv');