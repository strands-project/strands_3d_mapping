function generate_decay_plot()

% SHOT:
ajax_1 = [ 0.283333 0.275 0.283333 0.266667 0.266667 0.258333 0.258333 0.258333 0.25 ];
ajax_2  = [ 0.1375 0.14375 0.14375 0.14375 0.14375 0.14375 0.14375 0.14375 0.14375 ];
ball_1  = [ 0.538462 0.530769 0.515385 0.492308 0.476923 0.476923 0.476923 0.476923 0.476923 ];
cereal_1  = [ 0.278947 0.278947 0.273684 0.263158 0.252632 0.247368 0.247368 0.247368 0.247368 ];
cereal_2  = [ 0.633333 0.633333 0.633333 0.626667 0.593333 0.593333 0.593333 0.593333 0.593333 ];
cereal_3  = [ 0.310526 0.310526 0.310526 0.310526 0.310526 0.310526 0.310526 0.310526 0.310526 ];
clips_1  = [ 0.20625 0.20625 0.20625 0.2 0.2 0.2 0.2 0.19375 0.19375 ];
graphics_1  = [ 0.410345 0.410345 0.4 0.4 0.375862 0.372414 0.368966 0.358621 0.351724 ];
kinect_1  = [ 0.46 0.46 0.46 0.46 0.445 0.445 0.435 0.43 0.42 ];
muesli_1  = [ 0.212 0.212 0.212 0.208 0.204 0.204 0.2 0.2 0.2 ];

% SIFT:
%color_weight = 1.0
%sajax_1 = [ 0.108333 0.1 0.1 0.0916667 0.0916667 0.0916667 0.0916667 0.0916667 0.0916667 ];
%sajax_2 = [ 0.1375 0.1375 0.1375 0.1375 0.1375 0.13125 0.13125 0.125 0.125 ];
%sball_1 = [ 0.138462 0.138462 0.138462 0.130769 0.123077 0.123077 0.123077 0.123077 0.123077 ];
%scereal_1 = [ 0.147368 0.136842 0.136842 0.136842 0.136842 0.136842 0.136842 0.136842 0.136842 ];
%scereal_2 = [ 0.246667 0.246667 0.246667 0.24 0.24 0.226667 0.22 0.22 0.22 ];
%scereal_3 = [ 0.205263 0.194737 0.184211 0.173684 0.173684 0.168421 0.163158 0.163158 0.163158 ];
%sclips_1 = [ 0.1125 0.1125 0.1125 0.1125 0.1125 0.1125 0.1125 0.1125 0.1125 ];
%sgraphics_1 = [ 0.806897 0.806897 0.796552 0.789655 0.786207 0.786207 0.786207 0.786207 0.786207 %];
%skinect_1 = [ 0.225 0.225 0.22 0.215 0.215 0.205 0.205 0.2 0.2 ];
%smuesli_1 = [ 0.188 0.188 0.188 0.184 0.18 0.176 0.176 0.176 0.176 ];

%color_weight = 0.3
sajax_1 = [ 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1  ]; 
sajax_2 = [ 0.125 0.11875 0.125 0.125 0.125 0.125 0.125 0.125 0.1125  ]; 
sball_1 = [ 0.107692 0.107692 0.107692 0.107692 0.107692 0.107692 0.107692 0.107692 0.107692  ]; 
scereal_1 = [ 0.173684 0.173684 0.173684 0.173684 0.173684 0.173684 0.173684 0.173684 0.173684  ]; 
scereal_2 = [ 0.34 0.326667 0.326667 0.326667 0.326667 0.326667 0.326667 0.326667 0.326667  ]; 
scereal_3 = [ 0.363158 0.363158 0.352632 0.347368 0.347368 0.347368 0.347368 0.347368 0.347368  ]; 
sclips_1 = [ 0.09375 0.09375 0.09375 0.09375 0.09375 0.09375 0.09375 0.09375 0.09375  ]; 
sgraphics_1 = [ 0.931035 0.931035 0.927586 0.92069 0.917241 0.917241 0.917241 0.913793 0.906897  ]; 
skinect_1 = [ 0.4 0.395 0.395 0.395 0.39 0.38 0.375 0.375 0.37  ]; 
smuesli_1 = [ 0.232 0.232 0.228 0.228 0.228 0.224 0.22 0.216 0.212  ];

mean_decay = (ajax_1/ajax_1(1)+ajax_2/ajax_2(1)+ball_1/ball_1(1)+cereal_1/cereal_1(1)+cereal_2/cereal_2(1)+cereal_3/cereal_3(1)+clips_1/clips_1(1)+graphics_1/graphics_1(1)+kinect_1/kinect_1(1)+muesli_1/muesli_1(1))/10;

smean_decay = (sajax_1/sajax_1(1)+sajax_2/sajax_2(1)+sball_1/sball_1(1)+scereal_1/scereal_1(1)+scereal_2/scereal_2(1)+scereal_3/scereal_3(1)+sclips_1/sclips_1(1)+sgraphics_1/sgraphics_1(1)+skinect_1/skinect_1(1)+smuesli_1/smuesli_1(1))/10;

scenes = linspace(18389, 18389+17557, 9);

figure
hold on
xlabel('Number of segments') % should be number of scenes instead
ylabel('Mean recognition rate drop')

plot(scenes, mean_decay, 'LineWidth', 2, scenes, smean_decay, 'LineWidth', 2)
[legh,objh,outh,outm] = legend('SHOT', 'SIFT+RGB');
set(objh,'linewidth',2);

figure
hold on
xlabel('Number of segments') % should be number of scenes instead
ylabel('Recognition rate')

plot(scenes, ajax_1, '-o', 'LineWidth', 2, scenes, ajax_2, '-o', 'LineWidth', 2, scenes, ball_1, '-o', 'LineWidth', 2, scenes, cereal_1, '-o', 'LineWidth', 2, scenes, cereal_2, '-o', 'LineWidth', 2, scenes, cereal_3, 'LineWidth', 2, scenes, clips_1, 'LineWidth', 2, scenes, graphics_1, 'LineWidth', 2, scenes, kinect_1, 'LineWidth', 2, scenes, muesli_1, 'LineWidth', 2)
[legh,objh,outh,outm] = legend('ajax_1','ajax_2', 'ball_1', 'cereal_1', 'cereal_2', 'cereal_3', 'clips_1', 'graphics_1', 'kinect_1', 'muesli_1', 'Location','southwest');
set(objh,'linewidth',2);

figure
hold on
xlabel('Number of segments') % should be number of scenes instead
ylabel('Recognition rate')

plot(scenes, sajax_1, '-o', 'LineWidth', 2, scenes, sajax_2, '-o', 'LineWidth', 2, scenes, sball_1, '-o', 'LineWidth', 2, scenes, scereal_1, '-o', 'LineWidth', 2, scenes, scereal_2, '-o', 'LineWidth', 2, scenes, scereal_3, 'LineWidth', 2, scenes, sclips_1, 'LineWidth', 2, scenes, sgraphics_1, 'LineWidth', 2, scenes, skinect_1, 'LineWidth', 2, scenes, smuesli_1, 'LineWidth', 2)
[legh,objh,outh,outm] = legend('ajax_1','ajax_2', 'ball_1', 'cereal_1', 'cereal_2', 'cereal_3', 'clips_1', 'graphics_1', 'kinect_1', 'muesli_1', 'Location','southwest');
set(objh,'linewidth',2);

end
