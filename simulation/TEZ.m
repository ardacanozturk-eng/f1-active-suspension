%% FORMULA 1 AKTIF SUSPANSIYON VE AKTIF YUKSEKLIK SIMULASYONU
% Dumlupinar Universitesi - Elektrik-Elektronik Muhendisligi Bitirme Tezi
clear; clc; close all;

%% 1. Sistem Parametreleri ve Durum-Uzay Modeli
ms = 200; mus = 40; ks = 150000; cs = 1000; kt = 350000;
nominal_yukseklik = 0.050; % 50 mm nominal sürüş yüksekliği

% Durum-Uzay Matrisleri (Giriş: xr - Yol Profilini)
A = [0 1 0 -1;
     -ks/ms -cs/ms 0 cs/ms;
     0 0 0 1;
     ks/mus cs/mus -kt/mus -cs/mus];
B_control = [0; 1/ms; 0; -1/mus]; 
B_road = [0; 0; -1; kt/mus]; 
C = [-ks/ms -cs/ms 0 cs/ms; 1 0 0 0]; 
D = [0; 0];

%% 2. Genişletilmiş LQR-I Kontrolör Tasarımı
A_aug = [A, zeros(4,1); -[1 0 0 0], 0]; 
B_aug = [B_control; 0];

% Optimal Ağırlık Matrisleri (Tuning)
Q_aug = diag([1e8, 1e2, 1e8, 1e2, 1e11]); 
R_aug = 0.01;
K_aug = lqr(A_aug, B_aug, Q_aug, R_aug);
K_x = K_aug(1:4); K_i = K_aug(5);

% Kapalı Döngü Sistem Denklemleri
A_cl_i = [A - B_control*K_x, -B_control*K_i; -[1 0 0 0], 0];
B_cl_i = [B_road; 0];
C_cl_i = eye(5); 
D_cl_i = zeros(5,1);

sys_pasif = ss(A, B_road, eye(4), zeros(4,1));
sys_aktif_i = ss(A_cl_i, B_cl_i, C_cl_i, D_cl_i);

%% 3. Zaman Tanım Kümesi Analizi (2cm Bordür Testi)
t = 0:0.01:2;
yol_h = 0.02; 
yol_step = (t >= 0) * yol_h;

[x_pasif] = lsim(sys_pasif, yol_step, t);
[x_aktif] = lsim(sys_aktif_i, yol_step, t);

y_p = (C * x_pasif')'; 
y_i = (C * x_aktif(:, 1:4)')'; 

% Aktüatör Kuvvet Hesabı ve Donanım Sınırlandırması (Saturasyon)
sat_limit = 2500; 
u_force = zeros(length(t), 1);
for k = 1:length(t)
    u_calc = -K_aug * x_aktif(k, :)';
    u_force(k) = max(-sat_limit, min(sat_limit, u_calc)); 
end

%% 4. Koyu Tema Zaman Kümesi Grafikleri
fig1 = figure('Name', 'F1 Zaman Kumesi Analiz Raporu', 'Color', [0.12 0.12 0.12]);

subplot(3,1,1);
plot(t, y_p(:,1), 'r--', t, y_i(:,1), 'g', 'LineWidth', 2); grid on;
title('Sasi Ivmesi (Aerodinamik Kararlilik & Konfor)', 'Color', 'w'); ylabel('Ivme (m/s^2)', 'Color', 'w');
legend('Pasif', 'Aktif LQR-I', 'Color', [0.2 0.2 0.2], 'TextColor', 'w');
set(gca, 'Color', [0.15 0.15 0.15], 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2);

subplot(3,1,2);
plot(t, y_p(:,2), 'r--', t, y_i(:,2), 'g', 'LineWidth', 2); grid on;
title('Suspansiyon Salinimi (Strok Kullanimi)', 'Color', 'w'); ylabel('Mesafe (m)', 'Color', 'w');
set(gca, 'Color', [0.15 0.15 0.15], 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2);

subplot(3,1,3);
plot(t, u_force, 'b', 'LineWidth', 2); grid on;
title(sprintf('Aktuator Kuvveti (Saturasyon Limiti: +-%d N)', sat_limit), 'Color', 'w'); 
ylabel('Kuvvet (Newton)', 'Color', 'w'); xlabel('Zaman (s)', 'Color', 'w');
set(gca, 'Color', [0.15 0.15 0.15], 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2);

%% 5. Frekans ve RMS Performans Analizi
fig2 = figure('Name', 'Bode Diyagrami', 'Color', [0.12 0.12 0.12]);
opts = bodeoptions; opts.FreqUnits = 'Hz'; opts.XLim = [0.1, 100]; opts.Grid = 'on';

opts.Title.String = 'Sistem Gecirgenligi (Bode Plot)'; 
opts.Title.Color = 'w'; opts.Title.FontSize = 12;
opts.XLabel.String = 'Frekans (Hz)'; 
opts.XLabel.Color = 'w'; opts.XLabel.FontSize = 12;
opts.YLabel.String = {'Genlik (dB)'; 'Faz (Derece)'}; 
opts.YLabel.Color = 'w'; opts.YLabel.FontSize = 12;

sys_pasif_bode = ss(A, B_road, C(1,:), D(1));
sys_aktif_bode = ss(A - B_control*K_x, B_road, C(1,:), D(1));
bode(sys_pasif_bode, 'r--', sys_aktif_bode, 'g', opts);

bode_axes = findobj(fig2, 'Type', 'Axes');
set(bode_axes, 'XColor', 'w', 'YColor', 'w', 'Color', [0.15 0.15 0.15], 'FontSize', 12);

% Stokastik Yol Profili Üzerinde RMS İyileşme Hesabı
t_rand = 0:0.01:4; yol_rand = 0.005 * randn(size(t_rand));
[y_p_r] = lsim(ss(A, B_road, C, D), yol_rand, t_rand);
[y_i_r] = lsim(sys_aktif_i, yol_rand, t_rand); 

% Sadece ivme verisini (1. sütun) alarak 2 elemanlı matris hatasını önlüyoruz!
rms_p = rms(y_p_r(:,1)); 
rms_i_matris = (C * y_i_r(:,1:4)')';
rms_i = rms(rms_i_matris(:,1)); 

iyilesme = ((rms_p - rms_i) / rms_p) * 100;
fprintf('Analiz Tamamlandi. LQR-I ile Ortalama RMS Iyilesme Orani: %% %.2f\n', iyilesme);

%% 6. 3D STL Geometri ve Rotasyon İşlemleri
set(groot, 'DefaultFigureGraphicsSmoothing', 'off'); 
reduce_ratio = 0.2; stl_dosya_adi = 'f1_model.STL';

f1_model_data = stlread(stl_dosya_adi);
f1_struct.faces = f1_model_data.ConnectivityList; f1_struct.vertices = f1_model_data.Points;
reduced_f1 = reducepatch(f1_struct, reduce_ratio);

v_raw = reduced_f1.vertices * 0.001; 

% Z Ekseni Etrafında 90 Derece Rotasyon Kalibrasyonu
theta = pi/2; R_mat = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
v_centered = v_raw - [mean(v_raw(:,1)), mean(v_raw(:,2)), 0];
v_rotated = (R_mat * v_centered')';

% Taban Noktasını Asfalta Sıfırlama
v_scaled = v_rotated - [mean(v_rotated(:,1)), mean(v_rotated(:,2)), min(v_rotated(:,3))];

vurgu = 5; num_steps = length(t);
z_yol_all = (t >= 0) * yol_h; 
move_p_all = (x_pasif(:,1)' + z_yol_all) * vurgu; 
move_a_all = (x_aktif(:,1)' + z_yol_all) * vurgu; 

all_vertices_p = zeros(size(v_scaled,1), 3, num_steps);
all_vertices_a = zeros(size(v_scaled,1), 3, num_steps);
for k = 1:num_steps
    all_vertices_p(:,:,k) = v_scaled + [-1.6, 1.5, move_p_all(k)];
    all_vertices_a(:,:,k) = v_scaled + [1.6, 1.5, move_a_all(k)];
end

%% 7. 3D Nitro-Simülasyon Sahne Kurulumu
fig3 = figure('Name', 'F1 LQR-I Nitro Simulator', 'Color', 'k', 'NumberTitle', 'off');
set(fig3, 'Renderer', 'opengl'); hold on; view(40, 25); axis equal; grid off;
h_ax = gca; 
set(h_ax, 'Color', 'k', 'XColor', 'none', 'YColor', 'none', 'ZColor', 'none');
axis([-4.5 4.5 -2.5 6.5 -0.1 0.8]);

% Pist Kaplaması ve Bordürler
patch([-4.5 4.5 4.5 -4.5], [-2.5 -2.5 6.5 6.5], [0 0 0 0], [0.15 0.15 0.15], 'EdgeColor', 'none');
for i = -2.5:0.4:6.1
    renk = [0.8 0 0]; if mod(round(i*2.5),2) == 0; renk = [1 1 1]; end
    patch([-3.8 -3.3 -3.3 -3.8], [i i i+0.4 i+0.4], [yol_h yol_h yol_h yol_h], renk, 'EdgeColor', 'none');
    patch([3.3 3.8 3.8 3.3], [i i i+0.4 i+0.4], [yol_h yol_h yol_h yol_h], renk, 'EdgeColor', 'none');
end
plot3([0 0], [-2.5 6.5], [0.001 0.001], 'w--', 'LineWidth', 2); 
plot3([-3.1 -3.1], [-2.5 6.5], [0.001 0.001], 'w', 'LineWidth', 1.5); plot3([3.1 3.1], [-2.5 6.5], [0.001 0.001], 'w', 'LineWidth', 1.5); 

% Grafik Objelerinin Çizilmesi
h_p = patch('Faces', reduced_f1.faces, 'Vertices', v_scaled + [-1.6, 1.5, 0], 'FaceColor', [0.8 0.1 0.1], 'EdgeColor', 'none', 'AmbientStrength', 0.6);
h_a = patch('Faces', reduced_f1.faces, 'Vertices', v_scaled + [1.6, 1.5, 0], 'FaceColor', [0.1 0.7 0.1], 'EdgeColor', 'none', 'AmbientStrength', 0.6);
camlight('headlight'); lighting gouraud; material shiny;

% İlk açılıştaki ana başlık
title(h_ax, ['LQR-I Performans Analizi: %', num2str(iyilesme, '%.2f'), ' Iyilesme'], 'Color', 'w');

% Havada Asılı Telemetri Ekranları
t_pasif = text(-3.5, 1.5, 0.40, 'Pasif Taban: 50.0 mm', 'Color', [1 0.3 0.3], 'FontSize', 11, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'BackgroundColor', [0 0 0 0.7], 'Margin', 5);
t_aktif = text(3.5, 1.5, 0.40, 'LQR-I Taban: 50.0 mm', 'Color', [0.3 1 0.3], 'FontSize', 11, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'BackgroundColor', [0 0 0 0.7], 'Margin', 5);

oynat_fcn = @(~,~) nitro_oynat(h_p, h_a, t_pasif, t_aktif, all_vertices_p, all_vertices_a, num_steps, h_ax);
uicontrol('Style', 'pushbutton', 'String', 'Yeniden Oynat', 'Position', [20 20 120 40], 'BackgroundColor', [0.2 0.2 0.2], 'ForegroundColor', 'w', 'FontSize', 10, 'FontWeight', 'bold', 'Callback', oynat_fcn);

nitro_oynat(h_p, h_a, t_pasif, t_aktif, all_vertices_p, all_vertices_a, num_steps, h_ax);

% --- ENJEKTE EDILMIS AKICI TELEMETRI VE OYNATMA MOTORU ---
function nitro_oynat(h_p, h_a, t_pasif, t_aktif, all_vertices_p, all_vertices_a, num_steps, h_ax)
    x_p_base = evalin('base', 'x_pasif'); x_a_base = evalin('base', 'x_aktif'); 
    iyilesme_base = evalin('base', 'iyilesme'); nom_h = 0.050; 
    for k = 1:num_steps
        if ~ishandle(h_p), return; end 
        set(h_p, 'Vertices', all_vertices_p(:,:,k)); set(h_a, 'Vertices', all_vertices_a(:,:,k));
        
        curr_hp = (nom_h + x_p_base(k, 1)) * 1000; curr_ha = (nom_h + x_a_base(k, 1)) * 1000; 
        set(t_pasif, 'String', sprintf('Pasif Taban: %.1f mm', curr_hp)); set(t_aktif, 'String', sprintf('LQR-I Taban: %.1f mm', curr_ha));
        set(t_pasif, 'Position', [-3.5, 1.5, 0.40 + x_p_base(k, 1)*5]); set(t_aktif, 'Position', [3.5, 1.5, 0.40 + x_a_base(k, 1)*5]);
        
        
        title(h_ax, ['LQR-I Performans Analizi: %', num2str(iyilesme_base, '%.2f'), ' Iyilesme'], 'Color', 'w');
        drawnow; pause(0.02); 
    end
end
