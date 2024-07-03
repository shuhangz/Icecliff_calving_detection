clear
clc
%% settings
sigma_alignment=0.05; % alignment
sigma_s = 0.0270/100; % scale
% cuboid
a=5; % width
b=10; % length
c=2; % hight
depth_snow = 0.66;
density_snow=440; % snow density
density_ice = 917;
sigma_snow_density = 50; % std of snow density
sigma_depth = 0.28; % std of snow depth


%% begin calculation
sigma_a = sigma_alignment+sigma_s*a;
sigma_b = sigma_alignment+sigma_s*b;
sigma_c = sigma_alignment+sigma_s*c;

% case 1: full snow
disp( "case 1: full snow");
v=a*b*c;
sigma_v = sqrt(sigma_a^2*b^2*c^2+a^2*sigma_b^2*c^2+a^2*b^2*sigma_c^2);
m=density_snow*v;
sigma_m = sqrt(v^2*sigma_snow_density^2+density_snow^2*sigma_v^2);
mRatio = sigma_m/m;
vRatio = sigma_v/v;
% display volume and snow mass
disp(['volume:', num2str(v), ' m^3']);
disp(['volume std:', num2str(sigma_v), ' m^3']);
disp(['mass:', num2str(m), ' kg']);
disp(['mass std:', num2str(sigma_m), ' kg']);
% display relative precision
disp(['mass relative precision:', num2str(mRatio)]);
disp(['volume relative precision:', num2str(vRatio)]);

% case 2: ice+snow
disp("case 2: ice+snow");
surface_area = a*b; 
V_snow = surface_area*depth_snow;
V_ice = v - V_snow;
assert(V_ice>0);

sigma_v_snow = surface_area*sigma_depth;
sigma_v_ice = sqrt(sigma_v_snow^2+sigma_v^2);
sigma_m_snow = sqrt(V_snow^2*sigma_snow_density^2+density_snow^2*sigma_v_snow^2);
sigma_m_ice = density_ice*sigma_v_ice;
snow_mass = V_snow*density_snow;
ice_mass = V_ice*density_ice;

mRatio_snow = sigma_m_snow/snow_mass;
mRatio_ice = sigma_m_ice/ice_mass;
vRatio_snow = sigma_v_snow/V_snow;
vRatio_ice = sigma_v_ice/V_ice;

disp(['ice volume:', num2str(V_ice), ' m^3']);
disp(['ice volume std:', num2str(sigma_v_ice), ' m^3']);
disp(['snow volume:', num2str(V_snow), ' m^3']);
disp(['snow volume std:', num2str(sigma_v_snow), ' m^3']);

disp(['ice mass:', num2str(ice_mass), ' kg']);
disp(['ice mass std:', num2str(sigma_m_ice), ' kg']);
disp(['snow mass:', num2str(snow_mass), ' kg']);
disp(['snow mass std:', num2str(sigma_m_snow), ' kg']);

disp(['ice mass relative precision:', num2str(mRatio_ice)]);
disp(['snow mass relative precision:', num2str(mRatio_snow)]);
disp(['ice volume relative precision:', num2str(vRatio_ice)]);
disp(['snow volume relative precision:', num2str(vRatio_snow)]);

% vRatio2 = 3*sqrt(2*pi)*sigma_l / (pi*l^2);
% mRatio2 = sqrt((sigma_snow_density^2/density_snow^2)+(sigma_v^2/v^2));