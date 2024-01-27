clear
clc

sigma_a=0.05; % alignment
sigma_s = 0.0250/100; % scale
% cuboid
a=5; % width
b=10; % length
c=2; % hight
depth_snow = 0.66;
density_snow=440; % snow density
density_ice = 917;
sigma_snow_density = 50 % std of snow density
sigma_depth = 0.28;

sigma_l = sigma_a+sigma_s*l

% case 1: full snow
disp( "case 1: full snow");
v=a*b*c;
sigma_v = sqrt(a^2*b^2+a^2*c^2+b^2*c^2)*sigma_l;
m=density_snow*v;
sigma_m = sqrt(v^2*sigma_snow_density^2+density_snow^2*sigma_v^2)
mRatio = sigma_m/m
vRatio = sigma_v/v

% case 2: ice+snow
disp("case 2: ice+snow");
surface_area = a*b; 
V_snow = surface_area*depth_snow;
V_ice = v - V_snow;
assert(V_ice>0);

sigma_v_snow = surface_area*sigma_depth
sigma_v_ice = sqrt(sigma_v_snow^2+sigma_v^2)
sigma_m_snow = sqrt(V_snow^2*sigma_snow_density^2+density_snow^2*sigma_v_snow^2)
sigma_m_ice = density_ice*sigma_v_ice
snow_mass = V_snow*density_snow;
ice_mass = V_ice*density_ice;

mRatio_snow = sigma_m_snow/snow_mass
mRatio_ice = sigma_m_ice/ice_mass
vRatio_snow = sigma_v_snow/V_snow
vRatio_ice = sigma_v_ice/V_ice

vRatio2 = 3*sqrt(2*pi)*sigma_l / (pi*l^2);
mRatio2 = sqrt((sigma_snow_density^2/density_snow^2)+(sigma_v^2/v^2));