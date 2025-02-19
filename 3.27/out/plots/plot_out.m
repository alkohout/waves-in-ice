
clear
format compact

% main.h
m = 256;
dt = 0.5;
period = 10.6;
amp = 0.405/2;

% response
load ../response.out
for i = 1:2048
    fa(i) = (i-1)/(2048*.5);
end
clf
plot(fa,response)
ylabel('response function')
saveas(gcf,'response','jpg')

%integration
clf
t1 = 1:2048;
t2 = (2048-1280)/2+1:(2048-1280)/2 + 1280;
load ../disp_full.out
load ../disp_z.out
plot(t1,disp_full)
hold on
plot(t2,disp_z)
legend({'disp full','disp section'},'Location','best')
saveas(gcf,'displacement','jpg')

%accel v disp
clf
load ../accel_2hz.out
t = 0;
accel_true = (2*pi/period)^2*amp;
plot(accel_2hz)
hold on
plot(disp_z)
plot([1,1280],[accel_true,accel_true],'k');
plot([1,1280],[-accel_true,-accel_true],'k');
legend({'acceleration','displacement'})
saveas(gcf,'accel_v_disp','jpg')

%favg
clf
t1 = zeros*(1:257)+1;
t2 = zeros*(1:55)+1;
load ../f.out
load ../f_avg.out
% plot(1./f,t1,'b+')
hold on
plot(1./f_avg,t2,'r+')
axis([0 40 0 2]);
saveas(gcf,'f_avg','jpg')

% direction test
clf
load ../roll_spectra.out
load ../pitch_spectra.out
load ../accel_spectra.out
plot(roll_spectra,'b')
hold on
plot(pitch_spectra,'r')
plot(accel_spectra,'k')
legend({'roll spectra','pitch spectra','heave spectra'},'Location','best')
saveas(gcf,'direction_spectra','jpg')

clf
load ../direction_full.out
load ../c11.out
load ../processed.out
rspns_f7 = 0.04;
rspns_f8 = 0.25;
mn_dir = processed(81);
pk_dir = processed(82);
sprd = processed(83);
ratio = processed(84);
subplot(2,1,1)
plot(1./f,direction_full*180/pi)
hold on
plot([1/rspns_f7 1/rspns_f7],[0 360],'r');
plot([1/rspns_f8 1/rspns_f8],[0 360],'r');
xlabel('Period (s)')
ylabel('Direction (degrees)')
fname = sprintf('Mean = %f, Peak = %f, Spread = %f, ratio = %f',mn_dir, pk_dir, sprd, ratio);
title(fname);
legend('Direction', 'rspns f7','rspns f8')
axis([0 30 min(direction_full*180/pi) max(direction_full*180/pi)])
subplot(2,1,2)
plot(1./f,c11*2*m*dt)
hold on
plot(1./f_avg,processed(19:73),'r')
legend('C11','out')
xlabel('Period (s)')
ylabel('c11')
axis([0 30 min(c11*2*m*dt) max(c11(5:end)*2*m*dt)])
saveas(gcf,'direction_s','jpg')

clf
subplot(2,1,1)
semilogx(f,direction_full*180/pi)
hold on
semilogx([rspns_f7 rspns_f7],[0 360],'r');
semilogx([rspns_f8 rspns_f8],[0 360],'r');
xlabel('Frequency (Hz)')
ylabel('Direction (degrees)')
fname = sprintf('Mean = %f, Peak = %f, Spread = %f, ratio = %f',mn_dir, pk_dir, sprd, ratio);
title(fname);
legend('Direction', 'rspns f7','rspns f8')
axis([0 1 min(direction_full*180/pi) max(direction_full*180/pi)])
subplot(2,1,2)
loglog(f,c11*2*m*dt)
hold on
loglog(f_avg,processed(19:73),'r')
legend('C11','out')
xlabel('Frequency (Hz)')
ylabel('c11')
axis([0 1 min((c11*2*m*dt)) max([max((c11(5:end)*2*m*dt)),max((processed(19:73)))])])
saveas(gcf,'direction_hz','jpg')

clf
plot(1./f,c11*2*m*dt)
hold on
plot(1./f_avg,processed(19:73),'r')
legend('C11','out')
xlabel('period (s)')
ylabel('c11')
axis([0 30 min(c11*2*m*dt) max([max(c11(5:end)*2*m*dt),max(processed(19:73))])])
saveas(gcf,'c11','jpg')

% plot raw
clf
load ../roll_raw.out
load ../pitch_raw.out
load ../yaw_raw.out
subplot(3,1,1)
plot(roll_raw*180/pi)
ylabel('roll')
subplot(3,1,2)
plot(pitch_raw*180/pi)
ylabel('pitch')
subplot(3,1,3)
plot(yaw_raw*180/pi)
ylabel('yaw')
saveas(gcf,'rpy_raw','jpg')

clf
load ../accel_clean_k.out
load ../accel_clean_x.out
load ../accel_clean_y.out
load ../accel_clean_z.out
subplot(4,1,1)
plot(accel_clean_k)
ylabel('Kistler accel')
subplot(4,1,2)
plot(accel_clean_x)
ylabel('X accel')
subplot(4,1,3)
plot(accel_clean_y)
ylabel('Y accel')
subplot(4,1,4)
plot(accel_clean_z)
ylabel('Z accel')
saveas(gcf,'accel_clean','jpg')

clf
load ../gyro_clean_x.out
load ../gyro_clean_y.out
load ../gyro_clean_z.out
subplot(3,1,1)
plot(gyro_clean_x)
ylabel('X gyro')
subplot(3,1,2)
plot(gyro_clean_y)
ylabel('Y gyro')
subplot(3,1,3)
plot(gyro_clean_z)
ylabel('Z gyro')
saveas(gcf,'gyro_clean','jpg')

clf
load ../magno_clean_x.out
load ../magno_clean_y.out
load ../magno_clean_z.out
subplot(3,1,1)
plot(magno_clean_x)
ylabel('X magno')
subplot(3,1,2)
plot(magno_clean_y)
ylabel('Y magno')
subplot(3,1,3)
plot(magno_clean_z)
ylabel('Z magno')
saveas(gcf,'magno_clean','jpg')

clf
load ../roll_2hz.out
load ../pitch_2hz.out
subplot(2,1,1)
plot(roll_2hz)
ylabel('roll_2hz')
subplot(2,1,2)
plot(pitch_2hz)
ylabel('pitch_2hz')
saveas(gcf,'roll_pitch_2hz','jpg')

% accel_vert
clf
load ../accel_vert.out
plot(accel_clean_k-mean(accel_clean_k),'k')
hold on
plot(accel_vert,'r')
legend({'Kistler','Vertical'})
saveas(gcf,'vertical_accel','jpg')

% processed.out
fname = sprintf('../f_avg.out');
f = dlmread(fname);
fname = sprintf('../f.out');
f_all = dlmread(fname);
fname = sprintf('../psdf1f2.out');
psd_all = dlmread(fname);
fname = sprintf('../psdf3f4.out');
psd_f3f4 = dlmread(fname);
fname = sprintf('../processed.out');
processed = dlmread(fname);
fname = sprintf('../psd_z.out');
psd_z = dlmread(fname);
std = log(processed(88));
f2 = processed(89);    
a = processed(86);
b = processed(87);
for j = 1:257;    
    psd_slope(j) = (exp(b)*f_all(j)^(a));
end
Hs = 4*sqrt(processed(76));
Tp = processed(17);
det(:) = processed(1:16);
k = 2;
cnt = 0;
alpha = .5;
dof = 2*k/(1 + (0.4*(k-1))/k);
for i = 1:257
    ci_u(i) = psd_z(i)*dof/(chi2inv((1-alpha)/2,dof));
    ci_l(i) = psd_z(i)*dof/(chi2inv((1+alpha)/2,dof));
end
hs_u = 4*sqrt(sum(ci_u)/(2.0*256*.5));
hs_l = 4*sqrt(sum(ci_l)/(2.0*256*.5));
hs_out = 4*sqrt(sum(psd_f3f4)/(2.0*256*.5));

clf
loglog(f_all,psd_all,'k')
hold on
loglog(f_all,psd_z,'g')
loglog(f,processed(19:73),'b')
loglog(f_all,psd_slope,'r')
loglog(f_all,ci_l,'g--')
loglog(f_all,ci_u,'g--')
legend({'PSD All','PSD z','PSD out','Noise slope','PSD CIs'},'Location','NorthWest')
ylabel('PSD');
xlabel('Frequency (Hz)');
fname = sprintf('Hs z = %f, Hs out = %f, Tp = %f, f2 = %f',Hs,hs_out,Tp,1/f2);
title(fname);
fname = sprintf('psd_slope');
saveas(gcf,fname,'jpg')

clf
plot(1./f_all,psd_z,'g')
hold on
plot(1./f,processed(19:73),'b')
plot(1./f_all,ci_l,'g--')
plot(1./f_all,ci_u,'g--')
ylabel('PSD out');
xlabel('Period (s)');
% axis([0 35 0 max(ci_u)])
axis([0 35 0 max(psd_z)])
legend({'PSD out','PSD z','PSD CIs'});
fname = sprintf('psd_out');
saveas(gcf,fname,'jpg')

clf
ht_mn = processed(101)
tp_mn = mean(det(13:16))
Tp
Hs
bar(det(1:4))
fname = sprintf('Hz max, Mean = %f',ht_mn);
title(fname)
ylabel('Wave height (m)')
saveas(gcf,'Hz_max','jpg')

clf
bar(det(5:8))
title('Hcm max')
ylabel('Wave amplitude (m)')
saveas(gcf,'Hcm_max','jpg')

clf
bar(det(9:12))
title('Htm max')
ylabel('Wave amplitude (m)')
saveas(gcf,'Htm_max','jpg')

clf
bar(det(13:16))
title('Tz mean')
ylabel('Mean wave period (s)')
saveas(gcf,'Tz_mean','jpg')

mn_dir
pk_dir
