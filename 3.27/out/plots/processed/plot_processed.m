
clear

% main.h
m = 256;
dt = 0.5;
k = 2;

% load data
load ../../processed.out
load ../../psdf1f2.out
load ../../f.out
f_all = f;
load f.out
det(:) = processed(1:16);
tp = processed(17);
hs_det =processed(18);
psd = processed(19:73);
moments = processed(74:80);
hs_mom = 4*sqrt(processed(76));
mn_dir = processed(81);
pk_dir = processed(82);
sprd = processed(83);
ratio = processed(84);
hs_dir = processed(85);
a = processed(86);
b = processed(87);
std = log(processed(88));
f2 = processed(89);
q_mvar = processed(90);
q_kist = processed(91);
q_imu = processed(92);
q_pkist = processed(93);
q_paccel = processed(94);
q_pgyro = processed(95);
q_pmag = processed(96);
q_head = processed(97);
q_pd = processed(98);
q_stdyaw = processed(99);
q_h2o = processed(100);
if (size(processed,1)>100)
    dir = processed(101:154);
end 

% PSD slope
for j = 1:55;    
    psd_slope(j) = (exp(b)*f(j)^(a));
end

% PSD confidence intervals
alpha = .5;
dof = 2*k/(1 + (0.4*(k-1))/k);
for i = 1:55
    ci_u(i) = psd(i)*dof/(chi2inv((1-alpha)/2,dof));
    ci_l(i) = psd(i)*dof/(chi2inv((1+alpha)/2,dof));
end
hs_u = 4*sqrt(sum(ci_u)/(2.0*256*.5))
hs_l = 4*sqrt(sum(ci_l)/(2.0*256*.5))
hs_out = 4*sqrt(sum(psd)/(2.0*256*.5))

% plots
clf
subplot(2,1,1)
loglog(f,psd,'k')
hold on
% loglog(f_all,psdf1f2,'m')
loglog(f,psd_slope,'r')
loglog([f2 f2],[min(psd_slope) max(psd)],'b')
loglog(f,ci_l,'g--')
loglog(f,ci_u,'g--')
% legend({'PSD','PSD_all','Noise slope','f2','PSD CIs'})
legend({'PSD','Noise slope','f2','PSD CIs'})
axis([0.01 1 min(min(psd_slope),min(psd)) max(ci_u)])
ylabel('PSD');
xlabel('Frequency (Hz)');
fname = sprintf('Hs (det) = %.2f, Hs (mom) = %.2f, Hs (psd) = %.2f, Hs (dir) = %.2f, Tp = %.1f, f2 = %.1f',hs_det,hs_mom,hs_out,hs_dir,tp,1/f2);
title(fname);

subplot(2,1,2)
semilogx(f_all(11:64),dir/10)
fname = sprintf('Dir (mean) = %.2f, Dir (peak) = %.2f', mn_dir, pk_dir);
title(fname)
xlabel('Frequency (seconds)')
ylabel('Direction (degrees)')
axis([0.01 1 -180 180])
fname = sprintf('psd_processed');
saveas(gcf,fname,'jpg')

clf
bar(det(1:4))
title('Hz max')
ylabel('Wave height (m)')
saveas(gcf,'Hz_max','jpg')

clf
bar(det(5:8))
hold on
bar(det(9:12))
title('Hcm max')
ylabel('Wave amplitude (m)')
saveas(gcf,'Hcm_Htm','jpg')

clf
bar(det(13:16))
title('Tz mean')
ylabel('Mean wave period (s)')
saveas(gcf,'Tz_mean','jpg')

clf
plot(1./f_all(11:64),dir/10)
fname = sprintf('Dir (mean) = %.2f, Dir (peak) = %.2f', mn_dir, pk_dir);
title(fname)
xlabel('Frequency (seconds)')
ylabel('Direction (degrees)')
saveas(gcf,'Direction','jpg')

