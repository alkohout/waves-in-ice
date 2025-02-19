% load ../disp_full.out
% plot(disp_full)
% saveas(gcf,'displacement','jpg')    
% 
% % direction test
% 
% clf
% load ../heave.out
% load ../pitch.out
% load ../roll.out
% plot(heave,'k')
% hold on
% plot(pitch,'r')
% plot(roll,'b')
% axis([0 40 -1 1])
% legend('heave','pitch','roll')
% saveas(gcf,'heave_pitch_roll','jpg')    
% 
% for i=1:257
%     f(i) = (i-1)/(2.0*256*.5);
% end
% 
% clf
% load ../roll_spectra.out
% load ../pitch_spectra.out
% load ../heave_spectra.out
% plot(roll_spectra,'b')
% hold on
% plot(pitch_spectra,'r')
% plot(heave_spectra,'k')
% legend('roll','pitch','heave')
% saveas(gcf,'direction_spectra','jpg')
% 
% clf
% load ../direction.out
% plot(f,direction*180/pi)
% saveas(gcf,'direction','jpg')
% 
% clf
% load ../c11.out
% plot(f,c11)
% saveas(gcf,'c11','jpg')
% 
% clf
% load ../aa1.out
% plot(f,aa1)
% saveas(gcf,'aa1','jpg')
% 
% clf
% load ../bb1.out
% plot(f,bb1)
% saveas(gcf,'bb1','jpg')

% test_north_south
pitch = 5.0*pi/180.0;
roll =0.0*pi/180.0;
yaw = -45.0*pi/180.0;

% roll = (((sin(yaw)*sin(pitch))/cos(pitch)) - ((cos(yaw)*sin(roll))/(cos(pitch)*cos(roll))))*180.0/pi
% pitch = (((cos(yaw)*sin(pitch))/cos(pitch)) - ((sin(yaw)*sin(roll))/(cos(pitch)*cos(roll))))*180.0/pi

rolli = (tan(pitch)*sin(yaw)) %- (tan(roll)/cos(pitch))*cos(yaw))*180/pi  
pitchi = (tan(pitch)*cos(yaw) + (tan(roll)/cos(pitch))*sin(yaw))*180/pi 

