
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_code.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/gsn_csm.ncl"
load "$NCARG_ROOT/lib/ncarg/nclscripts/csm/contributed.ncl"

begin

pi = 3.1419

; processed
fname = "../processed.out"
processed = asciiread(fname,193,"double")
hz_mean = processed(100)
tp = processed(16)

; actual wave
fname = "actual.in"
actual = asciiread(fname,2,"float")
period = actual(0)
amp = actual(1)

; POS raw
fname = "../roll_raw.out"
roll_raw = asciiread(fname,5120,"double")
roll_raw = roll_raw*180/3.14156
fname = "../pitch_raw.out"
pitch_raw = asciiread(fname,5120,"double")
pitch_raw = pitch_raw*180/3.14156
fname = "../yaw_raw.out"
yaw_raw = asciiread(fname,5120,"double")
yaw_raw = yaw_raw*180/3.14156
x = ispan(1,5120,1)
res = True
wks = gsn_open_wks("png","roll_raw")
plot = gsn_csm_xy(wks,x,roll_raw,res)
wks = gsn_open_wks("png","pitch_raw")
plot = gsn_csm_xy(wks,x,pitch_raw,res)
wks = gsn_open_wks("png","yaw_raw")
plot = gsn_csm_xy(wks,x,yaw_raw,res)

; POS 2 Hz
fname = "../roll_2hz.out"
roll_2hz = asciiread(fname,1280,"double")
roll_2hz = roll_2hz*180/3.14156
fname = "../pitch_2hz.out"
pitch_2hz = asciiread(fname,1280,"double")
pitch_2hz = pitch_2hz*180/3.14156
delete(x)
x = ispan(1,1280,1)
res = True
wks = gsn_open_wks("png","roll_2hz")
plot = gsn_csm_xy(wks,x,roll_2hz,res)
wks = gsn_open_wks("png","pitch_2hz")
plot = gsn_csm_xy(wks,x,pitch_2hz,res)

; Acceleration 
fname = "../accel_2hz.out"
accel_2hz = new((/3,1280/),double)
accel_2hz(0,:) = asciiread(fname,1280,"double")
accel_2hz(1,:) = (2*pi/period)^2*amp;
accel_2hz(2,:) = -((2*pi/period)^2*amp);
delete(x)
x = ispan(1,1280,1)
delete(res)
res = True
res@tiMainString = "Amp actual = "+accel_2hz(1,0)+""
wks = gsn_open_wks("png","accel_2hz")
plot = gsn_csm_xy(wks,x,accel_2hz,res)

; Displacement
fname = "../disp_z.out"
disp = new((/3,1280/),double)
disp(0,:) = asciiread(fname,1280,"double")
disp(1,:) = amp;
disp(2,:) = -(amp);
delete(x)
x = ispan(1,1280,1)
delete(res)
res = True
res@tiMainString = "Amp actual = "+amp+", Amp rig ="+hz_mean/2+""
wks = gsn_open_wks("png","disp")
plot = gsn_csm_xy(wks,x,disp,res)

; PSD
fname = "../psd_z.out"
psd = asciiread(fname,55,"double")
fname = "../f.out"
delete(res)
res = True
res@trXMaxF = 25
res@tiMainString = "Tp actual = "+period+", Tp rig ="+tp+""
res@trYLog = True
res@trXLog = True
f = asciiread(fname,55,"double")
wks = gsn_open_wks("png","psd")
plot = gsn_csm_xy(wks,1/f(1:54),psd(1:54),res)

end

