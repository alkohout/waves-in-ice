import numpy as np
import Ngl

#-- Read data 
fname = "../psd_z.out"
nrows = 257 
ncols = 1
psd = Ngl.asciiread(fname,(nrows,1),"double")
psd = np.array(np.squeeze(psd))

fname = "../f.out"
nrows = 257 
ncols = 1
f = Ngl.asciiread(fname,(nrows,1),"double")
f = np.array(np.squeeze(f))

fname = "../accel_vert.out"
nrows = 5120 
ncols = 1
accel_vert = Ngl.asciiread(fname,(nrows,1),"double")
accel_vert = np.array(np.squeeze(accel_vert))

fname = "../disp_z.out"
nrows = 1280 
ncols = 1
disp = Ngl.asciiread(fname,(nrows,1),"double")
disp = np.array(np.squeeze(disp))

x = np.array(range(0,5119))
xdisp = np.array(range(0,1279))

res = Ngl.Resources()
res.nglMaximize             = True 
res.nglDraw                 = False 
res.nglFrame                = False 
res.trYMinF = np.amin(psd)
res.trYMaxF = np.amax(psd) 
res.trXMaxF = 0.25
res.trXLog = True
res.trYLog = True

wks_type = "png"
wres = Ngl.Resources()
wks = Ngl.open_wks(wks_type,"psd")
plot = Ngl.xy(wks,f,psd,res)
Ngl.draw(plot)
Ngl.frame(wks)

res.trYMinF = np.amin(accel_vert)
res.trYMaxF = np.amax(accel_vert) 
res.trXMaxF = 5120
res.trXLog = False 
res.trYLog = False 
wks_type = "png"
wres = Ngl.Resources()
wks = Ngl.open_wks(wks_type,"accel")
plot = Ngl.xy(wks,x,accel_vert,res)
Ngl.draw(plot)
Ngl.frame(wks)

res.trYMinF = np.amin(disp)
res.trYMaxF = np.amax(disp) 
res.trXMaxF = 1280 
res.trXLog = False 
res.trYLog = False 
wks_type = "png"
wres = Ngl.Resources()
wks = Ngl.open_wks(wks_type,"disp")
plot = Ngl.xy(wks,xdisp,disp,res)
Ngl.draw(plot)
Ngl.frame(wks)
