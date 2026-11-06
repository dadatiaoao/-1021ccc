import os,uhashlib,ubinascii
for p in ('/sdcard/yolo11n_det_320.kmodel','/sdcard/codex_yolo/colors.kmodel'):
 h=uhashlib.sha256()
 try:
  with open(p,'rb') as f:
   while True:
    b=f.read(32768)
    if not b:break
    h.update(b)
  print('MODEL_CHECK',p,os.stat(p)[6],ubinascii.hexlify(h.digest()).decode())
 except OSError as e:print('MODEL_MISSING',p,str(e))
