import uhashlib,ubinascii,os
h=uhashlib.sha256()
with open('/sdcard/codex_yolo/colors.kmodel','rb') as f:
 while True:
  b=f.read(32768)
  if not b:break
  h.update(b)
s=ubinascii.hexlify(h.digest()).decode()
assert s=='24f39546f886b179c98f887d4cd95dfb295d2c90d0650d68c28338b9f7a5dfcb'
print('SIX_COLOR_MODEL_HASH_VERIFIED',s,os.stat('/sdcard/codex_yolo/colors.kmodel')[6])