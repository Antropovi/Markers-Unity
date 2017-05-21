import os
import shutil
import subprocess

#print os.getcwd()
cwd = os.getcwd()
files = [f for f in os.listdir(cwd) if os.path.isfile(os.path.join(cwd, f)) and f.endswith("mp4")]

for f in files:
	dirname = os.path.splitext(f)[0]
	shutil.rmtree(dirname, ignore_errors=True)
	os.mkdir(dirname)

	print 'Converting ' + dirname
	subprocess.call("ffmpeg -i \"" + f +"\" -f image2 \"" + dirname + "/%05d.png\"", shell=True)
	print 'done'