import sys, os
import shutil
import json


def SilentMkdir(theDir):
	try:
		os.mkdir(theDir)
	except:
		pass
	return 0

def Run_Meshroom(overrides, srcImageDir, output, work, pipeline, runStep, version, scale):

	cmdLine = "C:\Server\Meshroom\meshroom_photogrammetry.exe --input " + srcImageDir + " --output " + output + " --cache " + work + "  --toNode " + runStep + " --overrides " + overrides + " --pipeline " + pipeline + " --scale " + scale
	
	print(cmdLine)
	os.system(cmdLine)

	return 0


def main():
	print("Prepping Scan, v2.")

	print(sys.argv)

	print (len(sys.argv))
	if (len(sys.argv) != 2):
		print("usage: python run_alicevision.py <version>")
		print("Must pass 1 arguments.")
		sys.exit(0)
		
	version = sys.argv[1]
	scale = "8"
	overrides = "C:\\Server\\config\\settingsv8.json"
	srcImageDir = "F:\\Viv\Documents\\Uni\\sem1_2019\\FYP\MESHROOM_LIVESCAN\\box\\v8\\img"
	output = "F:\\Viv\\Documents\\Uni\\sem1_2019\\FYP\MESHROOM_LIVESCAN\\Auto\\box1\\" + version + "\\s" + scale
	work = "F:\\Viv\Documents\\Uni\\sem1_2019\\FYP\\MESHROOM_LIVESCAN\\Auto\\Work\\" + version + "\\s" + scale
	pipeline = "C:\\Server\\config\\setup.mg"
	runStep = "Publish"
	
	Fov = "40.64392"
	f = "1458.1"
	ppy = "908.7"
	ppx = "554"
	intMat = f + ";0;" + ppx + ";0;" + f + ";" + ppy + ";0;0;1"
	
	SilentMkdir(output)
	SilentMkdir(work)
	print("Image dir   : %s" % srcImageDir)
	print("Work dir    : %s" % work)
	print("Output dir  : %s" % output)
	print("Version     : %s" % version)
	print("Step        : %s" % runStep)
	print("Scale        : %s" % scale)
	

	#Run_Meshroom(overrides, srcImageDir, output, work, pipeline, runStep, version, scale)
	writeSettings()
	print("Scale        : %s" % scale)

	return 0

def writeSettings(Fov, f, ppy, ppx):
	settings = {'CameraInit_1': 
			{'intrinsics':[
				{'intrinsicId': 2561805385, 
				'pxInitialFocalLength': -1.0,
				'pxFocalLength': 1458.1,
				'type': 'radial3',
				'width': 1080,
				'height': 1920,
				'serialNumber': 'C:/Server/img',
				'principalPoint': {
					'x': 554.0,
					'y': 908.7
				},
				'initializationMode': 'unknown',
				'distortionParams': [
					0.0, 
					0.0, 
					0.0
				],
				"locked": False
				}
			],
			'defaultFieldOfView': 40.64392
			}
		}
	jstr = json.dumps(settings, indent=4)
	print(jstr)
	
	return 0
main()



