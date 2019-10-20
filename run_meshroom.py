import sys, os
import shutil
import json
import random
import socket, select
from time import gmtime, strftime
from random import randint


def SilentMkdir(theDir):
	if not os.path.isdir(theDir):
		os.makedirs(theDir)
		print("Made Directory %s" %theDir)
	return 0

def Run_Meshroom(overrides, srcImageDir, output, work, pipeline, runStep, version, scale):
	#" --overrides " + overrides +
	cmdLine = "C:\\Server\Meshroom\\aliceVision\\bin\\aliceVision_utils_keyframeSelection.exe --mediaPaths " + srcImageDir + " --sensorDbPath C:\\Server\\Meshroom\\aliceVision\\share\\aliceVision\\cameraSensors.db --outputFolder " + srcImageDir + "\key  --voctreePath C:\\Server\\Meshroom\\aliceVision\\share\\aliceVision\\vlfeat_K80L3.SIFT.tree --useSparseDistanceSelection 1 --minFrameStep 1 --maxFrameStep 3  --sharpnessPreset very_low --pxFocals 1458.1 --sharpSubset 1"
	print(cmdLine)
	os.system(cmdLine)
	cmdLine = "C:\Server\Meshroom\meshroom_photogrammetry.exe --input " + srcImageDir + "\key --output " + output + " --cache " + work + "  --toNode " + runStep +  " --pipeline " + pipeline + " --scale " + scale #+ " --overrides " + overrides
	
	print(cmdLine)
	os.system(cmdLine)

	return 0

def RecServer(srcImageDir):
	imgcounter = 0
	fileName = srcImageDir + "\IMGw"+str(imgcounter)+".jpg"

	HOST = '192.168.8.118'
	PORT = 6666

	sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)

	sock.bind((HOST, PORT))
	sock.listen(10)
	print('Waiting For Connection....')
	
	conn,addr=sock.accept()
	print('Connected...')	
	try:
		dataTot = conn.recv(4096)
		totalImages = int(dataTot.decode('ascii'))
		#print(f"Recieving {totalImages} Images..")
		print(f'Recieving {totalImages}.')
		#print('Recieving %d Images' %totalImages)	
	except:
		print("Error:", sys.exc_info()[0])
		
	while imgcounter < totalImages:
		try:
			data = conn.recv(4096)
			txt = str(data.decode('ascii'))
			#print(txt)			
			tmp = txt.split()
			size = int(tmp[1])
			print(F'Image {imgcounter} Size = {size}...')					
		except:
			print('Error Recieving Size',  sys.exc_info()[0])
			break
		
		try:
			print ('RECIEVING IMG')
			#dataIM = b''
			fileName = srcImageDir + "\\IMGw"+str(imgcounter)+".jpg"
			#print ('Opened File')
			dataIM = conn.recv(4096)
			while len(dataIM) < size:
				dataIM += conn.recv(4096)
			
			print(f'Recieved Image {imgcounter}... Writing to file')
			with open(fileName, 'wb') as myfile: myfile.write(dataIM)
			#myfile.close()
			imConf = ('ENDR')
			imgcounter += 1
			conn.sendall(imConf.encode('ascii'))
			print('Sent GOT IMAGE Succesfully....')
			
		except UnicodeError as err:
			print("Unicode Error:", sys.exc_info()[0])
			print(err)
		#	print(err.reason)
		#	print('Object {0}', err.object[err.start:err.end]
			failString = ('Read Failed')
			conn.sendall(failString.encode('ascii'))
			print('Closing Socket....')
			conn.close()
			break
		except IndexError as err:
			print(f'Recieved {imgcounter} Images')
			conn.close()
			break
	sock.close()
	return 0


def main():
	print("Prepping Scan, v2.")
	print(sys.version_info)

	print(sys.argv)

	print (len(sys.argv))
	if (len(sys.argv) != 2):
		print("usage: python run_alicevision.py <version>")
		print("Must pass 1 arguments.")
		sys.exit(0)
		
	version = sys.argv[1]
	scale = "4"
	overrides = "C:\\Server\\config\\settings"+version+".json"
	srcImageDir = "F:\\Viv\Documents\\Uni\\sem1_2019\\FYP\MESHROOM_LIVESCAN\\Images\\"+version+"\\img"
	output = "F:\\Viv\\Documents\\Uni\\sem1_2019\\FYP\MESHROOM_LIVESCAN\\Output\\" + version + "\\s" + scale
	work = "F:\\Viv\Documents\\Uni\\sem1_2019\\FYP\\MESHROOM_LIVESCAN\\Work\\" + version + "\\s" + scale
	pipeline = "C:\\Server\\config\\setup.mg"
	runStep = "Publish"
	SilentMkdir(output)
	SilentMkdir(work)
	SilentMkdir(srcImageDir)
	SilentMkdir(srcImageDir + "\\key")

	RecServer(srcImageDir)
	
	Fov = "40.64392"
	f = "1458.1"
	ppy = "908.7"
	ppx = "554"
	intMat = f + ";0;" + ppx + ";0;" + f + ";" + ppy + ";0;0;1"
	

	print("Image dir   : %s" % srcImageDir)
	print("Work dir    : %s" % work)
	print("Output dir  : %s" % output)
	print("Version     : %s" % version)
	print("Step        : %s" % runStep)
	print("Scale        : %s" % scale)
	
	writeSettings(Fov, f, ppy, ppx, overrides, srcImageDir)
	Run_Meshroom(overrides, srcImageDir, output, work, pipeline, runStep, version, scale)
	print(f'SUCCESS!! Output Folder.... {output}')

	

	return 0

def writeSettings(Fov, f, ppy, ppx,overrides, srcImageDir):
	settings = {'CameraInit_1': 
			{
			'intrinsics':[
				{'intrinsicId': 3769820399, 
				'pxInitialFocalLength': float(f),
				'pxFocalLength': float(f),
				'type': 'radial3',
				'width': 1080,
				'height': 1920,
				'serialNumber': srcImageDir+"\\key",
				'principalPoint': {
					'x': float(ppx),
					'y': float(ppy)
				},
				'initializationMode': 'estimated',
				'distortionParams': [
					0.0, 
					0.0, 
					0.0
				],
				"locked": False
				}
			],
			'defaultFieldOfView': float(Fov),
			'groupCameraFallback': 'folder',
			}
		}
	jstr = json.dumps(settings, indent=4)
	#print(jstr)
	with open(overrides, "w") as fileOut:
		json.dump(settings,fileOut,indent=4,sort_keys=False)
	
	return 0
	
	
main()



