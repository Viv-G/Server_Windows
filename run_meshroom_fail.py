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

	cmdLine = "C:\Server\Meshroom\meshroom_photogrammetry.exe --input " + srcImageDir + " --output " + output + " --cache " + work + "  --toNode " + runStep + " --overrides " + overrides + " --pipeline " + pipeline + " --scale " + scale
	
	print(cmdLine)
	os.system(cmdLine)

	return 0

def RecServer(srcImageDir):
	imgcounter = 1
	fileName = srcImageDir + "\IMGw"+str(imgcounter)+".jpg"

	HOST = '192.168.8.118'
	PORT = 6666

	connected_clients_sockets = []

	server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

	server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	server_socket.bind((HOST, PORT))
	server_socket.listen(10)
	print('Waiting For Connection....')
	
	connected_clients_sockets.append(server_socket)
	
	read_sockets, write_sockets, error_sockets = select.select(connected_clients_sockets, [], [])

	for sock in read_sockets:

		if sock == server_socket:

			sockfd, client_address = server_socket.accept()
			connected_clients_sockets.append(sockfd)
			print('Connected...')
	while True:
		try:
			data = sock.recv(4096)
			txt = str(data.decode('ascii'))
			print (txt)
			if data:

				if data.decode('ascii').startswith('SIZE'):
					tmp = txt.split()
					size = int(tmp[1])

					print ('got size %d' %size)
					confString = 'RSIZE'
					sock.sendall(confString.encode('ascii'))

				elif data.decode('ascii').startswith('BYE'):
					print('Shutting Socket')
					print (data.decode('ascii'))
					sock.shutdown()

				else :
					print ('RECIEVING IMG')
					myfile = open(fileName, 'wb')
					myfile.write(data)

					data = sock.recv(40960000)
					if not data:
						print('Not DATA')
						myfile.close()
						break
					myfile.write(data)
					myfile.close()
					imConf = 'GOT IMAGE'
					sock.sendall(imConf.encode('ascii'))
					sock.shutdown()
		except:
			print('Closing Socket....')
			sock.close()
			connected_clients_sockets.remove(sock)
			continue
			imgcounter += 1
	server_socket.close()
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
	overrides = "C:\\Server\\config\\settings"+version+".json"
	srcImageDir = "F:\\Viv\Documents\\Uni\\sem1_2019\\FYP\MESHROOM_LIVESCAN\\box\\"+version+"\\img"
	output = "F:\\Viv\\Documents\\Uni\\sem1_2019\\FYP\MESHROOM_LIVESCAN\\Auto\\box1\\" + version + "\\s" + scale
	work = "F:\\Viv\Documents\\Uni\\sem1_2019\\FYP\\MESHROOM_LIVESCAN\\Auto\\Work\\" + version + "\\s" + scale
	pipeline = "C:\\Server\\config\\setup.mg"
	runStep = "Publish"
	SilentMkdir(output)
	SilentMkdir(work)
	SilentMkdir(srcImageDir)

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
	
	writeSettings(Fov, f, ppy, ppx, overrides)
	#Run_Meshroom(overrides, srcImageDir, output, work, pipeline, runStep, version, scale)
	

	return 0

def writeSettings(Fov, f, ppy, ppx,overrides):
	settings = {'CameraInit_1': 
			{'intrinsics':[
				{'intrinsicId': 2561805385, 
				'pxInitialFocalLength': -1.0,
				'pxFocalLength': float(f),
				'type': 'radial3',
				'width': 1080,
				'height': 1920,
				'serialNumber': 'C:/Server/img',
				'principalPoint': {
					'x': float(ppx),
					'y': float(ppy)
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
			'defaultFieldOfView': float(Fov)
			}
		}
	jstr = json.dumps(settings, indent=4)
	print(jstr)
	with open(overrides, "w") as fileOut:
		json.dump(settings,fileOut,indent=4,sort_keys=False)
	
	return 0
	
	
main()



