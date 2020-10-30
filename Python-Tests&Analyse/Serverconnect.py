'''
Created on 01.01.2017

@author: Max-Ole/Lennart
'''
import socket, time
import sys, traceback  # debugging

def createsocket(host, port=12345, UDP=False): # aufgerufen in ServerSetup
	if UDP:
		s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
		s.bind(('', port))
	else:
		s = socket.socket()
		s.bind(('', port))
		s.listen(5)
	
	print("socket erstellt")
	return s

def gettingtext(client):
	data = client.recv(128).decode()
	return data
	
def gettingUDPtext(s):
	data, address = s.recvfrom(8192)
	#print("UDP:", address, data)
	return data.decode()


def ServerSetup(port=12345, UDP=False):
#	try:
#		host = '192.168.137.1'
#		s = createsocket(host)
#		print("host1: %s" %(host))
#	except:
#		try:
#			host = '192.168.2.119'
#			s = createsocket(host)
#			print("host2: %s" %(host))
#		except:
#			host = '192.168.1.234'
#			s = createsocket(host)
#			print("host3: %s" %(host))
	
	host = socket.gethostname()
	print("aktual host: %s" %(host))
	s = createsocket(host, port=port, UDP=UDP)
	
	if not UDP:
		client, addr = s.accept()
		print("Adresse", addr)
		print("Client", client)
		return client, s
	else:
		return None, s
		
		
def chooseType():
	# choose connection type (UDP?)
	hostName = socket.gethostname()
	hostIP = socket.gethostbyname(hostName)
	print("The host is:\tname: {} \tIP: {}".format(hostName, hostIP))
	UDP_enable = input("enable connection via UDP?: (y/n) (will be TCP else)") in ['y', 'j', 'yes', 'ja', 'Y', 'J', 'Yes', 'Ja']
	return UDP_enable

		

if __name__ == "__main__":
	try:
		print("\n########\nIN MAIN\n#######")
		# Inputs
		if input("UDP?: (y/n)") in ['y', 'j', 'yes', 'ja', 'Y', 'J', 'Yes', 'Ja']:
			UDP = True
		else:
			UDP=False
		port = int(input("port?"))
		
		client, s = ServerSetup(port=port, UDP=UDP)
		running = True
		while running:
			if UDP:
				data = gettingUDPtext(s)
			else:
				data = gettingtext(client)
			print("data:", data)
			if data == "" or data == "done"or data == "quit":
				print("quit")
				running = False #beenden
			#time.sleep(1)
	
	except:
		traceback.print_exc(file=sys.stdout)
		input("beenden...")
	finally:
		s.close()