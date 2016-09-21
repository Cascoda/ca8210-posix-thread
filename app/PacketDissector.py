import xmltodict
import itertools

XML = 'MAC.xml'
LOG = 'exchange.log'

#sorry
def printDissected( packetString ):
	global doc
	inData = bytearray.fromhex(packetString)
	iterCounter = 0
	#Look though every item to find the description for the correct command ID
	for item in doc['List']['RadioPacket']:
		if int(item['byte']['#text'], 0) == inData[0]:
			#Found command, print the name
			print item['@Name']
			#Skip command and length bytes
			iterCounter = 2
			#Go through all of the parameters and print them
			for param in item['List']['Parameter']:
				s = param['@Name']

				#Displaylength controls how many characters are printed, but many fields still need to be fully read
				displayLengthCount = 0
				displayLengthMax = 255

				#Visibility hides parameters which are included in the packet but not used
				visible = True

				#Apply displaylength if relevent
				if 'DisplayLength' in param:
					displayLengthMax = int(param['DisplayLength'], 0)

				#Apply Visibility if relevent
				if 'Visibility' in param:
					if str(param['Visibility']) == 'false':
						visible = False

				#This loop handles scrolling through the received bytes and printing them
				for _ in itertools.repeat(None, int(param['@Length'], 0)):
					if displayLengthCount < displayLengthMax:
						s += ' ' + hex(inData[iterCounter]) + ' '
						displayLengthCount += 1

					#Certain parameters are discrete, and come with accompanying descriptions
					#Discrete parameters are currently limited to length 1
					if param['@Type'] == 'Discrete':
						for ValueDes in param['PotentialValues']['Value']:
							if int(ValueDes['@Value'], 0) == inData[iterCounter]:
								s += '(' + ValueDes['@Name'] + ')'

								#Discrete parameters can have effects on the properties of upcoming parameters
								if 'ParameterDependancy' in ValueDes:
									for parDep in ValueDes['ParameterDependancy']:
										for targetPar in item['List']['Parameter']:
											if targetPar['@Name'] == str(parDep['@Name']):
												if '#text' in parDep:
													targetPar[str(parDep['@Value'])] = parDep['#text']
												else:
													targetPar['@' + str(parDep['@Value'])] = hex(inData[iterCounter])
								break;

					iterCounter += 1
					if iterCounter >= len(inData):
						if visible:
							print s
						return;
				if visible:
					print s

				#Some parameter dependancies can cause a future property to be affected by the value of itself (eg length)
				#These kinds of parameters are currently limited to length 1
				if 'ParameterDependancy' in param:
					for parDep in param['ParameterDependancy']:
						for targetPar in item['List']['Parameter']:
							if targetPar['@Name'] == str(parDep['@Name']):
								if '#text' in parDep:
									targetPar[str(parDep['@Value'])] = parDep['#text']
								else:
									targetPar['@' + str(parDep['@Value'])] = hex(inData[iterCounter-1])

	#Print leftover data (this should never happen)
	if iterCounter < len(inData):
		s = ' '
		for _ in itertools.repeat(None, len(inData) - iterCounter):
			s += ' ' + hex(inData[iterCounter]) + ' '
			iterCounter += 1
		print s

	return;

def main():
	global doc

	#Open and parse the XML file
	with open(XML) as fd:
		doc = xmltodict.parse(fd.read(), force_list=('ParameterDependancy', 'Parameter', 'Value',))

	#Open the logfile and parse it line by line
	with open(LOG) as fp:
		for line in fp:
			line = line.rstrip()
			if 'Received  Sync: ' in line:
				print 'Received  Sync: '
				printDissected(line[15:])
				print '---'
			elif 'Writing data:   ' in line:
				print 'Writing data:   '
				printDissected(line[15:])
				print '---'
			elif 'Received Async: ' in line:
				print 'Received Async: '
				printDissected(line[15:])
				print '---'
			else:
				print line


	return;

if __name__ == "__main__": main()
