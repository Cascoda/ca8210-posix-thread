import xmltodict
import itertools

XML = 'MAC.xml'
LOG = 'exchange.log'

#sorry
def printDissected( packetString ):
	global doc
	inData = bytearray.fromhex(packetString)
	iterCounter = 0
	for item in doc['List']['RadioPacket']:
			if int(item['byte']['#text'], 0) == inData[0]:
					print item['@Name']
					iterCounter = 2
					for param in item['List']['Parameter']:
							s = param['@Name']
							displayLengthCount = 0
							displayLengthMax = 255

							visible = True

							if 'DisplayLength' in param:
								displayLengthMax = int(param['DisplayLength'], 0)

							if 'Visibility' in param:
								if str(param['Visibility']) == 'false':
									visible = False

							for _ in itertools.repeat(None, int(param['@Length'], 0)):
								if displayLengthCount < displayLengthMax:
									s += ' ' + hex(inData[iterCounter]) + ' '
									displayLengthCount += 1

								if param['@Type'] == 'Discrete':
									for ValueDes in param['PotentialValues']['Value']:
										if int(ValueDes['@Value'], 0) == inData[iterCounter]:
											s += '(' + ValueDes['@Name'] + ')'

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
							if 'ParameterDependancy' in param:
								for parDep in param['ParameterDependancy']:
									for targetPar in item['List']['Parameter']:
										if targetPar['@Name'] == str(parDep['@Name']):
											if '#text' in parDep:
												targetPar[str(parDep['@Value'])] = parDep['#text']
											else:
												targetPar['@' + str(parDep['@Value'])] = hex(inData[iterCounter-1])

	if iterCounter < len(inData):
		s = ' '
		for _ in itertools.repeat(None, len(inData) - iterCounter):
			s += ' ' + hex(inData[iterCounter]) + ' '
			iterCounter += 1
		print s

	return;

def main():
	global doc
	with open(XML) as fd:
		doc = xmltodict.parse(fd.read(), force_list=('ParameterDependancy', 'Parameter', 'Value',))

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
