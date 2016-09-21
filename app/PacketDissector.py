import xmltodict
import itertools

XML = 'MAC.xml'
inputString = "6e 03 00 50 00"

inData = bytearray.fromhex(inputString)
iterCounter = 0

with open(XML) as fd:
    doc = xmltodict.parse(fd.read())

for item in doc['List']['RadioPacket']:
	if int(item['byte'], 0) == inData[0]:
		print item['@Name']
		iterCounter = 2
		for param in item['List']['Parameter']:
			s = param['@Name']
			for _ in itertools.repeat(None, param['Length']):
				s += ' ' + inData[iterCounter] + ' '
				iterCounter++
			print s

