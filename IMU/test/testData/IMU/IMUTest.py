def getData(fileName, row = 100):
	f = open(fileName)
	datas = []
	i = 0
	for l in f.readlines():
		if(i == 0):
			i = i + 1
			continue

		line = l.split(',')
		lineData = []
		for data in line:
			lineData.append(float(data))
		i = i + 1
		datas.append(lineData)
		if(i == row):
			break

	return datas



if __name__ == '__main__':
	datas = getData('data.csv')
	print(datas)
