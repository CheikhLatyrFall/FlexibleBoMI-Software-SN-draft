
class bcolors :
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN= '\033[92m'
    WARNING= '\033[93m'
    FAIL = '\033[91m'
    ENDC ='\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    

def printProgressBar(iteration,total,prefix='',suffix='',decimals=1, length=100, fill='*'):#u"\x219"):
		percent=("{0:."+str(decimals)+"f}").format(100*(iteration/float(total)))
		filledLength=int(length*iteration//total)
		if iteration == total:
			suffix=bcolors.OKGREEN + suffix + bcolors.ENDC
		bar=fill*filledLength+'-'*(length-filledLength)
		sys.stdout.write('\r%s |%s| %s%% %s\r' %(prefix, bar, percent,suffix))
		sys.stdout.flush()
		if iteration==total:
				sys.stdout.write('\n')


