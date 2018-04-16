import subprocess
# MASTER CODE 
# Swith Mission type for Push/Pull Operations
def main():
	print('start')

	Push = 1
	Pull = 2

	MissionType = 1

	if MissionType == Push:
		subprocess.call(['python','PushOpen.py'])

	if MissionType == Pull:
		subprocess.call(['python','PullOpen.py'])

	print('end')

if __name__=="__main__":
	main()