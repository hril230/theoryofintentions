
from datetime import datetime

import subprocess



if __name__ == "__main__":
		sparc_path = "$HOME/work/solverfiles/sparc.jar"

		answer_set = subprocess.check_output('java -jar '+sparc_path + ' simulation/ASP_files/Test.sp -A ', shell=True)
		print 'this is it: *' + answer_set + '*'
