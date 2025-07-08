"""Run the raytracer with different parameters and save the output."""

import subprocess

base_cmdline_args = " -f fouranimals.obj -w 1000 "

variable_args = [" --method", " --max-depth", " --max-trig", " --num-buckets"]

method = [0, 1, 2, 3]
method = [3]
maximum_depth = 10
maximum_trig = [1024, 512, 256, 128, 64, 32, 16, 8, 4]
max_buckets = [1, 2, 4, 8, 10, 12, 14]

program = ".\\raytracer.exe"


for i in method:
    for j in range(maximum_depth + 1):
        for k in maximum_trig:
            if i == 3:
                for l in max_buckets:
                    bucketstring = variable_args[3] + " " + str(l) + " "
            else:
                bucketstring = ""
            cmdline_args = base_cmdline_args + variable_args[0] + " " + str(i) + " " + \
                                               variable_args[1] + " " + str(j) + " " + \
                                               variable_args[2] + " " + str(k) + " " + \
                                               bucketstring

            cmd = program + "".join(cmdline_args)
            print("Running command:", cmd)
            subprocess.run(cmd, shell=True)
            print("Command finished.")
            print("--------------------------------------------------")
