"""Run the raytracer with different parameters and save the output."""

import subprocess

base_cmdline_args = " -f .\\models\\fouranimals.obj -w 1000 "

variable_args = [" --method", " --max-depth", " --max-trig", " --num-buckets"]

method = [0, 1, 2, 3]
method = [3]
maximum_depth = 10
maximum_trig = [1024, 512, 256, 128, 64, 32, 16, 8, 4]
max_buckets = [1, 2, 4, 8, 10, 12, 14]

program = ".\\raytracer.exe"


skipped_to_last = False

for i in method:
    for j in range(maximum_depth + 1):
        for k in maximum_trig:
            for m in max_buckets:
                if not skipped_to_last:
                    if i == 3 and j == 8 and k == 64 and m == 1:
                        skipped_to_last = True
                        print("Skipped to command for method", i, "depth", j, "trig", k, "buckets", m)
                    else:
                        print("Skipping command for method", i, "depth", j, "trig", k, "buckets", m)
                        continue
                bucketstring = ""
                if i == 3:
                    bucketstring = variable_args[3] + " " + str(m) + " "
                cmdline_args = (base_cmdline_args +
                                variable_args[0] + " " + str(i) + " " +
                                variable_args[1] + " " + str(j) + " " +
                                variable_args[2] + " " + str(k) + " " +
                                bucketstring)

                cmd = program + "".join(cmdline_args)
                print("Running command:", cmd)
                subprocess.run(cmd, shell=True)
                print("Command finished.")
                print("--------------------------------------------------")
