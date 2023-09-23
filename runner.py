import subprocess
import threading
import time


def run_subprocess():
    try:
        process = subprocess.Popen(["python", "UAV\\final.py"],
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE)
        process.wait(timeout=2)  # Đợi tối đa 2 giây
        stdout, stderr = process.communicate()
        if process.returncode == 0:
            print("Subprocess completed successfully.")
            print("stdout:", stdout.decode())
        else:
            print("Subprocess failed.")
            print("stderr:", stderr.decode())
    except subprocess.TimeoutExpired:
        process.kill()
        print("Subprocess timed out and was killed.")


num_runs = 10

for _ in range(num_runs):
    print("Run", _)
    thread = threading.Thread(target=run_subprocess)
    thread.start()
    thread.join()