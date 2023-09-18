import multiprocessing
import subprocess

def run_camera_module():
    subprocess.call(["python3", "camera_module.py"])

def run_robot_module():
    subprocess.call(["python3", "robot_module.py"])

if __name__ == "__main__":

    print("Started.")

    process1 = multiprocessing.Process(target=run_camera_module)
    process2 = multiprocessing.Process(target=run_robot_module)

    process1.start()
    process2.start()

    process1.join()
    process2.join()

    print("Done.")
