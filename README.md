!pip install roboflow

from roboflow import Roboflow
rf = Roboflow(api_key="Lc5Zsror7KEAPwVHhhr4")
project = rf.workspace("eureka-c2uug").project("makerspace")
version = project.version(3)
dataset = version.download("yolov11")
