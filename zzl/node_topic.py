import subprocess
topic="/tf"
relate_node=[]
nodes=subprocess.run(['ros2', 'node', 'list'], stdout=subprocess.PIPE).stdout.decode("utf-8").splitlines()
for node in nodes:
    print(node)
    info=subprocess.run(["ros2","node","info",node],stdout=subprocess.PIPE).stdout.decode("utf-8")
    if topic in info:
        print("hhhhhh")
        relate_node.append(node)
print(relate_node)