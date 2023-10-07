from sys import argv

NAME = argv[1]

with open("map/{}.yaml".format(NAME), "r") as yaml:
    yml = yaml.read().split('\n')
    for line, kv in enumerate(yml):
        if kv.startswith("image:"):
            kv = kv.split(": ", 1)
            kv[1] = kv[1].replace("map/{}".format(NAME), NAME, 1)
            yml[line] = ": ".join(kv)
            break
with open("map/{}.yaml".format(NAME), "r") as yaml:
    yaml.write('\n'.join(yml))
