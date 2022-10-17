import yaml

class Point:

    def __init__(self, x=None, y=None, z=None) -> None:
        
        self.x = x
        self.y = y
        self.z = z

class BoundingBox:

    def __init__(self, c,r) -> None:
        
        self.c = c
        self.r = r

def aabb(bb_a, bb_b):

    if abs(bb_a.c.x - bb_b.c.x) > abs(bb_a.r.x + bb_b.r.x): return False
    if abs(bb_a.c.y - bb_b.c.y) > abs(bb_a.r.y + bb_b.r.y): return False
    if abs(bb_a.c.z - bb_b.c.z) > abs(bb_a.r.z + bb_b.r.z): return False

    return True

def parseYamlObjects(path):

    yamlfile = open(path, 'r')
    yaml_content = yaml.load(yamlfile, Loader=yaml.FullLoader)
    return yaml_content

def frameIsCenterd(a):

    if abs(a["x_max"]) != abs(a["x_min"]): return False
    if abs(a["y_max"]) != abs(a["y_min"]): return False
    if abs(a["z_max"]) != abs(a["z_min"]): return False

    return True

def getHalfwidths(value):

    return Point(abs(value["x_max"]), abs(value["y_max"]), abs(value["z_max"]))

def getCenter(pose):

    return Point(pose.position.x, pose.position.y, pose.position.z)

def getDefaultBoundingBox(pose):

    c = getCenter(pose)
    r = Point(0.1, 0.1, 0.1)

    return BoundingBox(c, r)

def getCenteredCenterAndHalfwidth(value, pose):

    r = Point()
    c = Point()

    if abs(value["x_min"]) != abs(value["x_max"]):
        width = value["x_max"] - value["x_min"]
        c.x = pose.position.x + value["x_min"] + width/2.0
        r.x = width/2.0
    else:
        c.x = pose.position.x
        r.x = abs(value["x_max"])
    
    if abs(value["y_min"]) != abs(value["y_max"]):
        width = value["y_max"] - value["y_min"]
        c.y = pose.position.y + value["y_min"] + width/2.0
        r.y = width/2.0
    else:
        c.y = pose.position.y
        r.y = abs(value["y_max"])

    if abs(value["z_min"]) != abs(value["z_max"]):
        width = value["z_max"] - value["z_min"]
        c.z = pose.position.z + value["z_min"] + width/2.0
        r.z = width/2.0
    else:
        c.z = pose.position.z
        r.z = abs(value["z_max"])

    return r, c

def getObjectBoundingBox(name, pose, bb_config_path):

    try:
        yamlfile = open(bb_config_path, 'r')
        yaml_content = yaml.load(yamlfile, Loader=yaml.FullLoader)
    except FileNotFoundError:
        print("[WARNING] Boundingbox.yaml not found, using default BB!")
        return getDefaultBoundingBox(pose)

    try:
        value = yaml_content[name]
    except KeyError:
        print(f"[WARNING] {name} not in BoundingBox.yaml, using default BB!")
        return getDefaultBoundingBox(pose)
    
    if frameIsCenterd(value):
        r = getHalfwidths(value)
        c = getCenter(pose)
    else:
        r, c = getCenteredCenterAndHalfwidth(value, pose)

    return BoundingBox(c, r)


def collision(obj1_name, obj1_pose, obj2_name, obj2_pose, bb_config_path):

    bb_1 = getObjectBoundingBox(obj1_name, obj1_pose, bb_config_path)
    bb_2 = getObjectBoundingBox(obj2_name, obj2_pose, bb_config_path)

    return aabb(bb_1, bb_2)
