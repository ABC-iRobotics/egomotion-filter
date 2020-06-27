

class RobotState():
    def __init__(self, timestamp, joints, x, y, z, rx, ry, rz):
        self.timestamp = timestamp
        self.joints = joints
        self.x = x
        self.y = y
        self.z = z
        self.rx = rx
        self.ry = ry
        self.rz = rz
        
    def __str__(self):
     return ("Timestamp: {}, Joint1: {}, Joint2: {}, Joint3: {}, Joint4: {}, Joint5: {}, Joint6: {}, " + 
             "x: {}, y: {}, z: {}, rx: {}, ry: {}, rz: {},").format(self.timestamp, self.joints[0],
             self.joints[1], self.joints[2], self.joints[3], self.joints[4], self.joints[5], 
             self.x, self.y, self.z, self.rx, self.ry, self.rz)

# Read robot states file
def read_robot_states(filename):
    file = open(filename, "r") 
    lines = file.readlines()

    robot_states = []
    
    #x_prev = 0.0
    #t_prev = 0.0
    for i in range(0,len(lines),13):
        if (lines[i].split(':')[0].strip()) != "Timestamp":
            raise NameError('Robot states file structure error!')
        timestamp = float(lines[i].split(':')[1].strip()) * 1000.0
        joints = []
        for j in range(6):
            joints.append(float(lines[i+1+j].split(':')[1].strip())) 
        x = float(lines[i+7].split(':')[2].strip())
        y = float(lines[i+8].split(':')[2].strip())
        z = float(lines[i+9].split(':')[2].strip())
        rx = float(lines[i+10].split(':')[2].strip())
        ry = float(lines[i+11].split(':')[2].strip())
        rz = float(lines[i+12].split(':')[2].strip())
        robot_states.append(RobotState(timestamp, joints, x, y, z, rx, ry, rz))
        
        #print((x - x_prev) / (timestamp - t_prev))
        #x_prev = x
        #t_prev = timestamp
    return robot_states
    
    
#robot_states = read_robot_states("robot_states.txt") 
#for state in robot_states:
   # print(state)
   #print(state.x)
 

