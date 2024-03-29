import numpy as np

def rotation_matrix_to_point_to_vector(TP_minus_EP):
    x = np.array([1, 0, 0])
    z = TP_minus_EP / np.linalg.norm(TP_minus_EP)  
    y = np.cross(z, x)
    y /= np.linalg.norm(y)  
    x = np.cross(y, z)  

    rotation_matrix = np.column_stack((x, y, z))

    if np.linalg.det(rotation_matrix) < 0:
        y *= -1
        rotation_matrix[:, 1] = y

    return rotation_matrix


EP = np.array([2,6,10])
TP = np.array([2,6,0])
vector_to_point_to = TP-EP
rotation_matrix = rotation_matrix_to_point_to_vector(vector_to_point_to)
print("Rotation matrix to point to vector {}: \n{}".format(vector_to_point_to, rotation_matrix))
print("Determinant: ", np.linalg.det(rotation_matrix))

nogozones = []

def create_blockage(x1, x2, y1, y2, z):
    for x in range(x1, x2+1):
        for y in range(y1, y2+1):
            nogozones.append(np.array([x,y,z]))

def remove_blockage(x, y, z):
    global nogozones
    i = 0
    for item in nogozones:
        if np.array_equal(np.array(item),np.array([x,y,z])):
            nogozones = np.delete(nogozones, i, axis=0)
        i+=1
            

create_blockage(1,4,2,9,3)
for z in range(1, 3):
    create_blockage(1,4, 0,1, z)
    create_blockage(0,1, 2,5, z)
    create_blockage(-1,0, 6,9, z)
    create_blockage(1,4, 8,9, z)
    create_blockage(6,7, 6,9, z)
    create_blockage(4,5, 2,5, z)

remove_blockage(2,6,3)




def find_xy_given_z(v, p, z):
    x0, y0, z0 = p
    v1, v2, v3 = v
    
    t = (z - z0) / v3
    
    x = x0 + t * v1
    y = y0 + t * v2
    
    return x, y, t

clear = True

for z in range(0,4):
    x,y= find_xy_given_z(vector_to_point_to, EP, z)[:2]
    intersection_point = np.array([x,y,z])

    for voxel in nogozones:
        if np.sqrt(np.sum((intersection_point - voxel)**2))<1:

            clear=False
            break

print("GOOD TO GO") if clear else print("NO GO")
