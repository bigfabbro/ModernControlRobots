import math
import array

def point_to_quat(point):
    quaternion = []
    quaternion.append(0)
    for i in range(len(point)):
        quaternion.append(point[i])
    return quaternion


def normalize(rot_axis):
    norm_vector = []
    temp = 0
    for i in range(len(rot_axis)):
        temp = temp + (rot_axis[i]*rot_axis[i])
    norm = math.sqrt(temp)
    for i in range(len(rot_axis)):
        norm_vector.append(rot_axis[i]/norm)
    return norm_vector

def generate_rot_quaternion(rot_axis,angle):
    quaternion = []
    quaternion.append(math.cos(angle/2))
    for i in range(len(rot_axis)):
        quaternion.append(rot_axis[i]*math.sin(angle/2))
    return quaternion

#inverse of a quaternion
def quat_inverse(quat):
    inverse = []
    inverse.append(quat[0])
    for i in range(len(quat)-1):
        inverse.append(0-quat[i+1])
    return inverse

def hamilton_product(quat_1,quat_2):
    result = []
    result.append(quat_1[0]*quat_2[0]-quat_1[1]*quat_2[1]-quat_1[2]*quat_2[2]-quat_1[3]*quat_2[3])
    result.append(quat_1[0]*quat_2[1]+quat_1[1]*quat_2[0]+quat_1[2]*quat_2[3]-quat_1[3]*quat_2[2])
    result.append(quat_1[0]*quat_2[2]-quat_1[1]*quat_2[3]+quat_1[2]*quat_2[0]+quat_1[3]*quat_2[1])
    result.append(quat_1[0]*quat_2[3]+quat_1[1]*quat_2[2]-quat_1[2]*quat_2[1]+quat_1[3]*quat_2[0])
    return result

#the rotation of a point "p" in r3 (e.g. a quaternion with "real part" = 0) is perfomed by q*p*q'
#with q the quaternion associated to the rotation axis and q' his inverse
def apply_rotation(point_quat,rot_quat):

    inv_rot_quat = quat_inverse(rot_quat)

    #first compute the hamilt_prod q*p
    first = hamilton_product(rot_quat,point_quat)
    #then the result is given by the hamilt_prod first*q'
    second = hamilton_product(first,inv_rot_quat)
    return second

def double_rotation(point_quat,rot_quat_1,rot_quat_2):

    inv_rot_quat_1 = quat_inverse(rot_quat_1)
    inv_rot_quat_2 = quat_inverse(rot_quat_2)

    #with first and second the first rotation is performed (e.g. the rotation on rot_quat_1)
    first = hamilton_product(rot_quat_1,point_quat)
    second = hamilton_product(first,inv_rot_quat_1)

    #with third and fourth the second rotation is performed (e.g. the rotation on rot_quat_1)
    third = hamilton_product(rot_quat_2,second)
    fourth = hamilton_product(third,inv_rot_quat_2)

    return fourth

#points defining the rigid polytope in R3

p1 = [2.5,4,1]
p2 = [2.5,8,2]
p3 = [5.2,6,3]
p4 = [3.2,5,6.2]

#define the polytope as list of quaternions (associated to the points in r3)
polytope = []
polytope.append(point_to_quat(p1))
polytope.append(point_to_quat(p2))
polytope.append(point_to_quat(p3))
polytope.append(point_to_quat(p4))

#axis as vectors in R3
rot_axis_1 = [1, 1, 1]
rot_axis_2 = [-1, 2.5, 0]

#normalize the rotation axis
norm_rot_axis_1 = normalize(rot_axis_1)
norm_rot_axis_2 = normalize(rot_axis_2)

#angles in degree
rot_angle_1 = 120
rot_angle_2 = 61

#convert angles in radians
rot_angle_1 = math.radians(rot_angle_1)
rot_angle_2 = math.radians(rot_angle_2)

#associate to the couple "rotation axis - angle" the corresponding rotation quaternion
rot_quaternion_1 = generate_rot_quaternion(norm_rot_axis_1,rot_angle_1)
rot_quaternion_2 = generate_rot_quaternion(norm_rot_axis_2,rot_angle_2)

#calculate the new polytope's coordinates generated by the rotation
rot_polytope = []
for i in range(len(polytope)):
    rot_polytope.append(apply_rotation(polytope[i],rot_quaternion_1))

#return in r3 coordinates
for i in range(len(polytope)):
    rot_polytope[i].pop(0)

#applying a double rotation of the polytope
double_rot_polytope = []
for i in range(len(polytope)):
    double_rot_polytope.append(double_rotation(polytope[i],rot_quaternion_1,rot_quaternion_2))

#return in r3 coordinates
for i in range(len(polytope)):
    double_rot_polytope[i].pop(0)

print("The single rotated polytope has coordinates: ")
print(rot_polytope)

print("The double rotated polytope has coordinates: ")
print(double_rot_polytope)