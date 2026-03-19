import pickle
import numpy as np
import sys

from google.protobuf.json_format import MessageToJson
import grpc
import ecloud_pb2 as ecloud
import ecloud_pb2_grpc as rpc

import carla

def save_obj(obj, name ):
    with open('obj/'+ name + '.pkl', 'wb') as f:
        pickle.dump(obj, f, pickle.HIGHEST_PROTOCOL)

def load_obj(name ):
    with open('obj/' + name + '.pkl', 'rb') as f:
        return pickle.load(f)

def get_rotation_mat(start_x, start_y, end_x, end_y):
    diff_lanex = end_x - start_x
    diff_laney = end_y - start_y

    angle = np.pi - np.arctan(diff_laney/diff_lanex)
    rotation_mat = np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
    inverse_rotation_mat = np.array([[np.cos(-angle),-np.sin(-angle)],[np.sin(-angle),np.cos(-angle)]])
    return rotation_mat, inverse_rotation_mat

def get_base_offset(x,y,rotation_mat):
    rot_base = np.array([[x],[y]])
    rot_base = np.matmul(rotation_mat,rot_base)
    return -rot_base

def transform(waypoint_x,waypoint_y,rotation_mat,offset):
    point_vec = np.array([[waypoint_x],[waypoint_y]])
    return (np.matmul(rotation_mat,point_vec) + offset).astype(int)

def inverse_transform(waypoint_x,waypoint_y,rotation_mat,offset):
    point_vec = np.array([[waypoint_x],[waypoint_y]])
    return np.matmul(rotation_mat,point_vec+offset)

def get_scaling(waypoints):
    count = 0
    rotation_mat, inverse_rotation_mat = get_rotation_mat(waypoints[1]['x'][0],waypoints[1]['y'][0],waypoints[1]['x'][-1],waypoints[1]['y'][-1])
    offset = get_base_offset(waypoints[1]['x'][0],waypoints[1]['y'][0],rotation_mat)
    rot_1 = []
    scaling = []
    for i in waypoints.keys():
        rot_1.append(transform(waypoints[i]['x'][-1],waypoints[i]['y'][-1],rotation_mat,offset))
        count += 1 
        if count > 1:
            scaling.append(count/(rot_1[-1][1,0]-rot_1[0][1,0]))
    return scaling

def serialize_waypoint(waypoint):

    serialized_waypoint = ecloud.Waypoint()
    serialized_waypoint.id = str(waypoint.id)
    
    transform = ecloud.Transform()
    
    location   = ecloud.Location()
    location.x = waypoint.transform.location.x
    location.y = waypoint.transform.location.y
    location.z = waypoint.transform.location.z

    rotation       = ecloud.Rotation()
    rotation.yaw   = waypoint.transform.rotation.yaw
    rotation.pitch = waypoint.transform.rotation.pitch
    rotation.roll  = waypoint.transform.rotation.roll

    transform.location.CopyFrom(location)
    transform.rotation.CopyFrom(rotation)

    serialized_waypoint.transform.CopyFrom(transform)

    serialized_waypoint.road_id     = waypoint.road_id
    serialized_waypoint.section_id  = waypoint.section_id
    serialized_waypoint.lane_id     = waypoint.lane_id
    serialized_waypoint.s           = waypoint.s
    serialized_waypoint.is_junction = waypoint.is_junction
    serialized_waypoint.lane_width  = waypoint.lane_width

    # int32 lane_change = 9; // unused - enum if needed
    # int32 lane_type = 10; // unused - enum if needed
    # int32 right_lane_marking = 11; // unused - enum if needed
    # int32 left_lane_marking = 12; // unused - enum if needed

    return serialized_waypoint    

def deserialize_waypoint(serialized_waypoint, dao):
    '''
    world = self.vehicle_manager_list[0].vehicle.get_world()
    self._dao = GlobalRoutePlannerDAO(world.get_map(), 2)
    location = self._dao.get_waypoint(carla.Location(x=car_array[0][i], y=car_array[1][i], z=0.0))
    '''

    '''waypoint = carla.Waypoint

    waypoint.id = serialized_waypoint.id
    
    waypoint.transform.location.x = serialized_waypoint.transform.location.x 
    waypoint.transform.location.y = serialized_waypoint.transform.location.y
    waypoint.transform.location.z = serialized_waypoint.transform.location.z

    waypoint.transform.location.yaw   = serialized_waypoint.transform.rotation.yaw
    waypoint.transform.rotation.pitch = serialized_waypoint.transform.rotation.pitch
    waypoint.transform.rotation.roll  = serialized_waypoint.transform.rotation.roll

    waypoint.road_id     = serialized_waypoint.road_id
    waypoint.section_id  = serialized_waypoint.section_id
    waypoint.lane_id     = serialized_waypoint.lane_id
    waypoint.s           = serialized_waypoint.s
    waypoint.is_junction = serialized_waypoint.is_junction
    waypoint.lane_width  = serialized_waypoint.lane_width'''

    #print(f"deserializing waypint - x:{serialized_waypoint.transform.location.x}, y:{serialized_waypoint.transform.location.y}, z:{serialized_waypoint.transform.location.z}, rl:{serialized_waypoint.transform.rotation.roll}, pt:{serialized_waypoint.transform.rotation.pitch}, yw:{serialized_waypoint.transform.rotation.yaw}")

    waypoint = dao.get_waypoint(carla.Location(x=serialized_waypoint.transform.location.x, y=serialized_waypoint.transform.location.y, z=serialized_waypoint.transform.location.z))

    return waypoint

class transform_processor():
    def __init__(self,waypoints):
        self.rotation_mat, self.inverse_rotation_mat = get_rotation_mat(waypoints[1]['x'][0],waypoints[1]['y'][0],waypoints[1]['x'][-1],waypoints[1]['y'][-1])
        self.offset = get_base_offset(waypoints[1]['x'][0],waypoints[1]['y'][0],self.rotation_mat)
        self.scaling = get_scaling(waypoints)
        self.scaling = [1] + self.scaling
        self.waypoints = waypoints
        self.lanewidth = 3 #Difference between adjacent lane indices, rounded to int, has to be found or coded as 'edge configuration' parameter

    def process_single_waypoint_forward(self, waypoint_x, waypoint_y):
        np_waypoint = transform(waypoint_x, waypoint_y, self.rotation_mat, self.offset)
        # print("Preprocessed Waypoint: ", np_waypoint)
        lane_number = -int(np_waypoint[1,0]/self.lanewidth) #Sign change present since all waypoints turned out negative, handle that case later more cleanly
        lane_number = max(lane_number,0)
        lane_number = min(lane_number,3) #4 lanes, hardcoded for now.
        # print("Lane Found: ", lane_number)
        np_waypoint[1,0] = lane_number # max(np_waypoint[1,0]*self.scaling[lane_number] - 1,0) #Handle edge case of zero lane
        # print("Scaling: %s " %self.scaling)
        # print("Waypoint Processed: ", np_waypoint)
        # sys.exit()
        return (np_waypoint[0,0], np_waypoint[1,0])

    def process_waypoints_bidirectional(self,indice): #Present for test purposes mainly
        initial_compute_flag = 0
        counter = 0
        rot_end, rot_inverted = [], []

        for i in self.waypoints.keys():

            #print("Initial: ", self.waypoints[i]['x'][indice],self.waypoints[i]['y'][indice])

            rot_end.append(transform(self.waypoints[i]['x'][indice],self.waypoints[i]['y'][indice],self.rotation_mat,self.offset))
            # lane_number = -int(rot_end[-1][1,0]/self.lanewidth)
            # rot_end[-1][1,0] = lane_number

            if rot_end[-1][1,0] != 0:
                rot_end[-1][1,0] = self.scaling[counter]*rot_end[-1][1,0]-1

            #print("Transformed: ", rot_end[-1])
            if rot_end[-1][1,0] != 0:
                rot_end[-1][1,0] = (rot_end[-1][1,0]+1)/self.scaling[counter]

            rot_inverted.append(inverse_transform(rot_end[-1][0,0],rot_end[-1][1,0],self.inverse_rotation_mat,-self.offset))
            #print("Inverted: ", rot_inverted[-1])
            counter += 1

        return rot_end, rot_inverted

    def process_forward(self,indice):
        initial_compute_flag = 0
        counter = 0
        rot_end = []

        for i in self.waypoints.keys():
            #print("Initial: ", self.waypoints[i]['x'][indice],self.waypoints[i]['y'][indice])
            rot_end.append(transform(self.waypoints[i]['x'][indice],self.waypoints[i]['y'][indice],self.rotation_mat,self.offset))
            # lane_number = -int(rot_end[-1][1,0]/self.lanewidth)
            # rot_end[-1][1,0] = lane_number
            if rot_end[-1][1,0] != 0:
                rot_end[-1][1,0] = self.scaling[counter]*rot_end[-1][1,0]-1
            #print("Transformed: ", rot_end[-1])
            counter += 1
        return rot_end

    def process_back(self,processed_forward_array):
        initial_compute_flag = 0
        counter = 0
        rot_inverted = []

        for i in range(0,len(processed_forward_array)):# len(self.waypoints.keys())):
            if processed_forward_array[i][1,0] != 0:
                processed_forward_array[i][1,0] = (processed_forward_array[i][1,0]+1)/self.scaling[int(processed_forward_array[i][1,0])]# counter]

            rot_inverted.append(inverse_transform(processed_forward_array[i][0,0],processed_forward_array[i][1,0],
                self.inverse_rotation_mat,-self.offset))
            #print("Inverted: ", rot_inverted[-1])
            counter += 1

        return rot_inverted

#waypoints = load_obj('waypoints')
#print(waypoints.keys())

#processor = transform_processor(waypoints)
#_, _ = processor.process_waypoints_bidirectional(0)
#inverted = processor.process_forward(0)

#print("List of transformed points: ", inverted)
#back = processor.process_back(inverted)

