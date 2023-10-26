#!/usr/bin/env python
import rospy
from soft_align.srv import GetPath, GetPathResponse
from soft_align.msg import PathPoint
import RRT_Star_MG as rrt_star
import numpy as np
import rospkg, os



# push_pos = [[[-0.2375, -0.184375], 3.141592653589793], 
#             [[0, -0.184375], 3.141592653589793], 
#             [[0.2375, -0.184375], 3.141592653589793], 
#             [[0.3, -0.12187500000000001], -1.5707963267948966], 
#             [[0.3, 0], -1.5707963267948966], 
#             [[0.3, 0.12187500000000001], -1.5707963267948966], 
#             [[0.2375, 0.184375], 0], 
#             [[0, 0.184375], 0], 
#             [[-0.2375, 0.184375], 0], 
#             [[-0.3, 0.30625], 1.5707963267948966], 
#             [[-0.3, 0], 1.5707963267948966], 
#             [[-0.3, -0.12187500000000001], 1.5707963267948966]]

push_pos = [[-0.2375, -0.184375], 
            [0, -0.184375], 
            [0.2375, -0.184375], 
            [0.3, -0.12187500000000001], 
            [0.3, 0], 
            [0.3, 0.12187500000000001],
            [0.2375, 0.184375],
            [0, 0.184375],
            [-0.2375, 0.184375], 
            [-0.3, 0.30625],
            [-0.3, 0],
            [-0.3, -0.12187500000000001]]

def handle_get_path(req):
    control_file = [
        'control_cloth_50cm.csv',
        'control_pants_50cm_1.csv'
    ]
    # TODO: Implement your path calculation logic here

    # Change coordinate frame from real to sim: change x to y, y to x and theta to -theta
    start_config = [req.start_y, req.start_x, -req.start_theta]
    target_config = [req.target_y, req.target_x, -req.target_theta]

    # set constraints, in the coordinate frame of simulation
    table_center = (0,-0.9)
    table_size = (1.4, 1.0)
    reachable_length = 1.1
    object_size = (0.33, 0.48)

    # set_constraints(self, table_center=(0,1), table_size=(1,1), reachable_length = 1.1, object_size=(0.3, 0.3))

    # set parameters
    params = {
        'table_center': table_center,
        'table_size': table_size,
        'reachable_length': reachable_length,
        'object_size': object_size,
    }

    rospack = rospkg.RosPack()
    package_name = "soft_align"

    print("Query start: ", start_config)
    print("Query target: ", target_config)

    file_path = os.path.join(rospack.get_path(package_name), "script", control_file[0])
    path = rrt_star.plan(start_config, target_config, params=params, control_file=file_path, save_plot=False, repeat=10)

    path_response = []
    # convert path to be in the coordinate frame of real world
    u_path = path[1:]
    num = len(u_path)
    for i in range(num):
        (next_node, action) = u_path[i]
        subgoal = (next_node[1], next_node[0], -next_node[2])

        action['push_x'], action['push_y'], action['push_ori']  
        # find get index of the push_pose in the push_pos list, which is the closest to the action['push_x'], action['push_y']
        push_idx = np.argmin(np.linalg.norm(np.array(push_pos) - np.array([action['push_x'], action['push_y']]), axis=1))

        # path_response.append(subgoal, push_idx)

        point = PathPoint(x=subgoal[0], y=subgoal[1], theta=subgoal[2], action=push_idx)
        path_response.append(point)

    # have to record contact pose as a integer

    # For now, we'll return a dummy path
    
    response = GetPathResponse(path=path_response)

    # return path of intermediate states and the trigger push_pose integer

    # the real receive intermediate states and the trigger push_pose integer
    # make mapping of the integer to the real push_pose, sim and real should map to the same push_pose
    # this is just the primary action and subgoals for the robot to follow, it has to achieve the goal 
    # in the refinement phase by p2p control
    
    return response

def get_path_server():
    rospy.init_node('get_path_server')
    s = rospy.Service('get_path', GetPath, handle_get_path)
    rospy.loginfo("Ready to calculate path.")
    rospy.spin()

if __name__ == "__main__":
    get_path_server()
