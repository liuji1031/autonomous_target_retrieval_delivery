import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

class Planner:
    def __init__(self,
                 arena_xrange=[-1*30.48,9*30.48],
                 arena_yrange=[-1*30.48,9*30.48],
                 border_offset=15,
                 k_near=15,
                 max_theta=35,
                 target_offset_radius=40):
        self.obstacle_radius = 15
        self.n_max_node = 100
        self.target_offset_radius = target_offset_radius
        self.k_near = k_near
        self.border_offset = border_offset
        self.arena_xrange = arena_xrange
        self.arena_yrange = arena_yrange
        self.max_theta = max_theta
            
    def gen_path(self, target_loc_, obstacle_center, mode, curr_pose=(0,0,0),debug=0):
        target_loc = np.copy(target_loc_)
        target_loc = np.reshape(target_loc,(1,2))
        path = None
        niter = 0
        max_niter = 10
        target_offset_radius = 50
        beta = 0.95
        while path is None and niter<max_niter:
            
            if mode == 'target_block' or mode == 'target_czone':
                target_loc_input_1 = target_loc
            elif mode == 'open_space':
                # plan to open space
                target_loc_input_1 = None
            else:
                print('unclear mode!')
                return 0
            node_loc, road_map, edge_weight = self.PRM(n_max_node=self.n_max_node,
                                         k_near=self.k_near,
                                         obstacle_radius=self.obstacle_radius,
                                         obstacle_center=obstacle_center,
                                         r_range=[5,100],
                                         theta_range=[-self.max_theta/180*np.pi,self.max_theta/180*np.pi],
                                         n_node_around=30, 
                                         target_offset_radius=target_offset_radius*np.power(beta,niter),
                                         target_loc_input=target_loc_input_1,
                                         curr_pose=curr_pose)
            
            # target_offset_radius is reduced every time a path cannot be found

            if mode == 'target_block' or mode == 'target_czone':
                target_loc_input_2 = target_loc
            elif mode == 'open_space':
                print('open space target:', target_loc)
                # choose the node loc that is closest to the target_loc
                # target_loc_input_2 = self.pick_node_close_to_direct_line(target_loc, node_loc, target_offset_radius)
                target_loc_input_2 = self.pick_close_node(target_loc, node_loc, min_proj=np.cos(np.pi/6))

            path = self.a_star_search(target_loc=target_loc_input_2,
                         node_loc=node_loc,
                         road_map=road_map,
                         edge_weight=edge_weight)

            niter+=1
            
        if path is not None:
            path_trimmed,_,is_straight_path = self.trim_current_path(path, obstacle_center, self.obstacle_radius, mode=mode, target_offset_radius=target_offset_radius)
            print('path trimmed')
        else:
            path_trimmed = None
            is_straight_path = 0
            print('path not found!')

        #if debug == 1:
        #    self.plot_result(target_loc, obstacle_center, self.obstacle_radius, path, path_trimmed, road_map, node_loc)
        return path_trimmed, path, is_straight_path
    
    def plot_result(self,target_loc, obstacle_center, obstacle_radius, path_ori, path_trimmed, road_map, node_loc):
        plt.style.use('my_style.mpstyle')
        plt.figure()
        first_obs=1
        for x,y in obstacle_center:
            phi = np.linspace(0,2*np.pi,50)
            x1 = x + obstacle_radius*np.cos(phi)
            y1 = y + obstacle_radius*np.sin(phi)
            if first_obs==1:
                plt.plot(x1,y1,color=[0.2,0.2,0.2],label='obstacles',zorder=1)
                first_obs = 0
            else:
                plt.plot(x1,y1,color=[0.2,0.2,0.2],zorder=1)
        first_rmp = 1
        for i,n in enumerate(road_map):
            x1,y1 = node_loc[i]
            for j in n:
                x2,y2 = node_loc[j]
                if first_rmp==1:
                    plt.plot([x1,x2],[y1,y2],color=[0.5,0.5,0.5],linewidth=0.5,label='road map',zorder=2)
                    first_rmp = 0
                else:
                    plt.plot([x1,x2],[y1,y2],color=[0.5,0.5,0.5],linewidth=0.5,zorder=2)
        if path_ori is not None:
            plt.plot(path_ori[:,0],path_ori[:,1],color='r',linewidth=4,label='original path',zorder=3)
            plt.plot(path_trimmed[:,0],path_trimmed[:,1],color=[52./255, 70./255, 235./255],linewidth=2,label='trimmed path',zorder=4)
        plt.scatter(target_loc[0,0],target_loc[0,1],s=200,color=[255./255, 135./255, 243./255],edgecolors=[0.4,0.4,0.4],marker='*',label='target',zorder=5)
        plt.gca().axis('equal')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend(frameon=False)
        now = datetime.now() # current date and time
        date_time = now.strftime("%Y-%m-%d_%H-%M-%S")
        fn = 'planning_'+date_time+'.pdf'
        plt.savefig(fn)
        plt.show(block=True)


    def pick_node_close_to_direct_line(self, target_loc, node_loc, target_offset_radius=50):
        dist_to_target = np.linalg.norm( node_loc-target_loc, axis=1)
        target_loc_ = target_loc.copy().flatten()
        target_loc_norm = np.linalg.norm(target_loc)
        n_node = node_loc.shape[0]
        proj = np.zeros((n_node,))
        for inode, node in enumerate(node_loc):
            node_norm = np.linalg.norm(node)
            if node_norm == 0:
                proj[inode]=-2
                continue
            # compute normalized projection
            proj[inode] = np.inner(node, target_loc_) / node_norm / target_loc_norm
        # sort projection value descending (closest to the straight line to target)
        isort_proj = np.flip(np.argsort(proj))
        
        target_offset_radius_lb = target_offset_radius
        target_offset_radius_ub = target_offset_radius*1.2

        while True:
            ind_node = None
            for isort in isort_proj:
                if dist_to_target[isort]>=target_offset_radius_lb and dist_to_target[isort]<=target_offset_radius_ub:
                    ind_node = isort
                    break
            if ind_node is not None:
                target_loc_input_2 = node_loc[[ind_node],:]
                break
            else:
                # increase range
                target_offset_radius_lb *= 0.9
                target_offset_radius_ub *= 1.1
        
        print('Chosen surragate target node:',target_loc_input_2,'dist to target,',dist_to_target[ind_node])
        return target_loc_input_2

    def pick_close_node(self, target_loc, node_loc, min_proj=np.cos(np.pi/4)):
        dist_to_target = np.linalg.norm( node_loc-target_loc, axis=1)
        target_loc_ = target_loc.copy().flatten()
        target_loc_norm = np.linalg.norm(target_loc)
        n_node = node_loc.shape[0]
        proj = np.zeros((n_node,))
        for inode, node in enumerate(node_loc):
            node_norm = np.linalg.norm(node)
            if node_norm == 0:
                proj[inode]=-2
                continue
            # compute normalized projection
            proj[inode] = np.inner(node, target_loc_) / node_norm / target_loc_norm

        isort_dist = np.argsort(dist_to_target)
        while True:
            ind_node = None
            for isort in isort_dist:
                # print('dist, proj:',dist_to_target[isort],proj[isort])
                if np.abs(proj[isort]) >= min_proj:
                    ind_node = isort
                    break
            if ind_node is not None:
                target_loc_input_2 = node_loc[[ind_node],:]
                break
            else:
                # increase range
                min_proj = min_proj*0.9
        
        print('Chosen surragate target node:',target_loc_input_2,'dist to target,',dist_to_target[ind_node])
        return target_loc_input_2

    def trim_current_path(self,path, obstacle_center, obstacle_radius, mode, target_offset_radius=40):
        trim_path = []
        trim_node_id = []
        # if mode=='target_block':
        #     N = path.shape[0]-1 # ignore the final block location for now
        # else:
        #     N = path.shape[0]
        N = path.shape[0]
        i = 0
        while i < N:
            if i==0:
                trim_path.append(path[i,:]) # first node is always [0,0]
                trim_node_id.append(i)
                i+=1
            else:
                last_node = trim_path[-1]
                curr_node = path[i,:]
                if self.is_collision_free(last_node,curr_node,obstacle_center, obstacle_radius):
                    potential_next_node = curr_node
                    potential_node_id = i
                    if i == N-1: # last node in the path
                        trim_path.append(potential_next_node)
                        trim_node_id.append(potential_node_id)
                    i+=1 # explore next node
                else:
                    trim_path.append(potential_next_node)
                    trim_node_id.append(potential_node_id)
        if trim_node_id[-1] != N-1:
            trim_path.append(path[N-1,:])
            trim_node_id.append(N-1)
        if len(trim_path) == 2:
            is_straight_path = 1
        else:
            is_straight_path = 0
        if mode =='target_block':
            # put the block back
            # trim_path.append(path[-1,:])

            # check if we are only left with two nodes
            if len(trim_path) == 2:
                # split the path accoriding to target_offset_radius
                dxy = trim_path[1] - trim_path[0]
                dxy_len = np.linalg.norm(dxy) 
                if np.linalg.norm(dxy) > target_offset_radius:
                    # insert a mid point
                    e = dxy/dxy_len # unit vector
                    mid_point = (dxy_len-target_offset_radius)*e + trim_path[0]
                    trim_path.insert(1,mid_point)

        trim_path = np.array(trim_path)
        return trim_path, trim_node_id, is_straight_path

    def calculate_dist(self, node_loc,target_loc):
        dist_to_target = node_loc - target_loc
        dist_to_target = np.sqrt(np.power(dist_to_target[:,0],2)+np.power(dist_to_target[:,1],2))
        imin = np.argmin(dist_to_target)
        return dist_to_target,imin

    def a_star_search(self, target_loc,node_loc,road_map,edge_weight):
        n_node = len(road_map)
        # connect target loc to closest node
        dist_to_target,end_node = self.calculate_dist(node_loc,target_loc)
        
        # find the starting node, which is closest to (0,0)
        dist_to_origin,start_node = self.calculate_dist(node_loc,np.zeros((1,2)))
        
        # initiate cost arrays
        past_cost = 1e9*np.ones((n_node,1))
        optimist_cost_to_go,_ = self.calculate_dist(node_loc,node_loc[end_node,:])
        est_total_cost = 1e9*np.ones((n_node,))
        
        past_cost[start_node]=0
        est_total_cost[start_node]=past_cost[start_node]+optimist_cost_to_go[start_node]
        
        # append imin_origin to the open list
        open_list = np.array([[start_node,est_total_cost[start_node]]])
        closed_list = np.zeros((0,))
        
        # parent array
        parent = -1*np.ones((n_node,1))
        while open_list.shape[0]>0:
            i = int(open_list[0,0])
            # print(i)
            past_cost_this = past_cost[i]
            if i == end_node: # target found
                print('path to target found')
                break
            # remove 
            np.append(closed_list,i)
            open_list = open_list[1:,:]
            
            # explore the connected nodes
            for n in road_map[i]:
                #print('node ',n)
                if np.argwhere(closed_list==n).shape[0]!=0: # in close list
                    continue
                    
                # find cost to reach n
                cost = edge_weight[i,n] + past_cost_this
                #print(cost,past_cost[n])
                if cost < past_cost[n]:
                    parent[n] = i
                    past_cost[n] = cost
                    est_total_cost[n] = past_cost[n]+optimist_cost_to_go[n]
                    # also update if in the open list
                    j = np.argwhere(open_list[:,0]==n)
                    if j.shape[0]==0: # add to open list
                        open_list = np.append(open_list, np.array([[n,est_total_cost[n]]]),axis=0)
                    else: # update
                        open_list[j,1] = est_total_cost[n]
                        # sort
                        open_list = open_list[np.argsort(open_list[:,1]),:]
            # print(open_list)
        if parent[end_node]!=-1: # a path is found
            # construct path using location
            path = node_loc[[end_node],:]
            p = int(parent[end_node])
            while True:
                path = np.append(path,node_loc[[p],:],axis=0)
                if p==start_node:
                    break
                p = int(parent[p])
            path = np.flipud(path)
        else:
            path = None
            
        return path

    def within_arena_range(self, x, y, curr_pose):
        x_, y_ = self.convert_to_world_frame(curr_pose, np.array([x,y])).flatten()
        offset = self.border_offset
        if x_>=self.arena_xrange[0]+offset and x_<=self.arena_xrange[1]-offset and \
        y_>=self.arena_yrange[0]+offset and y_<=self.arena_yrange[1]-offset:
            return True
        else:
            return False

    def PRM(self,n_max_node,k_near,obstacle_radius,obstacle_center,r_range,theta_range,
       n_node_around=8, target_offset_radius=50,target_loc_input=[0,0],n_r=10,n_theta=15,use_random=1,curr_pose=(0,0,0)):
        
        r_list, theta_list = np.meshgrid(np.linspace(r_range[0],r_range[1],n_r,endpoint=True),
                                         np.linspace(theta_range[0],theta_range[1],n_theta,endpoint=True))
        r_list = r_list.flatten()
        theta_list = theta_list.flatten()
        ind_r_theta = 0
        # if target_loc is None, it means we are generating a path to an open space
        if target_loc_input is not None: 
            target_loc = np.squeeze(target_loc_input)
        # generate n_max_node
        inode = 0
        node_loc = []
        iaround = 0
        around_pt_phi = np.linspace(0.25*np.pi,1.75*np.pi,n_node_around,endpoint=True)
        while True:
            if use_random == 1 and inode>=n_max_node:
                break

            if target_loc_input is not None:
                around_pt = 0
                if inode == 0:
                    x = 0.0
                    y = 0.0
                elif inode == 1:
                    x = target_loc[0]
                    y = target_loc[1]
                elif iaround>=0 and iaround<n_node_around:
                    x = target_loc[0] + target_offset_radius*np.cos(around_pt_phi[iaround])
                    y = target_loc[1] + target_offset_radius*np.sin(around_pt_phi[iaround])
                    around_pt = 1
                    iaround+=1
                else:
                    if use_random==1:
                        r = np.random.rand()*(r_range[1] - r_range[0]) + r_range[0]
                        theta = np.random.rand()*(theta_range[1] - theta_range[0]) + theta_range[0]
                        # remember x is pointing forward

                    else:
                        # use grid
                        if ind_r_theta < r_list.size:
                            r = r_list[ind_r_theta]
                            theta = r_list[ind_r_theta]
                            ind_r_theta += 1
                        else:
                            break
                    
                    x = r*np.cos(theta)
                    y = r*np.sin(theta)
                    # print(x,y)
            else:
                if inode == 0:
                    x = 0.0
                    y = 0.0
                else:
                    if use_random==1:
                        r = np.random.rand()*(r_range[1] - r_range[0]) + r_range[0]
                        theta = np.random.rand()*(theta_range[1] - theta_range[0]) + theta_range[0]
                        # remember x is pointing forward

                    else:
                        # use grid
                        if ind_r_theta < r_list.size:
                            r = r_list[ind_r_theta]
                            theta = r_list[ind_r_theta]
                            ind_r_theta += 1
                        else:
                            break
                    
                    x = r*np.cos(theta)
                    y = r*np.sin(theta)
            
            # check if the point is in the arena
            in_arena = self.within_arena_range(x, y, curr_pose)
            if in_arena==False:
                continue

            if obstacle_center.size > 0:
                dist = (obstacle_center[:,0]-x)**2 + (obstacle_center[:,1]-y)**2
                dist = np.sqrt(dist)
                no_collision = (np.sum(dist<=obstacle_radius)==0)
            else:
                no_collision = True
            
            if target_loc_input is not None:
                # also not within target_offset_radius to the target loc
                dist2 = (target_loc[0]-x)**2 + (target_loc[1]-y)**2
                dist2 = np.sqrt(dist2)
                if inode==0 or inode==1:
                    inode+=1
                    node_loc.append([x,y])
                elif no_collision and (around_pt==1 or dist2>=target_offset_radius):
                    inode+=1
                    node_loc.append([x,y])
            else: # target loc is a node in the map determined later
                # add to road map if not within the obstacle
                if inode==0 or no_collision:
                    inode+=1
                    node_loc.append([x,y])
        
        n_max_node = len(node_loc)
        node_loc = np.array(node_loc)
        # build road map
        road_map = [[] for _ in range(n_max_node)]

        for i in range(n_max_node):
            # find k nearest point
            dist,_ = self.calculate_dist(node_loc,node_loc[i])
            idx = np.argsort(dist)
            if target_loc_input is not None and i == 1: 
                # target loc is the actual block
                # connect the block with all the surrounding points
                idx = idx[1:n_node_around+1]
            else:
                idx = idx[1:k_near+1]
            for j in idx:
                if self.is_collision_free(node_loc[i], node_loc[j],obstacle_center,obstacle_radius)==True:
                    road_map[i].append(j)
                    road_map[j].append(i)
                    
        for i in range(n_max_node):
            road_map[i] = np.unique(road_map[i])
        
        edge_weight = np.sqrt(np.power(node_loc[:,[0]]-node_loc[:,[0]].T,2)+np.power(node_loc[:,[1]]-node_loc[:,[1]].T,2))
        
        return node_loc, road_map, edge_weight
    
    def is_collision_free(self,pt1,pt2,obstacle_center, obstacle_radius):
        if obstacle_center.size == 0:
            return True
        n_obstacle = obstacle_center.shape[0]
        min_dist = np.zeros((n_obstacle,1))
        A1 = -pt1[0] + pt2[0]
        A2 = -pt1[1] + pt2[1]
        B1 = pt1[0] - obstacle_center[:,0]
        B2 = pt1[1] - obstacle_center[:,1]
        rho_min = -(A1*B1 + A2*B2)/(A1*A1+A2*A2)
        rho_min = np.minimum(rho_min,1)
        rho_min = np.maximum(rho_min,0)
        min_dist = (A1*A1+A2*A2)*rho_min*rho_min + (2*A1*B1+2*A2*B2)*rho_min + B1*B1+B2*B2
        min_dist = np.sqrt(min_dist)
        s = np.sum(min_dist<obstacle_radius)
        return s==0

    def convert_to_world_frame(self, curr_pose, base_coord_input):
        curr_x, curr_y, curr_yaw = curr_pose
        base_coord = np.copy(base_coord_input)
        if base_coord.ndim == 1:
            base_coord = np.reshape(base_coord,(2,1))
        else: # 2d array, transpose, 2 by N matrix after transpose
            base_coord = base_coord.T 
            assert(base_coord.shape[0]==2)
        rot_mat = self.calculate_rot_matrix_from_rad(curr_yaw)
        t = np.array([curr_x, curr_y]).reshape((2,1))
        world_coord = np.linalg.multi_dot([rot_mat, base_coord]) + t
        return world_coord.T # 2 by N array

    def calculate_rot_matrix_from_rad(self,rad):
        c = np.cos(rad)
        s = np.sin(rad)
        rot_mat = np.array([[c,-s],[s,c]])
        return rot_mat