import rospy
import numpy as np
from .util import get_ros_param, get_text_msg, RefPath, extract_state_odom
from .boss import BossPlanner
from racecar_routing.srv import Plan, PlanRequest
from final_project.srv import Task, TaskResponse, Reward, RewardResponse, Schedule, ScheduleResponse
from std_srvs.srv import Empty, EmptyResponse
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
import threading
from threading import Lock
import yaml
from typing import List, Tuple

# ! IMPORTANT: You do not need to modify this file for the final project


class SwiftHaul:
    def __init__(self) -> None:
        self.boss_v_ref = 0.5
        self.boss_task_discount = 0.5
        self.student_pose = None
        self.finished = False
        self.total_income = 0 
        self.start_time = None
        self.cur_student_task = -1 # record the last task that was assigned to the student
        self.cur_student_reward = 0 # record the reward of the last task that was assigned to the student
        self.doing_boss_task = False
        self.load_params()
        self.setup_publisher()
        self.setup_subscriber()
        self.setup_service()
        self.setup_warehouse()
        self.create_boss_schedule()
        
        self.boss_truck = BossPlanner(self.boss_v_ref)
        threading.Thread(target=self.main_thread).start()
        
        
    def setup_warehouse(self):
        self.update_lock = Lock()
        # Load parameters
        with open(self.warehouse_yaml, "r") as stream:
            warehouse_info = yaml.safe_load(stream)
            
        # HACK: No failsafe is implemented for invalid warehouse config
        self.warehouse_location = []
        self.warehouse_probability = []
        for warehouse in warehouse_info.values():
            location_info = warehouse['location']
            location_info.extend(warehouse['dxdy'])
            self.warehouse_location.append(location_info)
            self.warehouse_probability.append(warehouse['probability'])
        self.num_warehouse = len(self.warehouse_location)
        # Initialize the current task as starting from the current warehouse
        self.cur_task = [i for i in range(self.num_warehouse)]
        self.refresh() # do a refresh to get the first goal
        
        # get reference trajectory and reward  
        self.ref_path = []
        for i in range(self.num_warehouse):
            temp = []
            for j in range(self.num_warehouse):
                if i == j:
                    temp.append(None)
                    continue
                plan_request = PlanRequest(self.warehouse_location[i], self.warehouse_location[j])
                temp.append(self.generate_path(self.plan_client(plan_request)))
                
            self.ref_path.append(temp)
    
    def generate_path(self, plan_respond):
        path_msg = plan_respond.path
        x = []
        y = []
        width_L = []
        width_R = []
        speed_limit = []
        
        for waypoint in path_msg.poses:
            x.append(waypoint.pose.position.x)
            y.append(waypoint.pose.position.y)
            width_L.append(waypoint.pose.orientation.x)
            width_R.append(waypoint.pose.orientation.y)
            speed_limit.append(waypoint.pose.orientation.z)
                    
        centerline = np.array([x, y])
        
        ref_path = RefPath(centerline, width_L, width_R, speed_limit, loop=False)
        reward = ref_path.length ** 1.5
        return ref_path, int(np.ceil(reward))
        
    def create_boss_schedule(self):
        
        cur_warehouse = 0
        cur_start_time = 0
        
        task_schedule = []
        
        while True:
            # randomly select a warehouse
            cur_target = np.random.randint(self.num_warehouse)
            if cur_target == cur_warehouse:
                continue
            # get path from cur_warehouse to cur_target
            cur_path = self.ref_path[cur_warehouse][cur_target][0]
            cur_path_length = cur_path.length
            t_approx = np.ceil((cur_path_length / self.boss_v_ref)/self.dt_refresh) * self.dt_refresh + 10 # extra 10 seconds buffer
            next_start_time = cur_start_time + t_approx
            
            if next_start_time > self.time_limit:
                break
            task_schedule.append((cur_warehouse, cur_target, cur_start_time, cur_path))
            
            # update cur_warehouse and cur_start_time
            cur_warehouse = cur_target
            cur_start_time = next_start_time
        
        self.boss_task_schedule = task_schedule
        self.cur_boss_task_idx = None
        
    def load_params(self):
        self.time_limit = get_ros_param("~time_limit", 300) # 300 seconds
        self.dt_refresh = get_ros_param("~dt_refresh", 5) # 5 seconds
        self.random_seed = get_ros_param("~random_seed", 0)
        np.random.seed(self.random_seed) # Set random seed
        self.warehouse_yaml = get_ros_param("~warehouse_yaml", None)
        self.student_pose_topic = get_ros_param("~student_pose_topic", "/SLAM/Pose")
        
    def setup_publisher(self):
        self.info_pub = rospy.Publisher('vis/info', MarkerArray, queue_size=10)
        
    def setup_subscriber(self):
        self.student_pose_sub = rospy.Subscriber(self.student_pose_topic, Odometry, self.student_odometry_callback, queue_size=10)
        self.student_pose_lock = Lock()
    
    def setup_service(self):
        self.start_service = rospy.Service('/SwiftHaul/Start', Empty, self.start_callback)
        self.side_task_service = rospy.Service('/SwiftHaul/SideTask', Task, self.request_sidetask_callback)
        self.boss_task_service = rospy.Service('/SwiftHaul/BossTask', Task, self.request_bosstask_callback)
        self.boss_schedule_service = rospy.Service('/SwiftHaul/BossSchedule', Schedule, self.request_bossschedule_callback)
        self.get_reward_service = rospy.Service('/SwiftHaul/GetReward', Reward, self.get_reward_callback)
        rospy.wait_for_service('/routing/plan')
        self.plan_client = rospy.ServiceProxy('/routing/plan', Plan)
        rospy.loginfo("Service /routing/plan is ready")
        
    def student_odometry_callback(self, msg):
        self.student_pose_lock.acquire()
        self.student_pose = extract_state_odom(msg)
        self.student_pose_lock.release()
            
    def main_thread(self):
        rate = rospy.Rate(30) # 30hz
        t_last_refresh = None
        while rospy.is_shutdown() is False:         
            # publish info
            marker_array = MarkerArray()
            marker_array.markers.append(self.display_time())
            marker_array.markers.append(self.display_income())
            marker_array.markers.extend(self.display_warehouse())
            marker_array.markers.extend(self.display_task())
            marker_array.markers.extend(self.display_boss_schedule())
            marker_array.markers.extend(self.display_current_task())
            self.boss_truck.visualize_car(ns='boss_truck', id=0, color=[1,0,0,0], marker_array=marker_array)
            self.info_pub.publish(marker_array)
            
            # Update state
            if self.start_time is None:
                rate.sleep()
                continue
            elif t_last_refresh is None:
                t_last_refresh = self.start_time
                
            t_now = rospy.get_time()
                
            # Update path for the boss
            update_route = False
            if self.cur_boss_task_idx is None:
                self.cur_boss_task_idx = 0
                update_route = True
            elif self.cur_boss_task_idx+1 < len(self.boss_task_schedule) and \
                (t_now - self.start_time) >= self.boss_task_schedule[self.cur_boss_task_idx+1][2]:
                self.cur_boss_task_idx += 1
                update_route = True
            
            if update_route:
                rospy.loginfo("Boss truck is updating path to warehouse %d", self.boss_task_schedule[self.cur_boss_task_idx][1])
                self.boss_truck.update_path(self.boss_task_schedule[self.cur_boss_task_idx][-1])
            
            t_since_last_refresh = rospy.get_time() - t_last_refresh
            # Refresh the start and goal every second
            if t_since_last_refresh >= self.dt_refresh:
                t_last_refresh = rospy.get_time()
                self.refresh()
                
            rate.sleep()
    
    def refresh(self):
        '''
        This function is used to refresh the task by randomly choosing a goal for each warehouse
        '''
        self.update_lock.acquire()
        if self.finished:
            self.cur_task = [i for i in range(self.num_warehouse)]
        else:
            for i in range(self.num_warehouse):
                self.cur_task[i] = np.random.choice(self.num_warehouse, p=self.warehouse_probability[i])
        self.update_lock.release()
        
    def display_time(self)->Marker:
        '''
        This function is used to display the time count by creating a text marker
        '''
        if self.start_time is not None:
            time_elapsed = rospy.get_time() - self.start_time
            r = 0.0
            g = 1.0
            b = 0.0
            if time_elapsed >= self.time_limit:
                self.finished = True
                time_elapsed = self.time_limit
                r = 1.0
                g = 0.0                
        else:
            time_elapsed = 0
            r = g = b = 1.0
        
        text = f"{int(time_elapsed/60)}:{int(time_elapsed%60):02d}"
        return get_text_msg(text, r=r, g=g, b=b, ns='info', id=0)
        
    def display_income(self)->Marker:
        '''
        This function is used to display the total income by creating a text marker
        '''
        text = f"Total Income: {self.total_income}"
        return get_text_msg(text, x = 3, y= 6.5, scale = 0.3, r=0.2, g=0.8, b=0.2, ns='info', id=1)
        
    def display_warehouse(self)->List[Marker]:
        '''
        This function is used to make visualization of the warehouse
        '''
        marker_list = []
        
        for i in range(self.num_warehouse):
            id = chr(ord('A') + i)
            location = self.warehouse_location[i]
            marker_list.append(get_text_msg(id, x=location[0], y=location[1], scale=0.25, r=1.0, g=1.0, b=1.0, ns='warehouse', id=i))

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = 'warehouse_marker'
            marker.id = i
            
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose.position.x = location[0]
            marker.pose.position.y = location[1]
            marker.pose.position.z = 0.0
            
            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.01  
            marker.lifetime = rospy.Duration(1.0)
            marker.color.r = 0.1
            marker.color.g = 0.1
            marker.color.b = 1
            marker.color.a = 0.5
            
            marker_list.append(marker)
                    
        return marker_list
    
    def display_task(self)-> List[Marker]:
        marker_list = []
        
        x = 7.5
        y = 5.5
        dy = 0.3
        
        marker_list.append(
            get_text_msg('Side Tasks', x=x, y=y+dy, scale=0.25, r=0.8, g=0.1, b=0.1, ns='tasks_header', id=self.num_warehouse))
        for i in range(self.num_warehouse):
            id = chr(ord('A') + i)
            marker_list.append(get_text_msg(id, x=x-0.8, y=y-i*dy, scale=0.25, r=1, g=1, b=1, ns='tasks_start', id=i))
            
            self.update_lock.acquire()
            goal = self.cur_task[i]
            self.update_lock.release()
            
            ref =  self.ref_path[i][goal]
            if ref is None:
                reward = 0
                text = ' Not available'
                marker_list.append(get_text_msg(text, x=x, y=y-i*dy, scale=0.25, r=1.0, g=0, b=0, ns='tasks_goal', id=i))
            else:
                reward = ref[1]
                text = f" -> {chr(ord('A') + goal)}, Fee: {reward}"
                marker_list.append(get_text_msg(text, x=x, y=y-i*dy, scale=0.25, r=1, g=1.0, b=1, ns='tasks_goal', id=i))

        return marker_list
    
    def display_current_task(self)->List[Marker]:
        marker_list = []
        
        x = 7.5
        y = 1.5
        dy = 0.3
        
        if self.cur_student_task == -1:
            return []
        
        marker_list.append(
            get_text_msg('Current Task', x=x, y=y+dy, scale=0.25, r=0.8, g=0.1, b=0.1, ns='current_tasks_header', id=0))
        
        goal = chr(ord('A') + self.cur_student_task)
        
        if self.doing_boss_task:
            text = f'Boss Task: Goal {goal}, Fee {self.cur_student_reward}'
        else:
            text = f'Side Task: Goal {goal}, Fee {self.cur_student_reward}'
        
        marker_list.append(get_text_msg(text, x=x, y=y, scale=0.25, r=1, g=1, b=1, ns='current_tasks', id=0))
        
        
        return marker_list
    
    def display_boss_schedule(self)->List[Marker]:
        marker_list = []
        
        x = 7.5
        y = 3.5
        dy = 0.3
        
        marker_list.append(
            get_text_msg('Boss Schedule', x=x, y=y+dy, scale=0.25, r=0.8, g=0.1, b=0.1, ns='boss_tasks_header', id=self.num_warehouse))
        
        for i, (start, goal, start_time, _) in enumerate(self.boss_task_schedule):
            text = f"{int(start_time/60)}:{int(start_time%60):02d}: {chr(ord('A') + start)} -> {chr(ord('A') + goal)}, Fee: {self.ref_path[start][goal][1]*self.boss_task_discount}"
            if i == self.cur_boss_task_idx:
                marker_list.append(
                    get_text_msg(text, x=x, y=y-i*dy, scale=0.25, r=0.1, g=0.8, b=0.1, ns='boss_tasks', id=i))
            else:
                marker_list.append(
                    get_text_msg(text, x=x, y=y-i*dy, scale=0.25, r=1, g=1, b=1, ns='boss_tasks', id=i))
        return marker_list
    
    def start_callback(self, _):
        self.start_time = rospy.get_time()
        rospy.loginfo("Start the session! Welcome to SwiftHaul Transportation Company!")
        return EmptyResponse()
    
    def request_sidetask_callback(self, _)->TaskResponse:
        
        if self.start_time is None:
            rospy.logwarn("The session has not started yet")
            return TaskResponse(-1, 0)
        
        task  = -1
        reward = 0
        
        # get the current odometry
        self.student_pose_lock.acquire()
        cur_student_pose = self.student_pose
        self.student_pose_lock.release()
                
        # loop through all the warehouses
        for i, location in enumerate(self.warehouse_location):
            if cur_student_pose is None:
                rospy.logwarn('No odometry received for student truck')
                break # sanity check
            
            dis_x = abs(location[0] - cur_student_pose[0])
            dis_y = abs(location[1] - cur_student_pose[1])
            if dis_x <= location[2] and dis_y <= location[3]:
                self.update_lock.acquire()
                task = self.cur_task[i]
                self.update_lock.release()
                if task == i: # task not available
                    rospy.loginfo("Student truck is at warehouse %c, but no side task is available", chr(ord('A') + i))
                    return TaskResponse(-1, 0)
                ref =  self.ref_path[i][task]
                if ref is not None:
                    reward = ref[1]
                rospy.loginfo("Student truck is at warehouse %c, receiving task %d, and reward %f", chr(ord('A') + i), task, reward)
                self.doing_boss_task = False
                self.cur_student_reward = reward
                self.cur_student_task = task
                return TaskResponse(task, reward)

        rospy.logwarn("Student truck is not at any warehouse")
        return TaskResponse(task, reward)
    
    def request_bosstask_callback(self, _)->TaskResponse:
        if self.start_time is None:
            rospy.logwarn("The session has not started yet")
            return TaskResponse(-1, 0)
        
        task  = -1
        reward = 0
        
        # get the current odometry
        self.student_pose_lock.acquire()
        cur_student_pose = self.student_pose
        self.student_pose_lock.release()
        
        warehouse_idx, boss_task, start_time, _ = self.boss_task_schedule[self.cur_boss_task_idx] #(warehouse, target, start_time, path)
        
        t_now = rospy.get_time() - self.start_time
        if t_now > (start_time + 5): # 5 seconds buffer
            rospy.logwarn("Boss task %d has expired", boss_task)
        
        cur_warehouse = self.warehouse_location[warehouse_idx]
        dis_x = abs(cur_warehouse[0] - cur_student_pose[0])
        dis_y = abs(cur_warehouse[1] - cur_student_pose[1])
        if dis_x <= cur_warehouse[2] and dis_y <= cur_warehouse[3]:
            task = boss_task
            ref =  self.ref_path[warehouse_idx][task] 
            if ref is not None:
                reward = ref[1] * self.boss_task_discount # 50% discount
            rospy.loginfo("Student truck is at warehouse %c, receiving task %d, and reward %f", chr(ord('A') +warehouse_idx), task, reward)
            self.cur_student_reward = reward
            self.cur_student_task = task
            self.doing_boss_task = True
            return TaskResponse(task, reward)
        else:
            rospy.logwarn("Student truck is not at warehouse %d", chr(ord('A') +warehouse_idx))
            
        return TaskResponse(task, reward)
    
    def request_bossschedule_callback(self, _)->ScheduleResponse:
        start_warehouse_index = []
        goal_warehouse_index = []
        rewards = []
        schedule = []
        for start, goal, start_time, _ in self.boss_task_schedule:
            start_warehouse_index.append(start)
            goal_warehouse_index.append(goal)
            rewards.append(self.ref_path[start][goal][1]*self.boss_task_discount)
            schedule.append(start_time)
        
        return ScheduleResponse(start_warehouse_index, goal_warehouse_index, rewards, schedule)
        
    def get_reward_callback(self, req):        
        req_task = req.task
        
        if self.start_time is None:
            rospy.logwarn("The session has not started yet")
            return RewardResponse(False, self.total_income)
        
        if req_task != self.cur_student_task:
            rospy.logwarn(f"Request task {req_task} does not match the previosly assigned task {self.cur_student_task}")
            return RewardResponse(False, self.total_income)
        
        # get the current odometry
        self.student_pose_lock.acquire()
        cur_student_pose = self.student_pose
        self.student_pose_lock.release()
        
        target_location = self.warehouse_location[req_task]
        # dis = np.sqrt((cur_student_pose[0] - target_location[0])**2 + (cur_student_pose[1] - target_location[1])**2)
        dis_x = abs(target_location[0] - cur_student_pose[0])
        dis_y = abs(target_location[1] - cur_student_pose[1])
        
        if dis_x <= target_location[2] and dis_y <= target_location[3]:
            self.total_income += self.cur_student_reward
            self.cur_student_reward = 0 # reset the reward
            self.cur_student_task = -1 # reset the task
            rospy.loginfo(f"Current task {req_task} is completed, and the new total income is {self.total_income}")
            return RewardResponse(True, self.total_income)
        else:
            rospy.loginfo(f"Current task {req_task} is not completed yet since you are ({dis_x},{dis_y}) meters away from the warehouse")
            return RewardResponse(False, self.total_income)