#!/usr/bin/env python
"""
TSIA full pipeline
This script implements the full TSIA pipeline for a robotic arm using MoveIt and Dual Quaternions.
"""

import rospy, csv, argparse, numpy as np
from moveit_commander import RobotCommander, MoveGroupCommander, roscpp_initialize
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from pyquaternion import Quaternion
from dual_quaternions import DualQuaternion
import matplotlib.pyplot as plt
import roboticstoolbox as rtb
from spatialmath import SE3
import matplotlib.pyplot as plt
from threading import Lock
from scipy.spatial.transform import Rotation as R


# This is because that in franka the TCP frame is not aligned with the link8 frame
# and we need to correct the orientation of the pose to align with the link8 frame
tool_transform = SE3(0.1034, 0, 0) * SE3.RPY([-0.10, 2.83, -44.91], unit='deg') 

q_tcp  = R.from_quat([ -0.011767, -0.03009, 0.6962, 0.71712 ])
q_link8 = R.from_quat([ -0.022386, -0.023296, 0.36877, 0.92896 ])
#q_offset = q_link8 * q_tcp.inv()
q_offset = q_tcp * q_link8.inv()
current_point_position = [None]
position_lock = Lock()

def correct_orientation_only(pose: Pose) -> Pose:
   
    q = R.from_quat([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    ])
    q_corrected = q_offset * q
    q_out = q_corrected.as_quat()

    pose_corrected = Pose()
    pose_corrected.position = pose.position 
    pose_corrected.orientation.x = q_out[0]
    pose_corrected.orientation.y = q_out[1]
    pose_corrected.orientation.z = q_out[2]
    pose_corrected.orientation.w = q_out[3]
    return pose_corrected


def dq_from_pose(pose):
    trans = [pose.position.x, pose.position.y, pose.position.z]
    quat  = [pose.orientation.w, pose.orientation.x,
             pose.orientation.y, pose.orientation.z]
    q_r = Quaternion(quat)
    q_d = 0.5 * Quaternion(0, *trans) * q_r
    print(q_r, q_d)
    return DualQuaternion(q_r, q_d)

def compute_fk_from_q_list_rtb(q_list):
    panda = rtb.models.DH.Panda()
    panda.tool = tool_transform
    dqs = []

    for q in q_list:
        T = panda.fkine(q)     # homogeneous transform: SE(3)
        pos = T.t              # translation [x, y, z]
        rot = T.R              # rotation matrix 3x3
        q_r = Quaternion(matrix=rot)  
        q_d = 0.5 * Quaternion(0, *pos) * q_r
        dq = DualQuaternion(q_r, q_d)
        dqs.append(dq)

    return dqs

def find_closest_dq_idx(path_dqs, current_dq):
    dists = [np.linalg.norm(np.array(dq.translation()) - np.array(current_dq.translation())) for dq in path_dqs]
    return np.argmin(dists)

def pose_from_dq(dq):
    t = dq.translation()
    q = dq.q_r
    p = Pose()
    p.position.x, p.position.y, p.position.z = t
    p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z = q.w, q.x, q.y, q.z
    return p

def compute_imitated_path(demo_dqs, new_goal_dq):
    demo_end = demo_dqs[-1]
    path = []
    for dq_i in demo_dqs[:-1]:
        delta = dq_i.inverse() * demo_end
        imi_dq = new_goal_dq * delta.inverse()
        path.append(imi_dq.normalized())
    path.append(new_goal_dq.normalized())
    return path

def sclerp_path(start_dq, end_dq, steps=20):
    return [DualQuaternion.sclerp(start_dq, end_dq, t) for t in np.linspace(0, 1, steps)]

def extract_xy(dqs):
    return np.array([[d.translation()[0], d.translation()[1]] for d in dqs])

def check_and_fix_joint_trajectory_continuity(q_dense, threshold=0.1, fix_steps=10):
    new_q_dense = [q_dense[0]]
    bad_points = []

    for i in range(1, len(q_dense)):
        q_prev = np.array(new_q_dense[-1])
        q_curr = np.array(q_dense[i])
        delta = np.abs(q_curr - q_prev)

        if np.any(delta > threshold):
            print(f"[Warning] Discontinuity at step {i}: Δq = {delta}")
            bad_points.append((i, delta))
            
            for j in range(1, fix_steps + 1):
                alpha = j / (fix_steps + 1)
                q_interp = (1 - alpha) * q_prev + alpha * q_curr
                new_q_dense.append(q_interp.tolist())
        new_q_dense.append(q_curr.tolist())

    if not bad_points:
        print("------------------✅ Trajectory is smooth.-------------------")
    else:
        print(f"-----❌ Found and fixed {len(bad_points)} discontinuities.------")

    return new_q_dense


# -----------------------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser()
    rospy.wait_for_service('/compute_ik')
    ik_service = rospy.ServiceProxy('/compute_ik', GetPositionIK)

    parser.add_argument("--csv", default="joint_trajectory.csv", help="Recorded joint CSV file")
    parser.add_argument("--visualize", action="store_true", help="Show matplotlib plot")
    args = parser.parse_args()

    # ---------------------------0. Initialize MoveIt & ROS -------------------------------
    roscpp_initialize([])
    rospy.init_node("tsia_full_pipeline")
    rospy.loginfo("Waiting for simulated time from /clock...")
    while rospy.Time.now().to_sec() == 0:
        rospy.sleep(0.1)
    rospy.loginfo("Sim time active. Starting TSIA pipeline.")

    group = MoveGroupCommander("panda_arm")
    print("Default end effector link:", group.get_end_effector_link())
    group.set_planning_time(1.5) 
    joint_names = group.get_active_joints()

    # ---------------------------1. Read demo joint states from CSV -------------------------------
    demo_q_list = []
    with open(args.csv) as f:
        reader = csv.DictReader(f)
        for row in reader:
            demo_q_list.append([float(row[n]) for n in joint_names])
            #print(f"Read joint state: {row['time']}, {row[joint_names[0]]}, ...")

    # ---------------------------2. Compute FK from demo joint states ------------------------------
    demo_dqs = compute_fk_from_q_list_rtb(demo_q_list)

    # ---------------------------3. Goal DQ Setup --------------------------------------------------

    #-------------Goal DQ Setup-------------
    '''
    Here we can change the goal (rot * trans)
    '''
    goal_rot = demo_dqs[-1].q_r # copy from last demo dq
    goal_trans = [0.3, 0.5, 0.3]
    goal_dq = DualQuaternion(goal_rot, 0.5 * Quaternion(0, *goal_trans) * goal_rot)

    #-------------Compute Imitated Path-------------
    imitated_dqs = compute_imitated_path(demo_dqs, goal_dq)

    #-------------Start DQ Setup (fix)-------------
    '''
    In case of we want to start from a specific position, not from the real current position'''
    start_rot = Quaternion(axis=[0, 0, 1], angle=0)
    start_trans = [0.4, -0.3, 0.7]
    start_dq = DualQuaternion(start_rot, 0.5 * Quaternion(0, *start_trans) * start_rot)

    #-------------Current DQ Setup-----------------

    joint_state = rospy.wait_for_message("/franka_state_controller/joint_states", JointState)
    current_q = joint_state.position[:7]

    T = rtb.models.DH.Panda().fkine(current_q)
    rot = T.R
    pos = T.t
    q_r = Quaternion(matrix=rot)
    q_d = 0.5 * Quaternion(0, *pos) * q_r
    current_dq = DualQuaternion(q_r, q_d)

    closest_idx = find_closest_dq_idx(imitated_dqs, current_dq)

    # take 1/N of imitated dq
    '''
    This is for shortening the imitated path, when we suppose some specific part of the path is not needed.
    '''
    start_idx = len(imitated_dqs) // 3

    #-------------Blend Path Setup----------------

    blended_dqs = sclerp_path(current_dq, imitated_dqs[start_idx], steps=50)

    #-------------Full Path Setup----------------
    full_dqs = blended_dqs + imitated_dqs[start_idx:]  
    print(f"Full path length: {len(full_dqs)}")
    #print(full_dqs)

    # ------------From DQ to q trajectory--------------
    joint_traj = []
    group.get_end_effector_link() 
    group.set_end_effector_link("panda_hand_tcp")  # Ensure the end effector link is set correctly

    joint_state = rospy.wait_for_message("/franka_state_controller/joint_states", JointState)
    prev_q = list(joint_state.position[:7])
    
    for dq in full_dqs:
        print(f"---IK solving DQ ---: {dq.q_d}, ")
        pose = pose_from_dq(dq)
        robot_state = RobotState()
        robot_state.joint_state.name = joint_names
        robot_state.joint_state.position = prev_q

        req = GetPositionIKRequest()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.pose_stamped.header.frame_id = "panda_link0"
        req.ik_request.pose_stamped.pose = pose
        req.ik_request.ik_link_name = "panda_hand_tcp"
        req.ik_request.robot_state.joint_state.name = joint_names
        req.ik_request.robot_state.joint_state.position = prev_q

        try:
            res = ik_service(req)
            if res.error_code.val == 1:
                q = res.solution.joint_state.position[:7]
                if joint_traj and np.linalg.norm(np.array(q) - np.array(joint_traj[-1])) > 0.5:
                    rospy.logwarn("Large jump detected, skipping this IK solution.")
                    continue
                joint_traj.append(q)
                prev_q = q
            else:
                rospy.logwarn("IK failed for a pose, skipping!")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)


    # interpolate to ensure smoothness
    from roboticstoolbox import jtraj  
    q_dense = []
    qd_dense = []
    for q0, q1 in zip(joint_traj[:-1], joint_traj[1:]):
        seg = jtraj(q0, q1, 80)  
        q_dense.extend(seg.q)
        qd_dense.extend(seg.qd)

    q_dense = check_and_fix_joint_trajectory_continuity(q_dense, threshold=0.05)

    # ---------------------------4. Visualize the paths -----------------------------------
    if args.visualize:

        def extract_xyz_and_z_axis(dqs):
            pos = []
            z_dirs = []
            for dq in dqs:
                t = dq.translation()
                
                R = dq.q_r.rotation_matrix
                z_dir = R[:, 2]  #  Z axes
                pos.append(t)
                z_dirs.append(z_dir)
            return np.array(pos), np.array(z_dirs)
        
        panda = rtb.models.DH.Panda()
        panda.tool = SE3(0, 0, 0.107) 
        real_dq = []  
        for q in q_dense:
            T = panda.fkine(q) 
            pos = T.t
            rot = T.R
            q_r = Quaternion(matrix=rot) 
            q_d = 0.5 * Quaternion(0, *pos) * q_r
            dq = DualQuaternion(q_r, q_d)
            real_dq.append(dq)



        pos_demo, z_demo = extract_xyz_and_z_axis(demo_dqs)
        pos_imi, z_imi = extract_xyz_and_z_axis(imitated_dqs)
        pos_blend, z_blend = extract_xyz_and_z_axis(blended_dqs)
        pos_real, z_real = extract_xyz_and_z_axis(real_dq)

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')

    
        ax.plot(pos_demo[:, 0], pos_demo[:, 1], pos_demo[:, 2], 'g.-', label="Demo")
        ax.plot(pos_imi[:, 0], pos_imi[:, 1], pos_imi[:, 2], 'b.-', label="Imitated")
        ax.plot(pos_blend[:, 0], pos_blend[:, 1], pos_blend[:, 2], 'r.--', label="Blend")
        ax.plot(pos_real[:, 0], pos_real[:, 1], pos_real[:, 2], 'k--', label="RTB FK")

        current_pos_marker, = ax.plot([], [], [], 'ro', label="Current EE")
    

        ax.text(pos_demo[0, 0], pos_demo[0, 1], pos_demo[0, 2], "Start", color='g')
        ax.text(pos_demo[-1, 0], pos_demo[-1, 1], pos_demo[-1, 2], "End", color='g')
        ax.text(pos_imi[0, 0], pos_imi[0, 1], pos_imi[0, 2], "Start", color='b')
        ax.text(pos_imi[-1, 0], pos_imi[-1, 1], pos_imi[-1, 2], "End", color='b')
        ax.text(pos_blend[0, 0], pos_blend[0, 1], pos_blend[0, 2], "Start", color='r')
        ax.text(pos_blend[-1, 0], pos_blend[-1, 1], pos_blend[-1, 2], "End", color='r')
        ax.text(pos_real[0, 0], pos_real[0, 1], pos_real[0, 2], "Start", color='k')
        ax.text(pos_real[-1, 0], pos_real[-1, 1], pos_real[-1, 2], "End", color='k')

    # Arrow settings
        skip = 3
        for i in range(0, len(pos_imi), skip):
            p = pos_imi[i]
            z = z_imi[i]
            ax.quiver(p[0], p[1], p[2], z[0], z[1], z[2], length=0.04, color='b', normalize=True)

        for i in range(0, len(pos_blend), skip):
            p = pos_blend[i]
            z = z_blend[i]
            ax.quiver(p[0], p[1], p[2], z[0], z[1], z[2], length=0.04, color='r', normalize=True)

        for i in range(0, len(pos_demo), skip):
            p = pos_demo[i]
            z = z_demo[i]
            ax.quiver(p[0], p[1], p[2], z[0], z[1], z[2], length=0.04, color='g', normalize=True)

        # for i in range(0, len(pos_real), 50):
        #     p = pos_real[i]
        #     z = z_real[i]
        #     ax.quiver(p[0], p[1], p[2], z[0], z[1], z[2], length=0.03, color='k', normalize=True)    

    
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title("Automatica")
        ax.view_init(elev=15, azim=25)
        ax.legend()
        ax.grid(True)
        plt.tight_layout()
        plt.show(block=False)
        


    curr_q = rospy.wait_for_message("/franka_state_controller/joint_states", JointState).position[:7]
    #q_dense.insert(0, curr_q) 
    transition = jtraj(curr_q, q_dense[0],500).q
    transition_qd = jtraj(curr_q, q_dense[0], 500).qd

    q_dense = list(transition) + q_dense
    qd_dense = list(transition_qd) + qd_dense
    np.savetxt("command_trajectory.csv", np.array(q_dense), delimiter=",")

    print("q_dense saved to command_trajectory.csv")
    print(f"Total initial trajectory points: {len(joint_traj)}")
    print(f"Total interpolated trajectory points: {len(q_dense)}")

    cmd_pub = rospy.Publisher('/joint_velocity_example_controller/command',JointState, queue_size=1)

    rate = rospy.Rate(1000)    # Here change the rate to 1000Hz
    js_msg = JointState()
    js_msg.name = joint_names              


    #----type Enter on keyboard to continue---
    import threading
    def update_marker():
        with position_lock:
            p = current_point_position[0]
        if p is not None:
            current_pos_marker.set_data_3d([p[0]], [p[1]], [p[2]])
            fig.canvas.draw_idle()


    def send_commands():
        print("Sending commands now...")
        count = 0
        panda = rtb.models.DH.Panda()
        panda.tool = SE3(0, 0, 0.107) 

        for q, dq in zip(q_dense, qd_dense):
            percentage = count / len(q_dense) * 100
            print("---Sending Command No.", count, "(", f"{percentage:.2f}", "%)---")
            js_msg.header.stamp = rospy.Time.now()
            js_msg.position = list(q)
            #js_msg.velocity = list(dq)

            cmd_pub.publish(js_msg)

            T = panda.fkine(q)
            p = T.t

            with position_lock:
                current_point_position[0] = p

            rate.sleep()
            count += 1



    print("Ready to send commands. Press Enter to start...")
    input()
    send_thread = threading.Thread(target=send_commands)
    send_thread.start()
    
    while plt.fignum_exists(fig.number) and send_thread.is_alive():
        update_marker()
        plt.pause(0.1)

    send_thread.join()



if __name__ == "__main__":
    main()
