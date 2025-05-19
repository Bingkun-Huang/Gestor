#!/usr/bin/env python
"""
TSIA full pipeline

1. 读取关节 CSV : time, panda_joint1 … panda_joint7
2. 用 MoveIt 做 FK  → Dual Quaternion 列表 (demo_dqs)
3. 生成模仿轨迹 + ScLERP 过渡轨迹
4. 对每个目标 Pose 做 IK  → 关节轨迹 joint_list
5. 发布 trajectory_msgs/JointTrajectory 给 Panda 控制器
6. (可选) matplotlib 可视化三条路径
"""

import rospy, csv, argparse, numpy as np
from moveit_commander import RobotCommander, MoveGroupCommander, roscpp_initialize
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from pyquaternion import Quaternion
from dual_quaternions import DualQuaternion
import matplotlib.pyplot as plt
import roboticstoolbox as rtb
from spatialmath import SO3

# ---------- 工具函数 ----------

def dq_from_pose(pose):
    trans = [pose.position.x, pose.position.y, pose.position.z]
    quat  = [pose.orientation.w, pose.orientation.x,
             pose.orientation.y, pose.orientation.z]
    q_r = Quaternion(quat)
    q_d = 0.5 * Quaternion(0, *trans) * q_r
    print(q_r, q_d)
    return DualQuaternion(q_r, q_d)

def compute_fk_from_q_list_rtb(q_list):
    """
    使用 roboticstoolbox (RTB) 对给定关节角 q_list 批量执行正向运动学
    返回对应的 DualQuaternion 序列
    """
    panda = rtb.models.DH.Panda()
    dqs = []

    for q in q_list:
        T = panda.fkine(q)     # homogeneous transform: SE(3)
        pos = T.t              # translation [x, y, z]
        #print(f"FK: -> {pos}")
        rot = T.R              # rotation matrix 3x3
        #quat = SO3(rot).q  # [w, x, y, z]
        #quat = tr2quat(rot)    # [w, x, y, z]
        q_r = Quaternion(matrix=rot)  # 直接从旋转矩阵构造四元数
        #print(f"FK: -> {q_r}")
        #q_r = Quaternion(quat)  # rotation
        q_d = 0.5 * Quaternion(0, *pos) * q_r
        dq = DualQuaternion(q_r, q_d)
        dqs.append(dq)
        #print(f"FK: {q} -> {dq}")

    return dqs


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

# ---------- 主流程 ----------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", default="joint_trajectory.csv", help="Recorded joint CSV file")
    parser.add_argument("--visualize", action="store_true", help="Show matplotlib plot")
    args = parser.parse_args()

    # === 0. 初始化 MoveIt & ROS ===
    roscpp_initialize([])
    rospy.init_node("tsia_full_pipeline")
    rospy.loginfo("Waiting for simulated time from /clock...")
    while rospy.Time.now().to_sec() == 0:
        rospy.sleep(0.1)
    rospy.loginfo("Sim time active. Starting TSIA pipeline.")

    group = MoveGroupCommander("panda_arm")
    group.set_planning_time(1.5)  # 默认只有 0.5s，可改为 2~3s
    joint_names = group.get_active_joints()
    pub = rospy.Publisher('/position_trajectory_controller/command',
                          JointTrajectory, queue_size=10)

    # === 1. 读取关节 csv ===
    demo_q_list = []
    with open(args.csv) as f:
        reader = csv.DictReader(f)
        for row in reader:
            demo_q_list.append([float(row[n]) for n in joint_names])
            #print(f"Read joint state: {row['time']}, {row[joint_names[0]]}, ...")

    # === 2. FK → Dual Quaternion demo_dqs ===
    demo_dqs = compute_fk_from_q_list_rtb(demo_q_list)

    # === 3. 构造新目标 DQ, 生成模仿 + 插值轨迹 ===
    goal_rot = Quaternion(axis=[0, 0, 1], angle=np.pi/2)
    goal_trans = [1.0, 0.5, 0.5]
    goal_dq = DualQuaternion(goal_rot, 0.5 * Quaternion(0, *goal_trans) * goal_rot)

    imitated_dqs = compute_imitated_path(demo_dqs, goal_dq)

    current_dq = demo_dqs[0]   # 也可用实时当前姿态
    blended_dqs = sclerp_path(current_dq, imitated_dqs[0], steps=20)

    full_dqs = blended_dqs + imitated_dqs   # 执行整条路径
    #print(f"Full path length: {len(full_dqs)}")
    #print(full_dqs)

    # === 4. 逐帧 IK  → joint_list ===
    joint_traj = []
    for dq in full_dqs:
        pose = pose_from_dq(dq)
        group.set_pose_target(pose)
        success, plan, _, _ = group.plan()
        if success:
            joint_traj.append(plan.joint_trajectory.points[-1].positions)
        else:
            rospy.logwarn("IK failed for a pose, skipping!")

    # === 5. 发布 JointTrajectory ===
    traj_msg = JointTrajectory()
    traj_msg.joint_names = joint_names
    time_from_start = 0.0
    for q in joint_traj:
        pt = JointTrajectoryPoint()
        pt.positions = q
        pt.time_from_start = rospy.Duration(time_from_start)
        traj_msg.points.append(pt)
        time_from_start += 0.2
    rospy.sleep(1.0)  # 等待 publisher 连接
    pub.publish(traj_msg)
    rospy.loginfo("Trajectory sent with %d points.", len(joint_traj))

    # === 6. 可视化 (可选) ===
    if args.visualize:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D

        def extract_xyz_and_z_axis(dqs):
            pos = []
            z_dirs = []
            for dq in dqs:
                t = dq.translation()
                # 提取末端姿态的 z 轴方向（末端朝向）
                R = dq.q_r.rotation_matrix
                z_dir = R[:, 2]  # 第三列是 Z 轴方向
                pos.append(t)
                z_dirs.append(z_dir)
            return np.array(pos), np.array(z_dirs)

    # 提取路径和方向
        pos_demo, z_demo = extract_xyz_and_z_axis(demo_dqs)
        pos_imi, z_imi = extract_xyz_and_z_axis(imitated_dqs)
        pos_blend, z_blend = extract_xyz_and_z_axis(blended_dqs)

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')

    # 路径线
        ax.plot(pos_demo[:, 0], pos_demo[:, 1], pos_demo[:, 2], 'g.-', label="Demo")
        ax.plot(pos_imi[:, 0], pos_imi[:, 1], pos_imi[:, 2], 'b.-', label="Imitated")
        ax.plot(pos_blend[:, 0], pos_blend[:, 1], pos_blend[:, 2], 'r.--', label="Blend")

    # 姿态方向箭头（每隔几个画一个）
        skip = 3
        for i in range(0, len(pos_imi), skip):
            p = pos_imi[i]
            z = z_imi[i]
            ax.quiver(p[0], p[1], p[2], z[0], z[1], z[2], length=0.05, color='b', normalize=True)

        for i in range(0, len(pos_blend), skip):
            p = pos_blend[i]
            z = z_blend[i]
            ax.quiver(p[0], p[1], p[2], z[0], z[1], z[2], length=0.05, color='r', normalize=True)

        for i in range(0, len(pos_demo), skip):
            p = pos_demo[i]
            z = z_demo[i]
            ax.quiver(p[0], p[1], p[2], z[0], z[1], z[2], length=0.05, color='g', normalize=True)

    # 轴设置
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title("TSIA 3D Trajectory with End-Effector Orientation")
        ax.legend()
        ax.grid(True)
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
