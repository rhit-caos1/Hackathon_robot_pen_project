from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import modern_robotics as mr
# The robot object is what you use to control the robot
robot = InterbotixManipulatorXS("px100", "arm", "gripper")
mode = 'h'
# Let the user select the position
while mode != 'q':
    mode=input("[h]ome, [s]leep, [o]pen gripper, [c]lose gripper, [q]uit ")
    if mode == "h":
        robot.arm.go_to_home_pose()
    elif mode == "s":
        robot.arm.go_to_sleep_pose()
    elif mode =="c":
        robot.gripper.grasp()
    elif mode == "o":
        robot.gripper.release()
    elif mode == "f":
        print(robot.arm.group_info)
        robot.arm.set_single_joint_position("waist",1.2)
        robot.arm.set_single_joint_position("elbow",0.01)
        robot.arm.set_single_joint_position("shoulder",1.2)
    elif mode == "b":
        robot.arm.set_single_joint_position("waist",0.01)
        robot.arm.set_single_joint_position("elbow",-1.2)
        robot.arm.set_single_joint_position("shoulder",0.01)
    elif mode == "w":
        i = float(input("angle(rad):"))
        robot.arm.set_single_joint_position("waist",i)
    elif mode == "e":
        i = float(input("angle(rad):"))
        robot.arm.set_single_joint_position("elbow",i)
    elif mode == "sh":
        i = float(input("angle(rad):"))
        robot.arm.set_single_joint_position("shoulder",i)
    elif mode == "wr":
        i = float(input("angle(rad):"))
        robot.arm.set_single_joint_position("wrist_angle",i)
    elif mode == "t":
        joints = robot.arm.get_joint_commands()
        T = mr.FKinSpace(robot.arm.robot_des.M,robot.arm.robot_des.Slist,joints)
        [R,p]=mr.TransToRp(T)
        print(R,p)
        print(joints)
    elif mode == "a":
        x=float(input("x:"))
        y=float(input("y:"))
        z=float(input("z:"))
        robot.arm.set_ee_cartesian_trajectory(x,y,z)

    elif mode == "p":
        #r=float(input("x:"))
        p=float(input("p:"))
        #y=float(input("z:"))
        robot.arm.set_ee_cartesian_trajectory(0,0,0,0,p,0)
