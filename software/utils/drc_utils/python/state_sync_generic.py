import lcm
import time
import drc
import bot_core
import math

try:
    import schunk_driver
    from schunk_driver.lcmt_schunk_status import lcmt_schunk_status
    have_schunk_driver = True
except Exception as e:
    print "Exception ", e, " while importing schunk driver."
    have_schunk_driver = False

class StateAssembler:
    def __init__(self):
        self.lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

        self.joint_state_store = {}

        # subscribe on a ton of channels.
        # TODO(gizatt) this should come from config
        self.utime = 0

        # dangerous to pretend we know state without actually knowing it...
        self.main_pose = bot_core.position_3d_t()
        self.main_pose.translation = bot_core.vector_3d_t()
        self.main_pose.translation.x = -0.17 # 0.0 #HACKY FIXME READ FROM irb140.cfg n = [ 0, 0, .911 ]; = [ -0.17, 0, 0 ]; # -.17 , 0, 0  #
        self.main_pose.translation.y = 0.0
        self.main_pose.translation.z = 0.911
        self.main_pose.rotation = bot_core.quaternion_t()
        # rotate by x axis by -90 degrees
        self.main_pose.rotation.w = 1.0 
        self.main_pose.rotation.x = 0.0
        self.main_pose.rotation.y = 0.0
        self.main_pose.rotation.z = 0.0
        self.main_twist = bot_core.twist_t()
        self.main_twist.linear_velocity = bot_core.vector_3d_t()
        self.main_twist.linear_velocity.x = 0.0
        self.main_twist.linear_velocity.y = 0.0
        self.main_twist.linear_velocity.z = 0.0
        self.main_twist.angular_velocity = bot_core.vector_3d_t()
        self.main_twist.angular_velocity.x = 0.0
        self.main_twist.angular_velocity.y = 0.0
        self.main_twist.angular_velocity.z = 0.0
        self.main_force_torque = bot_core.force_torque_t()
        self.main_force_torque.l_foot_force_z = 0.
        self.main_force_torque.l_foot_torque_x = 0.
        self.main_force_torque.l_foot_torque_y = 0.
        self.main_force_torque.r_foot_force_z = 0.
        self.main_force_torque.r_foot_torque_x = 0.
        self.main_force_torque.r_foot_torque_y = 0.
        self.main_force_torque.l_hand_force = [0., 0., 0.]
        self.main_force_torque.l_hand_torque = [0., 0., 0.]
        self.main_force_torque.r_hand_force = [0.,0.,0.]
        self.main_force_torque.r_hand_torque = [0.,0.,0.]

        self.lc.subscribe("ARM_STATE", self.arm_state_handler)
        self.joint_state_store["ARM_STATE"] = {"joint_name" : [], "joint_position" : [], "joint_velocity" : [], "joint_effort" : []}

        if (have_schunk_driver):
            self.lc.subscribe("WSG_STATUS", self.wsg_status_handler)
            self.joint_state_store["WSG_STATUS"] = {"joint_name" : [], "joint_position" : [], "joint_velocity" : [], "joint_effort" : []}


    def arm_state_handler(self,channel,data):
        msgIn = drc.robot_state_t.decode(data)

        self.utime = msgIn.utime
        self.main_pose = msgIn.pose
        self.main_twist = msgIn.twist
        self.main_force_torque = msgIn.main_force_torque

        self.joint_state_store[channel]["joint_name"] = msgIn.joint_name
        self.joint_state_store[channel]["joint_position"] = msgIn.joint_position
        self.joint_state_store[channel]["joint_velocity"] = msgIn.joint_velocity
        self.joint_state_store[channel]["joint_effort"] = msgIn.joint_effort
        self.assembleAndSend()

    def wsg_status_handler(self,channel,data):
        msgIn = schunk_driver.lcmt_schunk_status.decode(data)

        self.joint_state_store[channel]["joint_name"] = ["wsg_50_finger_left_joint", "wsg_50_finger_right_joint"]
        self.joint_state_store[channel]["joint_position"] = [msgIn.actual_position_mm/2./1000., msgIn.actual_position_mm/2./1000.]
        self.joint_state_store[channel]["joint_velocity"] = [msgIn.actual_speed_mm_per_s/2./1000., msgIn.actual_speed_mm_per_s/2./1000.]
        self.joint_state_store[channel]["joint_effort"] = [msgIn.actual_force, msgIn.actual_force]

        print self.joint_state_store[channel]
        self.assembleAndSend()


    def assembleAndSend(self):
        msgOut = bot_core.robot_state_t()

        ### Msg Conversion

        if (self.main_pose):
            msgOut.utime = self.utime
            msgOut.pose = self.main_pose
            msgOut.twist = self.main_twist

            msgOut.num_joints = 0
            for entry in self.joint_state_store.keys():
                msgOut.num_joints += len(self.joint_state_store[entry]["joint_name"])
                msgOut.joint_name += self.joint_state_store[entry]["joint_name"]
                msgOut.joint_position += self.joint_state_store[entry]["joint_position"]
                msgOut.joint_velocity += self.joint_state_store[entry]["joint_velocity"]
                msgOut.joint_effort += self.joint_state_store[entry]["joint_effort"]
            msgOut.force_torque = self.main_force_torque
 
        #Msg Publish
        self.lc.publish("EST_ROBOT_STATE", msgOut.encode())

if __name__ == "__main__":
    sa = StateAssembler()
    print "State Assembler Setup Success.  Running..."
    try:
        while True:
            sa.lc.handle()
    except KeyboardInterrupt:
        print "KeyboardInterrupt detected.  Convertor Terminated"    
