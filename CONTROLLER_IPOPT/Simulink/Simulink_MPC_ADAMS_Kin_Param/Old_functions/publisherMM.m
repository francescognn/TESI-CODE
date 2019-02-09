function []= publisherMM(u)

persistent pub msg1 msgpt msgpt2
if isempty(pub)
pub=rospublisher('/ur_driver/joint_speed','trajectory_msgs/JointTrajectory');
msg1=rosmessage(pub);
msg1.JointNames={'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};
msgpt=rosmessage('trajectory_msgs/JointTrajectoryPoint');
msgpt2=rosmessage('trajectory_msgs/JointTrajectoryPoint');
end

msg1.Header.Stamp=rostime('now','system');
msgpt.TimeFromStart=rosduration(0.5);
msgpt2.TimeFromStart=rosduration(0);
msgpt.Velocities=[u(1), u(2), u(3), u(4), u(5), u(6)];
msg1.Points=[msgpt];

send(pub,msg1)

end

