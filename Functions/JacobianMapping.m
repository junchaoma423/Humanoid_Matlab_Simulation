function tau = JacobianMapping(uin)
x = uin(1:13);
q = uin(14:23);
u = uin(24:35);

R = quat2rotm(x(1:4)');
global Contact_Jacobian
RR=reshape(R,[9,1]);
Jc=Contact_Jacobian(q(1),q(2),q(3),q(4),q(5),q(6),q(7),q(8),q(9),q(10),RR(1),RR(2),RR(3),RR(4),RR(5),RR(6),RR(7),RR(8),RR(9));
contact_mapping=[blkdiag(Jc(1:3,:)',Jc(4:6,:)'),blkdiag(Jc(7:9,:)',Jc(10:12,:)')]; % torque=contact_mapping*u

tau=-contact_mapping*u;

end