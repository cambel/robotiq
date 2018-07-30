def myProg():
	# This example script moves to the joint angles below at full speed.
    textmsg("start")
    q = [87, -89, 50, -58, -90, 7]
    movej([d2r(q[0]), d2r(q[1]), d2r(q[2]), d2r(q[3]), d2r(q[4]), d2r(q[5])])
    textmsg("end")
end
