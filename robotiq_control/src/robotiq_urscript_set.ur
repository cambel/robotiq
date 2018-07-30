def myProg():
  # THIS IS A MODIFIED VERSION OR THE ROBOTIQ URCAP  
  #aliases for the gripper variable names
  ACT = 1
  GTO = 2
  ATR = 3
  ARD = 4
  FOR = 5
  SPE = 6
  OBJ = 7
  STA = 8
  FLT = 9
  POS = 10
  PRE = 11
  def rq_init_connection(gripper_sid=9, gripper_socket="1"):
  	socket_open("127.0.0.1",63352, gripper_socket)
  	socket_set_var("SID", gripper_sid,  gripper_socket)
  	ack = socket_read_byte_list(3, gripper_socket)
  end
  ##########################
  # Returns True if list_of_bytes is [3, 'a', 'c', 'k']
  def is_ack(list_of_bytes):
  	if (list_of_bytes[0] != 3):
  		return False
  	end
  	if (list_of_bytes[1] != 97):
  		return False
  	end
  	if (list_of_bytes[2] != 99):
  		return False
  	end
  	if (list_of_bytes[3] != 107):
  		return False
  	end
  	return True
  end
  def is_not_ack(list_of_bytes):
  	if (is_ack(list_of_bytes)):
  		return False
  	else:
  		return True
  	end
  end
  ##################
  def rq_set_var(var_name, var_value, gripper_socket="1"):
  	sync()
  	if (var_name == ACT):
  		socket_set_var("ACT", var_value, gripper_socket)
  	elif (var_name == GTO):
  		socket_set_var("GTO", var_value, gripper_socket)
  	elif (var_name == ATR):
  		socket_set_var("ATR", var_value, gripper_socket)
  	elif (var_name == ARD):
  		socket_set_var("ARD", var_value, gripper_socket)
  	elif (var_name == FOR):
  		socket_set_var("FOR", var_value, gripper_socket)
  	elif (var_name == SPE):
  		socket_set_var("SPE", var_value, gripper_socket)
  	elif (var_name == POS):
  		socket_set_var("POS", var_value, gripper_socket)
  	else:
  	end
  	sync()
  	ack = socket_read_byte_list(3, gripper_socket)
  	sync()
  	while(is_not_ack(ack)):
  		textmsg("rq_set_var : retry", " ...")
  		textmsg("rq_set_var : var_name = ", var_name)
  		textmsg("rq_set_var : var_value = ", var_value)
  		if (ack[0] != 0):
  			textmsg("rq_set_var : invalid ack value = ", ack)
  		end
  		socket_set_var(var_name , var_value,gripper_socket)
  		sync()
  		ack = socket_read_byte_list(3, gripper_socket)
  		sync()
  	end
  end
  #################################
  def rq_set_force(force, gripper_socket="1"):
  	rq_set_var(FOR,force, gripper_socket)
  end
  
  def rq_set_speed(speed, gripper_socket="1"):
  	rq_set_var(SPE,speed, gripper_socket)
  end
  #################################
  # The main content should be inserted manually here
  rq_init_connection()
  rq_set_force()
  rq_set_speed()
end
