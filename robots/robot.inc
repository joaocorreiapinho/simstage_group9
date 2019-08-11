define laser ranger(
	sensor(
		range [0.0 8]
		fov 120.0
		samples 120
	)

	size [0.45 0.3 0.15]

	block(
		points 4
		point[0] [0 0]
		point[1] [0 1.5]
		point[2] [1 1.5]
		point[3] [1 0]
		z [0.2 1.2]
	)
color "red"
)

define corpo actuator(
	size [0.3 0.3 0.3]
	block(
		points 4
		point[0] [0 0]
		point[1] [0.0 1]
		point[2] [1 1]
		point[3] [1 0.0]
		z [0 1]
	)
color "red"
)

define capo actuator(
	size [0.3 0.45 0.15]
	block(
		points 4
		point[0] [0 1]
		point[1] [1 1]
		point[2] [1 1.5]
		point[3] [0 1.5]
		
		z [0 1]
	)
color "red"
)

define roda actuator(
	size [0.08 0.08 0.08]
	block(
		points 7
		point[0] [0 0]
		point[1] [0 1]
		point[2] [1 1]
		point[3] [1 0]
		z [0 1]
	)
color "black"
)

			
	
define robot position(
	odom_error [0.03 0.03 0.00 0.05]
	localization "odom"
	pose [0 0 1 0]
	size [0.01 0.01 0.01]
	gui_nose 1
	drive "diff"
	color "white"
	corpo(pose [0.0 0.0 0.05 -90.0])
	
	roda(pose [-0.15 0.15 0.0 -90.0])
	roda(pose [-0.15 -0.15 0.0 -90.0])
	roda(pose [ 0.15 0.15 0.0 -90.0])
	roda(pose [ 0.15 -0.15 0.0 -90.0])

	laser()
)
