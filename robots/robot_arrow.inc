define laser ranger(
	sensor(
		range [0.0 8]
		fov 240.0
		samples 240
	)

	size [0.01 0.01 0.01]

	block(
		points 4
		point[0] [0 0]
		point[1] [0 1]
		point[2] [1 1]
		point[3] [1 0]
		z [4.5 4.8]
	)
color "goldenrod"
)

define wing actuator(
	size [0.8 0.1 0.05]
	block(
		points 6
		point[0] [0 0]
		point[1] [0 0.05]
		point[2] [0.1 0.1]
		point[3] [0.6 0.1]
		point[4] [0.7 0.05]
		point[5] [0.7 0.0]
		z [2.5 2.8]
	)
color "goldenrod"
)

define nose actuator(
	size [0.1 0.15 0.05]
	block(
		points 8
		point[0] [0.2 0]
		point[1] [0.22 0.2]
		point[2] [0.24 0.3]
		point[3] [0.3 0.4]
		point[4] [0.36 0.3]
		point[5] [0.38 0.2]
		point[6] [0.4 0.0]
		point[7] [0.3 0.2]
		z [2.5 2.8]
	)
color "goldenrod"
)

define boom actuator(
	size [0.01 0.4 0.1]
	block(
		points 7
		point[0] [0 0]
		point[1] [0 1]
		point[2] [1 1]
		point[3] [1 0]
		z [2.5 2.8]
	)
color "goldenrod"
)

define tail actuator(
	size [0.4 0.1 0.1]
	block(
		points 6
		point[0] [0 0]
		point[1] [0 0.05]
		point[2] [0.1 0.1]
		point[3] [0.6 0.1]
		point[4] [0.7 0.05]
		point[5] [0.7 0.0]
		z [2.5 2.8]
	)
color "goldenrod"
)


define robot position(
	odom_error [0.03 0.03 0.00 0.05]
	localization "odom"
	pose [0 0 1 0]
	size [0.01 0.01 0.01]
	gui_nose 1
	drive "diff"
	color "white"
	nose(pose [0.1 0.0 0.0 -90.0])
	laser(pose [0.1 0.0 0.0 0.0])
)
