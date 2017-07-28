power_info.pdf: move_map_power.csv move_map_info.csv ../src/plotter/plot_power_info.py
	../src/plotter/plot_power_info.py move_map_power.csv move_map_info.csv power_info.pdf

move_map_power.bag: move_map.bag
	rosbag filter move_map.bag move_map_power.bag 'topic.find("power") != -1 or topic.find("energy") != -1'

move_map_power.csv: move_map_power.bag ../src/plotter/bag_to_power.py
	../src/plotter/bag_to_power.py move_map_power.bag move_map_power.csv

move_map_blobs.bag: move_map.bag
	rosbag filter move_map.bag move_map_blobs.bag 'topic in ["/robot_00/camera_0/blob_list", "/robot_01/camera_0/blob_list", "/tf"]'

move_map_points.csv: move_map_blobs.bag ../src/plotter/bag_to_blobs.py
	../src/plotter/bag_to_blobs.py move_map_blobs.bag move_map_points.csv

move_map_info.csv: move_map_points.csv ../src/plotter/points_to_info.py
	../src/plotter/points_to_info.py move_map_points.csv move_map_info.csv

