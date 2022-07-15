current_time=$(date "+%Y%m%d%H%M%S")
camera_save_path="./data/${current_time}.mkv"
lidar_save_path="./data/${current_time}.pcap"
echo $current_time
echo $camera_save_path
echo $lidar_save_path


echo 1127 | sudo -S sudo tcpdump -i enx108286149028 -w $lidar_save_path & ffmpeg -f v4l2 -video_size 1920x1024 -input_format rawvideo -i /dev/video4 -framerate 30 $camera_save_path 

#(trap 'kill 0' SIGINT; ffmpeg -f v4l2 -video_size 1920x1024 -input_format rawvideo -i /dev/video4 -framerate 30 $camera_save_path -movflags frag_keyframe+empty_moov & sudo tcpdump -i enx108286149028 -w $lidar_save_path)
