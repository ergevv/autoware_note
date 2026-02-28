note of autoware!
autoware_data：链接: https://pan.baidu.com/s/11ky1pT5gm8khG1TvZk2cOA?pwd=9wxm 提取码: 9wxm

编译：colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --parallel-workers 2

添加ros2环境变量：
source /home/developer/autoware/autoware_devel/install/setup.bash

echo 'source /home/developer/autoware/autoware_devel/install/setup.bash' >> ~/.bashrc
当前用户生效
nano ~/.bashrc

echo 'source /home/developer/autoware/autoware_devel/install/setup.bash' | sudo tee -a /etc/bash.bashrc  所有用户生效
sudo nano /etc/bash.bashrc

运行：
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=/home/developer/autoware/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit

挂载文件：
ln -s /home/developer/autoware/autoware_data $HOME/autoware_data

不使用cuda：
  autoware_devel/src/universe/autoware_universe/launch/tier4_perception_launch/launch/traffic_light_recognition/traffic_light.launch.xml
    <arg name="enable_image_decompressor" default="true" description="enable image decompressor"/> 改为false
  autoware_devel/src/universe/autoware_universe/perception/autoware_traffic_light_classifier/config/car_traffic_light_classifier.param.yaml
  classifier_type: 1改为0（存在4个文件，暂不知哪个文件是正确的）