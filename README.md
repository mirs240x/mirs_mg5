# mirs_mg5
mirs_mg5の標準的機能を備えたROS 2パッケージ

# Attention!
launchはyamlファイルへのパスを自動捜索するように書いているが環境によっては見つからずにエラーが起きる模様。\
cloneした後、それぞれ絶対パスに書き換えることを推奨。

それぞれのlaunchファイル内の
```
config_file_path = os.path.join(get_package_share_directory('mirs'),'config','various_file_names.yaml')
```
を
```
config_file_path = '絶対パス'
```
に変更。

# installation

```bash
$ cd ~/your_ws/src
$ git clone https://github.com/mirs240x/mirs_mg5.git
$ cd ~/your_ws
$ colcon build --symlink-install
$ source ~/your_ws/install/setup.bash
```
micro-rosのセットアップも必要。

# How to use

## launch
```
$ ros2 launch mirs launch_file_name
```
### mirs.launch.py

- Lidarの起動
- odometry_nodeの起動
- micro_rosの起動
- tf2_ros_nodeの起動
- 初期パラメータの更新（updateの呼び出し）

を行うlaunchファイル。基本これ一つで完結。\

設定項目はesp_portとlidar_portの２つ。それぞれマイコンとlidarのUSBポートの指定。
```
$ ros2 launch mirs mirs.launch.py esp_port:=/dev/ttyUSB0 lidar_port:=/dev/ttyUSB1
```
のようにすれば起動時に設定可能。launch内の初期パラメータをいじれば設定項目に触らなくても呼び出せる。

### mirs_minimam.launch.py

- odometry_nodeの起動
- micro_rosの起動
- 初期パラメータの更新（updateの呼び出し）

を行うファイル。ゲイン調整などLidarが不要な際に最低限の構成で起動ができる。主にデバッグ用。
設定項目はesp_portのみ。内容と使用方法は上記と同様。

## run
```
$ ros2 run mirs name
```

### odometry_publisher
odometry_nodeの起動

### reset
エンコーダーのカウントを0にリセットするサービス

### update
yamlファイルを参照してパラメーターの更新を行う
```
ros2 run mirs update --ros-args --params-file YourFilePath
```
ファイルパスはmirs_mg5/mirs/config/config.yamlまでの絶対パス






