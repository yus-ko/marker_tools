
# marker_tools

## パッケージ概要

Interactive Markerを使用して、rviz2で地図編集・軌道記録・ウェイポイント管理・インタラクティブマーカー操作などを行うためのツール群を提供するパッケージです。グラフ地図データの可視化・編集や、ロボットの軌道・ウェイポイントの記録・編集を支援します。yamlファイルによるマーカー管理に対応しています。

## 対応ROSディストリビューション

| ディストリビューション | 対応状況 |
|----------------------|----------|
| humble               | ○        |
| その他               | ×        |

## インストール方法

1. 必要な依存パッケージをインストールします。
   ```bash
   source /opt/ros/humble/setup.bash
   sudo apt update
   sudo apt install ros-$ROS_DISTRO-navigation2
   ```
   ※ `<distro>` はご利用のROS2ディストリビューション名（例: humble, iron）に置き換えてください。

2. ワークスペースでビルドします。
   ```bash
   mkdir -p ~/colcon_ws/src && cd ~/colcon_ws/src
   git clone https://github.com/yus-ko/potbot_core.git -b humble
   git clone https://github.com/yus-ko/marker_tools.git
   cd ~/colcon_ws
   colcon build --packages-up-to marker_tools
   source install/setup.bash
   ```

## 実行方法

launchファイルを使って各ノードを起動できます。
例：
```bash
ros2 launch marker_tools base_marker.launch.py
```

マーカーデータを```marker_yaml_path```で指定したファイルに保存できますが、デフォルト値ではinstallディレクトリのyamlファイルに保存されることに注意してください。

## launchファイルの説明

- **base_marker.launch.py**
  - 基本的なマーカー管理ノードを起動します。
- **graph_map.launch.py**
  - グラフマップの編集・表示ノードを起動します。
- **trajectory_recoder.launch.py**
  - マーカーの軌跡をパブリッシュするノードを起動します。
- **tree_map.launch.py**
  - ツリーマップの編集・表示ノードを起動します。
- **waypoint_editor.launch.py**
  - ウェイポイントの編集ノードを起動します。

## ROSパラメータ一覧

| パラメータ名                | 説明                           | デフォルト値           |
|----------------------------|--------------------------------|-----------------------|
| frame_id_global             | マーカー座標のソースフレームID  | map |
| marker_yaml_path            | マーカーデータの保存ファイル    | interactive_markers.yaml |
| mesh_resource_files         | マーカーとしてmeshを使用する場合のリソースファイル  | ["file:///opt/ros/$ROS_DISTRO/share/rviz_default_plugins/test_meshes/<br>pr2-base.dae","package://rviz_default_plugins/test_meshes/pr2-base.dae"] |
