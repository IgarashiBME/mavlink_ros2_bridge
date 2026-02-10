# mavlink_ros2_bridge 設計ドキュメント

## 1. 概要

QGroundControl (GCS) と ROS2 を MAVLink v2 プロトコルで接続するブリッジノード。
GCS に対して ArduPilot 互換の Ground Rover（模擬 FCU）として振る舞い、
ミッション（ウェイポイント）受信・ARM/DISARM 制御・パラメータ管理・ジョイスティック入力転送を行う。

```
ROS2 トピック群                          QGroundControl
  /gnss ──────┐                          (GCS)
  /auto_log ──┤                            ▲
  /heading ───┤                            │
              ▼                            │
      ┌──────────────────┐     UDP/MAVLink v2
      │ mavlink_ros2_bridge   │◄───────────────►│
      │     _node        │   port 14551    port 14550
      └──────────────────┘
              │
              ├──► /mav/mission
              ├──► /mav/modes
              ├──► /mav/joystick
              └──► /mav/mission_set_current
```

## 2. ファイル構成

```
mavlink_ros2_bridge/
├── CMakeLists.txt
├── package.xml
├── docs/
│   └── design.md                        ← 本ドキュメント
├── include/
│   ├── mavlink_ros2_bridge/
│   │   ├── udp_socket.hpp               ← UDPSocket クラス定義
│   │   └── mavlink_ros2_bridge_node.hpp      ← MAVLinkBridgeNode クラス定義
│   └── c_library_v2/                    ← MAVLink v2 C ライブラリ (git submodule)
│       └── common/
│           └── mavlink.h
├── src/
│   ├── udp_socket.cpp                   ← UDPSocket 実装
│   └── mavlink_ros2_bridge_node.cpp          ← MAVLinkBridgeNode 実装 + main()
└── reference/
    └── reference.cpp                    ← 旧 ROS1 実装 (参考資料)
```

## 3. ビルド

### 依存パッケージ

| パッケージ | 用途 |
|---|---|
| `rclcpp` | ROS2 C++ クライアントライブラリ |
| `std_msgs` | Float64MultiArray, Int32MultiArray, UInt16 |
| `geometry_msgs` | TwistStamped |

### CMake 構成

- `include/` — プロジェクトヘッダの検索パス
- `include/c_library_v2/` — MAVLink v2 ヘッダの検索パス（`SYSTEM` 指定で警告抑制）
- C++17 / C99
- 実行ファイル: `mavlink_ros2_bridge_node`

### ビルド手順

```bash
cd ~/ros2_ws
colcon build --packages-select mavlink_ros2_bridge
source install/setup.bash
```

## 4. 起動方法

```bash
# localhost の QGC に接続（デフォルト）
ros2 run mavlink_ros2_bridge mavlink_ros2_bridge_node

# 別 PC の QGC に接続
ros2 run mavlink_ros2_bridge mavlink_ros2_bridge_node --ros-args -p gcs_ip:=192.168.11.100
```

## 5. クラス設計

### 5.1 UDPSocket

非同期 UDP 通信を担うクラス。

| メソッド | 説明 |
|---|---|
| `open(local_port, remote_ip, remote_port)` | ソケット作成、bind、非ブロッキング設定、送信先設定 |
| `close()` | ソケット解放 |
| `is_open()` | ソケットが有効か |
| `send(data, length)` | `remote_addr_` へ UDP 送信 |
| `receive(buffer, max_length)` | UDP 受信。送信元アドレスで `remote_addr_` を更新（GCS アドレス自動学習） |

**ポート割当:**
- 受信: 14551（bind）
- 送信先: 14550（GCS）

**GCS アドレス自動学習:**
`receive()` は `recvfrom` の送信元アドレスを `remote_addr_` に書き戻す。
これにより、GCS が最初にパケットを送信した時点で送信先が自動的にその GCS のアドレスに切り替わる。
`gcs_ip` パラメータの初期値に関わらず、GCS 側から接続を開始すれば通信が確立する。

### 5.2 MAVLinkBridgeNode

`rclcpp::Node` を継承するメインノード。

**MAVLink システム情報:**

| フィールド | 値 |
|---|---|
| System ID | 1 |
| Component ID | MAV_COMP_ID_AUTOPILOT1 (1) |
| Vehicle Type | MAV_TYPE_GROUND_ROVER |
| Autopilot | MAV_AUTOPILOT_ARDUPILOTMEGA |

## 6. ROS2 パラメータ

| パラメータ名 | 型 | デフォルト | 説明 |
|---|---|---|---|
| `gcs_ip` | string | `"127.0.0.1"` | GCS の IP アドレス（初期送信先） |
| `mission_total_seq` | int | `0` | ミッション総ウェイポイント数 |
| `Kp` | double | `0.0` | PID 比例ゲイン (QGC公開) |
| `Ki` | double | `0.0` | PID 積分ゲイン (QGC公開) |
| `Kd` | double | `0.0` | PID 微分ゲイン (QGC公開) |
| `look_ahead` | double | `0.0` | 前方注視距離 (QGC公開) |
| `i_control_dist` | double | `0.0` | 積分制御距離 (QGC公開) |
| `i_limit` | double | `0.0` | 積分リミット (QGC公開) |
| `linear_velocity` | double | `0.0` | 目標直進速度 (QGC公開) |

「QGC公開」のパラメータは QGroundControl の Parameters 画面から読み書き可能。
QGC から PARAM_SET を受信すると、MAVLink 応答と同時に ROS2 パラメータも更新される。

## 7. ROS2 トピック

### 7.1 サブスクライブ（センサーデータ入力 → GCS へ送信）

| トピック名 | 型 | QoS | データ配列 |
|---|---|---|---|
| `/gnss` | `std_msgs/Float64MultiArray` | 10 | `[lat_deg, lon_deg, alt_mm, fix_status, num_satellites]` |
| `/auto_log` | `std_msgs/Float64MultiArray` | 1 | `[waypoint_seq, cross_track_error, angular_z]` |
| `/heading` | `std_msgs/Float64MultiArray` | 1 | `[yaw_rad, fix_status]` |

**fix_status の値:**

| 値 | 意味 | GPS_FIX_TYPE / ESTIMATOR_FLAGS |
|---|---|---|
| 0 | DGPS / 未Fix | `GPS_FIX_TYPE_DGPS` / `0` |
| 1 | RTK Float | `GPS_FIX_TYPE_RTK_FLOAT` / `0` |
| 2 | RTK Fixed | `GPS_FIX_TYPE_RTK_FIXED` / `ESTIMATOR_ATTITUDE(1)` |

### 7.2 パブリッシュ（GCS からのコマンド → ROS2 へ出力）

| トピック名 | 型 | QoS | データ内容 |
|---|---|---|---|
| `/mav/mission` | `std_msgs/Float64MultiArray` | 1000 | `[seq, total_seq, command, latitude, longitude]` |
| `/mav/modes` | `std_msgs/Int32MultiArray` | 1 | `[base_mode, custom_mode, mission_start]` |
| `/mav/joystick` | `geometry_msgs/TwistStamped` | 1 | `linear.x=pitch, y=roll, z=throttle, angular.z=yaw` |
| `/mav/mission_set_current` | `std_msgs/UInt16` | 10 | ウェイポイント番号 |

## 8. タイマー

| タイマー | 周期 | 初期状態 | 処理内容 |
|---|---|---|---|
| `heartbeat_timer_` | 900ms | 有効 | テレメトリ一式を GCS に送信 + `/mav/modes` パブリッシュ |
| `receive_timer_` | 10ms | 有効 | UDP 受信ポーリング + PARAM_SET 応答処理 |
| `mission_request_timer_` | 30ms | **停止** | MISSION_REQUEST_INT 送信（ミッションダウンロード中のみ有効） |

## 9. GCS への定期送信メッセージ (900ms 毎)

`onHeartbeatTimer()` で以下を順に送信する。

| # | MAVLink メッセージ | 主な送信内容 |
|---|---|---|
| 1 | HEARTBEAT | type=GROUND_ROVER, autopilot=ARDUPILOTMEGA, base_mode, custom_mode |
| 2 | SYS_STATUS | battery=11V, load=50% (固定値) |
| 3 | LOCAL_POSITION_NED | 位置 (現状ゼロ固定) |
| 4 | ATTITUDE | roll=0, pitch=0, yaw=`/heading` から取得 |
| 5 | GPS_RAW_INT | lat, lon, alt, fix_type, satellites (`/gnss` から取得) |
| 6 | ESTIMATOR_STATUS | flags, angular_z, cross_track_error (`/heading`, `/auto_log` から取得) |
| 7 | MISSION_CURRENT | 現在のウェイポイント番号 (ARM 時のみ) |

## 10. GCS 受信メッセージ処理

### 10.1 対応メッセージ一覧

| MSG ID | メッセージ名 | ハンドラ | 処理概要 |
|---|---|---|---|
| 0 | HEARTBEAT | `handleHeartbeat` | GCS 生存確認 (将来の接続タイムアウト検出用) |
| 11 | SET_MODE | `handleSetMode` | base_mode, custom_mode を更新 (旧プロトコル) |
| 21 | PARAM_REQUEST_LIST | `handleParamRequestList` | 全7パラメータを PARAM_VALUE で送信 |
| 23 | PARAM_SET | `handleParamSet` | パラメータ更新 → PARAM_VALUE 応答 + ROS2パラメータ同期 |
| 41 | MISSION_SET_CURRENT | `handleMissionSetCurrent` | `/mav/mission_set_current` にパブリッシュ |
| 44 | MISSION_COUNT | `handleMissionCount` | ミッションダウンロード開始 |
| 69 | MANUAL_CONTROL | `handleManualControl` | `/mav/joystick` にパブリッシュ |
| 73 | MISSION_ITEM_INT | `handleMissionItemInt` | ウェイポイント受信 → `/mav/mission` にパブリッシュ |
| 76 | COMMAND_LONG | `handleCommandLong` | コマンド処理 (下記参照) |

### 10.2 COMMAND_LONG (MSG ID 76) の対応コマンド

| Command | MAV_CMD | 処理 |
|---|---|---|
| 176 | `DO_SET_MODE` | param1→base_mode (uint8), param2→custom_mode (uint32) に設定 |
| 300 | `MISSION_START` | `mission_start_` = true |
| 400 | `COMPONENT_ARM_DISARM` | param1=1.0→ARM (base_mode=217), param1=0.0→DISARM (base_mode=89) |
| 519 | `REQUEST_PROTOCOL_VERSION` | COMMAND_ACK を返送 |

全コマンドに対して COMMAND_ACK (MAV_RESULT_ACCEPTED) を返す。
受信時に全フィールド (cmd, target_system, target_component, confirmation, param1-7) をログ出力する。

## 11. ミッションダウンロードプロトコル

MAVLink Mission Protocol に基づくウェイポイント転送シーケンス。

```
QGroundControl                           mavlink_ros2_bridge_node
     |                                           |
     |─── MISSION_COUNT (count=N) ──────────────►|
     |                                           | mission_request_timer_ 開始 (30ms)
     |                                           |
     |◄── MISSION_REQUEST_INT (seq=0) ──────────-|
     |─── MISSION_ITEM_INT (seq=0) ────────────►│| → /mav/mission パブリッシュ
     |                                           |
     |◄── MISSION_REQUEST_INT (seq=1) ──────────-|
     |─── MISSION_ITEM_INT (seq=1) ────────────►│| → /mav/mission パブリッシュ
     |                                           |
     |         ... (seq=2 ~ N-2) ...             |
     |                                           |
     |◄── MISSION_REQUEST_INT (seq=N-1) ────────-|
     |─── MISSION_ITEM_INT (seq=N-1) ──────────►│| → /mav/mission パブリッシュ
     |                                           |
     |◄── MISSION_ACK (ACCEPTED) ───────────────-| mission_request_timer_ 停止
```

- 各 MISSION_ITEM_INT はシーケンス番号の重複チェックを行い、期待する seq のみ受理する
- ダウンロード完了時に `mission_total_seq` パラメータを更新する

## 12. ARM/DISARM 状態遷移

```
起動
 │
 ▼
base_mode_=0  custom_mode_=0
 │
 ├─ SET_MODE (msg 11) ────────────► base_mode_, custom_mode_ を GCS 指定値に設定
 │
 ├─ DO_SET_MODE (cmd 176) ────────► base_mode_ = uint8(param1)
 │                                  custom_mode_ = uint32(param2)
 │
 ├─ ARM (cmd 400, param1=1.0) ───► base_mode_ = 217 (ARDUPILOT_GUIDED_ARMED)
 │        │
 │        ├─ MISSION_START (cmd 300) ──► mission_start_ = true
 │        │
 │        ├─ current_seq_ > mission_total_seq_ ──► base_mode_ = 89 (自動 DISARM)
 │        │                                         mission_start_ = false
 │        │
 │        └─ DISARM (cmd 400, param1=0.0) ────────► base_mode_ = 89
 │                                                   mission_start_ = false
 │
 └─ DISARM (cmd 400, param1=0.0) ► base_mode_ = 89
                                    mission_start_ = false
```

`base_mode_` と `custom_mode_` は 900ms 毎に HEARTBEAT で GCS に通知され、
同時に `/mav/modes` トピックで `[base_mode, custom_mode, mission_start]` として ROS2 にパブリッシュされる。

## 13. MAVLink v1 → v2 対応箇所

リファレンス (v1) から以下の変更を適用済み。

| 関数 | v2 で追加されたパラメータ |
|---|---|
| `gps_raw_int_pack` | `alt_ellipsoid`, `h_acc`, `v_acc`, `vel_acc`, `hdg_acc`, `yaw` (6個) |
| `command_ack_pack` | `progress`, `result_param2`, `target_system`, `target_component` (4個) |
| `mission_current_pack` | `total`, `mission_state`, `mission_mode`, `mission_id`, `fence_id`, `rally_points_id` (6個) |
| `sys_status_pack` | `sensors_present_ext`, `sensors_enabled_ext`, `sensors_health_ext` (3個) |
| `mission_ack_pack` | `opaque_id` (1個) |
| `mission_request_int_pack` | `mission_type` (1個) |

追加パラメータはすべて 0 を設定（拡張フィールドを使用しない）。

## 14. 定数

| 定数名 | 値 | 説明 |
|---|---|---|
| `LOCAL_PORT` | 14551 | 受信ポート |
| `REMOTE_PORT` | 14550 | GCS 送信先ポート |
| `ARDUPILOT_GUIDED_ARMED` | 217 | ArduPilot Guided+Armed の base_mode 値 |
| `ARDUPILOT_GUIDED_DISARMED` | 89 | ArduPilot Guided+Disarmed の base_mode 値 |

## 15. GPS デフォルト座標

サブスクライバ未受信時の初期値。

| フィールド | 値 | 備考 |
|---|---|---|
| `gps_lat_` | 35.736805 * 1e7 | 東京・田無 |
| `gps_lon_` | 139.539676 * 1e7 | 東京・田無 |
| `gps_alt_` | 10000 | 10m (mm 単位) |
| `gps_fix_type_` | GPS_FIX_TYPE_RTK_FIXED | |
