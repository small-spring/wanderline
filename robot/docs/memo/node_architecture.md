
## CanvasNode
Publisher
- `/canvas_state`

- 
- publishしたcanvas_stateは、以下に使用される
    - OpenCVの描画
    - motion計画

Subscriber
- 接触しているかどうか
- ペンのxy座標


## TransformFrameNode
サービス
- ペンのxy座標（カンバス上空間） → ペンのxyz座標（Rviz上空間）
- ペンのxyz座標 → ペンのxy座標 


## ロボットを動かすNode
ロボットの状態を更新するのはここだけ！ロボットの状態を配信するのはここだけ！
publish
- robotの状態（姿勢。xyz空間中のどこにジョイントやエフェクタがあるか）
- robotの状態から計算できるペン先の位置

subscriber
- target plannnerからの次の目標座標

中でやる計算
- target plannnerから次のxy座標trajectoryを受け取る→transformnframenodeにaskしてして3dの目標trajectoryを計算してもらう。→姿勢計算nodeに渡して、ロボットのトルクやjoint角度を得る？ここUR5eのパッケージ仕様とかを見る必要がある→そのようにrobotを動かし、publishする



## 接触判定Node
subscribe
- robotの状態

publish 
- 接触しているかどうか
    - ロボットの状態（姿勢）とカンバスの位置を受け取って、接触判定



## CentralNode
1. 現在のロボットの状態joint_status取得
2. canvas取得
3. motion_plannnerに渡し、motion_planを受け取る
4. motion_planに基づき動作実行action
    1. 実行の1ステップごと？、接触判定、ロボット状態joint_statusを更新
5. 次のステップへ



# memo
## 座標
- xy座標：canvasの絵を書く座標。wanderlineの頭脳部分と共通
- xyz座標：ロボット含めた3d座標
- jointState
    - header: std_msgs.msg.Header型（タイムスタンプやframe_idなど）
    - name: List[str]（ジョイント名のリスト）
    - position: List[float]（各ジョイントの角度や位置）
    - velocity: List[float]（各ジョイントの速度）
    - effort: List[float]（各ジョイントのトルクや力）
```
self.joint_names = [
    'shoulder_pan_joint',
    'shoulder_lift_joint', 
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint'
]
```