# task2

## 概要
ロボットシステム学2020の課題です。ROSを利用したブロック崩しのゲームです。
ゲームウィンドウ中のバーをマウス入力によって動かすことでゲームが進行できます。

## 動作環境
仮想環境上のUbuntuでの動作を確認しています。
  ホスト：
- OS: windows10
- Virtual box 6.1
  ゲスト：
- OS: Ubuntu 20.04
- ROS: noetic

## 環境構築
```
cd ~/catkin_ws/src
git clone https://github.com/Kenta-Nakajima/task2.git
cd ~/catkin_ws
catkin_make
```

## 実行方法
1. roscoreを立ち上げます
```
roscore &
```

2. マウス入力を受け付けるノードConsole.pyとゲーム画面を表示するノードBlock.pyを立ち上げます。
```
rosrun task2 Console.py
(別の端末)
rosrun task2 Block.py
```

3. ゲームの開始<br>
  ウィンドウConsoleの青色のつまみにカーソルを合わせてドラッグすると、ウィンドウBlockにボールが表示されてゲームが開始します。

## LICENSE
[COPYING](https://github.com/Kenta-Nakajima/task2/blob/main/COPYING)
