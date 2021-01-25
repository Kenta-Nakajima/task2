# task2

## 概要
ROSを利用したブロック崩しのゲームです。<br>
マウス入力でゲーム画面中のバーを操作することでゲームが進行できます。

## 動作環境
仮想環境上のUbuntuでの動作を確認しています。
- ホスト：<br>
  - OS: windows10
  - Virtual box 6.1
- ゲスト：<br>
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

## 動作説明
1. すべてのブロックを崩したとき<br>
  Consoleウィンドウの"Score"の欄に"Game clear"と表示されてゲームが終了します。<br>
  このときに"q"を入力するとゲーム画面のウィンドウ"BALL_IMG"と"Console"ウィンドウを消去できます。
  
2. ボールを落としてしまったとき<br>
  ボールを落とすとConsoleウィンドウの"Remaining number of times"欄の数値が減少します。<br>
  この値が0になるとゲームオーバーです。欄内に"Game over"と表示されてゲームが終了します。

- [動画](https://www.youtube.com/watch?v=Xm3iQ-bipgA&feature=youtu.be)

## LICENSE
[COPYING](https://github.com/Kenta-Nakajima/task2/blob/main/COPYING)
