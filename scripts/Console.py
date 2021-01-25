#!/usr/bin/env python3

"""
マウス入力を受け付けるノード
"""

### ----------- 宣言 --------------
import rospy
import cv2
import numpy as np
from std_msgs.msg import Int8

### ------------------ ノード準備 ---------------------
# ノード名
rospy.init_node("Console")
# パブリッシャ
pub = rospy.Publisher("console", Int8, queue_size = 1)
# データ送信レート: XX[Hz]
rate = rospy.Rate(30)
### ----------------------------------------------------


### ----------------------- コンソール準備 ----------------------------
#=======================
# コンソール画像サイズ
img_h = 400
img_w = 400
#=======================

### ================= マウス入力の準備 =================
### マウス移動時のフラグ
push_flag = False
### カーソルの座標
current_x, current_y = 0, 0
### 過去の座標
past_x, past_y = int(img_w/2), int(img_h/2)

### ---------- マウス操作時のコールバック関数 ----------
# マウスイベントとカーソル座標を返す
def callback(event, x, y, flags, param):

        global push_flag, current_x, current_y
        
        if event == cv2.EVENT_LBUTTONDOWN:   ### 左ボダン＿押し下げ
                push_flag = True
                current_x, current_y = x, y
        
        elif event == cv2.EVENT_MOUSEMOVE:   ### マウスの移動
                current_x, current_y = x, y
                
        elif event == cv2.EVENT_LBUTTONUP:   ### 左ボダン＿離す
                push_flag = False
                current_x, current_y = x, y
### ----------------------------------------------------

### ================= ウィンドウの準備 =================
Con_img = np.zeros((img_h,img_w,3), np.uint8)	## 画像作成
cv2.namedWindow("Console")			## ウィンドウ作成
cv2.setMouseCallback("Console", callback)	## マウスとウィンドウの紐づけ
mouse_range = [img_h-150, img_h-20]	# マウス入力受付範囲（高さ方向）= つまみの大きさ

###---------- 操作用つまみ -----------
tab_x = 30						# つまみのX方向サイズ
tab_y = [mouse_range[0]+10, mouse_range[1]-10]	# つまみのY座標(固定)
### 操作用つまみ描画
cv2.rectangle(Con_img, tuple([past_x-tab_x, tab_y[0]]), tuple([past_x+tab_x, tab_y[1]]), (255,255,0), 1)

tab_x_range = [0+tab_x+70, img_w-tab_x-70]		# つまみ中心の移動範囲（横方向）
tab_range_length = tab_x_range[1] - tab_x_range[0]	# 範囲の大きさ
cv2.rectangle(Con_img, tuple([tab_x_range[0]-tab_x-2, mouse_range[0]]), tuple([tab_x_range[1]+tab_x+2, mouse_range[1]]), (255,255,255), 1)	# 移動範囲の描画

###---------- 情報表示エリア（文字） -----------
### 情報を表示する枠を作成
info_box1 = [[40, 20], [img_w-40, img_h-int(img_h/2.5)]]
info_box2 = [info_box1[0], [info_box1[1][0], int(info_box1[0][1]+info_box1[1][1]/2)-10]]
cv2.rectangle(Con_img, tuple(info_box1[0]), tuple(info_box1[1]), (255,255,160), 1)
cv2.rectangle(Con_img, tuple(info_box2[0]), tuple(info_box2[1]), (255,255,160), 1)

### 文字の準備
def put_text(string, location, size, mode=True):
	if mode==True:
		cv2.putText(Con_img, string, tuple(location), cv2.FONT_HERSHEY_SIMPLEX, size, (255,255,255), 1, cv2.LINE_AA)
	else:
		cv2.putText(Con_img, string, tuple(location), cv2.FONT_HERSHEY_SIMPLEX, size, (0,0,0), 2, cv2.LINE_AA)

str1_loc = [info_box1[0][0]+2, info_box1[0][1]+24]                ## 文字を描く場所
str2_loc = [info_box2[0][0]+2, info_box2[0][1]+info_box2[1][1]]
put_text('Remaining number of times', str1_loc, 0.7)
put_text('Scores', str2_loc, 0.7)

R_game = 3	# 残りゲーム回数
B_count = 0	# 崩したブロックカウント
Clear_flag = 0	# ゲームクリアのフラグ

## 枠の中心に来るようにする
str1_loc = [int((info_box2[1][0]-info_box2[0][0])/2+info_box2[0][0])-30, int((info_box2[1][1]-info_box2[0][1])/2+info_box2[0][1])+20]
str2_loc = [int((info_box2[1][0]-info_box2[0][0])/2+info_box2[0][0])-30, int((info_box2[1][1]-info_box1[1][1])/2+info_box1[1][1])+20]
put_text(f"{R_game}", str1_loc, 0.9)
put_text(f"{B_count}", str2_loc, 0.9)

###--------------サブスクライバ作成----------------
### サブスクライバのコールバック関数
def sub_call(message):
	global R_game, B_count, Clear_flag
	Get_value = message.data	# 受け取った値
	if Get_value >= 0:
		put_text(f"{B_count}", str2_loc, 0.9, False)
		B_count = Get_value
		put_text(f"{B_count}", str2_loc, 0.9)
	elif Get_value==-1:
		put_text(f"{R_game}", str1_loc, 0.9, False)
		R_game += Get_value
		put_text(f"{R_game}", str1_loc, 0.9)
	elif Get_value==-2:
		Clear_flag = 1	# ゲームクリアの合図

### サブスクライバ
sub = rospy.Subscriber("block", Int8, sub_call)
### ----------------------------------------------


### パブリッシュする値
Rate_tab = 50  # 最初は中心にある

###--------送信するメッセージの意味-------
"""
0~100:	バーの位置
101:	終了信号
-1:		ポーズの信号
"""

### パブリッシャが生きているときに繰り返す
while not rospy.is_shutdown():

		cv2.imshow("Console", Con_img)
		
		### ============ 処理の終了 =============
		key = cv2.waitKey(1) & 0xFF
		if key == ord('q'):
			pub.publish(101)
			break
		elif R_game == 0:		# ゲームオーバー
			pub.publish(-1)
			cv2.waitKey(2000)
			put_text(f"{R_game}", str1_loc, 0.9, False)
			put_text("Game over", [str1_loc[0]-50,str1_loc[1]], 0.9)
			cv2.imshow("Console", Con_img)
			while True:
				if cv2.waitKey(200) & 0xFF == ord('q'):
					pub.publish(101)
					break
			break
		elif Clear_flag == 1:		# ゲームクリア
			cv2.waitKey(1500)
			put_text(f"{B_count}", str2_loc, 0.9, False)
			put_text("Game clear", [str2_loc[0]-50, str2_loc[1]], 0.9)
			cv2.imshow("Console", Con_img)
			while True:
				if cv2.waitKey(200) & 0xFF == ord('q'):
					pub.publish(101)
					break
			break
        
		### マウス入力がある
		if push_flag == True:
				
				### マウス入力受付範囲か？
				if mouse_range[0]<=current_y<=mouse_range[1]:
				        
				        ### つまみ移動範囲外のとき
				        if current_x < tab_x_range[0]:
				                current_x = tab_x_range[0]
				        elif tab_x_range[1] < current_x:
				                current_x = tab_x_range[1]
				        
				        ### 前回のつまみを消す
				        cv2.rectangle(Con_img, tuple([past_x-tab_x, tab_y[0]]), tuple([past_x+tab_x, tab_y[1]]), (0,0,0), 1)
				        ### 新しいつまみを描く
				        cv2.rectangle(Con_img, tuple([current_x-tab_x, tab_y[0]]), tuple([current_x+tab_x, tab_y[1]]), (255,255,0), 1)

				        past_x, past_y = current_x, current_y
				        
				        Rate_tab = int( (current_x - tab_x_range[0]) * 100 / tab_range_length )	## 比率

		### パブリッシュ
		pub.publish(Rate_tab)

		### パブリッシャの待機
		rate.sleep()

print("concole: Normaly finished")
cv2.destroyAllWindows()



