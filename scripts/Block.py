#!/usr/bin/env python3

"""
ブロック崩しを表示するノード
"""
### ----------- 宣言 --------------
import rospy
import cv2
import numpy as np
from std_msgs.msg import Int8
from Myfunc import serching
from Myfunc import Check_obj

### -------------------- ノード準備 -----------------------
### ノード名
rospy.init_node("Block")
### パブリッシャ
pub = rospy.Publisher("block", Int8, queue_size = 1)
### データ送信レート: XX[Hz]
rate = rospy.Rate(35)

### 他のノードから受け取った値
Get_value = -1
### サブスクライバのコールバック関数
def sub_call(message):
	global Get_value
	Get_value = message.data
	#rospy.loginfo(Get_value)

### サブスクライバ
sub = rospy.Subscriber("console", Int8, sub_call)
### -------------------------------------------------------


### ======================= ベース画像 =======================
IMG_H = 420
IMG_W = 500
OBJ_img = np.zeros((IMG_H, IMG_W, 3), np.uint8)

### ======================= ボール設定 =======================

### --------- パラメータ ----------
CD = 14	# 半径
vx = 2		# 移動量
vy = 3
cx = 250	# 現在の座標
cy = 250
offset_x = 0  # オブジェクトに埋まったときの脱出用
offset_y = 0

### --------- 探索領域(円・オブジェクトの接触判定) ----------
# 探索領域サイズ(縦=横)
Serching_size = CD*2+1

### マスク画像の作成 (探索領域作成用)
Circle_img = np.zeros((Serching_size, Serching_size, 3))
cv2.circle(Circle_img, (CD,CD), CD, (255,255,255), -1)
Mask = Circle_img / 255

### 探索用画像, (描画した円の上書きにも使用)
Serch_img = OBJ_img[cy-CD : cy+CD+1, cx-CD : cx+CD+1].copy()

### ======================= オブジェクト設置 =======================

### --------- ボール移動可能範囲の描画 ----------
# 画像の縁・壁のスペース
EWS = 20
border = np.array([[EWS, EWS], [IMG_W-EWS, IMG_H-EWS]])                   # ボール移動可能範囲
cv2.rectangle(OBJ_img, tuple(border[0]), tuple(border[1]), (0,255,0), 1)  # 外枠（緑色)

### --------- ボックスの配置(ボールが当たると消える) ----------
# ボックスの数
h_num = 5
w_num = 10
# ボックスの大きさ
h_size = 30
w_size = h_size
# ボックス同士のスペース
h_space = 10
w_space = 10

### ボックス座標のテーブルの作成

# 描画を始める位置の座標
start_w = int( ((IMG_W-EWS*2) - w_size * w_num - w_space * (w_num-1)) / 2 ) + EWS
if start_w <= 0:
        print("Can not be stored in the border.")
start = np.array([border[0,0]+10, start_w])  ## 開始座標

# 座標テーブル
Object_table = np.zeros((h_num, w_num, 2, 2), np.uint16)
current_h, current_w = start[0], start[1]

### ボックスの配置, 座標格納
for h in range(h_num):
    
        ### 四角形の始点座標を進める
        if h!=0:  current_h += (h_size + h_space)
        current_w = start[1]
        
        for w in range(w_num):
            
            ### 四角形の始点座標を進める
            if w!=0:  current_w += (w_size + w_space)
            
            h0, w0 = current_h, current_w        ## 始点
            h1, w1 = h0 + h_size, w0 + w_size   ## 終点
            
            ### 四角形の描画
            cv2.rectangle(OBJ_img, tuple(np.array([w0, h0])), tuple(np.array([w1, h1])), (255,255,0), 1)
            ### 座標格納
            Object_table[h,w,0] = np.array([h0,w0])
            Object_table[h,w,1] = np.array([h1,w1])


### ======================= バーの設定 =======================
### 新しい座標
current_Bx = int(IMG_W/2)
### 過去の座標
past_Bx = current_Bx
### バーのサイズ(半分)
hH = 20
hW = 40
### バーのY座標
Bar_Cy = 350
Bar_y = [Bar_Cy - hH, Bar_Cy + hH]
### バーの移動距離(範囲)
Bar_move_dist = IMG_W - hW*2 - EWS*2 -2
### バーの描画関数
def draw_bar(xy1, xy2):
	cv2.rectangle(OBJ_img, tuple(xy1), tuple(xy2), (0,0,255), 1)
draw_bar([current_Bx-hW, Bar_y[0]], [current_Bx+hW, Bar_y[1]])
### １フレームでバーが動ける距離
bar_move_ = 10

### ======================= 描画処理 =======================
# ウィンドウ作成
cv2.namedWindow("BALL_IMG")

# ボックスを消す処理のフラグ
rm_obj_flag = 0
# 探索領域
RangeY1, RangeY2, RangeX1, RangeX2 = cy-CD, cy+CD+1, cx-CD, cx+CD+1

# --- エッジの条件分岐（角度）---
A1 = np.round(40 /180*np.pi, 3)
A2 = np.round(50 /180*np.pi, 3)
A3 = np.round(90 /180*np.pi, 3)

# 消したブロックの数
count_blocks = 0
# ブロックの数
blocks_num = h_num * w_num


### 開始待機...相手のノードからの値が受け取れない or 操作されていない
cv2.imshow("BALL_IMG", OBJ_img)

while Get_value == -1 or Get_value == 50:
	if cv2.waitKey(500) & 0xFF == ord('q'):
		cv2.destroyAllWindows()
		exit()

###--------送信するメッセージの意味-------
"""
0~:		崩したブロックの数
-1:		ゲームリセットの合図
-2:		ゲームクリアの合図
"""

### パブリッシャが生きているとき
while not rospy.is_shutdown():
        
        ### ============ 処理の終了 =============
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or Get_value == 101:	# 終了の合図を受け取る
            break
        elif count_blocks >= blocks_num:	# すべてのブロックを崩した
        	print("Game_Cleared")
        	while Get_value != 101:
        		pub.publish(-2)				# ゲーム終了の合図を送信
        		cv2.waitKey(100)
        	break
        elif Get_value == -1:				# ポーズ	
        	while Get_value != 101:
        		cv2.waitKey(100)
        	break
        
        ### ============ 円に接触するオブジェクト探索 =============
        
        ### 前回描画した円を消す (円が描画されていない画像で上書き）
        Dell_img = Serch_img.copy(); Dell_img[:,:,2] = 0	## 赤の画素（バー）を消す
        OBJ_img[RangeY1: RangeY2, RangeX1: RangeX2] = Dell_img
        
        ### ボックスを消す
        if rm_obj_flag == 1:
        	if OBJ_img[Object_table[idx_h,idx_w,0, 0], Object_table[idx_h,idx_w, 0, 1], 0] > 0:	# 画素が入っているとき
        		cv2.rectangle(OBJ_img, tuple(Object_table[idx_h,idx_w,0,::-1]), tuple(Object_table[idx_h,idx_w,1,::-1]), (0,0,0), 1)
        		rm_obj_flag = 0				# ブロック接触時のフラグ
        		count_blocks += 1
        		### データをパブリッシュ
        		pub.publish(count_blocks)
        		
        ### マスク処理
        Serch_img = Serch_img * Mask
        
        ### 探索関数
        ret, info = serching(Serch_img, Serching_size)
        
        ### ============ 接触に対する処理 =============
        
        ### ------------------------ ボールの移動方向変更 ------------------------
        if ret==1:                        ### コーナー ( オフセットなし )
            vx *= -1; vy *= -1
            
        elif ret==2:
            if info[0][0] == "C":         ### コーナー ( オフセットあり )
                vx *= -1; vy *= -1
                
                if info[0][1] == "1":
                    if info[1][1] != 0:
                        offset_x = np.sign(vx) * (2*CD - info[1][1]); offset_y = np.sign(vy) * info[1][0]
                elif info[0][1] == "2":
                    if info[1][1] != 0:
                        offset_x = np.sign(vx) * info[1][1];          offset_y = np.sign(vy) * info[1][0]
                elif info[0][1] == "3":
                    if info[1][0] != 0:
                        offset_x = np.sign(vx) * info[1][1];          offset_y = np.sign(vy) * (2*CD - info[1][0])
                elif info[0][1] == "4":
                    if info[1][0] != 0:
                        offset_x = np.sign(vx) * (2*CD - info[1][1]); offset_y = np.sign(vy) * (2*CD - info[1][0])
                
                #=========================================================
                touch = np.array( [cy-CD+info[1][0], cx-CD+info[1][1]] )   	## 円とオブジェクトが接触した座標  (h, w) = (y,x)
                #=========================================================
                    
            elif info[0][0] == "L":       ### 直線
                
                if info[0][1] == "R" and vx>0:
                    vx *= -1
                    if info[1][1] != 0:
                        offset_x = np.sign(vx) * (2*CD - info[1][1])
                    #=========================================================
                    touch = np.array( [cy, cx-CD+info[1][1]] )
                    #=========================================================
                
                elif info[0][1] == "L" and vx<0:
                    vx *= -1
                    if info[1][1] != 0:
                        offset_x = np.sign(vx) * info[1][1]
                    #=========================================================
                    touch = np.array( [cy, cx-CD+info[1][1]] )
                    #=========================================================
                
                elif info[0][1] == "A":
                    vy *= -1
                    if info[1][0] != 0:
                        offset_y = np.sign(vy) * info[1][0]
                    #=========================================================
                    touch = np.array( [cy-CD+info[1][0], cx] )
                    #=========================================================
                
                elif info[0][1] == "U":
                    vy *= -1
                    if info[1][0] != 0:
                        offset_y = np.sign(vy) * (2*CD - info[1][0])
                    #=========================================================
                    touch = np.array( [cy-CD+info[1][0], cx] )
                    #=========================================================
                
            
            elif info[0][0] == "E":       ### エッジ
                
                if info[0][1] == "1":
                    x = info[1][1] - CD; y = CD - info[1][0]

                elif info[0][1] == "2":
                    x = CD - info[1][1]; y = CD - info[1][0]

                elif info[0][1] == "3":
                    x = CD - info[1][1]; y = info[1][0] - CD

                elif info[0][1] == "4":
                    x = info[1][1] - CD; y = info[1][0] - CD

                ### オフセット計算
                if x<0.001: x = 0.001
                angle = np.arctan(y/x)  # [rad]
                off_x, off_y = np.round( abs(CD*np.cos(angle) - x)), np.round( abs(CD*np.sin(angle) - y))
                
                ### 跳ね返る方向を決定
                if angle < A1:
                    vx *= -1
                    offset_x = int(np.sign(vx) * off_x)
                    #=========================================================
                    touch = np.array( [cy-CD+info[1][0], cx-CD+info[1][1]] )
                    #=========================================================
                
                elif angle < A2:
                    if (info[0][1]=="1" and vx>0 and vy<0) or (info[0][1]=="2" and vx<0 and vy<0)\
                                                or (info[0][1]=="3" and vx<0 and vy>0) or (info[0][1]=="4" and vx>0 and vy>0):
                        vx *= -1; vy *= -1
                        offset_x, offset_y = int(np.sign(vx)*off_x), int(np.sign(vy)*off_y)
                        #=========================================================
                        touch = np.array( [cy-CD+info[1][0], cx-CD+info[1][1]] )
                        #=========================================================
                
                elif angle < A3:
                    vy *= -1
                    offset_y = int(np.sign(vy) * off_y)
                    #=========================================================
                    touch = np.array( [cy-CD+info[1][0], cx-CD+info[1][1]] )
                    #=========================================================
                
                else:		##  例外
                    vx*=-1; vy*=-1
                    #=========================================================
                    touch = np.array( [cy-CD+info[1][0], cx-CD+info[1][1]] )
                    #=========================================================
        
            ### ----------------- 円とボックスの接触判定 ----------------- (ret == 2)
            if info[2] == 0 and info[0]!="D":
            	idx_h, idx_w = Check_obj(touch, Object_table, h_num, w_num)
            	rm_obj_flag = 1
        
        ### ========== 円の移動 ===========
        cx = cx + vx + offset_x
        cy = cy + vy + offset_y
        offset_x = 0; offset_y = 0
        
        ### ========== バーの移動 ===========
        
        ### 新しい操作量
        current_Bx = int(Bar_move_dist * Get_value / 100) + hW + EWS+1
        diff = current_Bx - past_Bx
        if diff > bar_move_:
        	current_Bx = past_Bx + bar_move_
        if diff < -1*bar_move_:
        	current_Bx = past_Bx - bar_move_
        
        ### ボールがバーと同じ高さにある... バーに円が埋まるのを防ぐ
        if ((cy+CD) > Bar_y[0]+10 and (cy-CD) < Bar_y[1]-10):
        	
        	### 円をどかす
        	# どかすためのスペースがあるか
        	if (EWS+bar_move_+hW < current_Bx < IMG_H-EWS-bar_move_-hW):
        		if cx>current_Bx:
        			bar_edge = current_Bx + hW; circle_edge = cx - CD
        			if bar_edge > circle_edge:
        				cx += (bar_edge - circle_edge)
        		else:
        			bar_edge = current_Bx - hW; circle_edge = cx + CD
        			if bar_edge < circle_edge:
        				cx -= (circle_edge - bar_edge)
        	else:
        		current_Bx = past_Bx		## バーを動かさない（スペースなし）
        
        ### 前回のバーを消す
        cv2.rectangle(OBJ_img, tuple([past_Bx-hW, Bar_y[0]]), tuple([past_Bx+hW, Bar_y[1]]), (0,0,0), 1)
        ### 新しいバー
        draw_bar([current_Bx-hW, Bar_y[0]], [current_Bx+hW, Bar_y[1]])
        past_Bx = current_Bx
        
        ### ========== 円のリセット ===========
        if cy >= border[1, 1] - CD:
        	# 円のパラメータ初期化
        	vx, vy = 2, 3
        	cx, cy = 250, 250
        	cv2.waitKey(300)
        	cv2.circle(OBJ_img, (cx,cy), CD, (0,255,255), 1)
        	cv2.imshow("BALL_IMG", OBJ_img)
        	# リセットの合図をパブリッシュ
        	pub.publish(-1)
        	cv2.waitKey(1000)
        	cv2.circle(OBJ_img, (cx,cy), CD, (0,0,0), 1)
        
        ### 探索領域の切り取り
        RangeY1, RangeY2, RangeX1, RangeX2 = cy-CD, cy+CD+1, cx-CD, cx+CD+1
        Serch_img = OBJ_img[RangeY1: RangeY2, RangeX1: RangeX2].copy()	## 円が描かれる前の領域
        
        ### 新しい円の描画
        cv2.circle(OBJ_img, (cx,cy), CD, (255,255,255), 1)
        
        
        cv2.imshow("BALL_IMG", OBJ_img)
        
        ### パブリッシャの待機
        rate.sleep()
        

print("block: Normaly finished")
cv2.destroyAllWindows()



