import numpy as np

"""
Block.pyで使う関数が置いてあるファイル
"""

### オブジェクト探索関数
def serching(Serch_img, Serching_size):
        
        ### =================== 色を確認  ===================
        if Serch_img[:,:,0].any() == True:    ## Blue
                color_idx = 0
        elif Serch_img[:,:,1].any() == True:  ## Green
                color_idx = 1
        elif Serch_img[:,:,2].any() == True:  ## Red
                color_idx = 2
        else:                                  ## 画素なし
                return 0, 0
        
        ## ----- オブジェクトの種類フラグ -----
        Edge   = 0         ## エッジ発見時のフラグ(第 1, 2, 3, 4 象限)
        H_line = 0         ## 縦
        W_line = 0         ## 横線を見つけたフラグ
        Dot    = 0         ## 1点
        Side   = []        ## イメージの縁 ( 1:上、2:左、3:下、5:右 )
        
        ### =================== 探索  ===================
        
        ### ----- 縁にある点の探索 -----
        middle = int((Serching_size-1)/2)    ## 画像の中心
        m2 = middle*2
        #print(m2, Serching_size)
        
        if Serch_img[0, middle, color_idx] > 0:     ## 上
                Side.append(1)
                Serch_img[0, middle, color_idx] = 0
        
        elif Serch_img[m2, middle, color_idx] > 0:  ## 下
                Side.append(3)
                Serch_img[m2, middle, color_idx] = 0
                
        if Serch_img[middle, 0, color_idx] > 0:     ## 左
                Side.append(2)
                Serch_img[middle, 0, color_idx] = 0
                
        elif Serch_img[middle, m2, color_idx] > 0:  ## 右
                Side.append(5)
                Serch_img[middle, m2, color_idx] = 0
        
        if len(Side)>1:
                return 1, 1      ## 省略...( コーナー検出 )
        """if len(Side)>1:                             ## 2つの縁と接触
                Sum = Side[0] + Side[1]
                if Sum == 3:                        ## 第2象限にコーナー   1+2= 3
                    return 1, "C2"
                elif Sum == 5:                      ## 第3象限             2+3= 5
                    return 1, "C3"
                elif Sum == 8:                      ## 第4象限             3+5= 8
                    return 1, "C4"
                else: # 6                           ## 第1象限             5+1= 6
                    return 1, "C1"
        """
        
        ### ----- 画像を1行ずつ探索 -----
        OBJ    = np.array([0,0])       ## オブジェクト座標
        OBJ2   = np.array([0,0])       #  for 直線_横
        count_h = 0
        count_w = 0       ## 連続するドットの数
        
        for h in range(Serching_size):
                
                if Edge:
                    break
                
                for w in range(Serching_size):

                        ### 画素に値が入っている（ドット発見）
                        if Serch_img[h, w, color_idx].any() == True:

                                ### 続くドットを探索
                                W = Serch_img[h, w+1, color_idx].any()
                                H = Serch_img[h+1, w, color_idx].any()

                                ### イメージ右下(第4象限)にエッジ
                                if H == True and W == True:                    ## エッジ |~~ (第4象限)
                                    OBJ[:] = h, w
                                    Edge = 4
                                    break

                                ### 縦方向(下)に直線が伸びる
                                elif H==True:
                                    h_idx = h + 1

                                    ### 直線の終わりまで進める
                                    while Serch_img[h_idx, w, color_idx].any():
                                        count_h += 1
                                        h_idx += 1
                                    h_idx -= 1

                                    ### 横方向＿1つ前を確認
                                    if Serch_img[h_idx, w-1, color_idx].any():   ## エッジ _| (第2象限)
                                        OBJ[:] = h_idx, w
                                        Edge = 2
                                        break
                                        
                                    elif Serch_img[h_idx, w+1, color_idx].any(): ## エッジ |_ (第1象限)
                                        OBJ[:] = h_idx, w
                                        Edge = 1
                                        break
                                        
                                    elif H_line==0 and count_h > 2:             ## 直線_縦
                                        OBJ[:] = h_idx, w
                                        H_line = 1
                                        break

                                ### 横方向に直線が伸びる
                                elif W==True:
                                    w_idx = w + 1

                                    ### 直線の終わりまで進める
                                    while Serch_img[h, w_idx, color_idx].any():
                                        count_w += 1
                                        w_idx += 1
                                    w_idx -= 1

                                    ### 縦方向＿1つ先を確認
                                    if Serch_img[h+1, w_idx, color_idx].any():   ## エッジ ~~| (第3象限)
                                        OBJ[:] = h, w_idx
                                        Edge = 3
                                        break
                                        
                                    elif W_line==0 and count_w > 2:             ## 直線_横
                                        OBJ2[:] = h, w_idx
                                        W_line = 1
                                        break

                                ### 点
                                elif W_line==0 and H_line==0:
                                    OBJ[:] = h, w
                                    Dot = 1
        
        ### =========== 結果を整理して返す ============
        
        info   = []                          ## オブジェクト情報
        
        ### エッジ or コーナー
        if Edge > 0:
                if Edge==1:
                    if OBJ[0]>middle and OBJ[1]<middle:  ## コーナー(第3象限) __1のエッジが第3象限に達する
                        info.append("C3")
                elif Edge==2:
                    if OBJ[0]>middle and OBJ[1]>middle:  ## コーナー(第4象限)
                        info.append("C4")
                elif Edge==3:
                    if OBJ[0]<middle and OBJ[1]>middle:  ## コーナー(第1象限)
                        info.append("C1")
                elif Edge==4:
                    if OBJ[0]<middle and OBJ[1]<middle:  ## コーナー(第2象限)
                        info.append("C2")
                
                if len(info)==0:
                    info.append(f"E{Edge}")               ### エッジ_ E(1～4)
                info.append(OBJ)
        
        ### 直線 or コーナー
        elif len(Side) > 0:
                
                if Side[0] == 1:    # 横線（上）
                    
                        if H_line:  # 縦線
                            if (OBJ[1]-middle) > 0: # (右寄り)
                                info.append("C1")        ### コーナー (第1象限)
                            else:                   # (左寄り)
                                info.append("C2")        ### コーナー (第2象限)
                        else:
                            info.append("LA")            ### 直線 (上 Above)
                            
                        OBJ[0] = 0
                        info.append(OBJ)
                        
                elif Side[0] == 2:        # 縦線 (左)
                    
                        if W_line:  # 横線
                            if (OBJ2[0]-middle) > 0: # (下寄り)
                                info.append("C3")        ### コーナー (第3象限)
                            else:                    # (上寄り)
                                info.append("C2")        ### コーナー_(第2象限)
                        else:
                            info.append("LL")            ### 直線 (左 Left)
                            
                        OBJ2[1] = 0
                        info.append(OBJ2)
                        
                elif Side[0] == 3:        # 横線 (下)
                    
                        if H_line:  # 縦線
                            if (OBJ[1]-middle) > 0: # (右寄り)
                                info.append("C4")        ### コーナー (第4象限)
                            else:                   # (左寄り)
                                info.append("C3")        ### コーナー (第3象限)
                        else:
                            info.append("LU")            ### 直線 (下 Under)
                        
                        OBJ[0] = Serching_size-1
                        info.append(OBJ)

                elif Side[0] == 5:        # 縦線 (右)
                
                        if W_line:  # 横線
                            if (OBJ2[0]-middle) > 0: # (下寄り)
                                info.append("C4")        ### コーナー (第4象限)
                            else:                    # (上寄り)
                                info.append("C1")        ### コーナー_(第1象限)
                        else:
                            info.append("LR")            ### 直線 (右 Right))
                        
                        OBJ2[1] = Serching_size-1
                        info.append(OBJ2)
        
        ### 直線
        elif H_line or W_line:
                if H_line and W_line:                   ### コーナー  (縦・横の2本線からコーナーを検出)
                        C = np.array([OBJ2[0], OBJ[1]])
                        dH, dW = C[0]-middle, C[1]-middle     ### コーナーの場所を特定
                        
                        if dH > 0:     ## 下
                            if dW > 0: # 右
                                info.append("C4")
                            else:      # 左
                                info.append("C3")
                        else:          ## 上
                            if dW > 0: # 右
                                info.append("C1")
                            else:      # 左
                                info.append("C2")
                        info.append(C)
                
                elif H_line:        # 縦線
                        if (OBJ[1]-middle) > 0: # (右寄り)
                            info.append("LR")            ### 直線 (右 Right))
                        else:                   # (左寄り)
                            info.append("LL")            ### 直線 (左 Left)
                        OBJ[0] = 0
                        info.append(OBJ)
                
                elif W_line:        # 横線
                        if (OBJ2[0]-middle) > 0: # (下寄り)
                            info.append("LU")            ### 直線 (下 Under)
                        else:                   # (上寄り)
                            info.append("LA")            ### 直線 (上 Above)
                        OBJ2[1] = 0
                        info.append(OBJ2)
        ### 点
        elif Dot==1:
                info.append("D")
                info.append(OBJ)
        
        ### 色(0:Blue, 1:Green, 2:Red)
        info.append(color_idx)
        
        return 2, info


### オブジェクト(箱)の接触確認（円が箱と接触したかを判定）
def Check_obj(target, Object_table, current_h, current_w): ## 個数
    
        ### w
        max_w = current_w    ## 最大
        min_w = 0            ## 最小
        remaining = max_w - min_w    ## 残りの列数
        current_w = int(current_w/2) ## 中間の値が基準
        
        while remaining > 1:
            #print(f"論理：{(min_w + current_w-1)}")
            if target[1] <= Object_table[0 , (min_w + current_w-1), 1, 1]:  # 中間以下
                    #print("MIN")
                    max_w = min_w + current_w
            else:
                    #print("BIG")
                    min_w = current_w + min_w
            
            remaining = max_w - min_w
            
            current_w = int(remaining/2)
        
        ## Object_tableに属さない座標をなくす
        w = max_w-1
        if w > 0:   # 自分の1つ前と1つ次を比較して近い方を探す
            if (target[1] - Object_table[0, w-1, 1, 1]) < (Object_table[0, w, 0, 1] - target[1]):
            		w = w-1
            
        
        ### h
        max_h = current_h    ## 最大
        min_h = 0            ## 最小
        remaining = max_h - min_h    ## 残りの行数
        current_h = int(current_h/2) ## 中間の値が基準
        
        while remaining > 1:
            if target[0] <= Object_table[(min_h + current_h-1), 0, 1, 0]:  # 中間以下
                    max_h = min_h + current_h
            else:
                    min_h = current_h + min_h
            
            remaining = max_h - min_h
            
            current_h = int(remaining/2)
        
        ## Object_tableに属さない座標をなくす
        h = max_h-1
        if h > 0:
            if (target[0] - Object_table[h-1, 0, 1, 0]) < (Object_table[h, 0, 0, 0] - target[0]):
                    h = h-1
        
        return h, w
