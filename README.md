# 点群レジストレーション（ICP）

このプロジェクトは、2つの点群データを使って「ICP（反復最近傍法）」で位置合わせ（レジストレーション）を行うものです。整列後の結果を保存し、3Dで可視化もできます。

---

### 使い方

### 1. 仮想環境を作る（初回だけ）

```bash
python -m venv .venv
```

### 2. 仮想環境を有効化
Windows (PowerShell)
```bash
.venv\Scripts\Activate.ps1
```

### 3. ICP 実行
```bash
python main.py
```
transformed_full.txt に整列後の点群が保存されます
Dawnsampling_icp_rmse.png にRMSEのグラフが出力されます

### 4. 点群を3D表示（画像として保存）
```bash
python visualize_point_cloud.py
```
transformed_full.png が出力されます

入力ファイルについて
data フォルダに以下のような .txt ファイルが必要です：

（例）
x y z R G B
1.0 2.0 3.0 255 0 0
4.5 1.2 6.3 0 255 0

出力ファイル
transformed_full.txt : 位置合わせ後の点群

transformed_full.png : 3Dプロット画像

Dawnsampling_icp_rmse.png : RMSEの推移グラフ

説明
main.py : 点群の位置合わせ（ICPの実装）

visualize_point_cloud.py : 点群を3Dで表示・画像保存