import os
import numpy as np
import matplotlib.pyplot as plt

def load_point_cloud(filepath: str) -> np.ndarray:
    """
    テキストファイルから点群を読み込む。

    引数:
        filepath (str): 点群のファイルパス

    戻り値:
        np.ndarray: x, y, z, R, G, B を含む Nx6 の配列
    """
    return np.loadtxt(filepath)

def find_correspondences(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    """
    各 source 点に対して、最も近い target 点を見つける。

    引数:
        source (np.ndarray): Nx3 のソース点群
        target (np.ndarray): Mx3 のターゲット点群

    戻り値:
        np.ndarray: Nx3 の配列で、各行は source の各点に対応する最も近い target の点
    """
    correspondences = []
    for src_point in source:
        dists = np.linalg.norm(target - src_point, axis=1)
        closest_idx = np.argmin(dists)
        correspondences.append(target[closest_idx])
    return np.array(correspondences)

def compute_rigid_transform(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    ソース点群をターゲット点群に最小二乗誤差で一致させる最適な回転行列Rと並進ベクトルtを計算する。

    引数:
        source (np.ndarray): Nx3 のソース点群
        target (np.ndarray): Nx3 の対応するターゲット点群

    戻り値:
        tuple[np.ndarray, np.ndarray]: 
            - (3, 3) の回転行列
            - (3,) の並進ベクトル
    """
    source_mean = np.mean(source, axis=0)
    target_mean = np.mean(target, axis=0)

    source_centered = source - source_mean
    target_centered = target - target_mean

    H = source_centered.T @ target_centered
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = Vt.T @ U.T

    t = target_mean - R @ source_mean
    return R, t

def compute_rmse(source: np.ndarray, target: np.ndarray) -> float:
    """
    ２つの点群間の RMSE（平均二乗平方根誤差）を計算する。

    引数:
        source (np.ndarray): Nx3 のソース点群
        target (np.ndarray): Nx3 のターゲット点群

    戻り値:
        float: RMSE の値
    """
    
    return np.sqrt(np.mean(np.sum((source - target) ** 2, axis=1)))

def save_point_cloud(points: np.ndarray, filename: str) -> None:
    """
    点群データをファイルに保存する。
    
    引数:
        points (np.ndarray): 点群データ（Nx6の配列）
        filename (name): 保存先ファイルのパス
        
    戻り値:
        None
    """
    np.savetxt(filename, points, fmt="%.6f")

def main() -> None:
    """
    ICP（反復最近傍法）を用いた点群の位置合わせを実行するメイン処理。

    この関数では以下の処理を行う
    - 点群ファイル２つを読み込む
    - ソース点群をダウンサンプリングして高速化
    - ダウンサンプリングデータでICPを実行し、変換の計算とRMSE値の出力
    - 得られた変換をフル解像度の点群に適用する
    - フルデータでRMSEを1回だけ計算・表示する
    - 結果を可視化用に保存する
    - ダウンサンプリング時のRMSEの変化をグラフ描画する

    戻り値:
        None
    """
    base_dir = os.path.dirname(__file__)
    root = os.path.abspath(os.path.join(base_dir, "../../../"))
    
    source_path = os.path.join(root, "data", "会議室1_19_53_16.txt")
    target_path = os.path.join(root, "data", "会議室2_19_53_44.txt")


    source = load_point_cloud(source_path)
    target = load_point_cloud(target_path)

    source_xyz = source[:, :3]
    target_xyz = target[:, :3]
    source_rgb = source[:, 3:]
    target_rgb = target[:, 3:]
    
    # 処理用のダウンサンプリング
    source_small = source_xyz[:1000]
    transformed_small = source_small.copy()

    rmse_list = []
    max_iterations = 50
    threshold = 1e-4

    # ダウンサンプリングのRMSE計算
    for i in range(max_iterations):
        matched_small = find_correspondences(transformed_small, target_xyz)
        R, t = compute_rigid_transform(transformed_small, matched_small)
        transformed_small = (R @ transformed_small.T).T + t
        
        rmse = compute_rmse(transformed_small, matched_small)
        rmse_list.append(rmse)
        print(f"Iteration {i+1}: RMSE = {rmse:.6f}")

        if i > 0 and abs(rmse_list[-2] - rmse) < threshold:
            print("Converged.")
            break

    # ダウンサンプリングで得た変換行列R, tをフル点群に使用。（精度が少し向上）
    transformed_full_xyz = (R @ source_xyz.T).T + t
    
    # スライド（可視化）用の保存
    transformed_full = np.hstack([transformed_full_xyz, source_rgb])
    target_full = np.hstack([target_xyz, target_rgb])
    combined = np.vstack([transformed_full, target_full])
    save_point_cloud(combined, "transformed_full.txt")
    
    # 評価のためのフルデータに対して１回のRMSE計算（計算時間が莫大なため）
    matched_full = find_correspondences(transformed_full_xyz, target_xyz)
    rmse_full = compute_rmse(transformed_full_xyz, matched_full)
    print(f"FULL RMSE: {rmse_full:.6f}")
    
    # 収束後にRMSEグラフを描画（ダウンサンプリング）
    plt.plot(rmse_list, marker="o")
    plt.title("DawnSampling ICP RMSE over iterations")
    plt.xlabel("Iteration")
    plt.ylabel("RMSE")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig("Dawnsampling_icp_rmse.png")
    plt.show()

if __name__ == "__main__":
    main()