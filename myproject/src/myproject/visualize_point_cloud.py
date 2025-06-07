import os
import numpy as np
import matplotlib.pyplot as plt


def visualize_point_cloud(file_path: str, output_image_path: str) -> None:
    """
    可視化用に点群データ（x, y, z, R, G, B）を読み込み、3Dプロットとして保存・表示する・

    引数:
        file_path (str): 点群テキストファイル（x, y, z, R, G, B）が含まれているファイルのパス
        output_image_path (str, optional): 画像を保存するパス

    戻り値:
        None
    """
    points = np.loadtxt(file_path)
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    rgb = points[:, 3:6] / 255.0  # 0〜1に正規化

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(x, y, z, c=rgb, s=0.01)
    ax.set_title("3D Point Cloud with RGB")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    # 視点の設定
    ax.view_init(elev=-20, azim=90)

    plt.tight_layout()
    plt.savefig(output_image_path, dpi=300)
    plt.show()


base_dir = os.path.dirname(os.path.abspath(__file__))
root = os.path.abspath(os.path.join(base_dir, "../../"))
file_path = os.path.join(root, "transformed_full.txt")
output_path = os.path.join(root, "transformed_full.png")

visualize_point_cloud(file_path, output_path)
