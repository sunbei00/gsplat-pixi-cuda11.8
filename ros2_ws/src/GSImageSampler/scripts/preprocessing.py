#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
COLMAP: known poses로 points3D 생성 (단일 스크립트)

전제:
- data_root/
    images/                      # 이미지 폴더
    sparse/0/                    # 여기에 cameras.txt, images.txt 존재 (KNOWN POSES)
        cameras.txt
        images.txt
        [points3D.txt] (없어도 됨, 스크립트가 헤더만 생성)
동작:
1) cameras.txt / images.txt 존재 확인 및 points3D.txt 헤더 생성
2) feature_extractor → (exhaustive|sequential|vocab_tree) matcher
3) point_triangulator (포즈는 고정, 매칭된 트랙을 삼각화)
4) 옵션: triangulated 모델을 TXT로 덤프

작성: you
"""
import argparse
import os
from pathlib import Path
import subprocess
import sys

POINTS3D_HEADER = """# 3D point list with one line of data per point:
# POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)
"""

def run(cmd):
    print("[RUN]", " ".join(cmd), flush=True)
    p = subprocess.run(cmd)
    if p.returncode != 0:
        sys.exit(p.returncode)

def ensure_points3d_txt(p_txt: Path):
    if p_txt.exists():
        return
    p_txt.parent.mkdir(parents=True, exist_ok=True)
    with open(p_txt, "w") as f:
        f.write(POINTS3D_HEADER)
    print(f"[OK] wrote empty {p_txt}")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data_root", required=True, help="COLMAP 데이터 루트(이미지/스파스 포함)")
    ap.add_argument("--matcher", default="exhaustive", choices=["exhaustive", "sequential", "vocab_tree"])
    ap.add_argument("--dump_txt", action="store_true", help="결과를 TXT로도 덤프")
    args = ap.parse_args()

    root = Path(args.data_root).resolve()
    image_path = root / "images"
    sparse_known = root / "sparse" / "camera"
    cameras_txt = sparse_known / "cameras.txt"
    images_txt = sparse_known / "images.txt"
    points3d_txt = sparse_known / "points3D.txt"
    db_path = root / "database.db"
    out_sparse = root / "sparse" / "0"
    out_txt = root / "sparse" / "0_txt"

    # 0) sanity checks
    if not image_path.is_dir():
        print(f"[ERR] images 폴더 없음: {image_path}", file=sys.stderr); sys.exit(1)
    if not cameras_txt.is_file():
        print(f"[ERR] cameras.txt 없음: {cameras_txt}", file=sys.stderr); sys.exit(1)
    if not images_txt.is_file():
        print(f"[ERR] images.txt 없음: {images_txt}", file=sys.stderr); sys.exit(1)

    # 1) 빈 points3D.txt 헤더 생성(없을 때만)
    ensure_points3d_txt(points3d_txt)

    # 2) 데이터베이스 초기화 후 특징 추출
    if db_path.exists():
        print(f"[INFO] remove existing database: {db_path}")
        db_path.unlink()
    run([
        "colmap", "feature_extractor",
        "--database_path", str(db_path),
        "--image_path", str(image_path),
    ])

    # 3) 매칭
    if args.matcher == "exhaustive":
        run(["colmap", "exhaustive_matcher", "--database_path", str(db_path)])
    elif args.matcher == "sequential":
        run(["colmap", "sequential_matcher",
             "--database_path", str(db_path),
             "--image_path", str(image_path)])
    else:
        run(["colmap", "vocab_tree_matcher", "--database_path", str(db_path)])

    # 4) known poses로 삼각화
    out_sparse.mkdir(parents=True, exist_ok=True)
    run([
        "colmap", "point_triangulator",
        "--database_path", str(db_path),
        "--image_path", str(image_path),
        "--input_path", str(sparse_known),
        "--output_path", str(out_sparse),
        # 필요시 BA refine 옵션 조정:
        # "--Mapper.ba_refine_focal_length", "0",
        # "--Mapper.ba_refine_principal_point", "0",
        # "--Mapper.ba_refine_extra_params", "0",
    ])

    # 5) (옵션) TXT 덤프
    if args.dump_txt:
        out_txt.mkdir(parents=True, exist_ok=True)
        run([
            "colmap", "model_converter",
            "--input_path", str(out_sparse),
            "--output_path", str(out_txt),
            "--output_type", "TXT",
        ])

    print("\n[DONE] Triangulated model:", out_sparse)
    if args.dump_txt:
        print("[INFO] TXT dump:", out_txt)

if __name__ == "__main__":
    main()

