# Báo cáo Day 6: Độ nhạy calibration LiDAR-camera

- **Họ tên:** Le Nguyen Tram Anh
- **MSSV:** 2A202602760
- **Lớp:** VinUni AI20K — Track 4
- **Link repo:** https://github.com/itskathy05/LeNguyenTramAnh-2A202602760-Track4-Day21
- **Topic:** A — LiDAR-camera projection QA
- **Dataset:** `data/synthetic`, `data/kitti_mini`, `data/nuscenes_mini_subset`
- **Các frame đã dùng:** synthetic `000000`; KITTI `000004, 000011, 000012, 000019, 000049, 000061`; nuScenes `scene-0103_010, scene-0103_020, scene-1094_010, scene-1094_020`.

## 1. Claim

Trên các frame đã chọn, yaw drift 1° làm tỷ lệ điểm nguồn nằm trong cùng GT 2D box giảm còn 78.9–81.8% trên KITTI và 84.1–85.6% trên nuScenes. Edge score nhạy với yaw trên KITTI nhưng không ổn định giữa frame và có thể bỏ sót lỗi hình học lớn.

## 2. Evidence

432 cấu hình calibration, mỗi lần chỉ đổi một trục; so sánh cảnh báo theo retention `<0.95` với edge score thấp hơn mức sạch tối thiểu của dataset (KITTI 0.420, nuScenes 0.310). Synthetic self-check cho `(10,0,0)` đạt `z_cam=9.727 m`, pixel `(613.96,175.01)`.

| Thí nghiệm | KITTI | nuScenes |
|---|---:|---:|
| Điểm/frame trung bình | 119,318 | 34,719 |
| Yaw +1°: retention / edge score | 0.818 / 0.593 | 0.856 / 0.464 |
| Projection p50 / p95, 30 lần/frame | 25.82 / 35.27 ms | 4.98 / 6.71 ms |

Stress test có random dropout và Gaussian noise, mỗi loại 4 mức, seed 42; dropout keep 0.3 giữ lại trung bình 27.8% điểm đối tượng KITTI và 29.8% nuScenes; noise σ=0.10 m giữ 94.4% và 92.1%. CSV đầy đủ: `results/calibration_sweep.csv`, `results/degradation_stress.csv`, `results/latency.csv`; latency đo trên Windows CPU, Python 3.11.9 (tên CPU không được hệ điều hành cung cấp).

Overlay KITTI theo khoảng cách: [gần, <6 m](../results/figures/demo_kitti_000019.png), [trung bình](../results/figures/demo_kitti_000011.png), [xa, >50 m](../results/figures/demo_kitti_000004.png). Synthetic: [self-check scene](../results/figures/demo_synthetic_000000.png).

Biểu đồ: [độ nhạy calibration](../results/figures/calibration_sensitivity.png), [so sánh retention và edge score](../results/figures/detector_comparison.png), [stress test](../results/figures/degradation_stress.png), [latency](../results/figures/latency_distribution.png). Overlay nuScenes: [demo](../results/figures/demo_nuscenes_scene-0103_010.png).

## 3. Failure case

![failure pitch drift](../results/figures/fail_01_pitch_drift_edge_false_negative.png)

Trên KITTI `000012`, pitch drift −3° làm object-point retention giảm còn 0.000, nhưng edge score vẫn 0.764, cao hơn ngưỡng 0.420 nên detector theo cạnh không cảnh báo. Drift tạo ra là lỗi **Geometry**; bỏ sót là giới hạn **Metric** vì điểm vẫn gần các cạnh ảnh khác. Threshold được hiệu chỉnh trên năm frame sạch KITTI, nên cần thêm dữ liệu sạch trước khi áp dụng thực tế.

## 4. Khuyến nghị nếu triển khai thật

Với ADAS, dùng edge score như một tín hiệu rẻ để theo dõi calibration, không dùng đơn độc để quyết định an toàn. Kết hợp reprojection score theo vùng/đối tượng, lịch sử drift và kiểm tra extrinsic; khi vượt ngưỡng thì cảnh báo bảo trì hoặc chuyển sang trạng thái cảm biến suy giảm. Ghi log số điểm hợp lệ, phân bố residual, confidence theo range, timestamp và nhiệt độ/gia tốc giá đỡ. Projection thuần CPU mất khoảng 5.0–25.8 ms p50 trong phép đo này; giới hạn chính là tập frame nhỏ và ngưỡng cần hiệu chỉnh theo xe/camera thực.

## 5. Cách chạy lại

Từ thư mục gốc repo trên Windows PowerShell:

```powershell
python -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements.txt
.venv\Scripts\python.exe tools/verify_data.py --data-root data/kitti_mini
.venv\Scripts\python.exe tools/verify_data.py --data-root data/nuscenes_mini_subset
.venv\Scripts\python.exe -m starter.data_health --data-root data/synthetic --out results/data_health_synthetic.csv
.venv\Scripts\python.exe -m starter.data_health --data-root data/kitti_mini --out results/data_health_kitti.csv
.venv\Scripts\python.exe -m starter.data_health --data-root data/nuscenes_mini_subset --out results/data_health_nuscenes.csv
.venv\Scripts\python.exe -m src.calibration_qa self-check
.venv\Scripts\python.exe -m src.calibration_qa all
.venv\Scripts\python.exe tools/check_submission.py
```

`all` tạo lại overlay, ba CSV benchmark/stress/latency và các biểu đồ trong `results/`.

## 6. Khai báo sử dụng AI

| Công cụ | Dùng cho việc gì | Bạn đã kiểm chứng thế nào |
|---|---|---|
| OpenAI Codex | Đọc yêu cầu, hỗ trợ viết projection, script đo và cấu trúc báo cáo | Chạy self-check tọa độ synthetic; chạy lại pipeline từ repo; đối chiếu số trong báo cáo với CSV sinh ra |
