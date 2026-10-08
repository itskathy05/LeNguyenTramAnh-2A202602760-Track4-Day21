# Kịch bản trình bày 3 phút

## 0:00–0:30 — Câu hỏi và claim

“Em kiểm tra ảnh hưởng của calibration LiDAR-camera drift lên phép chiếu. Trên các frame em thử, yaw drift 1° làm tỷ lệ điểm giữ đúng GT 2D box còn khoảng 79–86%, tùy dataset. Em cũng tìm được trường hợp edge score bỏ sót một lỗi hình học lớn.”

## 0:30–1:10 — Cách đo

“Em tự cài hai bước LiDAR sang camera và camera sang pixel, sau đó chiếu điểm lên ảnh và so với GT box. Em sweep riêng roll, pitch, yaw và dịch chuyển x/y/z; mỗi lượt chỉ đổi một tham số. Hai chỉ số là object-point retention theo correspondence điểm và edge score, tức tỷ lệ điểm gần cạnh ảnh. Ngưỡng edge đặt riêng cho mỗi dataset từ frame sạch.”

## 1:10–1:50 — Kết quả

“Em chạy 432 cấu hình trên 5 frame KITTI và 4 frame nuScenes. Em so sánh Canny inlier fraction với median Chamfer trên cùng cấu hình: Canny đạt F1 0.166/0.241, Chamfer 0.073/0.194 trên KITTI/nuScenes; cả hai không báo nhầm trên frame sạch nhưng bỏ sót nhiều drift nhỏ. KITTI có 119 nghìn điểm/frame, nuScenes 34.7 nghìn do sensor khác nhau.”

## 1:50–2:30 — Failure case

“Ở KITTI frame 000012, pitch drift −3° làm retention tụt về 0 nhưng edge score vẫn 0.764, cao hơn ngưỡng 0.420. Điểm bị chiếu lệch khỏi GT object nhưng vẫn rơi gần các cạnh khác trong ảnh, nên edge score báo âm tính giả. Đây là lỗi Geometry được tạo có chủ ý và điểm yếu của Metric.”

## 2:30–3:00 — Triển khai và giới hạn

“Với ADAS, em sẽ dùng edge score như tín hiệu giám sát rẻ, kết hợp residual theo object/range, lịch sử calibration và kiểm tra extrinsic. Khi bất thường, cảnh báo bảo trì hoặc đánh dấu cảm biến suy giảm. Giới hạn hiện tại là số frame nhỏ, ngưỡng hiệu chỉnh theo dataset và chưa đo trên xe thật.”

## Câu hỏi có thể gặp

- **Vì sao 64-beam KITTI và 32-beam nuScenes khác nhau?** Mật độ điểm, quy ước trục, độ phân giải ảnh, ánh sáng và đồng bộ thời gian khác nhau; vì vậy ngưỡng score cần hiệu chỉnh riêng.
- **NuScenes lệch thời gian thế nào?** Reader chuyển LiDAR và camera qua global frame bằng ego pose mặc định; tắt `use_ego_motion` sẽ tạo thêm lỗi Time.
- **Edge score có phát hiện mọi drift không?** Không. Pitch −3° cho retention 0 nhưng edge score vẫn cao; cần kết hợp hình học object và edge.
- **Production cần log gì?** Tỷ lệ điểm chiếu hợp lệ, residual theo range/class, timestamp offset, score từng vùng, confidence, lịch sử calibration và trạng thái rung/nhiệt của giá đỡ.
- **Khi đổi dataset thì sao?** Không chuyển nguyên ngưỡng. Đo baseline sạch trên sensor/camera mới rồi xác lập threshold và kiểm tra false alarm/miss.
