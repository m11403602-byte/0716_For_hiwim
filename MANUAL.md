# 使用手冊（Project Manual）— HIWIN 雙臂避障規劃器 workspace

本文件是「怎麼操作」的入口手冊，涵蓋整個 workspace 共通的環境準備、六個規劃器怎麼選 /
怎麼切換 / 怎麼比較。個別演算法的原理與數學推導在各 package 自己的 `README.md`；
逐步操作細節（standalone、接 MoveIt2、C++ API、疑難排解）在各 package 自己的
`MANUAL.md`。整體架構總覽見 [README.md](README.md)。

---

## 1. 一次性環境準備

```bash
distrobox enter hiwin-humble-env

# 容器內先清掉可能殘留的系統 ROS 環境變數，避免跟 humble 打架
unset ROS_VERSION ROS_ROOT AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/humble/setup.bash

cd ~/0706_hiwim         # workspace 根目錄
colcon build --symlink-install --cmake-args -DCMAKE_PREFIX_PATH=/opt/ros/humble
source install/setup.bash

export ROS_DOMAIN_ID=24   # 若多機/多容器共用網路，統一網域 ID
```

需求：ROS 2 Humble、MoveIt2、Eigen3、C++17 編譯器。詳細環境細節與容器內啟動流程見
[hiwin_dual_arm/README.md](hiwin_dual_arm/README.md) 第 4 節。

---

## 2. 六個規劃器是什麼、怎麼選

六個規劃器解的是**同一個**雙臂避障問題，差別只在內層最佳化用哪一種數學模型／哪一種
求解方法，可視為 2×3 的組合矩陣：

| | Newton（二階） | CG（共軛梯度） | GD（最陡下降） |
|---|---|---|---|
| **ALM 模型**（增廣拉格朗日） | [dual_arm_alm_newton_planner_1](dual_arm_alm_newton_planner_1/MANUAL.md) | [dual_arm_alm_cg_planner_1](dual_arm_alm_cg_planner_1/MANUAL.md) | [dual_arm_alm_gd_planner_1](dual_arm_alm_gd_planner_1/MANUAL.md) |
| **純 Lagrangian 模型** | [dual_arm_lag_newton_planner_1](dual_arm_lag_newton_planner_1/MANUAL.md) | [dual_arm_lag_cg_planner_1](dual_arm_lag_cg_planner_1/MANUAL.md) | [dual_arm_lag_gd_planner_1](dual_arm_lag_gd_planner_1/MANUAL.md) |

沒有特別偏好時的預設建議：**`dual_arm_alm_newton_planner_1`**（收斂最穩定快速，ALM
雙層結構的排程也整定得最溫和）。若要用純 Lagrangian 譜系，建議先看該 package
README.md 開頭的警語（決策變數含 λ/S，不可跟 ALM 系列混用參數）。

不知道怎麼選時，可以先用 standalone 模式（見第 3 節）分別跑同一組 waypoints，
比較各自的 CSV 輸出（`inner.csv` 的收斂曲線、`summary.csv` 的最終 maxD）。

---

## 3. 三種使用情境（六個規劃器共通）

| 情境 | 適用時機 |
|---|---|
| A. Standalone 獨立執行 | 只想看避障結果、產 CSV 畫圖、離線測試，不用啟動 MoveIt |
| B. MoveIt2 插件 | 要接上 `move_group`，在 RViz 規劃/執行到（模擬或實體）機器人 |
| C. 自己的 C++ 程式呼叫 | 要把避障邏輯嵌進別的專案 |

每個規劃器三種情境的**逐步操作**（怎麼編譯、怎麼改 waypoints、怎麼看 CSV、怎麼調
yaml 參數、常見錯誤）都寫在各自的 `MANUAL.md`，見上面第 2 節的連結表。

---

## 4. 切換 / 同時比較六個規劃器（MoveIt2 pipeline 層級）

`hiwin_dual_arm` 是「底盤」（機器人模型、控制器），六個 `dual_arm_*_planner_1` 是可插拔
的「規劃大腦」。要讓 RViz 的 Planning Pipeline 下拉選單能六選一切換，需要：

1. `hiwin_dual_arm/package.xml` 補上六個規劃器 package 的 `exec_depend`。
2. launch 檔（如 `move_group.launch.py`）用 `MoveItConfigsBuilder(...).planning_pipelines(...)`
   把六個 pipeline id（`dual_arm_alm_newton_1` 等）都列進去。
3. 重新編譯 `hiwin_dual_arm`（`--packages-up-to hiwin_dual_arm` 會連六個規劃器一起建）。

完整步驟、目前工作區的接線狀態（尚未接進 launch）、以及完整啟動流程
（`demo.launch.py` / 分件手動啟動 / `brain.launch.py`）見
[hiwin_dual_arm/README.md](hiwin_dual_arm/README.md) 第 2–4 節。

接好之後可以：

```bash
# RViz MotionPlanning 面板「Planning Pipeline」下拉選單即時切換，不用重啟

# 或不重啟 move_group，動態改某個規劃器的參數：
ros2 param set /move_group dual_arm_alm_newton_1.danger_threshold 0.35
```

---

## 5. 常見問題 FAQ（六個規劃器共通）

| 問題 | 排查方向 |
|---|---|
| 編譯後在 MoveIt 內崩潰（segfault） | 確認對應 package 的 `CMakeLists.txt` 沒加 `-march=native`（各 package README.md「編譯選項」有詳細原因） |
| RViz 下拉選單看不到自訂 pipeline | 六個 pipeline 還沒接進 launch，見第 4 節 |
| `has_collision` 一直是 `true` | 依序調 `max_refinement_iter` → `danger_threshold`/`path_weight` → 內層排程參數（ALM 系列調 `alm_c_max`/`alm_beta_c`；Lagrangian 系列調 `lag_max_iter`），見各 package `MANUAL.md` 第 2.4 節 |
| TOTG 印「使用預設加速度限制」警告 | `hiwin_dual_arm/config/joint_limits.yaml` 未設加速度限制，`time_optimal: true` 時會用預設 `1 rad/s²` |
| 執行軌跡時找不到 action server | 檢查 `moveit_controllers.yaml` 與 `ros2_controllers.yaml` 的 namespace/控制器命名是否一致，見 `hiwin_dual_arm/README.md` 第 1 節 |
| 想換機器人（非 RA610/RA605 雙臂） | 需同步調整六個 package 的機器人幾何常數（見各 `PARAMETERS.md`）與 `hiwin_dual_arm` 的 URDF/SRDF，見 `hiwin_dual_arm/README.md` 第 6 節 |

---

## 6. 文件地圖

| 文件 | 內容 |
|---|---|
| [README.md](README.md) | Workspace 架構總覽：7 個 package 關係、雙層最佳化框架、ALM vs 純 Lagrangian、三種內層方法比較 |
| 本文件（`MANUAL.md`） | 環境準備、六選一怎麼選/切換、共通 FAQ |
| `dual_arm_*_planner_1/README.md` | 該規劃器的演算法推導、CSV 欄位定義、已知踩雷筆記 |
| `dual_arm_*_planner_1/MANUAL.md` | 該規劃器的逐步操作手冊（standalone / MoveIt2 / API 三種情境） |
| `dual_arm_*_planner_1/PARAMETERS.md` | 該規劃器逐項參數對照表（yaml 可調 vs 寫死、預設值、程式碼位置） |
| [hiwin_dual_arm/README.md](hiwin_dual_arm/README.md) | 機器人本體與 MoveIt2 設定包：SRDF/控制器/launch、六選一 pipeline 整合步驟、完整啟動流程 |

---

## 7. 授權

各 package 內文件標示為 MIT；`hiwin_dual_arm` 沿用 MoveIt2 產生的預設授權 BSD。
