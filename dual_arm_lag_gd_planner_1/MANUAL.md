# dual_arm_lag_gd_planner_1 — 使用手冊

本文件是「怎麼操作」的手冊：編譯、跑起來、切模式、調參數、排除常見錯誤。
演算法原理與數學推導請見 [README.md](README.md)；逐項參數的預設值與程式碼位置請見
[PARAMETERS.md](PARAMETERS.md)。

一句話定位：內層用 **純 Lagrangian 模型 + 最陡下降（GD）**求解雙臂避障問題
（決策變數 `V=[X;λ;S]`，與 ALM 系列是不同的數學模型，不可混用 yaml/常數）。

---

## 1. 三種使用情境，先選一種

| 情境 | 適用時機 | 對應章節 |
|---|---|---|
| A. Standalone 獨立執行 | 只想看避障結果、產 CSV 畫圖、離線測試 | 第 2 節 |
| B. MoveIt2 插件 | 要接上 `move_group`，在 RViz 規劃/執行 | 第 3 節 |
| C. 自己的 C++ 程式呼叫 | 要把避障邏輯嵌進別的專案 | 第 4 節 |

---

## 2. 情境 A：Standalone 獨立執行

### 2.1 編譯

```bash
cd ~/ros2_ws
colcon build --packages-select dual_arm_lag_gd_planner_1
source install/setup.bash
```

### 2.2 改起點/終點 waypoints

打開 [standalone/run_standalone.cpp](standalone/run_standalone.cpp) 的 `main()`，改這兩個矩陣
（單位 degree，每臂 2 列：第 0 列起點、第 1 列終點）：

```cpp
Eigen::MatrixXd A(2, 6), B(2, 6);
A << ...;   // A 臂起點, 終點
B << ...;   // B 臂起點, 終點
```

`path_weight` / `danger_threshold` 也寫在同一個 `main()` 裡（standalone 不讀 yaml，這兩個值只是示範預設）。

### 2.3 重編 + 執行

```bash
colcon build --packages-select dual_arm_lag_gd_planner_1
source install/setup.bash
ros2 run dual_arm_lag_gd_planner_1 run_standalone myout
```

### 2.4 怎麼看結果

- Console 會印 `is_optimized` / `has_collision` / 原始與優化後的軌跡點數。
- **回傳碼**：`0` = 成功避障，`1` = 仍碰撞。
- CSV 輸出在 `myout/<unix秒>_GD/` 資料夾下（`export_unified` 統一匯出，level 1=6 檔、
  level 2=9 檔），欄位定義見 README.md 的「CSV 匯出」章節。
- 收斂判定是 `phys_ok && stable_ok`（`stat_ok` 刻意停用，因純 Lagrangian 對 λ 是鞍點，
  `‖G‖` 不會收斂到 0，屬設計上的正確行為）。若 `has_collision` 一直是 `true`：先加大
  `max_refinement_iter` 或 `lag_max_iter`，再調 `danger_threshold` / `path_weight`。
  GD 是三種內層方法中最簡單、收斂通常最慢的，若迭代次數不夠也會表現成不收斂。

---

## 3. 情境 B：接上 MoveIt2 當 planning_plugin

### 3.1 這個規劃器對應哪個 pipeline 設定檔

`hiwin_dual_arm/config/dual_arm_lag_gd_1_planning.yaml` 已經寫好：

```yaml
planning_plugin: dual_arm_lag_gd_planner_1/DualArmLagGdPlannerManager
```

### 3.2 讓 `move_group` 真的載入這個 pipeline

工作區目前預設**還沒有**把六個自訂 pipeline 接進 `move_group.launch.py`（詳見
`hiwin_dual_arm/README.md` 第 2、3 節）。簡要步驟：

1. `hiwin_dual_arm/package.xml` 加 `<exec_depend>dual_arm_lag_gd_planner_1</exec_depend>`。
2. `move_group.launch.py`（或想用的 launch 檔）用
   `MoveItConfigsBuilder(...).planning_pipelines(pipelines=["ompl", "dual_arm_lag_gd_1", ...])`
   把 `dual_arm_lag_gd_1` 加進清單。
3. 重新編譯：
   ```bash
   colcon build --symlink-install --packages-up-to hiwin_dual_arm
   source install/setup.bash
   ```

### 3.3 送規劃請求

```bash
ros2 launch hiwin_dual_arm move_group.launch.py
ros2 launch hiwin_dual_arm moveit_rviz.launch.py
```

在 RViz 的 MotionPlanning 面板「Planning Pipeline」下拉選單選
`dual_arm_lag_gd_1`，設定起訖姿態後按 Plan / Execute。

### 3.4 調參數（不用重編，改 yaml 後重啟 `move_group`）

編輯 `config/dual_arm_lag_gd_planner_1.yaml`（或
`hiwin_dual_arm/config/dual_arm_lag_gd_1_planning.yaml`，視實際載入路徑而定，
兩者欄位相同）。也可以在 `move_group` 已啟動時動態改（免重啟）：

```bash
ros2 param set /move_group dual_arm_lag_gd_1.danger_threshold 0.35
```

常用欄位（完整表格見 PARAMETERS.md）：

| 參數 | 預設 | 說明 |
|------|------|------|
| `danger_threshold` | 0.4 | 危險因子閾值（避障鬆緊） |
| `collision_tolerance` | 0.1 | 碰撞判定緩衝帶 |
| `path_weight` | 0.5 | A/B 臂成本權重 |
| `max_refinement_iter` | 15 | 外層修復最多輪數 |
| `smooth_w` / `smooth_w_H` / `smooth_w_T` / `smooth_w_neighbor` | 0.3/1/1/1 | 軌跡平滑權重 |
| `lag_wd` | 1.0 | 對偶強度 w_d（約束項罰係數） |
| `lag_lam0` | 30.0 | λ 決策變數初值（⚠ 非 ALM 的外層乘子） |
| `lag_s0` | 1.0 | S 決策變數初值 |
| `lag_tol_phys_margin` / `lag_tol_stable` | 0.01/0.005 | 兩條收斂容差（本包不用 `lag_tol_stat`） |
| `lag_max_iter` | 500 | 內層主迴圈最大迭代 |
| `joint_prefix_A` / `joint_prefix_B` | `big_joint_` / `small_joint_` | 依 SRDF 調整 |
| `time_optimal` | true | true=TOTG；false=自訂等間隔 |

⚠ `lag_*` 參數是本包（純 Lagrangian 譜系）專屬，與 ALM 系列的 `alm_*` 參數不共用、
不可互相套用。

---

## 4. 情境 C：在自己的 C++ 程式呼叫

```cpp
#include "dual_arm_lag_gd_planner_1/avoidance_system.hpp"
using namespace dual_arm_lag_gd_planner_1;

Eigen::MatrixXd A(2,6), B(2,6);   // 每臂 2 列: 起點 / 終點 (degree)
AvoidanceSystem sys(A, B, /*path_weight=*/0.5, /*danger_threshold=*/0.4);
sys.run_optimization();
if (!sys.has_collision()) {
    auto traj = sys.get_optimized_trajectory();   // traj.pos: (T x 12)
    sys.export_unified("result", 2);
}
```

---

## 5. 疑難排解

| 問題 | 排查方向 |
|---|---|
| 編譯後執行時在 MoveIt 內崩潰（segfault） | 確認 `CMakeLists.txt` 沒加 `-march=native`（見 README.md「編譯選項」） |
| RViz 下拉選單看不到 `dual_arm_lag_gd_1` | 六個 pipeline 還沒接進 launch，見第 3.2 節 / `hiwin_dual_arm/README.md` 第 2 節 |
| `has_collision` 一直是 `true` | 先加大 `max_refinement_iter`/`lag_max_iter`，再調 `danger_threshold`/`path_weight` |
| `‖G‖` 一直不收斂到 0 | 屬設計上的正確行為（純 Lagrangian 對 λ 是鞍點方向），不是 bug；收斂判定看 `phys_ok && stable_ok` 即可 |
| 收斂很慢 | GD 是三種內層方法中最簡單、收斂通常最慢的，屬預期行為；要更快可換 CG 或 Newton 版 package |
| 誤把 ALM 系列的 `alm_*` 參數填進本包 yaml | 本包不吃 `alm_*`，改用 `lag_*` 系列參數 |
| TOTG 印「使用預設加速度限制」警告 | `joint_limits.yaml` 未設加速度限制，`time_optimal: true` 時會用預設 `1 rad/s²` |
| 執行軌跡時找不到 action server | 檢查 `moveit_controllers.yaml` 與 `ros2_controllers.yaml` 的 namespace/控制器命名是否一致（見 `hiwin_dual_arm/README.md` 第 1 節） |

---

## 6. 相關文件

- [README.md](README.md) — 數學模型、CSV 欄位定義
- [PARAMETERS.md](PARAMETERS.md) — 逐項參數對照表
- [../hiwin_dual_arm/README.md](../hiwin_dual_arm/README.md) — 機器人設定與六選一 pipeline 整合
- [../MANUAL.md](../MANUAL.md) — 專案總使用手冊（環境準備、六個規劃器怎麼選/怎麼比較）
