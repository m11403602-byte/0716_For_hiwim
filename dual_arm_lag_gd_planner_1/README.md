# dual_arm_lag_gd_planner_1

雙臂避障路徑規劃器 — ROS 2 Humble / MoveIt2 插件。**純 Lagrangian 模型 + 梯度下降 (GD)**。

使用 C++17 + Eigen3 實作。

---

## 數學模型

決策變數 **V = [X; λ; S] ∈ ℝ¹¹¹⁶**（λ、S 都是決策變數，非外層更新的乘子）：

```
原問題:     min f(X)  s.t.  D_i(X) ≤ θ
Slack 化:   h_i = D_i(X) − θ + s_i² = 0          (Bertsekas §3.3.2)
Lagrangian: L(V) = f(X) + w_d · λᵀ · (D − θ·1 + S²)
```

梯度為完整 KKT 一階殘差 **G = [G_X; G_λ; G_S]**：

```
G_X = ∇f + w_d Σ λ_i ∇D_i      (stationarity)
G_λ = w_d · (D − θ + S²)        (primal feasibility)
G_S = 2 w_d · S ⊙ λ            (complementarity)
```

內層 = **最陡下降** `d = −G` + **1D Newton 線搜索**（`LS_DELTA=0.01`）。

⚠ **收斂判定 = `phys_ok && stable_ok`（`stat_ok` 刻意停用）**：純 Lagrangian 對 λ 線性 → 鞍點，`‖G‖` 不會收斂到 0（λ 朝可行性漂移，而非 KKT stationarity）。故以「max_D ≤ θ + margin」+「max_D 穩定」為收斂依據。這是設計上的正確行為，非 bug。

---

## 架構：單一 .so

| 組成 | 角色 |
|------|------|
| `gd_solver` | 第 1 層：內層 GD 求解器 + FK + 包覆球 + 危險因子 |
| `avoidance_system` | 第 2 層：外層碰撞修復迴圈 + Spline 重建 |
| `data_io` | CSV 寫入工具 |
| `planner_manager` | 第 3 層：MoveIt2 PlannerManager / PlanningContext |

---

## 優化流程與變數說明

### 外層：碰撞修復迴圈 (`AvoidanceSystem::run_optimization`)

```
┌────────────────────────────────────────────────────────────────────┐
│ Clamped Cubic Spline 生成初始軌跡 (A/B 臂 起點 → 終點)                │
│        │                                                            │
│        ▼                                                           │
│ 逐步計算危險因子 D_m = calc_df(...) → max_D(t)                       │
│        │                                                            │
│        ▼                                                            │
│ max_D(t) > danger_threshold (0.4) ?  ──否──▶ 完成 (has_collision=false)│
│        │是                                                           │
│        ▼                                                            │
│ find_collision_targets(): 取危險段 5 個控制點                        │
│   [Head, q1, peak, q3, Tail]                                         │
│        │                                                            │
│        ▼                                                            │
│ 取 3 個內部點 (q1, peak, q3) 組成決策變數 X (36 維)                    │
│ 呼叫內層優化器 run_newton(V0) → V* = [X*; λ*; S*]                     │
│        │                                                            │
│        ▼                                                            │
│ regenerate_trajectory_global(): 用 X* 做局部 Spline 重建整段軌跡       │
│        │                                                            │
│        ▼                                                            │
│ 重新計算 max_D(t)，回到「危險判定」 (最多 max_refinement_iter 輪)      │
└────────────────────────────────────────────────────────────────────┘
```

### 內層：純 Lagrangian 最陡下降求解器 (`GdSolver::run_newton`)

```
┌──────────────────────────────────────────────────────────────────┐
│ 初始化: V0 = [X0; λ0=lag_lam0; S0=lag_s0]                          │
│        │                                                           │
│        ▼                                                           │
│ ┌── GD 迭代 (無外層 ALM 排程) ─────────────────────────────────┐   │
│ │  計算殘差:                                                    │   │
│ │    G_X = ∇f + w_d Σ λ_i ∇D_i        (stationarity)           │   │
│ │    G_λ = w_d · (D - θ + S²)          (primal feasibility)     │   │
│ │    G_S = 2 w_d · S ⊙ λ              (complementarity)         │   │
│ │  d = -G   (最陡下降方向，無 Hessian、無 LDLT)                    │   │
│ │  1D Newton 線搜索求步長 alpha (LS_DELTA=0.01)                   │   │
│ │  V ← V + alpha·d                                               │   │
│ │        ▼                                                      │   │
│ │  收斂判定: phys_ok (max_D ≤ θ+margin) &&                       │   │
│ │            stable_ok (max_D 穩定)                              │   │
│ │            （`stat_ok` 刻意停用：λ 為鞍點方向，‖G‖ 不收斂到 0）  │   │
│ │      ──否──▶ 回到 GD 迭代 (最多 lag_max_iter 步)                │   │
│ └── 是 ──▶ 回傳 V* (取 X* 分量)、SolverLog ──────────────────────┘   │
└──────────────────────────────────────────────────────────────────┘
```

### 主要變數

| 變數 | 意義 | 維度/型別 |
|------|------|-----------|
| `X` | 危險段 3 個內部控制點 × 雙臂各 6 軸角度（degree） | 36 維 |
| `λ` | Lagrange 乘子，本身是決策變數（非外層更新） | 與約束數同（`num_C`） |
| `S` | Slack 變數（`h_i = D_i - θ + s_i² = 0`） | 與約束數同（`num_C`） |
| `V` | 完整決策變數 `[X; λ; S]` | 1116 維 |
| `G` | 完整 KKT 一階殘差 `[G_X; G_λ; G_S]` | 1116 維 |
| `d` | GD 搜索方向 `-G`（最陡下降）；本包**無 Hessian、無 LDLT** | 1116 維 |
| `alpha` | 步長，由 1D Newton 線搜索決定（`LS_DELTA=0.01`） | scalar |
| `w_d` | Lagrangian 對約束項的權重（`lag_wd`） | scalar |
| `θ` | 危險因子閾值（同 `danger_threshold`） | scalar |
| `phys_ok` / `stable_ok` | 兩條收斂判定：物理可行 / max_D 穩定（`stat_ok` 停用） | bool |

各參數（`lag_wd`／`lag_lam0`／`lag_s0`／`lag_tol_*`／`lag_max_iter` 等）的 yaml 對應名稱、預設值與所在原始碼位置詳見 [PARAMETERS.md](PARAMETERS.md)。

---

## 目錄結構

在看個別檔案之前，先說明每個頂層資料夾在整個處理流程中分別代表的性質：

- **根目錄檔案**（`CMakeLists.txt` / `package.xml` / `*.xml` / `README.md` / `PARAMETERS.md`）：**建置與描述層**——定義怎麼編譯、怎麼被 ROS 2 / MoveIt2 辨識與載入，以及對外說明文件，不含任何演算法邏輯。
- **`config/`**：**執行期參數層**——存放 move_group 啟動時讀入的 yaml 參數，改參數只需重啟 move_group，不用重新編譯。
- **`include/dual_arm_lag_gd_planner_1/`**：**介面宣告層**——用 `.hpp` 定義各層的類別、資料結構與函式簽名，決定模組之間如何互相呼叫，不含實作細節。
- **`src/`**：**演算法實作層**——對應 `include/` 中宣告的邏輯本體，是實際執行運算（FK、危險因子、純 Lagrangian 最陡下降優化迭代、CSV 匯出、MoveIt 介面轉接）的地方。
- **`standalone/`**：**獨立驗證層**——提供不依賴 ROS/MoveIt runtime 的可執行進入點，用來單獨跑演算法、除錯、產生實驗資料，與正式的 MoveIt2 插件路徑互不影響。

四層資料夾的資料流方向大致是：`config/` 參數 → 經 `planner_manager`（第 3 層）讀入 → 呼叫 `avoidance_system`（第 2 層，外層碰撞修復）→ 呼叫 `gd_solver`（第 1 層，內層純 Lagrangian 最陡下降優化）→ 結果透過 `data_io` 匯出 CSV，或轉換回 `RobotTrajectory` 交還 MoveIt。

```
dual_arm_lag_gd_planner_1/
├── CMakeLists.txt
├── package.xml
├── dual_arm_lag_gd_planner_1.xml
├── README.md
├── PARAMETERS.md
├── config/
│   └── dual_arm_lag_gd_planner_1.yaml
├── include/dual_arm_lag_gd_planner_1/
│   ├── gd_solver.hpp
│   ├── avoidance_system.hpp
│   ├── data_io.hpp
│   └── planner_manager.hpp
├── src/
│   ├── gd_solver.cpp
│   ├── avoidance_system.cpp
│   ├── data_io.cpp
│   └── planner_manager.cpp
└── standalone/
    └── run_standalone.cpp
```

| 檔案 | 功能 |
|------|------|
| `CMakeLists.txt` | 建置設定：把 `src/` 下四個 .cpp 編進單一共享庫 `libdual_arm_lag_gd_planner_1.so`，並設定 `-O3 -DNDEBUG`（刻意不含 `-march=native`，見下「編譯選項」）；另外編出 `run_standalone` 執行檔。 |
| `package.xml` | ROS 2 套件描述（ament_cmake），宣告對 `rclcpp` / `pluginlib` / `moveit_core` / `moveit_msgs` / `eigen` 的相依，並透過 `<moveit_core plugin=...>` 匯出插件描述檔給 pluginlib 讀取。 |
| `dual_arm_lag_gd_planner_1.xml` | pluginlib 的插件描述 XML，宣告 `DualArmLagGdPlannerManager` 類別與其 `base_class_type`（`planning_interface::PlannerManager`），讓 MoveIt2 能動態載入本插件。 |
| `README.md` | 本文件：數學模型、架構、編譯、使用方式、CSV 匯出格式、部署前調整項目。 |
| `PARAMETERS.md` | 參數對照表：列出所有外部可調（yaml/命令列）與內部寫死的參數、預設值、所在原始碼位置。 |
| `config/dual_arm_lag_gd_planner_1.yaml` | move_group 載入的規劃器參數：`path_weight`／`danger_threshold`、軌跡平滑權重、純 Lagrangian 內層參數（`lag_wd`／`lag_lam0`／`lag_s0`／`lag_tol_*`／`lag_max_iter`）、關節名前綴、時間參數化與 CSV 診斷輸出開關。 |
| `include/.../gd_solver.hpp` + `src/gd_solver.cpp` | **第 1 層**：純 Lagrangian 內層優化器。實作 FK（`transmatrix`）、雙臂包覆球模型（`BubbleDef` / `BUBBLES_*` / `PEDESTAL_*`）、危險因子 `calc_df`，以及決策變數 `V=[X;λ;S]` 的最陡下降求解：`d = -G`（無 Hessian、無 LDLT）+ 1D Newton 線搜索求步長（`LS_DELTA=0.01`）。回傳的 `SolverLog` 記錄每步 L、f、maxD、KKT 殘差（stationarity/primal/complementarity）等收斂歷程。 |
| `include/.../avoidance_system.hpp` + `src/avoidance_system.cpp` | **第 2 層**：外層碰撞修復迴圈（純數學，不依賴 MoveIt，可獨立使用）。定義 `Trajectory`（各臂關節角軌跡）、`CollisionIndices`（危險段 5 個控制點）等資料結構；`AvoidanceSystem` 類別串起「Clamped Spline 生成初始軌跡 → 碰撞偵測 → 找危險段 → 呼叫 GD 內層優化 → Spline 局部重建 → 重新檢查」的完整流程，並提供 `export_unified` 統一 CSV 匯出函式。 |
| `include/.../data_io.hpp` + `src/data_io.cpp` | CSV 寫入工具（純 std，零依賴）：`write_csv` 寫純數值矩陣＋表頭；`write_csv_labeled` 寫「首欄為字串標籤＋其餘為數值」的表。只負責寫入，不負責讀取。 |
| `include/.../planner_manager.hpp` + `src/planner_manager.cpp` | **第 3 層**：MoveIt2 `PlannerManager` / `PlanningContext` 介面封裝，把 `AvoidanceSystem` 接到 MoveIt。`DualArmPlanningContext::solve()` 負責 radian↔degree 邊界轉換、讀取 yaml 參數、呼叫外層避障、將結果轉回 `RobotTrajectory` 並依 `time_optimal` 開關做時間參數化。 |
| `standalone/run_standalone.cpp` | 獨立執行檔，不依賴 MoveIt/ROS，直接呼叫 `AvoidanceSystem` 跑避障並匯出 CSV；waypoints 寫死在 `main()` 內，適合開發、除錯、產實驗資料。 |

---

## 執行順序（.cpp 呼叫順序）與參數查找方法

MoveIt2 插件路徑下，四支 .cpp 的實際呼叫順序如下：

1. **`planner_manager.cpp`**（`DualArmPlanningContext::solve()`）— 讀取 yaml 參數，建構 `AvoidanceSystem` 並傳入。
2. **`avoidance_system.cpp`**（`AvoidanceSystem::run_optimization()`）— 外層碰撞修復迴圈；找到危險段後建構 `GdSolver` 並呼叫 `run_newton()`。
3. **`gd_solver.cpp`**（`GdSolver::run_newton()`）— 內層純 Lagrangian 最陡下降優化迭代，各參數在這裡被實際用於計算（例如 `danger_threshold_` 用在約束項 `D_i(X) - danger_threshold_ + s_i²`）。
4. **`data_io.cpp`**（`write_csv` / `write_csv_labeled`）— 只負責把 `SolverLog` / `Trajectory` 寫成 CSV，不含任何決策參數。

standalone 模式（`standalone/run_standalone.cpp`）呼叫順序相同，只是省略第 1 步（不經 MoveIt/yaml，waypoints 與參數直接寫死在 `main()` 呼叫 `AvoidanceSystem` 建構子）。

**查參數意義的方法**：先在 [PARAMETERS.md](PARAMETERS.md) 找到參數對應的成員變數名（如 `danger_threshold_`），再依上面 1→2→3 的順序，用該成員名稱在對應檔案中搜尋（`grep -rn "danger_threshold_" src/`）：

- 在 `planner_manager.cpp` 看到它從 yaml 讀入、傳給 `AvoidanceSystem` 建構子。
- 在 `avoidance_system.cpp` 看到它被存成成員、傳給 `GdSolver` 建構子，並在 `run_optimization()` 用來跟 `max_D` 比較判斷是否碰撞。
- 在 `gd_solver.cpp` 看到它被用在約束項 `D_i(X) - danger_threshold_ + s_i²`。

照著「讀入 → 傳遞 → 實際使用」這個順序看下去，就能看到一個參數從 yaml 到影響哪段數學式的完整路徑。

---

## 編譯

```bash
cd ~/ros2_ws
colcon build --packages-select dual_arm_lag_gd_planner_1
source install/setup.bash
```

需求：ROS 2 Humble、MoveIt2、Eigen3、C++17。

### ⚠ 使用 `-O3`，**不要用 `-march=native`**

`-march=native` 會讓 Eigen 啟用 AVX（32-byte 對齊），與 MoveIt 的 .so（16-byte 對齊）在傳遞含 Eigen 的 `Trajectory` 時對齊不一致，於 `regenerate_trajectory_global` 回傳大矩陣後於 `__libc_free` 崩潰。CMake 預設僅 `-O3 -DNDEBUG`，部署前確認 `grep add_compile_options CMakeLists.txt` 無 `march=native`。

---

## 使用方式

### A. 獨立執行（不啟 ROS）

```bash
ros2 run dual_arm_lag_gd_planner_1 run_standalone myout
```

waypoints 寫在 `standalone/run_standalone.cpp` 的 `main()`（全 degree），換 case 改數值重編。回傳碼：`0`=成功避障，`1`=仍碰撞。

### B. 在 C++ 程式呼叫

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

### C. MoveIt2 插件

```yaml
planning_plugin: dual_arm_lag_gd_planner_1/DualArmLagGdPlannerManager
```

並載入 `config/dual_arm_lag_gd_planner_1.yaml`。

---

## CSV 匯出（`export_unified`）

目錄 `<prefix>/<unix秒>_GD/`，level 1=6 檔、level 2=9 檔。

⚠ **schema 與 ALM 不同**（純 Lagrangian 無 c/μ 排程，改帶 KKT 殘差 + λ-S 診斷）：

| 檔 | 內容 |
|----|------|
| `meta.csv` | 參數快照（含 `lag_wd/lag_lam0/lag_s0/lag_tol_*/lag_max_iter`）|
| `summary.csv` | 每輪末值：L/f/f_a/f_b/penalty/maxD/final_G_norm/final_r_stat/final_kkt |
| `inner.csv` | 逐迭代完整軌跡（21 欄）：L/f/f_a/f_b/penalty/maxD/G_norm/d_norm/alpha + KKT 殘差(r_stat/r_prim/r_comp/r_dual/lam_neg/kkt) + λ/S 範圍 |
| `danger_final` / `danger_rounds` / `targets` | 同 ALM（solver 無關）|
| level 2: `constraints_all` / `path_original` / `path_evolution` | 同 ALM |

---

## 部署前須調整

1. **關節名**：yaml `joint_prefix_A/B`（預設 `big_joint_` / `small_joint_`）依 SRDF。
2. **機器人底座**：`avoidance_system.cpp` 建構函數 A=`Ty(700)Rz(180)`、B=`Ty(-700)`。
3. **包覆球**：`gd_solver.cpp` 的 `BUBBLES_*` / `PEDESTAL_*`。
4. **package.xml maintainer**：改成你的 email。

## 授權

MIT
