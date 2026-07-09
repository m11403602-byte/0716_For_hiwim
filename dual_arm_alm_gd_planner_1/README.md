# dual_arm_alm_gd_planner_1

雙臂避障路徑規劃器 — ROS 2 Humble / MoveIt2 插件。

使用 C++17 + Eigen3 實作。內層採用 **ALM (PHR 增廣拉格朗日) + GD (最陡下降梯度法) + 1D Newton 線搜索**，無 Hessian、無 LDLT。

---

## 演算法概觀

兩隻機械臂（RA610 16 球 / RA605 18 球包覆模型），給定起點與終點關節角，求一條兩臂互不碰撞的關節空間軌跡。

採雙層最佳化：

```
外層 (avoidance_system):  生成初始軌跡 -> 碰撞偵測 -> 找危險段 5 點 ->
                          呼叫內層優化 -> Spline 重建 -> 重新檢查 (最多 15 輪)
內層 (gd_solver):         ALM 外層 (罰參數/乘子更新) + GD 內層 + 線搜索,
                          以平滑危險因子 sj = exp(ln0.5/(Ri+Rj)^2 * d^2) 為約束
```

危險因子閾值 `danger_threshold = 0.4`；碰撞判定用 `0.4 + 0.1 = 0.5`（保留緩衝帶）。

---

## 架構：單一 .so

所有原始碼（核心演算法 + MoveIt 插件介面）編進**單一共享庫** `libdual_arm_alm_gd_planner_1.so`。

| 組成 | 角色 |
|------|------|
| `gd_solver` | 第 1 層: 內層 GD 求解器 + FK + 包覆球 + 危險因子 |
| `avoidance_system` | 第 2 層: 外層碰撞修復迴圈 + Spline 重建 |
| `data_io` | CSV 寫入工具 |
| `planner_manager` | 第 3 層: MoveIt2 PlannerManager / PlanningContext |

另編出一個執行檔（連同一個 .so）：`run_standalone`（讀 waypoints 跑避障匯出 CSV）。

> **為何單一 .so（而非拆成 core.so + plugin.so）**
> 早期版本曾把核心演算法拆成獨立的 `dual_arm_avoidance_core.so`。但拆成兩個 .so 時，含 Eigen 矩陣的 `Trajectory` 物件需跨 .so 邊界傳遞，與 MoveIt 的 .so 之間易發生記憶體對齊不一致而崩潰。合併為單一 .so 後，Eigen 物件不跨自己的 .so 邊界，較穩定。代價是失去「純 Eigen、零 MoveIt 依賴」的獨立核心庫（standalone/test 改連這個含 MoveIt 依賴的主 .so）。

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
│ max_D(t) > ＝ 碰撞門檻 (0.5) 代表碰撞持續優化  ?  ──否──▶ 完成 (has_collision=false)│
│        │是                                                           │
│        ▼                                                            │
│ find_collision_targets(): 取危險段 5 個控制點                        │
│   [Head, q1, peak, q3, Tail]                                         │
│        │                                                            │
│        ▼                                                            │
│ 取 3 個內部點 (q1, peak, q3) 組成決策變數 X (36 維)                    │
│ 呼叫內層優化器 run_alm(X0) → X*                                       │
│        │                                                            │
│        ▼                                                            │
│ regenerate_trajectory_global(): 用 X* 做局部 Spline 重建整段軌跡       │
│        │                                                            │
│        ▼                                                            │
│ 重新計算 max_D(t)，回到「危險判定」 (最多 max_refinement_iter=15 輪)   │
└────────────────────────────────────────────────────────────────────┘
```

### 內層：ALM + GD 求解器 (`GdSolver::run_alm`)

```
┌─────────────────────────────────────────────────────────────────┐
│ 初始化: mu = mu0 (=10), c = c0 (=5)                                │
│        │                                                          │
│        ▼                                                          │
│  ┌── 外層 ALM 迴圈 ─────────────────────────────────────────┐     │
│  │   ┌── 內層 GD 迭代 ──────────────────────────────────┐   │     │
│  │   │  計算 G(X) = ∇f + Σ mu_i · max(0, g_i(X)) · ∇g_i │   │     │
│  │   │  d = -G   (最陡下降方向，無 Hessian、無 LDLT)        │   │     │
│  │   │  1D Newton 線搜索求步長 alpha (LS_DELTA=0.001)      │   │     │
│  │   │  X ← X + alpha·d                                   │   │     │
│  │   │  ‖G‖ 是否夠小?  ──否──▶ 繼續下一步                   │   │     │
│  │   └── 是 ──────────────────────────────────────────────┘  │     │
│  │        ▼                                                  │     │
│  │  更新乘子 mu_i ← max(0, mu_i + c·g_i(X))                    │     │
│  │  ‖V_k‖ > gamma_v·‖V_k-1‖ ?  ──是──▶ c ← beta_c·c (c ≤ c_max)│     │
│  │        ▼                                                  │     │
│  │  收斂判定: v_pure 夠小 && ‖G‖ 夠小 && compl 夠小              │     │
│  │      ──否──▶ 回到內層 GD 迭代                                │     │
│  └── 是 ──▶ 回傳 X*、SolverLog ─────────────────────────────────┘     │
└─────────────────────────────────────────────────────────────────┘
```

### 主要變數

| 變數 | 意義 | 維度/型別 |
|------|------|-----------|
| `X` | 決策變數：危險段 3 個內部控制點 × 雙臂各 6 軸角度（degree） | 36 維（`num_X_ = 2*M_*6`） |
| `D_m` | 第 m 個控制點的危險因子向量（雙臂包覆球兩兩距離換算） | `num_D_` 維（有效球對數，含遮罩） |
| `g_i(X)` | 第 i 個不等式約束：`D_i(X) - danger_threshold` | scalar，共 `num_C_ = M_*num_D_` 個 |
| `G` | Lagrangian 對 X 的梯度 | 36 維 |
| `d` | GD 搜索方向 `-G`（最陡下降）；本包**無 Hessian、無 LDLT** | 36 維 |
| `mu` | ALM 乘子估計，`mu0=10` warm-start | `num_C_` 維 |
| `c` | ALM 罰參數（純量），`c0=5` 起，依 `beta_c=8` 逐步放大，上限 `c_max=1e5` | scalar |
| `alpha` | 內層步長，由 1D Newton 線搜索決定（`LS_DELTA=0.001`） | scalar |
| `v_pure` / `‖G‖` / `compl` | 三條收斂判定：主可行性 / 梯度範數（平穩性）/ 互補性 | scalar |
| `gamma_v` | 觸發罰參數放大的比例門檻（`‖V_k‖ > gamma_v·‖V_k-1‖`） | scalar，預設 0.5 |

各參數的 yaml 對應名稱、預設值與所在原始碼位置詳見 [PARAMETERS.md](PARAMETERS.md)。

---

## 目錄結構

在看個別檔案之前，先說明每個頂層資料夾在整個處理流程中分別代表的性質：

- **根目錄檔案**（`CMakeLists.txt` / `package.xml` / `*.xml` / `README.md` / `PARAMETERS.md`）：**建置與描述層**——定義怎麼編譯、怎麼被 ROS 2 / MoveIt2 辨識與載入，以及對外說明文件，不含任何演算法邏輯。
- **`config/`**：**執行期參數層**——存放 move_group 啟動時讀入的 yaml 參數，改參數只需重啟 move_group，不用重新編譯。
- **`include/dual_arm_alm_gd_planner_1/`**：**介面宣告層**——用 `.hpp` 定義各層的類別、資料結構與函式簽名，決定模組之間如何互相呼叫，不含實作細節。
- **`src/`**：**演算法實作層**——對應 `include/` 中宣告的邏輯本體，是實際執行運算（FK、危險因子、ALM+GD 優化迭代、CSV 匯出、MoveIt 介面轉接）的地方。
- **`standalone/`**：**獨立驗證層**——提供不依賴 ROS/MoveIt runtime 的可執行進入點，用來單獨跑演算法、除錯、產生實驗資料，與正式的 MoveIt2 插件路徑互不影響。

四層資料夾的資料流方向大致是：`config/` 參數 → 經 `planner_manager`（第 3 層）讀入 → 呼叫 `avoidance_system`（第 2 層，外層碰撞修復）→ 呼叫 `gd_solver`（第 1 層，內層 ALM+GD 優化）→ 結果透過 `data_io` 匯出 CSV，或轉換回 `RobotTrajectory` 交還 MoveIt。

```
dual_arm_alm_gd_planner_1/
├── CMakeLists.txt
├── package.xml
├── dual_arm_alm_gd_planner_1.xml
├── README.md
├── PARAMETERS.md
├── config/
│   └── dual_arm_alm_gd_planner_1.yaml
├── include/dual_arm_alm_gd_planner_1/
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
| `CMakeLists.txt` | 建置設定：把 `src/` 下四個 .cpp 編進單一共享庫 `libdual_arm_alm_gd_planner_1.so`，並設定 `-O3 -DNDEBUG`（刻意不含 `-march=native`，見下「編譯選項」）；另外編出 `run_standalone` 執行檔並註冊 `run_standalone` ros2 run 目標。 |
| `package.xml` | ROS 2 套件描述（ament_cmake），宣告對 `rclcpp` / `pluginlib` / `moveit_core` / `moveit_msgs` / `eigen` 的相依，並透過 `<moveit_core plugin=...>` 匯出插件描述檔給 pluginlib 讀取。 |
| `dual_arm_alm_gd_planner_1.xml` | pluginlib 的插件描述 XML，宣告 `DualArmAlmGdPlannerManager` 類別與其 `base_class_type`（`planning_interface::PlannerManager`），讓 MoveIt2 能動態載入本插件。 |
| `README.md` | 本文件：演算法概觀、架構、編譯、使用方式、CSV 匯出格式、部署前調整項目。 |
| `PARAMETERS.md` | 參數對照表：列出所有外部可調（yaml/命令列）與內部寫死的參數、預設值、所在原始碼位置。 |
| `config/dual_arm_alm_gd_planner_1.yaml` | move_group 載入的規劃器參數：`path_weight`／`danger_threshold`／`collision_tolerance`／`fix_tolerance`／`max_refinement_iter`、軌跡平滑權重（`smooth_w*`）、ALM 排程參數（`alm_mu0`／`alm_c0`／`alm_c_max`／`alm_beta_c`／`alm_gamma_v`）、關節名前綴、時間參數化與 CSV 診斷輸出開關。 |
| `include/.../gd_solver.hpp` + `src/gd_solver.cpp` | **第 1 層**：內層求解器。實作 FK（`transmatrix`）、雙臂包覆球模型（`BubbleDef` / `BUBBLES_*` / `PEDESTAL_*`）、危險因子 `calc_df`、ALM 外層（罰參數/乘子排程）+ GD（最陡下降：`d = -G`）內層 + 1D Newton 線搜索求步長；無 Hessian、無 LDLT。回傳的 `SolverLog` 記錄每圈/每步的 L、f、maxD、‖G‖、‖d‖ 等收斂歷程，供外層與 CSV 匯出使用。 |
| `include/.../avoidance_system.hpp` + `src/avoidance_system.cpp` | **第 2 層**：外層碰撞修復迴圈（純數學，不依賴 MoveIt，可獨立使用）。定義 `Trajectory`（各臂關節角軌跡）、`CollisionIndices`（危險段 5 個控制點）、`IterLogEntry`（每輪修復快照）等資料結構；`AvoidanceSystem` 類別串起「Clamped Spline 生成初始軌跡 → 碰撞偵測 → 找危險段 → 呼叫 GD 內層優化 → Spline 局部重建 → 重新檢查」的完整流程（最多 `max_refinement_iter` 輪），並提供 `export_danger_factor` / `export_full_log` / `export_trajectory_data` / `export_diagnostics` 等 CSV 匯出函式。 |
| `include/.../data_io.hpp` + `src/data_io.cpp` | CSV 寫入工具（純 std，零依賴）：`write_csv` 寫純數值矩陣＋表頭；`write_csv_labeled` 寫「首欄為字串標籤＋其餘為數值」的表（例如 Targets 表）。只負責寫入，不負責讀取。 |
| `include/.../planner_manager.hpp` + `src/planner_manager.cpp` | **第 3 層**：MoveIt2 `PlannerManager` / `PlanningContext` 介面封裝，把 `AvoidanceSystem` 接到 MoveIt。`DualArmPlanningContext::solve()` 負責 radian↔degree 邊界轉換、讀取 yaml 參數、呼叫外層避障、將結果轉回 `RobotTrajectory` 並依 `time_optimal` 開關做時間參數化。 |
| `standalone/run_standalone.cpp` | 獨立執行檔，不依賴 MoveIt/ROS，直接呼叫 `AvoidanceSystem` 跑避障並匯出 CSV；waypoints 寫死在 `main()` 內，適合開發、除錯、產實驗資料。 |

---

## 執行順序（.cpp 呼叫順序）與參數查找方法

MoveIt2 插件路徑下，四支 .cpp 的實際呼叫順序如下：

1. **`planner_manager.cpp`**（`DualArmPlanningContext::solve()`）— 讀取 yaml 參數，建構 `AvoidanceSystem` 並傳入。
2. **`avoidance_system.cpp`**（`AvoidanceSystem::run_optimization()`）— 外層碰撞修復迴圈；找到危險段後建構 `GdSolver` 並呼叫 `run_alm()`。
3. **`gd_solver.cpp`**（`GdSolver::run_alm()`）— 內層 ALM+GD 優化迭代，各參數在這裡被實際用於計算（例如 `danger_threshold_` 用在約束 `g_i(X) = D_i(X) - danger_threshold_`）。
4. **`data_io.cpp`**（`write_csv` / `write_csv_labeled`）— 只負責把 `SolverLog` / `Trajectory` 寫成 CSV，不含任何決策參數。

standalone 模式（`standalone/run_standalone.cpp`）呼叫順序相同，只是省略第 1 步（不經 MoveIt/yaml，waypoints 與參數直接寫死在 `main()` 呼叫 `AvoidanceSystem` 建構子）。

**查參數意義的方法**：先在 [PARAMETERS.md](PARAMETERS.md) 找到參數對應的成員變數名（如 `danger_threshold_`），再依上面 1→2→3 的順序，用該成員名稱在對應檔案中搜尋（`grep -rn "danger_threshold_" src/`）：

- 在 `planner_manager.cpp` 看到它從 yaml 讀入、傳給 `AvoidanceSystem` 建構子。
- 在 `avoidance_system.cpp` 看到它被存成成員、傳給 `GdSolver` 建構子，並在 `run_optimization()` 用來跟 `max_D` 比較判斷是否碰撞。
- 在 `gd_solver.cpp` 看到它被用在約束式 `g_i(X) = D_i(X) - danger_threshold_`。

照著「讀入 → 傳遞 → 實際使用」這個順序看下去，就能看到一個參數從 yaml 到影響哪段數學式的完整路徑。

---

## 編譯

放進 ROS 2 workspace 的 `src/` 後：

```bash
cd ~/ros2_ws
colcon build --packages-select dual_arm_alm_gd_planner_1
source install/setup.bash
```

需求：ROS 2 Humble、MoveIt2、Eigen3、C++17 編譯器。

### ⚠️ 編譯選項：使用 `-O3`，**不要用 `-march=native`**

CMake 預設 Release `-O3 -DNDEBUG`（**刻意不含 `-march=native`**）。

> **為何不用 -march=native**
> `-march=native` 會讓 Eigen 啟用 AVX，將矩陣對齊到 32-byte；而 MoveIt 的 .so 是用標準選項編的（16-byte 對齊）。當含 Eigen 矩陣的 `Trajectory` 在本插件與 MoveIt 之間傳遞/析構時，兩邊對齊假設不一致，會在碰撞路徑 `regenerate_trajectory_global` 回傳大矩陣後於 `__libc_free` 崩潰（Segmentation fault）。
> 實測：去掉 `-march=native`（僅 `-O3`）即穩定，且規劃僅需約 0.02 秒，速度差異可忽略。
> 部署前可確認：`grep add_compile_options CMakeLists.txt` 應只見 `-O3 -DNDEBUG`，無 `march=native`。

---

## 使用方式

### A. 獨立執行（用核心演算法，仍需在含 MoveIt 的環境編譯）

最快上手，用來開發、除錯、產實驗資料：

```bash
ros2 run dual_arm_alm_gd_planner_1 run_standalone myout
# 或直接執行: ./install/.../lib/dual_arm_alm_gd_planner_1/run_standalone myout
```

waypoints 寫在 `standalone/run_standalone.cpp` 的 `main()` 內（全 degree），要換 case 改數值重編即可。輸出一系列 `myout_*.csv`。

回傳碼：`0` = 成功避障，`1` = 仍碰撞。

### B. 在自己的 C++ 程式呼叫 AvoidanceSystem

```cpp
#include "dual_arm_alm_gd_planner_1/avoidance_system.hpp"
using namespace dual_arm_alm_gd_planner_1;

Eigen::MatrixXd A(2,6), B(2,6);   // 每臂 2 列: 起點 / 終點 (degree)
A << -30,-30.8,38.6,0,-7.8,0,  30,-30.8,38.6,0,-7.8,0;
B << -30,-19.8,-29.8,0,49.6,0,  30,-19.8,-29.8,0,49.6,0;

AvoidanceSystem sys(A, B, /*path_weight=*/0.5, /*danger_threshold=*/0.4);
sys.run_optimization();

if (!sys.has_collision()) {
    auto traj = sys.get_optimized_trajectory();   // traj.pos: (T x 12)
    sys.export_full_log("result");
}
```

> 註：核心類別 `AvoidanceSystem` / `GdSolver` 本身只依賴 Eigen（程式碼層面無 ROS/MoveIt 呼叫）。但因合併為單一 .so，連結時會帶到 MoveIt 依賴，故需在含 MoveIt 的環境編譯。

### C. MoveIt2 插件

在 move_group 設定規劃插件：

```yaml
planning_plugin: dual_arm_alm_gd_planner_1/DualArmAlmGdPlannerManager
```

並載入 `config/dual_arm_alm_gd_planner_1.yaml` 的參數。規劃請求需含 joint goal constraints。

---

## CSV 匯出（除畫圖外的全部資料）

| 函式 | 輸出檔 | 內容 |
|------|--------|------|
| `export_danger_factor(p)` | `p_danger.csv` 之類 | 原始 vs 優化每步 max_D 對照 |
| `export_full_log(p)` | `p_Summary` / `p_Path_*` / `p_D_*` / `p_Targets_*` | 每輪修復完整追蹤 |
| `export_trajectory_data(p)` | `p_Meta` / `p_Original_*` / `p_Iter_*` / `p_Targets` | 繪圖工具用 |
| `export_diagnostics(p)` | `p_diag_summary` / `p_diag_inner` | ALM 收斂診斷 |

每項匯出各自獨立成多個 CSV 檔（純 std，零依賴）。所有 CSV 可用 Excel、Python `pandas.read_csv` 等工具直接開啟。

---

## 需要調整的地方

部署前請依實際機器人調整：

1. **關節名**（`src/planner_manager.cpp` 的 `solve()`）：目前用 A 臂 `big_joint_1~6`、B 臂 `small_joint_1~6`，改成你 SRDF 的實際關節名。
2. **planning group**：solve 用 `req.group_name`，確認 SRDF 的 group 設定。
3. **機器人底座**（`avoidance_system.cpp` 建構函數）：目前 A 臂 `Ty(700)Rz(180)`、B 臂 `Ty(-700)`，兩臂相距 1400mm 面對面 — 依實際擺位調整。
4. **包覆球參數**（`gd_solver.cpp` 的 `BUBBLES_*` / `PEDESTAL_*` 常數）：RA610/RA605 的球座標與半徑，依實際機型確認。
5. **package.xml maintainer**：改成你的 email。

---

## 已知踩雷筆記

- **不要用 `-march=native`**（見上「編譯選項」），會與 MoveIt 的 Eigen 對齊衝突而崩潰。
- **核心庫的 `std::cout`**：建構函數開頭設 `std::cout << std::unitbuf` 關閉緩衝，確保 `[Init]/[Outer]` 等訊息在 MoveIt（非終端機）環境下即時輸出，不被緩衝累積成「有時不印」。
- **MoveIt 用多執行緒執行器**：核心庫的 static 成員皆為 `const`（唯讀），多執行緒讀取安全。

---

## 授權

MIT
