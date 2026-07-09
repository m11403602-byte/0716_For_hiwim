# 參數對照表 — dual_arm_lag_newton_planner_1

本文件列出所有參數：外部可調 vs 寫死、預設值、所在位置。

---

## 一、yaml 可調參數（改 yaml 重啟 move_group 生效）

### 1-1. 外層 / 平滑（與 ALM 共用語意）

| 參數 | 預設 | 說明 |
|------|------|------|
| `path_weight` | 0.5 | A/B 手臂避障權重 `pw·fA+(1−pw)·fB` |
| `danger_threshold` | 0.4 | 危險因子閾值 |
| `collision_tolerance` | 0.1 | 碰撞判定緩衝帶 （此值＝ 判撞門檻（0.5） - 危險因子） |
| `fix_tolerance` | 0.1 | `find_targets` 的 fix_gap 比例 |
| `max_refinement_iter` | 15 | 外層修復最多輪數 |
| `smooth_w` | **1.0** | 平滑項主權重 ⚠ Lagrangian 為 1.0（≠ ALM 的 0.3）|
| `smooth_w_H` | 1.0 | Head 端權重 |
| `smooth_w_T` | 1.0 | Tail 端權重 |
| `smooth_w_neighbor` | 1.0 | 鄰點權重 |
| `joint_prefix_A` | `"big_joint_"` | A 臂關節名前綴（依 SRDF）|
| `joint_prefix_B` | `"small_joint_"` | B 臂關節名前綴（依 SRDF）|
| `time_optimal` | true | true=TOTG; false=自訂等間隔 |
| `path_total_time` | 5.0 | （time_optimal=false）目標總時間（秒）|
| `min_time_interval` | 0.05 | （time_optimal=false）每點最小間隔（秒）|

### 1-2. 純 Lagrangian 參數 [NEW]（取代 ALM 的 `alm_*`）

| 參數 | 預設 | 說明 |
|------|------|------|
| `lag_wd` | 1.0 | 對偶強度 w_d（penalty 係數）|
| `lag_lam0` | **30.0** | λ_0 初值 |
| `lag_s0` | 1.0 | S_0 初值（S²=1）|
| `lag_tol_phys_margin` | 0.01 | 物理收斂容差 max_D ≤ θ + margin |
| `lag_tol_stable` | 0.005 | 穩定收斂容差 \|Δmax_D\| ≤ 此值 |
| `lag_tol_stat` | 0.1 | ⚠ **Newton 專屬**: stationarity 收斂 ‖G‖ ≤ 此值 (GD/CG 不用) |
| `lag_max_iter` | 500 | 主迴圈最大迭代 |

> ⚠ `lag_lam0` / `lag_s0` 是**決策變數初值**（V=[X;λ;S] 的一部分），與 ALM 的 `alm_mu0`（外層乘子）概念不同，不可對照搬移。

### 1-3. 診斷輸出

| 參數 | 預設 | 說明 |
|------|------|------|
| `export_csv_prefix` | `""`（關）| 非空 → 每次規劃後匯出 CSV 子目錄 `<prefix>/<unix秒>_GD/` |
| `export_level` | 1 | 0=不匯出；1=標配 6 檔；2=完整 9 檔 |
| `solver_verbose` | false | 內層逐迭代詳細記錄 |

---

## 二、寫死參數（需改原始碼重編）

### 2-1. 內層收斂與線搜索（`newton_solver.cpp`）

| 參數 | 值 | 說明 |
|------|-----|------|
| `delta` | 0.01 | 有限差分擾動 h（FK 與 cost FD 共用）|
| 線搜索 `LS_DELTA` | **0.01** | ⚠ ≠ ALM 的 0.001 |
| 線搜索 `MAX_INNER` | **2000** | ⚠ ≠ ALM 的 50 |
| `H_MIN_ABS` | 1e-9 | 二階退化保護 |
| `ANEW_TOL` | 1e-3 | 步長收斂門檻 |
| `FALLBACK_ALPHA` | 0.0001 | 線搜索失敗退路 |
| 發散門檻 | ‖G‖ > 1e9 或 NaN/Inf | |

### 2-2. 危險因子（`newton_solver.cpp` `calc_df`）

| 公式 | `sj = exp( ln(0.5)/(R_i+R_j)² · d² )` |
|------|------|
| 球心距 = R_i+R_j 時 | sj = 0.5 |

### 2-3. 外層（`avoidance_system.cpp`）

| 參數 | 值 | 說明 |
|------|-----|------|
| `STEP_MAX_DEG_` | 0.5 | 軌跡切分最大角度差（度）|
| robotA_base | `Ty(700)·Rz(180°)` | A 臂底座 |
| robotB_base | `Ty(-700)·Rz(0°)` | B 臂底座（兩臂相距 1400mm 面對面）|

### 2-4. 機器人幾何（`newton_solver.cpp`，與 ALM/Newton 譜系位元一致）

| 項目 | 內容 |
|------|------|
| RA610 (A 臂) | 16 球（4 底盤 R=320 + 12 手臂）+ 7 連桿 FK |
| RA605 (B 臂) | 18 球（8 底盤 R=350 + 10 手臂）+ 7 連桿 FK |
| 球→連桿映射 | sA[16]、sB[18] |
| 跨臂 mask | cAB 第 4~8 連桿 → K_AB=180 |

---

## 三、決策變數維度（執行期決定，本案）

| 參數 | 值 | 說明 |
|------|-----|------|
| M（中間優化點）| 3 | 不含 Head/Tail |
| num_X | 36 = 2×3×6 | X 段 |
| num_D = K_AB | 180 | 跨臂約束（mask 過濾後）|
| num_C | 540 = M × num_D | λ / S 各自的維度 |
| **total_dim** | **1116 = num_X + 2·num_C** | V = [X; λ; S] |

---

## 四、函數說明

### 4-1. `avoidance_system`（第 2 層：外層碰撞修復，純數學，不依賴 MoveIt）

| 函式 | 說明 |
|------|------|
| `AvoidanceSystem(A_waypoints, B_waypoints, path_weight, danger_threshold, ...)` | 建構子：接收 A/B 臂各 2×6 起訖關節角與可調參數，初始化狀態 |
| `run_optimization()` | 對外主要入口：跑「生成軌跡 → 偵測碰撞 → 找危險段 → 呼叫內層優化 → Spline 重建 → 重新檢查」完整迴圈，最多 `max_refinement_iter_` 輪 |
| `generate_initial_trajectory()` | 用 clamped cubic spline 由起訖點生成初始整段軌跡 |
| `check_collision(traj, path_D_max, is_collision, path_D_all)` | 逐步呼叫 `calc_df` 算危險因子，得到每步 `max_D` 與是否超過 `danger_threshold_` |
| `find_collision_targets(path_D)` | 在超過閾值的區段找出 5 個控制點索引：`[Head, q1, peak, q3, Tail]` |
| `run_solver_global(traj, targets, Xa_opt, Xb_opt, solver_log)` | 用 5 點中的 3 個內部點組成決策變數 X0，建立並呼叫 `NewtonSolver::run_newton()` |
| `regenerate_trajectory_global(old_traj, Xa_opt, Xb_opt, indices, targets_out)` | 用優化後的 3 個內部點做局部 clamped spline，重建整段軌跡 |
| `clamped_cubic_spline(t_knots, Y, v0, v1, t_query)` | 靜態工具：給節點時間、值、端點斜率，解三對角系統求二階導後做分段三次插值 |
| `set_solver_verbose(v)` / `set_lag_params(...)` | 將 verbose 開關與純 Lagrangian 參數（wd/lam0/s0/容差/最大迭代）透傳給內層 solver |
| `export_unified(prefix, level)` | 對外唯一 CSV 匯出入口：`level=1` 匯出 6 個標配檔，`level=2` 額外匯出 3 個完整診斷檔 |

### 4-2. `newton_solver`（第 1 層：純 Lagrangian 全維度 Newton 求解器）

| 函式 | 說明 |
|------|------|
| `NewtonSolver(X, robotA_base, robotB_base, danger_threshold, path_weight, smooth_w...)` | 建構子：接收 (P+2)×12 控制點矩陣與機器人/問題參數，計算維度 `M_/num_D_/num_X_/num_C_/total_dim_`，並呼叫 `rebuild_initial_V_()` 建 `V_0=[X_0;λ_0;S_0]` |
| `run_newton()` | 對外主要入口：跑全維度 Newton 迭代（組 9-block KKT Hessian、LDLT 求解、α=1 純步）直到 `phys_ok && stable_ok && stat_ok` 或達 `lag_max_iter`，回傳 `SolverLog` |
| `get_X_final()` / `get_V_final()` | 取回結果的 X 段（36 維，給外層 reshape）與完整 V（1116 維）|
| `set_lag_params(wd, lam0, s0, ...)` | 注入 yaml 傳入的純 Lagrangian 參數，並呼叫 `rebuild_initial_V_()` 以新初值重建 V_0 |
| `rebuild_initial_V_()` | 以目前 `lam0_`/`s0_`/X 初值重建 `V_0`（建構子與 `set_lag_params` 共用）|
| `make_rotation(axis, angle_deg)` / `make_translation(axis, dist_mm)` | 靜態：建立 4×4 齊次旋轉 / 平移矩陣 |
| `calc_df(R1, R2, P1, P2)` | 靜態：計算兩組包覆球兩兩之間的危險因子矩陣 `sj = exp(ln0.5/(Ri+Rj)²·d²)` |
| `get_collision_masks()` | 靜態：回傳 16×18 的跨臂碰撞遮罩 |
| `robot_arm_bubble_RA610(...)` / `robot_arm_bubble_RA605(...)` | 靜態：正向運動學，由關節角算出包覆球球心座標、半徑與末端變換 |
| `idx_Xam/idx_Xbm/idx_Xm/idx_lam/idx_Sm/idx_lam_local` | 索引輔助：算 X/λ/S 各分量在完整向量 V 中的起始位置 |
| `compute_Dm(X, m)` / `compute_Dx_all(X)` | 算單點 / 全點的危險因子向量 |
| `compute_D_cache(V, D_base, D_plus, D_minus)` | 中心差分快取：算 D 的基準值與正負向擾動值，供梯度與 Hessian 共用 |
| `compute_G_smooth(V)` | 平滑項對 X 的梯度 `∇_X f` |
| `compute_G(V, D_base, D_plus)` | 完整 KKT 一階殘差 `G=[G_X;G_λ;G_S]`（`total_dim_` 維）|
| `cost_function_L(V, Dx_all_out)` | Lagrangian 值 `L(V)=f(X)+w_d·λᵀ·(D-θ+S²)` |
| `cost_function_F(V)` / `cost_function_F_split(V, f, fa, fb)` | 平滑總成本，及拆解為 A/B 臂分量 |
| `cost_Xm(Xa, Xori, XH, XT)` | 單一控制點相對平滑參考的成本 |
| `compute_H_smooth(V)` | 平滑項 Hessian `∇²_X f`（全 `num_X×num_X` 稠密有限差分）|
| `weighted_D_sum(Xm_local, lam_m)` | `w_d·Σλ⊙D(Xm_local)`，供 Hessian 非對角雙擾動使用 |
| `compute_H(V, D_base, D_plus, D_minus)` | 組出完整 1116×1116 的 9-block KKT Hessian，供 `run_newton` 解 `d=-H⁻¹G` |

### 4-3. `data_io`（CSV 寫入工具）

| 函式 | 說明 |
|------|------|
| `write_csv(path, header, mat)` | 寫純數值矩陣＋表頭到 CSV |
| `write_csv_labeled(path, header, row_labels, mat)` | 寫「首欄為字串標籤＋其餘為數值」的表 |

### 4-4. `planner_manager`（第 3 層：MoveIt2 介面）

| 函式 | 說明 |
|------|------|
| `DualArmPlanningContext::solve(res)` | MoveIt2 規劃入口：radian↔degree 邊界轉換、呼叫 `AvoidanceSystem`、轉回 `RobotTrajectory` 並依 `time_optimal_` 做時間參數化 |
| `DualArmLagNewtonPlannerManager::initialize(model, ns)` | 插件初始化：從 node 讀取 yaml 參數存入成員 |
| `getPlanningAlgorithms / setPlannerConfigurations / canServiceRequest` | pluginlib 標準介面：回報可用演算法、接受規劃器設定、判斷是否能服務某請求 |

---

## 摘要：常調的

| 場景 | 參數 | 在哪 |
|------|------|------|
| 改避障鬆緊 | `danger_threshold` | ✅ yaml |
| 改 A/B 臂優先 | `path_weight` | ✅ yaml |
| 改軌跡平滑度 | `smooth_w` 系列 | ✅ yaml |
| 改 λ/S 初值 | `lag_lam0` / `lag_s0` | ✅ yaml |
| 改收斂容差 | `lag_tol_phys_margin` / `lag_tol_stable` | ✅ yaml |
| 改主迴圈上限 | `lag_max_iter` | ✅ yaml |
| 改線搜索 / 危險因子 / 幾何 | — | ✗ `newton_solver.cpp` 重編 |
