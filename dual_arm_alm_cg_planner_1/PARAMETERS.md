# 參數對照表 — dual_arm_alm_cg_planner_1

本文件列出所有參數：外部可調 vs 寫死、預設值、所在位置。

---

## 一、外部輸入（不用改程式碼）

### 1-1. 規劃請求 / 命令列

| 參數 | 來源 | 預設 | 說明 |
|------|------|------|------|
| 起點關節角 | MoveIt `start_state` / standalone | — | A/B 臂各 6 軸（介面用 radian，內部 degree）|
| 終點關節角 | MoveIt `goal_constraints` / standalone | — | A/B 臂各 6 軸 |
| CSV filename | standalone 命令列 | `standalone_out` | 輸出資料夾/前綴 |

### 1-2. yaml 可調參數（共 11 個，改 yaml 重啟 move_group 生效）

| 參數 | 預設 | 對應成員 | 說明 |
|------|------|---------|------|
| `path_weight` | 0.5 | A/B 手臂避障權重 `pw·fA+(1−pw)·fB` |
| `danger_threshold` | 0.4 | 危險因子閾值 |
| `collision_tolerance` | 0.1 | 碰撞判定緩衝帶 （此值＝ 判撞門檻（0.5） - 危險因子） |
| `fix_tolerance` | 0.1 | fix_tolerance_ | find_targets 的 fix_gap 比例 |
| `max_refinement_iter` | 15 | max_refinement_iter_ | 外層修復最多輪數 |
| `smooth_w` | 0.3 | smooth_w_ | 平滑項主權重 |
| `smooth_w_H` | 1.0 | smooth_w_H_ | Head 端權重 |
| `smooth_w_T` | 1.0 | smooth_w_T_ | Tail 端權重 |
| `smooth_w_neighbor` | 1.0 | smooth_w_neighbor_ | 鄰點權重 |
| `joint_prefix_A` | "big_joint_" | joint_prefix_A_ | A 臂關節名前綴（依 SRDF）|
| `joint_prefix_B` | "small_joint_" | joint_prefix_B_ | B 臂關節名前綴（依 SRDF）|
| `time_optimal` | true | time_optimal_ | true=TOTG 時間最佳化; false=自訂等間隔 |
| `path_total_time` | 5.0 | path_total_time_ | （time_optimal=false 時）目標軌跡總時間（秒）|
| `min_time_interval` | 0.05 | min_time_interval_ | （time_optimal=false 時）每點最小時間間隔（秒）|

> 傳遞路徑：yaml → planner_manager（initialize 讀入）→ AvoidanceSystem 建構函數 → CgSolver 建構函數（smooth_w 系列）。standalone/test 用 4 參數呼叫仍相容（新參數有預設值）。
> 其餘參數（ALM 內部、機器人幾何等）皆寫死，需改原始碼重編。

### 1-3. 時間參數化（在插件 solve() 內處理）

時間參數化把幾何路徑（一串關節角）加上時間軸，決定機器人執行時每點之間隔多久。

- `time_optimal: true` → 用 **TOTG**，依關節速度/加速度限制算時間戳。需在 joint_limits.yaml 設加速度限制，否則用預設 1 rad/s²（會印警告）。
- `time_optimal: false` → **自訂等間隔**：`dt = path_total_time / (點數-1)`，但 `dt` 不小於 `min_time_interval`。若點數多到算出的 dt < min_time_interval，則強制用 min_time_interval（此時實際總時間會超過 path_total_time）。

> ⚠️ **重要**：因時間參數化改在插件內做，必須從 MoveIt pipeline 的 `request_adapters` **移除** `default_planner_request_adapters/AddTimeOptimalParameterization`，否則 adapter 會再跑一次 TOTG，覆蓋 `time_optimal=false` 的等間隔設定。

### 1-4. 三個時間輸出（solve() 印出，意義不同）

| 輸出 | 量什麼 | 單位 | 用途 |
|------|--------|------|------|
| `[純路徑規劃時間]` | 避障 run_optimization + 軌跡轉換（不含時間參數化）| 秒 | 看避障演算法快慢 |
| `[軌跡規劃時長]` | 純路徑規劃 + 時間參數化**計算**耗時（= 回報給 MoveIt 的 planning_time）| 秒 | 看整個生成軌跡花多久（電腦計算）|
| `[軌跡時長]` | 機器人**執行**這條軌跡要花的時間（getDuration）| 秒 | 看機器人動作多久 |

> 注意「軌跡規劃時長」是電腦**計算**花的時間（通常零點零幾秒），「軌跡時長」是機器人**執行**花的時間（數秒），兩者完全不同。

---

## 二、外層常數（avoidance_system）

寫死於 `src/avoidance_system.cpp` 建構函數 / `include/.../avoidance_system.hpp` 成員。
標 ✅ 者已開放為 yaml 可調（見第一節），其餘寫死。

| 參數 | 值 | yaml? | 說明 |
|------|-----|-------|------|
| `collision_tolerance_` | 0.1 | ✅ | 碰撞判定緩衝（判撞門檻 = 0.4+0.1 = 0.5）|
| `fix_tolerance_` | 0.1 | ✅ | find_targets 的 fix_gap 比例 |
| `max_refinement_iter_` | 15 | ✅ | 外層修復最多輪數 |
| `danger_threshold_` | 0.4 | ✅ | 危險因子閾值 |
| `path_weight_` | 0.5 | ✅ | A/B 臂成本權重 |
| `STEP_MAX_DEG_` | 0.5 | ✗ 寫死 | 軌跡切分最大角度差（度）|
| robotA_base | Ty(700)·Rz(180°) | ✗ 寫死 | A 臂底座變換 |
| robotB_base | Ty(-700)·Rz(0°) | ✗ 寫死 | B 臂底座變換（兩臂相距 1400mm 面對面）|

---

## 三、內層 ALM 參數（cg_solver）

寫死於 `include/.../cg_solver.hpp` 成員預設值。

| 參數 | 值 | 位置 | 說明 |
|------|-----|------|------|
| `mu_`（mu_0）| 10.0 | cpp:290 | 初始拉格朗日乘子（dual warm-start）|
| `c_`（c_0）| 5.0 | hpp:146 | 初始罰參數 |
| `c_max_` | 1e5 | hpp:147 | 罰參數上限 |
| `mu_max_safeguard_` | 1e8 | hpp:148 | 乘子安全上限（超出則 reset 0）|
| `beta_c_` | 8.0 | hpp:149 | 罰參數放大倍率 |
| `gamma_v_` | 0.5 | hpp:150 | violation 改善門檻 |
| `epsilon_v_` | 0.01 | hpp:151 | 主可行性收斂閾值 |
| `epsilon_g_` | 0.1 | hpp:152 | 一階最佳（梯度）收斂閾值 |
| `epsilon_compl_` | 0.1 | hpp:153 | 互補性收斂閾值 |
| `epsilon_inner_` | 1.0 | hpp:154 | 內層精度（起始）|
| `epsilon_inner_min_` | 0.1 | hpp:155 | 內層精度下限 |
| `epsilon_inner_decay_` | 0.5 | hpp:156 | 內層精度遞減率 |
| `K_outer_` | 50 | hpp:157 | 外層 ALM 最大輪數 |
| `K_inner_` | 200 | hpp:158 | 內層 CG 最大迭代 |
| `K_inner_first_` | 200 | hpp:159 | 首輪內層最大迭代 |

> KKT 三條件收斂（須同時）：v_pure ≤ eps_v、‖grad L‖ ≤ eps_g、compl ≤ eps_compl。

---

## 四、平滑項權重（cg_solver）

**已開放為 yaml 可調**（見第一節）。控制軌跡平滑度。
傳遞：yaml → AvoidanceSystem → CgSolver 建構函數。

| 參數 | 值 | yaml? | 說明 |
|------|-----|-------|------|
| `smooth_w_` | 0.3 | ✅ | 平滑項主權重 |
| `smooth_w_H_` | 1.0 | ✅ | Head 端權重 |
| `smooth_w_T_` | 1.0 | ✅ | Tail 端權重 |
| `smooth_w_neighbor_` | 1.0 | hpp:138 | 鄰點權重 |

---

## 五、線搜索與數值保護（cg_solver）

寫死於 `src/cg_solver.cpp` run_alm 內。

| 參數 | 值 | 位置 | 說明 |
|------|-----|------|------|
| `LS_DELTA` | 0.001 | cpp:464 | 1D Newton 線搜索差分步長 |
| `FALLBACK_ALPHA` | 0.0001 | cpp:468 | 線搜索失敗時退路步長 |
| `FAIL_CHECK_IT` | 10 | cpp:469 | 失敗檢查迭代數 |
| 發散門檻 | ‖G‖ > 1e10 | cpp:528 | 梯度爆炸視為發散 |
| 梯度停擺 | pre_G_norm² < 1e-18 | cpp:537 | CG 方向退化保護 |

---

## 六、危險因子公式（cg_solver）

寫死於 `src/cg_solver.cpp` `calc_df()`。**核心避障數學**。

| 參數 | 值 | 位置 | 說明 |
|------|-----|------|------|
| 危險因子公式 | `sj = exp(ln(0.5)/(Ri+Rj)²·d²)` | cpp:116 | 球對危險因子（d=球心距，R=半徑）|
| `ln(0.5)` | -0.6931 | cpp:107 | 衰減係數（球心距 = Ri+Rj 時 sj=0.5）|

---

## 七、Spline 重建數值閾值（avoidance_system）

寫死於 `src/avoidance_system.cpp`。

| 參數 | 值 | 位置 | 說明 |
|------|-----|------|------|
| 距離防除零 | 1e-4 → 1e-6 | cpp:138 | 初始 spline 弦長過小時的下限 |
| 重合過濾閾值 | 1e-4 | cpp:354 | regenerate 的 valid_mask（段距離 > 1e-4 才有效）|
| 退化 t_knots | 1e-6 | cpp:366 | anchor 全重合時的退化 spline 參數 |

---

## 八、機器人幾何（cg_solver）

寫死於 `src/cg_solver.cpp`。換機器人需整批修改。

| 項目 | 內容 | 位置 |
|------|------|------|
| RA610 (A 臂) 底盤球 | 4 球，R=320 | cpp:19 `PEDESTAL_A` |
| RA610 手臂球 | 12 球 | cpp:27 `BUBBLES_A` |
| RA605 (B 臂) 底盤球 | 8 球，R=350 | cpp:43 `PEDESTAL_B` |
| RA605 手臂球 | 10 球 | `BUBBLES_B` |
| 球→連桿映射 | sA[16]、sB[18] | cpp:129-130 |
| RA610 FK chain | 7 連桿 DH 變換 | `robot_arm_bubble_RA610` |
| RA605 FK chain | 7 連桿 DH 變換 | `robot_arm_bubble_RA605` |

> 球總數：A 臂 16（4 底盤 + 12 手臂）、B 臂 18（8 底盤 + 10 手臂）。

---

## 九、決策變數維度（cg_solver，執行期決定）

| 參數 | 值（本案）| 說明 |
|------|----------|------|
| M（中間優化點）| 3 | 不含 Head/Tail |
| num_X | 36 = 2×3×6 | 2 臂 × 3 點 × 6 軸 |
| K_AB（跨臂約束）| 180 | 16×18 經 mask 過濾後 |
| num_C | 540 = 3 × 180 | M × num_D |
| self_collision | false | 本版不含自我碰撞（num_D = K_AB）|

---

## 十、函數說明

### 10-1. `avoidance_system`（第 2 層：外層碰撞修復，純數學，不依賴 MoveIt）

| 函式 | 說明 |
|------|------|
| `AvoidanceSystem(A_waypoints, B_waypoints, path_weight, danger_threshold, ...)` | 建構子：接收 A/B 臂各 2×6 起訖關節角與可調參數，初始化狀態 |
| `run_optimization()` | 對外主要入口：跑「生成軌跡 → 偵測碰撞 → 找危險段 → 呼叫內層優化 → Spline 重建 → 重新檢查」完整迴圈，最多 `max_refinement_iter_` 輪 |
| `generate_initial_trajectory()` | 用 clamped cubic spline 由起訖點生成初始整段軌跡 |
| `check_collision(traj, path_D_max, is_collision, path_D_all)` | 逐步呼叫 `calc_df` 算危險因子，得到每步 `max_D` 與是否超過 `danger_threshold_` |
| `find_collision_targets(path_D)` | 在超過閾值的區段找出 5 個控制點索引：`[Head, q1, peak, q3, Tail]` |
| `run_solver_global(traj, targets, Xa_opt, Xb_opt, solver_log)` | 用 5 點中的 3 個內部點組成決策變數 X0，建立並呼叫 `CgSolver::run_alm()` |
| `regenerate_trajectory_global(old_traj, Xa_opt, Xb_opt, indices, targets_out)` | 用優化後的 3 個內部點做局部 clamped spline，重建整段軌跡 |
| `clamped_cubic_spline(t_knots, Y, v0, v1, t_query)` | 靜態工具：給節點時間、值、端點斜率，解三對角系統求二階導後做分段三次插值 |
| `set_solver_verbose(v)` / `set_alm_params(...)` | 將 verbose 開關與 ALM 排程參數（mu0/c0/c_max/beta_c/gamma_v）透傳給內層 solver |
| `export_unified(prefix, level)` | 對外唯一 CSV 匯出入口：`level=1` 匯出 6 個標配檔，`level=2` 額外匯出 3 個完整診斷檔 |

### 10-2. `cg_solver`（第 1 層：內層 ALM + CG-FR 求解器）

| 函式 | 說明 |
|------|------|
| `CgSolver(X, robotA_base, robotB_base, danger_threshold, path_weight, smooth_w...)` | 建構子：接收 (P+2)×12 控制點矩陣與機器人/問題參數，計算維度 `M_/num_D_/num_X_/num_C_` |
| `run_alm()` | 對外主要入口：跑 ALM 外層（罰參數/乘子排程）+ CG-FR 內層（方向 `d=-G+β·d_prev`，1D Newton 線搜索）直到收斂/發散/達上限，回傳 `SolverLog` |
| `make_rotation(axis, angle_deg)` / `make_translation(axis, dist_mm)` | 靜態：建立 4×4 齊次旋轉 / 平移矩陣 |
| `calc_df(R1, R2, P1, P2)` | 靜態：計算兩組包覆球（半徑 R、球心 P）兩兩之間的危險因子矩陣 `sj = exp(ln0.5/(Ri+Rj)²·d²)` |
| `get_collision_masks()` | 靜態：回傳 16×18 的跨臂碰撞遮罩，決定哪些球對要納入約束 |
| `robot_arm_bubble_RA610(T_base, J, bubble, r, T_ee)` / `robot_arm_bubble_RA605(...)` | 靜態：正向運動學，由 6 軸關節角算出各包覆球球心座標、半徑與末端 4×4 變換 |
| `compute_Dm(X, m)` | 算第 m 個控制點的危險因子向量（`num_D_` 維）|
| `compute_Dx_all(X)` | 算所有控制點的危險因子，拼成 `num_C_` 維向量 |
| `compute_D_cache(X, D_base, D_plus)` | 中心差分快取：算 D 的基準值與正向擾動值，供梯度共用（CG 不需 Hessian，故只需正向擾動）|
| `compute_G_smooth(X)` | 平滑項對 X 的梯度（`num_X_` 維）|
| `compute_G_c(X, D_base, D_plus, mu_loc, c_loc)` | 完整梯度 `G = G_smooth + 罰項梯度`，即 CG 方向要驅近 0 的量 |
| `cost_function_F(X)` / `cost_function_F_split(X, f, fa, fb)` | 平滑總成本，及拆解為 A/B 臂分量 |
| `cost_Xm(Xa, Xori, XH, XT)` | 單一控制點相對平滑參考的成本 |
| `cost_L_loc(X, mu_loc, c_loc)` | 局部 Lagrangian 值（成本 + ALM 罰項），供線搜索評估用 |
| `line_search_newton_1d(X, d, mu_loc, c_loc)` | 沿方向 `d` 做 1D Newton 線搜索求步長 `alpha`（`LS_DELTA=0.001` 差分估一階/二階導）|

### 10-3. `data_io`（CSV 寫入工具）

| 函式 | 說明 |
|------|------|
| `write_csv(path, header, mat)` | 寫純數值矩陣＋表頭到 CSV |
| `write_csv_labeled(path, header, row_labels, mat)` | 寫「首欄為字串標籤＋其餘為數值」的表（例如 Targets 表）|

### 10-4. `planner_manager`（第 3 層：MoveIt2 介面）

| 函式 | 說明 |
|------|------|
| `DualArmPlanningContext::solve(res)` | MoveIt2 規劃入口：radian↔degree 邊界轉換、呼叫 `AvoidanceSystem`、轉回 `RobotTrajectory` 並依 `time_optimal_` 做時間參數化 |
| `DualArmAlmCgPlannerManager::initialize(model, ns)` | 插件初始化：從 node 讀取 yaml 參數存入成員 |
| `getPlanningAlgorithms / setPlannerConfigurations / canServiceRequest` | pluginlib 標準介面：回報可用演算法、接受規劃器設定、判斷是否能服務某請求 |

---

## 十一、關節名（planner_manager）

**已開放為 yaml 可調**（`joint_prefix_A` / `joint_prefix_B`，見第一節）。
程式用 `prefix + 數字` 組成關節名，改 yaml 即可適配不同 SRDF，不用重編。

| 參數 | yaml 值 | 組成的關節名 | 說明 |
|------|---------|------------|------|
| `joint_prefix_A` | "big_joint_" | `big_joint_1` ~ `big_joint_6` | A 臂，須與 SRDF 一致 |
| `joint_prefix_B` | "small_joint_" | `small_joint_1` ~ `small_joint_6` | B 臂，須與 SRDF 一致 |

---

## 摘要：真正常需調整的

| 場景 | 要調的參數 | 在哪 |
|------|-----------|------|
| 改避障鬆緊 | `danger_threshold` | ✅ yaml（不用重編）|
| 改 A/B 臂優先 | `path_weight` | ✅ yaml |
| 改軌跡平滑度 | `smooth_w` 系列 | ✅ yaml |
| 改修復輪數 | `max_refinement_iter` | ✅ yaml |
| 改碰撞緩衝 | `collision_tolerance` | ✅ yaml |
| 換機器人關節名 | `joint_prefix_A/B` | ✅ yaml |
| 切換時間參數化方式 | `time_optimal` | ✅ yaml |
| 自訂軌跡總時間 | `path_total_time` / `min_time_interval` | ✅ yaml |
| 改軌跡密度 | `STEP_MAX_DEG_` | ✗ avoidance_system.hpp（重編）|
| 改收斂嚴格度 | `epsilon_v/g/compl` | ✗ cg_solver.hpp（重編）|
| 改 ALM 收斂行為 | `c_0/mu_0/beta_c` | ✗ cg_solver.hpp（重編，進階）|
| 換機器人幾何 | 包覆球 + FK + base | ✗ cg_solver.cpp + planner_manager.cpp（重編）|

> ✅ = yaml 可調（改 yaml 重啟 move_group 生效）；✗ = 寫死，需改原始碼重編。
> 目前共 **14 個參數**開放為 yaml 可調。



## 診斷輸出參數（yaml, 運行期動態, [NEW]）

| 參數 | 預設 | 說明 |
|------|------|------|
| `export_csv_prefix` | `""`（關） | 非空 → 每次規劃後匯出 CSV 三組（`<prefix>_<unix秒>_danger.csv`、ALM 診斷、軌跡繪圖）；規劃失敗也匯出 |
| `solver_verbose` | `false` | 內層逐迭代記錄 G/d 全向量（耗記憶體；深掘/論文用） |

### [NEW] export_unified 整合匯出（取代插件路徑的舊三組匯出）

| 參數 | 預設 | 說明 |
|------|------|------|
| `export_level` | 1 | **0=完全不匯出（總開關）**；1=論文標配 6 檔；2=+constraints_all / path_original / path_evolution 共 9 檔 |

輸出目錄：`<export_csv_prefix>/<unix秒>_<solver>/`，檔案數固定不隨修復輪數膨脹（長表 + round 欄設計）。舊四匯出器已刪除（standalone 亦改走 export_unified level 2）。

## ALM 參數（yaml, 運行期動態, [NEW] — 原等級 3 硬編碼開放為參數）

| 參數 | CG 預設 | 說明 |
|------|------|------|
| `alm_mu0` | 10 | 乘子 warm-start 初值（mu_ 向量以此重建）；⚠ 勿超過 1e8（程式內固定的 mu_max_safeguard）|
| `alm_c0` | 5 | 罰參數初值；⚠ 勿超過 `alm_c_max` |
| `alm_c_max` | 100000 | 罰參數上限 |
| `alm_beta_c` | 8 | 罰參數放大係數 |
| `alm_gamma_v` | 0.5 | 放大觸發比（程式內箝位至 (0,1)，非法值回落 0.5）|

五參數全數寫入每次規劃的 meta.csv（可追溯性）。`mu_max_safeguard` 為 Birgin 理論保護值，固定 1e8 於 solver 內，不開放調整。發散急救順序：先降 `alm_beta_c`，再降 `alm_c_max`，再動 `alm_c0`。
