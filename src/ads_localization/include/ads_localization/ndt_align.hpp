// Copyright 2026 孙帅
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ADS_LOCALIZATION__NDT_ALIGN_HPP_
#define ADS_LOCALIZATION__NDT_ALIGN_HPP_

// =============================================================================
//  NDT 配准：把一帧扫描对齐到体素高斯地图上
//
//  ## 代价函数
//
//  扫描点 p 经位姿 T 变换到 map 系得 x = T·p，落进某个体素后与那份高斯比：
//
//      e = x − μ                      残差
//      q = eᵀ Σ⁻¹ e                   马氏距离平方
//      s = −exp(−q/2)                 该点的得分（**越小越好**）
//
//  总代价是所有点的和。落不进任何体素的点**直接跳过**（不是记 0 分）——
//  草地上的实测点在地图里本来就没有对应，硬给它们一个分数等于让地图的
//  「洞」去拉扯位姿。跳过的代价是 inlier_ratio 会下降，那个量因此必须被报出来。
//
//  ## 位姿的参数化：增量式，不是欧拉角
//
//  优化变量是 6 维增量 ξ = (δt, δθ)，作用在**当前估计**上：
//
//      x(ξ) = R̂ · Exp(δθ) · p + (t̂ + δt)
//
//  每次迭代把解出来的 ξ 乘进 T̂ 再把 ξ 清零 —— 与 ESKF 的「注入 + 重置」
//  是同一个套路，理由也一样：**旋转活在流形上**，用欧拉角参数化会在
//  pitch = ±90° 处万向节死锁，而且雅可比里会出现 1/cos(pitch)。
//
//  于是雅可比非常简单（在 ξ = 0 处求）：
//
//      ∂x/∂δt = I               ∂x/∂δθ = −R̂ [p]×
//
//  ## Hessian 用 Gauss-Newton 近似，这不只是图省事
//
//      H ≈ Σ w · Jᵀ Σ⁻¹ J        （w = exp(−q/2)）
//
//  完整的 Hessian 还有一项 `−(JᵀΣ⁻¹e)(eᵀΣ⁻¹J)`，它是**负半定**的，
//  会让 H 在残差大的时候变成不定矩阵 —— 那样牛顿方向可能是上坡的。
//
//  丢掉它有两个好处，第二个才是关键的：
//    1. H 结构上就是半正定的，`−H⁻¹g` 一定是下降方向；
//    2. **`Σ w JᵀΣ⁻¹J` 恰好就是 Fisher 信息阵** —— 也就是这一片要输出给
//       ESKF 的那个协方差的正确对象。用完整 Hessian 反而拿不到它。
//
//  ⚠️ 解析梯度有专门的用例与**数值微分逐项对账**。符号写反的症状是
//     「配准朝反方向跑然后卡住」，看起来像收敛域太小 —— 而根因在一个负号。
//
//  ## 退化检测：看**法向散布**，不看信息阵条件数
//
//  P4-1 决策一的原话：「点云对位姿的约束来自表面法向，一个平面只约束沿它
//  法向的那一个自由度」。所以几何退化的判据就是**匹配上的体素法向是否平行**：
//
//      N = Σ w · n nᵀ        （n = 体素协方差最小特征值对应的特征向量）
//      normal_diversity = λ_min(N) / λ_max(N)
//
//  ⚠️ **不要用信息阵 H 的条件数当退化判据** —— 这是走过一次弯路的结论。
//     2026-08-10 实测：纯地面点云上 H 报出 σ_x = 8.7 mm，也就是"在一个
//     无特征平面上把 x 定到了 9 毫米"，而条件数只比结构齐全的情形差 3 倍。
//     原因是**体素离散化伪造了面内信息**：每个体素的高斯均值落在该格质心，
//     于是整张地图在面内构成一个 2 m 点阵，扫描平移时会撞上纹波。
//     那个信息相对**离散化后的地图**是真的，相对**物理世界**是假的 ——
//     而 NDT 会老老实实地收敛到纹波的某个谷底，给出一个看起来很准的错位姿。
//
//     换成法向散布之后，两种情形差 **6 个数量级**（3e-8 vs 0.1）。
//
//  完整推导与参数见 docs/modules/localization.md。**改这个文件前先读它。**
// =============================================================================

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>

#include "ads_localization/ndt.hpp"

namespace ads_localization
{

/// 配准参数。
struct NdtAlignParams
{
  /// 牛顿迭代上限。到了还没收敛就报 converged = false。
  int max_iterations{30};

  /// 收敛判据：一步里的平移与旋转都小于它才算收敛。
  double translation_epsilon_m{1e-3};
  double rotation_epsilon_rad{1e-4};

  /// 单步平移的上限，m。防止病态的 H 解出一个把扫描甩到几百米外的步长。
  /// 调小 → 更稳但迭代次数上升；调大 → 病态时可能一步跳出收敛域。
  double max_step_m{1.0};

  /// Levenberg 阻尼，加在 H 的对角上。纯粹是数值保险，
  /// 让 H 在极端情形下仍然可逆。调大会让步长变短（趋向梯度下降）。
  double levenberg_damping{1e-6};

  /// 回溯线搜索最多折半几次。
  ///
  /// **这是控制单帧耗时的主要杠杆** —— 每折半一次就是一次**全量**打分
  /// （遍历所有扫描点 × 27 个邻域体素），所以最坏情况是
  /// `max_iterations × max_line_search_trials` 次全量打分。
  ///
  /// 折了这么多次还不下降，说明已经在极小值附近了（代码把这种情形
  /// 当作收敛）。取 4 意味着最小步长是全步的 1/16。
  int max_line_search_trials{4};

  /// **退化判据：匹配上的体素法向的散布程度**（见 NdtAlignResult::normal_diversity）。
  ///
  /// ⚠️ 这里**不用信息阵的条件数**，那是走过一次弯路之后的结论：
  ///    实测纯地面点云上信息阵报出 σ_x = 8.7 mm，条件数只比结构齐全的情形
  ///    差 3 倍 —— 因为体素离散化会伪造面内信息（voxel 均值构成 2 m 点阵）。
  ///    换成法向散布之后两者差 6 个数量级。
  ///
  /// 取 1e-3：纯地面实测 3e-8、结构齐全 0.1，判据落在中间且两侧余量都
  /// 有好几个数量级 —— 不会 flake。
  double min_normal_diversity{1e-3};

  /// 信息阵最小特征值的绝对下限。只作为数值保险（H 病态到解不出来时兜底），
  /// **不承担几何退化判定**。
  double min_absolute_eigenvalue{1e-6};

  /// 落进体素的扫描点占比下限。低于它说明扫描与地图基本没有重叠，
  /// 此时哪怕 H 条件数正常，结果也不可信。
  double min_inlier_ratio{0.2};

  /// 输出协方差 = `covariance_scale · H⁻¹`。
  ///
  /// ⚠️ **这是一个经验标定量，不是从第一性原理推出来的。**
  /// 本实现的得分函数不是严格归一化的对数似然，所以 `H⁻¹` 只与真实协方差
  /// **成正比**。一个没标定的协方差喂进 ESKF 的后果是权重错，而那**不会报错**。
  ///
  /// **已于 2026-08-11 标定**（`scripts/calibrate_ndt_covariance.py`，NEES 法）：
  /// 实测中位数 4.331 / 5.175（两轮）对理论期望 6 ⟹ scale ≈ 0.72 / 0.86。
  /// **节点侧默认值取 0.8**（`ndt.covariance_scale`）。
  ///
  /// ⚠️ 这里的**库默认值仍是 1.0，那是有意的**：库不该假设自己跑在哪张地图、
  /// 哪种雷达上，而标定结果只对「本项目这张 campus 地图 + 这个 32 线雷达」成立。
  /// 换地图或换传感器必须重标 —— 所以标定值属于**部署侧参数**，不属于算法默认值。
  ///
  /// ⚠️ 标定**没有**解决那条重尾：实测 NEES 均值 10.5 / 11.8、95 分位 48 / 52
  /// （理论值的 8 倍）。那是「NDT 报告成功但其实锁偏了」的直接证据，
  /// 要靠**严格卡方新息门限**拦（尚未实现）。标定只是给了它可用的前提。
  double covariance_scale{1.0};

  void Validate() const;
};

/// 配准结果。
struct NdtAlignResult
{
  /// 估计出来的位姿（map ← body）。
  ///
  /// ⚠️ **`degenerate` 为真时不要用它。** 那种情况下代价函数沿某些方向是平的，
  ///    这个位姿在那些方向上是任意的 —— 而它看起来和一个好位姿毫无区别。
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};

  bool converged{false};
  /// 信息阵秩亏 / 重叠太少 —— 结果不可用。
  bool degenerate{false};

  int iterations{0};
  /// 最终代价（越小越好，恒 ≤ 0）。
  double score{0.0};
  /// 落进某个体素的扫描点占比。
  double inlier_ratio{0.0};

  /// Fisher 信息阵（6×6，顺序 = 平移 xyz、旋转 xyz）。
  Eigen::Matrix<double, 6, 6> information{Eigen::Matrix<double, 6, 6>::Zero()};
  /// `covariance_scale · H⁻¹`。见 NdtAlignParams::covariance_scale 的警告。
  Eigen::Matrix<double, 6, 6> covariance{Eigen::Matrix<double, 6, 6>::Zero()};

  /// 诊断量：信息阵的最小特征值与条件数。
  double smallest_eigenvalue{0.0};
  double condition_number{0.0};

  /// **几何退化度**：匹配上的体素法向散布矩阵 `Σ w·nnᵀ` 的 λ_min/λ_max。
  ///
  /// 这才是退化判据的依据，条件数不是。取值 0（所有法向平行 → 只约束一个
  /// 自由度）到 1/3（法向各向同性 → 三个方向都有约束）。
  /// 实测：纯地面 3e-8 量级，结构齐全的园区地图 0.1 量级 —— **差 6 个数量级**，
  /// 而信息阵条件数在两种情形下只差 3 倍。
  double normal_diversity{0.0};
};

/// 把扫描配准到地图上。
///
/// @param map           体素高斯地图。
/// @param scan_body     扫描点，**body 系**（= base_link）。
/// @param initial_guess 初值位姿（map ← body）。本项目由 ESKF 的预测位姿提供。
/// @param params        配准参数。
/// @throws std::invalid_argument 参数非法、扫描为空、或初值/扫描含非有限值。
NdtAlignResult AlignNdt(
  const NdtGrid & map, const std::vector<Eigen::Vector3d> & scan_body,
  const Eigen::Isometry3d & initial_guess, const NdtAlignParams & params);

/// 在给定位姿处求代价、梯度与（Gauss-Newton）信息阵。
///
/// 单独暴露出来是为了让测试能用**数值微分**对账解析梯度 ——
/// 那是消除符号错误最直接的办法，而符号错误的症状（配准朝反方向跑然后卡住）
/// 看起来像「收敛域太小」，会把人引到调参数上去。
struct NdtScoreTerms
{
  double score{0.0};
  Eigen::Matrix<double, 6, 1> gradient{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 6> information{Eigen::Matrix<double, 6, 6>::Zero()};
  /// 法向散布矩阵 `Σ w·n nᵀ`。几何退化判据的原料，见「退化检测」一节。
  Eigen::Matrix3d normal_scatter{Eigen::Matrix3d::Zero()};
  int inlier_count{0};
};

NdtScoreTerms ComputeNdtScoreTerms(
  const NdtGrid & map, const std::vector<Eigen::Vector3d> & scan_body,
  const Eigen::Isometry3d & pose);

/// 两个位姿之差的大小：平移的欧氏距离 + 相对旋转的转角。
struct PoseDelta
{
  double translation_m{0.0};
  /// 相对旋转 `aᵀb` 的转角，恒在 `[0, π]`。
  double rotation_rad{0.0};
};

/// 求两个位姿的差量 —— **新息门限的原料**。
///
/// 用途：把 NDT 的输出与滤波器的预测（也就是配准初值）比一比。差得离谱说明
/// NDT 锁到了错误的局部极小，那一帧必须丢掉。
///
/// ⚠️ **为什么需要这个门限**：NDT 到迭代上限仍会返回位姿，而它的协方差来自
/// 最终迭代点处的信息阵 —— 那个量反映「代价函数在这里有多陡」，
/// **不是「离真解有多远」**。半收敛的位姿带着毫米级协方差被喂进滤波器，
/// 把状态硬拽过去，下一帧初值更差…… 正反馈发散。
/// 实测（2026-08-10）：把 `max_iterations` 从 30 砍到 15，横向误差
/// 0.0664 m → **462 m**。也就是说在这个门限出现之前，挡住这个失效模式的
/// 是「迭代上限填得够大」这一个数 —— 一个看起来像调参旋钮的安全关键值。
///
/// ⚠️ 转角用 `acos((tr(R)−1)/2)` 而不是四元数分量：后者在接近 π 时符号会翻，
/// 给出一个接近 0 的假差值 —— 而「转了 180°」恰恰是最该被拦下的那种错误。
/// 数值上 `(tr(R)−1)/2` 可能因舍入略微越出 `[−1, 1]`，所以先夹再 acos。
///
/// @param a 位姿甲（本项目里是滤波器预测 / 配准初值）。
/// @param b 位姿乙（本项目里是 NDT 的输出）。
/// @return 平移距离（m）与转角（rad）。**输入含非有限值时原样返回非有限值**，
///         由调用方用 `isfinite` 拦 —— 见下。
///
/// ⚠️ 调用方**必须显式判有限性**，不能只写 `if (d.translation_m > limit) reject`。
/// `NaN` 参与任何比较都返回 `false`，于是那行代码对 NaN 恒为假、**原样放行**。
/// 这个坑本仓库已经咬过两次（`vehicle_cmd_bridge` 的指令限幅、
/// `ads_control` 的路径点距校验），所以这里写在接口注释里。
PoseDelta ComparePoses(const Eigen::Isometry3d & a, const Eigen::Isometry3d & b);

}  // namespace ads_localization

#endif  // ADS_LOCALIZATION__NDT_ALIGN_HPP_
