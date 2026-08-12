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

#ifndef ADS_PERCEPTION__TRACKER_HPP_
#define ADS_PERCEPTION__TRACKER_HPP_

// =============================================================================
//  多目标跟踪：恒速卡尔曼滤波 + 匈牙利关联 + 航迹生命周期
//
//  ## ⚠️ 它是**线性卡尔曼滤波**，不是 EKF
//
//  plan.md 里写的是「恒速 EKF」，但状态转移（匀速直线）与观测（直接测位置）
//  **都是线性的** —— 没有需要线性化的地方，所以 EKF 会退化成 KF。
//  这里如实叫它 KF。用 EKF 这个名字会让人以为存在非线性、去找雅可比。
//  （P4 的 ESKF 是真 EKF：姿态活在流形上。两者不是一回事。）
//
//      状态 x = (px, py, vx, vy)      观测 z = (px, py)
//      F = [[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]]      H = [I₂ 0]
//
//  ## ⚠️⚠️ 生命周期参数直接来自 S1 的实测，不是抄来的默认值
//
//  S1 体检发现：**目标在连续帧之间闪烁**（同一个锥桶连续三帧 8 点→8 点→0 点），
//  实测命中率只有 33–74%。这把生命周期从"锦上添花"变成了**必需品**，
//  也推翻了教科书上那个「连续 N 帧命中才确认」的写法：
//
//      p=0.5 时「连续 N 帧命中」的概率     N=2: 25%  N=3: 12.5%  N=4: 6.2%
//      ⟹ N=3 平均要等 8 次机会（0.8 s）才确认一个目标，而它可能只在
//        视野里待几秒。**这不是保守，是看不见。**
//
//  所以本实现：
//    · **确认用「累计命中数」**（不要求连续）—— p=0.5 时平均 6 帧达成 3 次命中
//    · **删除用「连续未命中」** —— 目标真的消失了才该删
//      p=0.5 时连续 5 帧未命中的概率 3.1%，那是可接受的误删率
//
//  ## ⚠️⚠️ 为什么直接滤「包围盒中心」是错的（2026-08-12 实测）
//
//  包围盒中心**不是目标身上的一个固定点** —— 它随观测几何漂移，因为雷达
//  只打得到朝向自己的那些面。CP-P5-B 实测（对向 NPC 车，真值 4.40×1.80）：
//
//      距离      沿视线位置误差   感知长度
//      0–10 m       0.004         4.38     ← 侧面露出来了，长度量得准
//      15–20 m      0.453         3.33
//      20–25 m      2.150         1.77     ← 只剩车头那一面
//      25–30 m      2.223         1.77       (1.77 ≈ 车**宽**，轴向已翻 90°)
//
//  误差**几乎全在视线方向上**（垂直分量中位数只有 0.026 m），88% 的样本
//  "感知比真值更近" —— 这是可见面伪影的三个签名。
//  **对照组**：行人（0.4 m，无面可遮）全程 0.047 m，同样的代码同样的距离。
//
//  后果不只是位置：中心一跳，卡尔曼就把它读成**速度**。实测最坏的几帧
//  在 3.6 m 处，相邻两帧长度 4.364 → 3.578，中心挪 0.4 m，
//  0.1 s 一帧 ⟹ **8.8 m/s 的假速度**，而真值只有 4.0 —— 连符号都反了。
//
//  所以本层做两件事：
//    ① **记住轴向一致时观测到的最大尺寸**（`Track::length_m/width_m`）；
//    ② 用它把每帧的中心**补全**到"整个目标的中心"再喂给滤波器
//       （`CompletedCenter`）—— 关联的门限也用补全后的位置，否则预测的是
//       补全中心、观测是原始中心，两者差 1 m 以上会**判不进门 ⟹ ID 跳变**。
//
//  ⚠️ 它**修不了**远处那个 2.2 m 的偏差：目标一直在远处时我们从没见过它的
//     侧面，没有可记的尺寸。那需要**形状先验**（"这是车 ⟹ 长 4.4 m"），
//     而先验又依赖分类、分类又依赖尺寸 —— 循环。如实记在这里，别再查一遍。
//
//  ## 180° 二义性在这里消歧
//
//  S3 的 L-Shape 只能给出**轴向**（`[0, π)`）—— 一帧点云里没有信息能区分
//  车头朝哪。本层有跨帧的速度估计，于是：
//
//      速度足够大  ⟹ 车头 = 速度方向（取与轴向差 < 90° 的那一个）
//      速度太小    ⟹ **仍然无解**，如实标 `heading_resolved = false`
//
//  ⚠️ 静止目标的朝向**永远定不下来**，这不是缺陷而是物理事实。
//     下游拿到 `heading_resolved = false` 时应当退回用轴向 + 不确定性，
//     而不是当成一个确定的朝向 —— 那会有 50% 的机会让 P6 预测出逆行轨迹。
// =============================================================================

#include <Eigen/Core>

#include <cstdint>
#include <vector>

namespace ads_perception
{

/// 跟踪参数。
struct TrackerParams
{
  /// 过程噪声：**加速度**的标准差，m/s²。
  ///
  /// 恒速模型假设目标不加速，而实际会 —— 这个量就是"允许它多不守规矩"。
  /// 取 2.0：园区目标（车 1.5 m/s²、行人变向）的量级。
  /// 调大 → 滤波器更信观测，速度估计跟得快但更抖；
  /// 调小 → 更平滑，但目标一变向就跟丢（新息持续超门限 → 关联失败 → 航迹删除）。
  double process_accel_stddev_mps2{2.0};

  /// 观测噪声：检测位置的标准差，m。
  ///
  /// 取 0.3：S2 实测**可见面**定位误差只有 0.003 m，但那是"面"的位置；
  /// 而检测给出的是包围盒中心，它随目标朝向与遮挡变化（雷达打不到背面），
  /// 帧间抖动远大于面的定位精度。0.3 是给这个抖动留的。
  /// 调小 → 滤波器过度相信每一帧的中心，速度估计被抖动放大；
  /// 调大 → 迟钝，真机动跟不上。
  double measurement_stddev_m{0.3};

  /// 关联门限：马氏距离平方的上限（卡方，2 自由度）。
  ///
  /// 取 9.21 = χ²(2, 99%)。**这次可以用卡方分布表**，因为这里的协方差
  /// 是滤波器自己按已知的 Q/R 递推出来的 —— 与 P4 那个未标定的 NDT 协方差
  /// 不同（那里只能用固定阈值，见 localization.md §10.6）。
  /// 调小 → 目标机动时关联失败，航迹断裂后重建 ⟹ **ID 跳变**；
  /// 调大 → 密集场景下配错对象，两个目标的 ID 互换。
  double gate_chi_square{9.21};

  /// 累计命中多少次才算「确认」。**不要求连续**，理由见文件头。
  ///
  /// 取 3：p=0.5 时平均 6 帧（0.6 s）达成。
  /// 调大到 5 → 平均 10 帧，快速穿过视野的目标可能到消失都没被确认；
  /// 调小到 1 → 一个噪点簇就能生成一条确认航迹（虚警）。
  int confirm_hits{3};

  /// 连续未命中多少帧就删除航迹。
  ///
  /// 取 5（0.5 s）：p=0.5 时误删概率 3.1%。
  /// 调小到 3 → 误删概率 12.5%，闪烁场景下 ID 会频繁跳变；
  /// 调大到 10 → 目标真的走了之后还留着"幽灵航迹"，而规划会绕它。
  int max_misses{5};

  /// 速度超过它才用来消歧朝向，m/s。
  ///
  /// 取 0.5：低于这个速度，速度方向由噪声主导 —— 拿噪声去定朝向
  /// 还不如老实说"不知道"。行人正常步速 1.2 m/s，所以走动的人能被消歧。
  double heading_min_speed_mps{0.5};

  /// 尺寸记忆：检测的轴向与航迹已知轴向的夹角**不超过它**时才合并尺寸，rad。
  ///
  /// ⚠️ 这道闸不是保守，是**必需**（2026-08-12 实测）：远处只看得见车头
  ///    那一面时，L-Shape 拟出的矩形是 1.76 × 0.08，而它会把较大的那个范围
  ///    叫做 `length` —— 于是"长轴"指的其实是**车宽**，轴向整个翻了 90°。
  ///    不判轴向一致就合并的话，会沿**错误的轴**把盒子外扩 1.3 m。
  ///    实测长/宽比：0–10 m 是 2.5（轴向稳），25–30 m 是 25.5（已翻）。
  ///
  /// 取 0.35 rad（20°）：比拟合噪声大得多，又远小于 90° 的翻转。
  /// 调大到 45° → 翻转与噪声分不开，会用错轴补全；
  /// 调小到 5°  → 目标转弯时记忆频繁失效，退化成不做补全。
  double extent_memory_max_axis_diff_rad{0.35};

  /// 记忆尺寸的上限，m。
  ///
  /// ⚠️ 这是给「记最大值」兜底的：一次欠分割（两辆车并成一簇）会**永久**
  ///    放大航迹尺寸，因为最大值只增不减。上限把损失框住。
  ///    取 6.0：大于园区里任何合法目标（车 4.4 m），小到能拦住明显的并簇。
  ///    ⚠️ 它拦不住 5 m 量级的欠分割 —— 那一档只能靠 S2 的聚类容差，
  ///       写在这里是为了下一个人不要以为有了上限就安全了。
  double max_extent_m{6.0};

  /// 两条航迹靠得比它还近就并成一条，m。
  ///
  /// ⚠️ **重复航迹是一个独立于 ID 跳变的缺陷**：规划会把一个目标当成两个
  ///    障碍物去做碰撞检查。它同时也是"ID 在跳"的来源 —— 两条航迹都活着，
  ///    评测逐帧在它们之间摇摆，看起来像跳变，其实是**并存**。
  ///
  /// 取 1.0，两边的余量都是实测出来的：
  ///   · 实测重复航迹的间距**中位 0.07 m、最大 0.44 m**（31/678 帧次）；
  ///   · 本 ODD 里两个真值目标的最小间距是**车与行人的 1.75 m**。
  /// 1.0 比最大重复远 2.3 倍、比最小真实间距近 1.75 倍，两侧都不贴边。
  /// 调大到 2.0 → 车与行人会被并成一个目标（灾难性：漏掉一个人）；
  /// 调小到 0.5 → 那 0.44 m 的一批留下来，重复照旧。
  double merge_distance_m{1.0};

  /// 被另一条已确认航迹挡住视线时，未命中最多滑行多少帧。
  ///
  /// ⚠️ 这条规则解决一个**实测钉死的设计冲突**（2026-08-12）：CP-P5-B 第 6 条
  ///    要求「遮挡后 ID 保持」，而实测遮挡持续 0.5–1.8 s（车 4.4 m 扫过视线），
  ///    删除窗口 max_misses=0.5 s —— 任何真实遮挡都会删航迹换 ID。
  ///    解法是多目标跟踪的标准做法：预测位置被**另一条已确认航迹**的盒子
  ///    挡住视线时，未命中不计入删除计数（目标在别人身后，看不见 ≠ 消失），
  ///    航迹按恒速模型在遮挡后面滑行，照常发布 —— 规划**应当**知道
  ///    车后面有个行人。
  ///
  /// 取 30（3 s @ 10 Hz）：本 ODD 最长的可信遮挡 ~2 s（自车静止、视线转得慢）
  /// 加 50% 余量。滑行是**有上限的**：没有它，一个在遮挡后面真的离开的目标
  /// 会留下永生幽灵 —— 与 max_misses 防幽灵是同一个理由，只是这里的
  /// "看不见"有一个可解释的原因，所以允许更久。
  /// 调小到 5 → 与不做此规则无异；调大到 100 → 幽灵在遮挡后面活 10 s。
  int max_occluded_misses{30};

  /// 两条航迹的速度差超过它就**不**合并，m/s。
  ///
  /// 位置近但速度截然不同 ⟹ 是两个恰好擦身而过的目标，不是重复。
  /// 取 1.0：实测两个目标的速度差 5.2 m/s（车 −4.0、行人 +1.2），
  /// 而同一目标的两条航迹速度几乎相同。
  ///
  /// ⚠️ **只在两条都已确认时才判这一条。** 新建的航迹初速恒为 0
  ///    （见 Update 里建航迹那段），拿它去比一条 −4 m/s 的确认航迹，
  ///    速度差 4.0 恒超门限 ⟹ **重复航迹永远合不掉**，而这恰恰是
  ///    重复最常见的来源。这个坑在实现前就想到了，写在这里免得被"简化"掉。
  double merge_speed_mps{1.0};

  /// 补全/重锚单次位移的物理上限，m。
  ///
  /// ⚠️ 这道闸守着一个实测抓到的**关联虫洞**（P6-S0，2026-08-12）：
  ///    补全/重锚把「观测与记忆的尺寸差」换算成位置修正，前提是**同一个
  ///    目标露出了更多（或被挡住了一截）**。而墙沿/杆件碎片的 L-Shape
  ///    拟合帧间剧变（0.03 m ↔ 6 m），重锚位移可达 3 m —— 恰好把 5.9 m
  ///    外**另一个**碎片"拉进"卡方门限：两个静止碎片被焊成一条带
  ///    9–12 m/s 假速度的航迹，再借遮挡滑行横穿地图，在自车车道内变成
  ///    6.00×N 的幻影（CP-P5-B 车道内虚警的真身，bag 逐帧钉死）。
  ///
  /// 上限 = ODD 最大目标（车 4.4 m）从"看不见"到"全露出"的几何极限
  /// 4.4/2 = 2.2。真实目标最大的合法修正是"只见车头 1.8 → 露出侧面
  /// 4.4"= 1.3 m（余量 1.7×）；被遮挡到只剩一角的补全 ≤ 4.4/2 也在内。
  /// 超限的那一半（补全或重锚）退出，没被补偿的尺寸差原样留在新息里、
  /// 直面卡方门限 —— 5.9 m 的跳变自然被拒。
  /// 调大到 3.5 → 墙沿虫洞重新打开；调小到 1.0 → 车侧面刚露出的合法
  /// 重锚（1.3 m）被拒，退化回 ID 在 17–22 m 反复切换的老毛病。
  double anchor_shift_max_m{2.2};

  /// 遮挡滑行的速度准入上限，m/s。
  ///
  /// ⚠️ 滑行是在**外推**状态。一条速度超出 ODD 物理上限的航迹，其状态
  ///    必然已经是坏的（园区里没有任何东西跑这么快）—— 外推它就是在
  ///    制造幽灵。实测（P6-S0）：被墙沿碎片焊出的 11.9 m/s 航迹靠滑行
  ///    免死 3 s、飞越 35 m 横穿自车车道；速度合法的正主（行人 1.2、
  ///    NPC 车 4.0）完全不受影响。
  /// 取 8.33 = ODD 设计最高车速 30 km/h（SPEC §2）。调大失去拦截力；
  /// 调小到 3 → 正常行驶的车一被遮挡就按普通未命中倒计时，遮挡 >0.5 s
  /// 必换 ID —— 恰好退化回滑行规则要解决的那个原始设计冲突。
  double coast_max_speed_mps{8.33};
};

/// 一次检测（S2 聚类 + S3 拟合的产物）。
struct Detection
{
  /// 位置（x, y）—— 本项目里用 L-Shape 的矩形中心。
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};
  /// 长轴方向，`[0, π)`。⚠️ 是**轴向**不是朝向，见 lshape_fit.hpp。
  double yaw_rad{0.0};
  double length_m{0.0};
  double width_m{0.0};
  double height_m{0.0};
};

/// 一条航迹。
struct Track
{
  std::uint32_t id{0};

  /// 状态 (px, py, vx, vy)。
  Eigen::Vector4d state{Eigen::Vector4d::Zero()};
  Eigen::Matrix4d covariance{Eigen::Matrix4d::Identity()};

  /// 最近一次关联上的检测的轴向。
  double yaw_rad{0.0};

  /// 长/宽：**轴向一致的前提下已观测到的最大值**，不是最近一帧的值。
  ///
  /// ⚠️ 这是本层最反直觉的一点，理由见 `TrackerParams::extent_memory_*`
  ///    与文件头「为什么滤包围盒中心是错的」。轴向翻转时会**重置**成当帧值。
  double length_m{0.0};
  double width_m{0.0};
  /// 高度：**最近一帧**的值，不做记忆（它不参与中心补全）。
  double height_m{0.0};

  /// 消歧之后的**车头朝向**，`[−π, π)`。
  ///
  /// ⚠️ 只有 `heading_resolved` 为 true 时才有意义。为 false 时目标太慢，
  /// 朝向**物理上无解** —— 下游应当退回用 `yaw_rad`（轴向）+ 不确定性。
  double heading_rad{0.0};
  bool heading_resolved{false};

  int hits{0};  ///< 累计命中次数（**不要求连续**）
  int consecutive_misses{0};
  /// 被遮挡状态下的连续未命中（不计入 consecutive_misses，另设上限）。
  int occluded_misses{0};
  bool confirmed{false};  ///< 累计命中达到 confirm_hits

  Eigen::Vector2d position() const { return state.head<2>(); }
  Eigen::Vector2d velocity() const { return state.tail<2>(); }
};

/// 多目标跟踪器。**有状态**，逐帧调用 `Update`。
class Tracker
{
public:
  explicit Tracker(const TrackerParams & params = TrackerParams{});

  /// 推进一帧。
  ///
  /// @param detections 本帧的检测。**允许为空**（那正是"目标闪烁"时的常态，
  ///                   此时所有航迹只做预测，靠 max_misses 决定去留）。
  /// @param dt_s       距上一帧的时间，s。必须为正。
  /// @param sensor_position 传感器在**与 detections 相同的坐标系**里的位置。
  ///                   用来判断"看不见的那半截在哪一侧"，见文件头。
  ///                   ⚠️ **故意不给默认值**：默认成原点在 map 系里就是
  ///                   "传感器在地图原点"，那会让补全方向系统性地错，
  ///                   而结果看起来完全合理 —— 这种错误最难发现。
  /// @throws std::invalid_argument dt 非正或非有限，或检测含非有限值。
  void Update(
    const std::vector<Detection> & detections, double dt_s,
    const Eigen::Vector2d & sensor_position);

  /// 全部航迹（含未确认的）。
  const std::vector<Track> & tracks() const { return tracks_; }

  /// 只要**已确认**的航迹 —— 这才是该发给下游的东西。
  ///
  /// ⚠️ 未确认的航迹**不要发**：它们可能只是噪点簇，而下游（规划）
  /// 会对每一个障碍物做碰撞检查，虚警的代价是车无故刹停。
  std::vector<Track> ConfirmedTracks() const;

  /// 把检测改写成与航迹**同一种轴向说法**（差 90° 就交换长宽并旋转 90°）。
  ///
  /// ⚠️ L-Shape 把较大的范围叫 `length`，这个约定与目标本身无关 ——
  ///    正对时可见范围 1.80 × 0.08，"长轴"指的其实是车**宽**。
  ///    而 `a×b @ ψ` 与 `b×a @ ψ+90°` **是同一个盒子**，所以这不是猜，
  ///    是换说法。不做的话，目标由远及近露出侧面的那一帧轴向翻转 ⟹
  ///    既不补全也不重锚 ⟹ 新息含整跳 ⟹ 判不进门 ⟹ **ID 跳变**。
  ///
  /// @param detection 本帧检测
  /// @param track     提供轴向约定的航迹
  /// @return 与 `track.yaw_rad` 同向的等价描述；本来就同向时原样返回
  Detection AlignedDetection(const Detection & detection, const Track & track) const;

  /// 检测的轴向与航迹的是否算"同一个轴"（差值折到 `[0, π/2]` 再比）。
  ///
  /// 公开是为了让测试能直接验这道闸 —— 它是尺寸记忆能否安全启用的前提。
  bool AxesConsistent(double detection_yaw_rad, double track_yaw_rad) const;

  /// 把观测到的盒子中心补全到航迹已知的尺寸。
  ///
  /// 看不见的那部分**一定藏在背离传感器的一侧**（雷达只打得到朝向自己的面），
  /// 所以沿盒子自身的两个轴，各朝背离传感器的方向挪半个缺口。
  /// 轴向不一致或没有缺口时原样返回。
  ///
  /// @param detection       本帧检测
  /// @param track           提供已知尺寸的航迹
  /// @param sensor_position 传感器位置，与 detection 同系
  /// @return 补全后的中心
  Eigen::Vector2d CompletedCenter(
    const Detection & detection, const Track & track,
    const Eigen::Vector2d & sensor_position) const;

  /// 观测**大于**航迹已知尺寸时，航迹位置需要挪多少才算按新盒子重新锚定。
  ///
  /// 与 `CompletedCenter` 是同一件事的两半：一个补观测、一个补航迹。
  /// 近边没动而盒子变长时，中心的位移是**重新锚定**不是运动 ——
  /// 不补的话卡尔曼会把它读成速度（实测一帧 4.4 m/s，真值 4.0）。
  ///
  /// @param detection       本帧检测
  /// @param track           航迹
  /// @param sensor_position 传感器位置，与 detection 同系
  /// @return 应当加到航迹位置上的偏移；轴向不一致或没有增量时为零
  Eigen::Vector2d TrackAnchorShift(
    const Detection & detection, const Track & track,
    const Eigen::Vector2d & sensor_position) const;

private:
  /// 把一个盒子中心沿「背离传感器」的方向挪半个缺口。`CompletedCenter` 与
  /// `TrackAnchorShift` 共用它 —— 两者只是缺口的符号不同。
  static Eigen::Vector2d AnchorOffset(
    double yaw_rad, double deficit_long_m, double deficit_lat_m, const Eigen::Vector2d & box_center,
    const Eigen::Vector2d & sensor_position);

  /// 航迹的预测位置是否被**另一条已确认航迹**挡住了视线。
  ///
  /// 公开是为了测试能直接验几何（线段-OBB 相交，slab 法精确解）。
  ///
  /// @param track           被查的航迹
  /// @param sensor_position 传感器位置，与航迹同系
  /// @return 视线被挡为 true
  bool IsOccludedByAnotherTrack(const Track & track, const Eigen::Vector2d & sensor_position) const;

  /// 把落在同一个目标上的重复航迹并成一条。
  ///
  /// 留命中多的那条（证据更足）；打平留 id 小的（更老，下游的历史更长）。
  void MergeDuplicateTracks();

  void Predict(double dt_s);
  void Associate(
    const std::vector<Detection> & detections, const Eigen::Vector2d & sensor_position,
    std::vector<int> * assignment);
  void ApplyUpdate(
    const Detection & detection, const Eigen::Vector2d & sensor_position, Track * track);
  void ResolveHeading(Track * track) const;

  TrackerParams params_;
  std::vector<Track> tracks_;
  std::uint32_t next_id_{1};
};

}  // namespace ads_perception

#endif  // ADS_PERCEPTION__TRACKER_HPP_
