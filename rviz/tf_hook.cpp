// rviz2 など tf2 を使うプロセスに LD_PRELOAD で入れ、tf2::BufferCore に入った TF（setTransform）と、時刻を指定して
// 引いた TF（lookupTransform の 3 引数の版）を、環境変数 TF_HOOK_LOG のファイルに 1 行ずつ書く（dev_log.md の 106）。
// 元の関数を呼んで、その結果を書くだけ（値は変えない）。TF_HOOK_LOG が無ければ何も書かない。
//   S <seq> <steady ns> <buffer> <stamp ns> <parent> <child> <static> <入った> x y z qx qy qz qw
//   L <引く前の seq> <引いた後の seq> <steady ns> <buffer> <target> <source> <時刻 ns（0 は最新）> <結果の stamp ns> x y z qx qy qz qw
//   E <引く前の seq> <引いた後の seq> <steady ns> <buffer> <target> <source> <時刻 ns>（例外。そのまま投げ直す）
// seq は setTransform が終わるごとに 1 増える。lookupTransform の前後の seq で、引いたときに入っていた TF が分かる。
// tf2 0.25（Humble）の x86-64 の ABI に合わせてある（メンバ関数の this と戻り値の置き場所は、普通の関数の引数と同じ並び）。
#include <dlfcn.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/buffer_core.h>

namespace
{
using TransformStamped = geometry_msgs::msg::TransformStamped;
using SetFn = bool (*)(tf2::BufferCore *, const TransformStamped &, const std::string &, bool);
using LookupFn = TransformStamped (*)(
  const tf2::BufferCore *, const std::string &, const std::string &, const tf2::TimePoint &);

std::atomic<uint64_t> g_seq{0};
std::mutex g_mutex;
FILE * g_log = nullptr;
bool g_opened = false;

FILE * logFile()  // g_mutex を持って呼ぶ
{
  if (!g_opened) {
    g_opened = true;
    const char * path = std::getenv("TF_HOOK_LOG");
    if (path != nullptr && *path != '\0') {
      g_log = std::fopen(path, "w");
      if (g_log == nullptr) {
        std::fprintf(stderr, "tf_hook: cannot open %s\n", path);
      }
    }
  }
  return g_log;
}

template<typename Fn>
Fn original(const char * mangled)
{
  // libtf2 は plugin（dlopen、RTLD_LOCAL）から読まれることがあり、そのときは RTLD_NEXT で見つからないので libtf2.so から引く
  void * lib = dlopen("libtf2.so", RTLD_LAZY | RTLD_NOLOAD);
  if (lib == nullptr) {
    lib = dlopen("libtf2.so", RTLD_LAZY);
  }
  void * p = lib != nullptr ? dlsym(lib, mangled) : nullptr;
  if (p == nullptr) {
    std::fprintf(stderr, "tf_hook: %s not found\n", mangled);
    std::abort();
  }
  return reinterpret_cast<Fn>(p);
}

long long steadyNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

long long stampNs(const builtin_interfaces::msg::Time & t)
{
  return static_cast<long long>(t.sec) * 1000000000LL + t.nanosec;
}

void writeValue(FILE * f, const TransformStamped & tf)
{
  const auto & p = tf.transform.translation;
  const auto & q = tf.transform.rotation;
  std::fprintf(
    f, " %.17g %.17g %.17g %.17g %.17g %.17g %.17g\n", p.x, p.y, p.z, q.x, q.y, q.z, q.w);
  std::fflush(f);
}
}  // namespace

bool tf2::BufferCore::setTransform(
  const TransformStamped & transform, const std::string & authority, bool is_static)
{
  static const SetFn orig = original<SetFn>(
    "_ZN3tf210BufferCore12setTransformERKN13geometry_msgs3msg17TransformStamped_ISaIvEEERKNSt7__"
    "cxx1112basic_stringIcSt11char_traitsIcESaIcEEEb");
  const bool inserted = orig(this, transform, authority, is_static);
  std::lock_guard<std::mutex> lock(g_mutex);
  const uint64_t seq = ++g_seq;
  if (FILE * f = logFile()) {
    std::fprintf(
      f, "S %llu %lld %p %lld %s %s %d %d", static_cast<unsigned long long>(seq), steadyNs(),
      static_cast<const void *>(this), stampNs(transform.header.stamp),
      transform.header.frame_id.c_str(), transform.child_frame_id.c_str(), is_static ? 1 : 0,
      inserted ? 1 : 0);
    writeValue(f, transform);
  }
  return inserted;
}

geometry_msgs::msg::TransformStamped tf2::BufferCore::lookupTransform(
  const std::string & target_frame, const std::string & source_frame, const TimePoint & time) const
{
  static const LookupFn orig = original<LookupFn>(
    "_ZNK3tf210BufferCore15lookupTransformERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES"
    "8_RKNSt6chrono10time_pointINS9_3_V212system_clockENS9_8durationIlSt5ratioILl1ELl1000000000EEE"
    "EEE");
  const unsigned long long before = g_seq.load();
  const long long t_ns = time.time_since_epoch().count();
  try {
    TransformStamped out = orig(this, target_frame, source_frame, time);
    const unsigned long long after = g_seq.load();
    std::lock_guard<std::mutex> lock(g_mutex);
    if (FILE * f = logFile()) {
      std::fprintf(
        f, "L %llu %llu %lld %p %s %s %lld %lld", before, after, steadyNs(),
        static_cast<const void *>(this), target_frame.c_str(), source_frame.c_str(), t_ns,
        stampNs(out.header.stamp));
      writeValue(f, out);
    }
    return out;
  } catch (const std::exception &) {
    const unsigned long long after = g_seq.load();
    std::lock_guard<std::mutex> lock(g_mutex);
    if (FILE * f = logFile()) {
      std::fprintf(
        f, "E %llu %llu %lld %p %s %s %lld\n", before, after, steadyNs(),
        static_cast<const void *>(this), target_frame.c_str(), source_frame.c_str(), t_ns);
      std::fflush(f);
    }
    throw;
  }
}
