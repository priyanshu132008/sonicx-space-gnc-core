// pil_server.cpp -- Standalone CM4 Processor-in-the-Loop TCP server (V4 6-DOF).
//
// The flight-ready half of the PIL link: a standalone, multi-threaded TCP
// server that runs the SAME flight stack as the SHM SIL bridge
// (aero-sim/sil_shm_bridge.cpp) -- the ONNX AiGncPolicy actor
// (models/ppo_actor_v4.onnx, RAW 13-obs in -> 3 actions out) + the Rust
// `#![no_std]` Gatekeeper (rust-gatekeeper/, C-ABI staticlib) -- ON THE TARGET
// HARDWARE (Raspberry Pi CM4, ARM64). The host Python side
// (aero-sim/run_sil_orbital.py --tcp / src/test_monte_carlo_v4_6dof.py --tcp)
// ships each control tick's raw 6-DOF state over TCP and gets the safe
// gimbal command back, so the certification gauntlet exercises the EXACT
// silicon that will fly.
//
// WIRE PROTOCOL (fixed-size binary frames, little-endian; x86-64 + ARM64 are
// both LE so no byte-swapping -- see run_sil_orbital.py, the mirror client):
//   request  (104 B): double state[13]   -- the raw 6-DOF obs
//     [0]alt [1]downrange [2]vr [3]vt [4]pitch(rad) [5]prate [6]mass [7]q
//     [8]roll(rad) [9]sideslip(rad) [10]roll_rate [11]yaw_rate [12]aoa(rad)
//   response (33 B): double action[3] + uint8 fts_latched + uint64 latency_ns
//     action[3] = SAFE clamped TVC gimbal [pitch, yaw, roll] DEG (+-5; frozen
//                 to [0,0,0] once the flight session latches),
//     fts_latched = the Range-Safety / IEEE-754 abort latch for THIS session,
//     latency_ns  = THIS tick's ONNX + gatekeeper execution time on the CM4
//                   (clock_gettime(CLOCK_MONOTONIC) around the compute,
//                   exclusive of network transit).
// One connection == one FLIGHT SESSION. The per-tick ping-pong runs over the
// persistent socket; TCP_NODELAY on both ends (latency > batching here).
//
// THREADING: std::thread per connection (the Monte Carlo gauntlet runs 8
// concurrent spawn workers; also trivially handles the 10k-flight stress
// test's serialized connection churn). Session state is per-connection:
//   * ONNX: one global OrtEnv + one global OrtSession (ORT sessions are
//     thread-safe for concurrent Run); each connection thread owns a private
//     OrtIoBinding + OrtValues wrapping its FIXED stack buffers -> the run
//     path is ZERO-ALLOCATION (fill obs_in, RunWithBinding, read action_out).
//   * RustGatekeeper: the staticlib keeps its state in ONE global static
//     (core-only no_std has no heap objects to hand out), so ALL gate calls
//     and resets are serialized under g_gate_mutex. The V4 gate's persistence
//     window is 1 tick (each process_telemetry call is self-contained), so a
//     session thread that gets `true` marks ITS OWN per-connection `latched`
//     flag (the sticky behavior, kept C++-side) and calls gatekeeper_reset()
//     under the SAME mutex before releasing it -- a latched flight can never
//     poison a concurrent healthy session. gatekeeper_reset() at session
//     start gives every flight a fresh gate (the /avx_sil_mc_{id}
//     fresh-bridge-per-flight discipline, over the network).
//
// Build on the CM4 (see build_cm4.sh -- it wraps exactly this):
//   g++ -std=c++17 -O2 -pthread -I onnxruntime \
//     pil_server.cpp rust-gatekeeper/target/release/libsonicx_gatekeeper_embedded.a \
//     -L $ORT_ROOT/lib -lonnxruntime -o pil_server
// Run (from the deploy_cm4/ directory):
//   ./pil_server [model_path] [--port N]        # defaults: models/ppo_actor_v4.onnx :5555

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

#include <mutex>
#include <thread>
#include <atomic>
#include <vector>
#include <string>

// send() flags: MSG_NOSIGNAL is Linux; macOS/BSD use SO_NOSIGPIPE (set on the
// socket) or a global SIGPIPE ignore. We do BOTH: portable flag + SIG_IGN.
#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

#include <onnxruntime_c_api.h>

// ---- RustGatekeeper #![no_std] staticlib C ABI (identical to the SIL bridge) --
// V4: process_telemetry ingests the raw 13-obs state + a 3-action IN/OUT gimbal
// command (deg) and returns the sticky FTS latch (true = abort latched).
extern "C" {
bool    process_telemetry(const double* state, double* action);
bool    gatekeeper_fault_latched(void);
uint8_t gatekeeper_fault_reason(void);
void    gatekeeper_reset(void);
}

// ---------------------------------------------------------------------------
// Constants (mirrored from sil_shm_bridge.cpp / run_solar_orbital.py).
// ---------------------------------------------------------------------------
#define GIMBAL_LIMIT_DEG   5.0     // TVC mechanical envelope (deg) -- 3 axes
#define PIL_REQ_BYTES      (13 * 8)          // 13 doubles in
#define PIL_RSP_BYTES      (3 * 8 + 1 + 8)   // 3 doubles + fts + latency out
#define PIL_DEFAULT_PORT   5555

// ---------------------------------------------------------------------------
// ORT C API -- one global env/session, per-connection zero-alloc run state.
// ---------------------------------------------------------------------------
static const OrtApi* g_ort = nullptr;

static inline bool ort_ok(OrtStatus* s, const char* ctx) {
  if (s == nullptr) return true;
  fprintf(stderr, "[pil] ORT error (%s): %s\n", ctx, g_ort->GetErrorMessage(s));
  g_ort->ReleaseStatus(s);
  return false;
}

/// Global ONNX state: created ONCE in main (session load incl. arena setup),
/// shared by every connection thread (ORT sessions are thread-safe for Run).
static struct {
  OrtEnv*            env = nullptr;
  OrtSession*        session = nullptr;
  OrtSessionOptions* opts = nullptr;
  char               input_name[64] = {0};
  char               output_name[64] = {0};
  bool               ready = false;
} g_ort_global;

static bool ort_global_init(const char* model_path) {
  g_ort = OrtGetApiBase()->GetApi(ORT_API_VERSION);
  if (!g_ort) { fprintf(stderr, "[pil] GetApi failed\n"); return false; }
  if (!ort_ok(g_ort->CreateEnv(ORT_LOGGING_LEVEL_WARNING, "pil", &g_ort_global.env),
              "CreateEnv"))
    return false;
  if (!ort_ok(g_ort->CreateSessionOptions(&g_ort_global.opts), "CreateSessionOptions"))
    return false;
  // Pre-allocate the CPU arena (embedded mandate: bounded, no per-run growth).
  if (!ort_ok(g_ort->EnableCpuMemArena(g_ort_global.opts), "EnableCpuMemArena"))
    return false;
  if (!ort_ok(g_ort->CreateSession(g_ort_global.env, model_path, g_ort_global.opts,
                                    &g_ort_global.session), "CreateSession"))
    return false;
  OrtAllocator* allocator = nullptr;
  if (!ort_ok(g_ort->GetAllocatorWithDefaultOptions(&allocator), "GetAllocator"))
    return false;
  char* in_name = nullptr;
  char* out_name = nullptr;
  if (!ort_ok(g_ort->SessionGetInputName(g_ort_global.session, 0, allocator, &in_name),
              "SessionGetInputName"))
    return false;
  if (!ort_ok(g_ort->SessionGetOutputName(g_ort_global.session, 0, allocator,
                                           &out_name), "SessionGetOutputName")) {
    allocator->Free(allocator, in_name); return false;
  }
  snprintf(g_ort_global.input_name, sizeof(g_ort_global.input_name), "%s", in_name);
  snprintf(g_ort_global.output_name, sizeof(g_ort_global.output_name), "%s", out_name);
  allocator->Free(allocator, in_name);
  allocator->Free(allocator, out_name);
  g_ort_global.ready = true;
  return true;
}

/// Per-connection ONNX run state: OrtValues wrapping the thread's FIXED stack
/// buffers, pre-bound to a private IoBinding. All allocation happens at session
/// setup; the per-tick run path is ZERO-ALLOCATION (the mandate).
struct ThreadOrt {
  OrtMemoryInfo* mem_info = nullptr;
  OrtIoBinding*  io_binding = nullptr;
  OrtValue*      input_ort = nullptr;
  OrtValue*      output_ort = nullptr;
  float          obs_in[13]    = {0};
  float          action_out[3] = {0};
  bool           ready = false;
};

static bool thread_ort_init(ThreadOrt* t) {
  if (!g_ort_global.ready) return false;
  if (!ort_ok(g_ort->CreateCpuMemoryInfo(OrtDeviceAllocator, OrtMemTypeDefault,
                                         &t->mem_info), "CreateCpuMemoryInfo"))
    return false;
  int64_t in_shape[2]  = {1, 13};
  int64_t out_shape[2] = {1, 3};
  if (!ort_ok(g_ort->CreateTensorWithDataAsOrtValue(
        t->mem_info, t->obs_in, sizeof(t->obs_in), in_shape, 2,
        ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT, &t->input_ort), "CreateInputValue"))
    return false;
  if (!ort_ok(g_ort->CreateTensorWithDataAsOrtValue(
        t->mem_info, t->action_out, sizeof(t->action_out), out_shape, 2,
        ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT, &t->output_ort), "CreateOutputValue"))
    return false;
  if (!ort_ok(g_ort->CreateIoBinding(g_ort_global.session, &t->io_binding),
              "CreateIoBinding"))
    return false;
  if (!ort_ok(g_ort->BindInput(t->io_binding, g_ort_global.input_name, t->input_ort),
              "BindInput"))
    return false;
  if (!ort_ok(g_ort->BindOutput(t->io_binding, g_ort_global.output_name,
                                t->output_ort), "BindOutput"))
    return false;
  t->ready = true;
  return true;
}

/// ZERO-ALLOCATION run path: fill obs_in, RunWithBinding, read action_out[3].
/// Returns true on success; on failure the caller latches a safe zero action.
static bool thread_ort_run(ThreadOrt* t) {
  if (!t->ready) return false;
  if (!ort_ok(g_ort->RunWithBinding(g_ort_global.session, nullptr, t->io_binding),
              "RunWithBinding")) {
    t->action_out[0] = 0.0f;
    t->action_out[1] = 0.0f;
    t->action_out[2] = 0.0f;
    return false;
  }
  return true;
}

static void thread_ort_destroy(ThreadOrt* t) {
  if (t->io_binding) g_ort->ReleaseIoBinding(t->io_binding);
  if (t->output_ort) g_ort->ReleaseValue(t->output_ort);
  if (t->input_ort)  g_ort->ReleaseValue(t->input_ort);
  if (t->mem_info)   g_ort->ReleaseMemoryInfo(t->mem_info);
  memset(t, 0, sizeof(*t));
}

// ---------------------------------------------------------------------------
// Socket helpers.
// ---------------------------------------------------------------------------

/// recv() until exactly n bytes arrive (TCP may fragment frames). Returns false
/// on EOF / error.
static bool recv_exact(int fd, void* buf, size_t n) {
  uint8_t* p = (uint8_t*)buf;
  size_t got = 0;
  while (got < n) {
    ssize_t r = recv(fd, p + got, n - got, 0);
    if (r == 0) return false;               // peer closed
    if (r < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    got += (size_t)r;
  }
  return true;
}

/// send() all n bytes (handles partial writes + EINTR). Returns false on error.
static bool send_all(int fd, const void* buf, size_t n) {
  const uint8_t* p = (const uint8_t*)buf;
  size_t sent = 0;
  while (sent < n) {
    ssize_t r = send(fd, p + sent, n - sent, MSG_NOSIGNAL);
    if (r < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    sent += (size_t)r;
  }
  return true;
}

static inline uint64_t mono_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (uint64_t)t.tv_sec * 1000000000ull + (uint64_t)t.tv_nsec;
}

// ---------------------------------------------------------------------------
// Session statistics (aggregated across the whole server, printed on exit).
// ---------------------------------------------------------------------------
static std::atomic<uint64_t> g_sessions(0);     // completed flight sessions
static std::atomic<uint64_t> g_frames(0);       // control ticks served
static std::atomic<uint64_t> g_latches(0);      // sessions that latched FTS
static std::atomic<uint64_t> g_lat_min_ns(UINT64_MAX);
static std::atomic<uint64_t> g_lat_max_ns(0);
static std::atomic<uint64_t> g_lat_sum_ns(0);
static std::atomic<int>      g_active(0);

static void note_frame_latency(uint64_t ns) {
  g_frames.fetch_add(1, std::memory_order_relaxed);
  g_lat_sum_ns.fetch_add(ns, std::memory_order_relaxed);
  uint64_t prev_min = g_lat_min_ns.load(std::memory_order_relaxed);
  while (ns < prev_min &&
         !g_lat_min_ns.compare_exchange_weak(prev_min, ns,
                                              std::memory_order_relaxed)) {}
  uint64_t prev_max = g_lat_max_ns.load(std::memory_order_relaxed);
  while (ns > prev_max &&
         !g_lat_max_ns.compare_exchange_weak(prev_max, ns,
                                              std::memory_order_relaxed)) {}
}

// ---------------------------------------------------------------------------
// The gatekeeper session mutex + per-connection handler.
// ---------------------------------------------------------------------------

/// The staticlib keeps its sticky latch in ONE global static, so every
/// process_telemetry / gatekeeper_reset call is serialized under this mutex.
/// V4 persistence=1 makes each call self-contained; the owning thread resets
/// the global latch (still under the mutex) immediately after ITS session
/// latches, so concurrent healthy sessions are never poisoned.
static std::mutex g_gate_mutex;

/// One connection == one flight session (per-tick ping-pong until EOF/err).
static void handle_session(int fd) {
  struct sockaddr_in peer;
  socklen_t peer_len = sizeof(peer);
  char peer_ip[64] = "?";
  if (getpeername(fd, (struct sockaddr*)&peer, &peer_len) == 0)
    inet_ntop(AF_INET, &peer.sin_addr, peer_ip, sizeof(peer_ip));

  int one = 1;
  setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));  // latency > batching

  // Fresh gate for every flight session (the /avx_sil_mc_{id} discipline).
  {
    std::lock_guard<std::mutex> lk(g_gate_mutex);
    gatekeeper_reset();
  }
  bool latched = false;      // sticky, PER-SESSION (this flight's FTS)
  uint8_t latched_reason = 0;
  uint64_t sess_frames = 0;

  ThreadOrt ort;
  bool ort_okay = thread_ort_init(&ort);
  if (!ort_okay) {
    fprintf(stderr, "[pil] session %s: per-connection ONNX init FAILED -- "
                    "refusing to fly (fail-closed)\n", peer_ip);
    close(fd);
    return;
  }

  // --- per-tick ping-pong: 104 B in -> ONNX + Rust gate -> 33 B out ---------
  double state[13];
  uint8_t rsp[PIL_RSP_BYTES];

  for (;;) {
    if (!recv_exact(fd, state, PIL_REQ_BYTES)) break;  // EOF / error: flight over

    uint64_t t0 = mono_ns();

    double action_io[3] = {0.0, 0.0, 0.0};
    if (!latched) {
      // AiGncPolicy: raw 13-obs -> 3 normalized action fractions (zero-alloc run
      // path). An inference fault degrades to a safe neutral command.
      for (int i = 0; i < 13; ++i) ort.obs_in[i] = (float)state[i];
      if (!thread_ort_run(&ort)) {
        ort.action_out[0] = 0.0f;
        ort.action_out[1] = 0.0f;
        ort.action_out[2] = 0.0f;
      }
      // AI command in degrees (action[i] ~[-1,1] * +-5 deg envelope, 3 axes).
      action_io[0] = (double)ort.action_out[0] * GIMBAL_LIMIT_DEG;
      action_io[1] = (double)ort.action_out[1] * GIMBAL_LIMIT_DEG;
      action_io[2] = (double)ort.action_out[2] * GIMBAL_LIMIT_DEG;
      // RustGatekeeper: state[13] IN + action[3] IN/OUT (AI cmd deg -> safe
      // clamped deg). Serialized: one global gate static.
      bool fts;
      {
        std::lock_guard<std::mutex> lk(g_gate_mutex);
        fts = process_telemetry(state, action_io);
        if (fts) {
          // THIS flight latches; clear the GLOBAL sticky immediately (under the
          // same mutex) so a concurrent healthy session never sees it. The
          // stickiness lives in our per-session `latched` flag from here on.
          latched_reason = gatekeeper_fault_reason();
          gatekeeper_reset();
        }
      }
      if (fts) {
        latched = true;
        g_latches.fetch_add(1, std::memory_order_relaxed);
        action_io[0] = 0.0;  // gate already froze it; keep the wire explicit
        action_io[1] = 0.0;
        action_io[2] = 0.0;
      }
    } else {
      // Post-latch: sticky safe state -- neutral nozzle, FTS flag stays set.
      // (No inference: the flight is over; the client breaks on this frame.)
      action_io[0] = action_io[1] = action_io[2] = 0.0;
    }

    uint64_t t1 = mono_ns();
    uint64_t lat_ns = t1 - t0;
    note_frame_latency(lat_ns);
    ++sess_frames;

    // 33 B response: action[3] (deg) + fts (u8) + execution latency (u64 ns).
    // memcpy packing (no pointer casts): the u8 at offset 24 makes the u64 at
    // offset 25 unaligned -- direct casts are UB on strict-alignment targets.
    memcpy(rsp + 0, action_io, sizeof(action_io));           // 3 doubles
    uint8_t fts_u8 = latched ? 1u : 0u;
    memcpy(rsp + 24, &fts_u8, 1);
    memcpy(rsp + 25, &lat_ns, 8);
    if (!send_all(fd, rsp, sizeof(rsp))) break;
  }

  thread_ort_destroy(&ort);
  close(fd);
  g_sessions.fetch_add(1, std::memory_order_relaxed);
  fprintf(stderr, "[pil] session %-15s done: ticks=%llu latched=%s%s\n", peer_ip,
          (unsigned long long)sess_frames, latched ? "YES" : "no",
          latched ? " (FTS)" : "");
  (void)latched_reason;  // reserved for the ground station's fault-code mapping
}

// ---------------------------------------------------------------------------
// Main: listen, accept, spawn a thread per flight session.
// ---------------------------------------------------------------------------
static std::atomic<bool> g_run(true);

static void on_sigint(int) { g_run.store(false); }

int main(int argc, char** argv) {
  const char* model_path = "models/ppo_actor_v4.onnx";
  int port = PIL_DEFAULT_PORT;

  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--port") == 0 && i + 1 < argc)
      port = (int)strtol(argv[++i], nullptr, 10);
    else model_path = argv[i];
  }

  // SIGINT: stop accepting, print the aggregate summary, exit (detached session
  // threads die with the process).
  struct sigaction sa;
  memset(&sa, 0, sizeof(sa));
  sa.sa_handler = on_sigint;
  sigaction(SIGINT, &sa, nullptr);
  signal(SIGPIPE, SIG_IGN);  // send() reports EPIPE instead of killing us

  if (!ort_global_init(model_path)) {
    fprintf(stderr, "[pil] FATAL: ONNX init failed for %s\n", model_path);
    return 1;
  }
  fprintf(stderr, "[pil] ONNX loaded: %s  in=%s out=%s\n", model_path,
          g_ort_global.input_name, g_ort_global.output_name);

  int lfd = socket(AF_INET, SOCK_STREAM, 0);
  if (lfd < 0) { perror("socket"); return 1; }
  int one = 1;
  setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons((uint16_t)port);
  if (bind(lfd, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
    perror("bind"); return 1;
  }
  if (listen(lfd, 32) != 0) { perror("listen"); return 1; }

  // Accept loop with a poll timeout so SIGINT shuts us down cleanly.
  fprintf(stderr, "[pil] SonicX CM4 Processor-in-the-Loop server listening on "
                  "0.0.0.0:%d (13-obs in -> ONNX + Rust gate -> 3-action + FTS "
                  "+ latency out; one connection per flight)\n", port);

  while (g_run.load()) {
    struct pollfd pfd = { lfd, POLLIN, 0 };
    int pr = poll(&pfd, 1, 500);
    if (pr <= 0) continue;                    // timeout -> re-check g_run
    int cfd = accept(lfd, nullptr, nullptr);
    if (cfd < 0) {
      if (errno == EINTR) continue;
      perror("accept");
      continue;
    }
    g_active.fetch_add(1, std::memory_order_relaxed);
    // std::thread per connection (the 8 concurrent MC workers + the 10k-flight
    // stress churn). Detached: sessions are self-terminating (client closes).
    std::thread(handle_session, cfd).detach();
  }

  close(lfd);
  uint64_t frames = g_frames.load();
  uint64_t sessions = g_sessions.load();
  fprintf(stderr, "\n[pil] shutdown summary: sessions=%llu frames=%llu "
          "latched=%llu\n",
          (unsigned long long)sessions, (unsigned long long)frames,
          (unsigned long long)g_latches.load());
  if (frames > 0) {
    uint64_t mn = g_lat_min_ns.load();
    if (mn == UINT64_MAX) mn = 0;
    fprintf(stderr, "[pil] on-target compute latency (ns): min=%llu med~=%llu "
            "max=%llu  over %llu frames\n",
            (unsigned long long)mn,
            (unsigned long long)(g_lat_sum_ns.load() / frames),
            (unsigned long long)g_lat_max_ns.load(),
            (unsigned long long)frames);
  }
  return 0;
}