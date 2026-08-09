/*
 * AsvJsonBridge — jembatan ArduPilot SITL (backend JSON) <-> Ignition Gazebo Fortress.
 *
 * Peran (menggantikan plugin resmi ardupilot_gazebo yang butuh Garden/Harmonic):
 *   1. Menerima paket servo biner dari SITL di UDP :9002
 *      (uint16 magic, uint16 frame_rate, uint32 frame_count, uint16 pwm[16|32]).
 *   2. Memetakan PWM kanal servo kiri/kanan -> gaya dorong (N) dan mem-publish
 *      ke topic cmd_thrust milik plugin Thruster bawaan Gazebo.
 *   3. Membalas state fisika sebagai JSON (posisi/kecepatan NED, attitude,
 *      gyro & accel_body FRD) sekali per langkah fisika -> SITL lockstep.
 *   4. Opsional: gaya arus air konstan (N, kerangka dunia ENU) dari SDF.
 *
 * Konvensi kerangka: Gazebo = ENU dunia / FLU badan; ArduPilot = NED / FRD.
 */

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/math/Matrix3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/msgs/double.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

using namespace ignition;

namespace asv
{

struct ServoPacket16
{
  uint16_t magic;       // 18458
  uint16_t frame_rate;
  uint32_t frame_count;
  uint16_t pwm[16];
} __attribute__((packed));

class AsvJsonBridge
    : public gazebo::System,
      public gazebo::ISystemConfigure,
      public gazebo::ISystemPreUpdate,
      public gazebo::ISystemPostUpdate
{
  // ---------- konfigurasi ----------
  private: int port{9002};
  private: int idxLeft{0};    // indeks array pwm utk thruster kiri  (SERVO1 -> 0)
  private: int idxRight{2};   // indeks array pwm utk thruster kanan (SERVO3 -> 2)
  private: double maxThrust{6.0};   // N pada simpangan PWM penuh
  private: double pwmTrim{1500.0};
  private: double pwmRange{400.0};  // 1500 +/- 400 -> 1100..1900
  private: double deadband{20.0};   // us di sekitar trim dianggap nol
  private: math::Vector3d currentForce{0, 0, 0};  // gaya arus konstan (N, ENU)
  private: std::string leftTopic, rightTopic;

  // ---------- hidrostatika sederhana (pengganti plugin Buoyancy yang tidak
  // stabil utk kapal permukaan): 4 titik apung per lambung di sudut-sudutnya.
  // Tiap titik menyumbang gaya angkat sebanding fraksi terendam kolom lambung
  // di atasnya + redaman vertikal -> keseimbangan heave/roll/pitch yang kokoh.
  private: bool hydrostatics{true};
  private: double hullLen{0.75}, hullWid{0.10}, hullHt{0.10};
  private: double hullY{0.175};     // jarak sumbu lambung dari sumbu kapal
  private: double hullTopZ{0.0};    // z atas lambung pada kerangka badan
  private: double waterLevel{0.0};  // z permukaan air (dunia)
  private: double waterRho{1000.0};
  private: double heaveDampPt{15.0};  // N/(m/s) per titik

  // ---------- state ----------
  private: gazebo::Model model{gazebo::kNullEntity};
  private: gazebo::Link link{gazebo::kNullEntity};
  private: transport::Node node;
  private: transport::Node::Publisher pubLeft, pubRight;
  private: int sock{-1};
  private: sockaddr_in remote{};
  private: bool haveRemote{false};
  private: bool pendingReply{false};
  private: uint32_t lastFrameCount{0};
  private: double thrustLeft{0.0}, thrustRight{0.0};
  private: double lastRecvSimTime{-1.0};
  private: double lastSentSimTime{-1.0};
  private: bool armedOnce{false};

  public: void Configure(const gazebo::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         gazebo::EntityComponentManager &_ecm,
                         gazebo::EventManager &) override
  {
    this->model = gazebo::Model(_entity);
    auto sdf = _sdf->Clone();

    const std::string linkName = sdf->Get<std::string>("link_name", "base_link").first;
    this->port      = sdf->Get<int>("port", this->port).first;
    this->idxLeft   = sdf->Get<int>("servo_left_index", this->idxLeft).first;
    this->idxRight  = sdf->Get<int>("servo_right_index", this->idxRight).first;
    this->maxThrust = sdf->Get<double>("max_thrust", this->maxThrust).first;
    this->pwmTrim   = sdf->Get<double>("pwm_trim", this->pwmTrim).first;
    this->pwmRange  = sdf->Get<double>("pwm_range", this->pwmRange).first;
    this->deadband  = sdf->Get<double>("pwm_deadband", this->deadband).first;
    this->currentForce.X(sdf->Get<double>("current_force_x", 0.0).first);
    this->currentForce.Y(sdf->Get<double>("current_force_y", 0.0).first);

    this->hydrostatics = sdf->Get<bool>("hydrostatics", true).first;
    this->hullLen  = sdf->Get<double>("hull_length", this->hullLen).first;
    this->hullWid  = sdf->Get<double>("hull_width", this->hullWid).first;
    this->hullHt   = sdf->Get<double>("hull_height", this->hullHt).first;
    this->hullY    = sdf->Get<double>("hull_offset_y", this->hullY).first;
    this->hullTopZ = sdf->Get<double>("hull_top_z", this->hullTopZ).first;
    this->waterLevel  = sdf->Get<double>("water_level", this->waterLevel).first;
    this->heaveDampPt = sdf->Get<double>("heave_damping_per_point",
                                         this->heaveDampPt).first;

    const std::string ns = sdf->Get<std::string>(
        "namespace", this->model.Name(_ecm)).first;
    const std::string jl = sdf->Get<std::string>(
        "left_joint", "left_prop_joint").first;
    const std::string jr = sdf->Get<std::string>(
        "right_joint", "right_prop_joint").first;
    this->leftTopic  = "/model/" + ns + "/joint/" + jl + "/cmd_thrust";
    this->rightTopic = "/model/" + ns + "/joint/" + jr + "/cmd_thrust";

    auto linkEnt = this->model.LinkByName(_ecm, linkName);
    if (linkEnt == gazebo::kNullEntity)
    {
      ignerr << "[AsvJsonBridge] link '" << linkName << "' tidak ditemukan\n";
      return;
    }
    this->link = gazebo::Link(linkEnt);
    this->link.EnableVelocityChecks(_ecm, true);
    this->link.EnableAccelerationChecks(_ecm, true);

    this->pubLeft  = this->node.Advertise<msgs::Double>(this->leftTopic);
    this->pubRight = this->node.Advertise<msgs::Double>(this->rightTopic);

    this->sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    int one = 1;
    ::setsockopt(this->sock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(this->port));
    if (::bind(this->sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
    {
      ignerr << "[AsvJsonBridge] gagal bind UDP :" << this->port << "\n";
      ::close(this->sock); this->sock = -1;
      return;
    }
    ::fcntl(this->sock, F_SETFL, O_NONBLOCK);

    ignmsg << "[AsvJsonBridge] siap: UDP :" << this->port
           << "  pwm[" << this->idxLeft << "]->" << this->leftTopic
           << "  pwm[" << this->idxRight << "]->" << this->rightTopic
           << "  max_thrust=" << this->maxThrust << " N\n";
  }

  private: double PwmToThrust(uint16_t _pwm) const
  {
    if (_pwm < 800 || _pwm > 2200)  // nilai tak masuk akal -> netral
      return 0.0;
    double d = static_cast<double>(_pwm) - this->pwmTrim;
    if (std::abs(d) < this->deadband)
      return 0.0;
    d = std::max(-this->pwmRange, std::min(this->pwmRange, d));
    return this->maxThrust * d / this->pwmRange;
  }

  public: void PreUpdate(const gazebo::UpdateInfo &_info,
                         gazebo::EntityComponentManager &_ecm) override
  {
    if (this->sock < 0 || _info.paused)
      return;

    const double simTime =
        std::chrono::duration<double>(_info.simTime).count();

    // Kuras socket; pakai paket terbaru. Dukung 16 kanal (40 B) & 32 kanal (72 B).
    uint8_t buf[512];
    sockaddr_in from{};
    socklen_t fromLen = sizeof(from);
    ssize_t n;
    while ((n = ::recvfrom(this->sock, buf, sizeof(buf), 0,
                           reinterpret_cast<sockaddr *>(&from), &fromLen)) > 0)
    {
      if (n < static_cast<ssize_t>(sizeof(ServoPacket16)))
        continue;
      auto *pkt = reinterpret_cast<ServoPacket16 *>(buf);
      if (pkt->magic != 18458 && pkt->magic != 29569)
        continue;
      this->remote = from;
      this->haveRemote = true;
      this->pendingReply = true;
      this->lastRecvSimTime = simTime;
      this->lastFrameCount = pkt->frame_count;
      this->thrustLeft  = PwmToThrust(pkt->pwm[this->idxLeft]);
      this->thrustRight = PwmToThrust(pkt->pwm[this->idxRight]);
      if (!this->armedOnce && (this->thrustLeft != 0 || this->thrustRight != 0))
      {
        this->armedOnce = true;
        ignmsg << "[AsvJsonBridge] thrust pertama: L=" << this->thrustLeft
               << " N  R=" << this->thrustRight << " N\n";
      }
    }

    // Publish thrust hanya setelah SITL pernah terhubung — supaya uji manual
    // (ign topic -p ke cmd_thrust) tidak tertimpa nol oleh jembatan ini.
    if (this->lastRecvSimTime >= 0)
    {
      // Failsafe: SITL diam > 1 s -> matikan thruster.
      if (simTime - this->lastRecvSimTime > 1.0)
        this->thrustLeft = this->thrustRight = 0.0;

      msgs::Double m;
      m.set_data(this->thrustLeft);
      this->pubLeft.Publish(m);
      m.set_data(this->thrustRight);
      this->pubRight.Publish(m);
    }

    if (this->link.Entity() == gazebo::kNullEntity)
      return;

    // Gaya arus air konstan (kerangka dunia).
    if (this->currentForce.SquaredLength() > 0.0)
      this->link.AddWorldForce(_ecm, this->currentForce);

    // Hidrostatika 8 titik (lihat komentar deklarasi anggota).
    if (this->hydrostatics)
    {
      auto poseOpt = this->link.WorldPose(_ecm);
      auto velOpt  = this->link.WorldLinearVelocity(_ecm);
      auto angOpt  = this->link.WorldAngularVelocity(_ecm);
      if (poseOpt && velOpt && angOpt)
      {
        const math::Quaterniond &R = poseOpt->Rot();
        const double halfL = 0.46 * this->hullLen;   // titik agak masuk dari ujung
        const double zBot  = this->hullTopZ - this->hullHt;
        // volume kolom air per titik (1/4 lambung)
        const double vPt = this->hullLen * this->hullWid * this->hullHt / 4.0;
        const double fFull = this->waterRho * 9.80665 * vPt;
        math::Vector3d fTot(0, 0, 0), tTot(0, 0, 0);
        for (int sx = -1; sx <= 1; sx += 2)
          for (int sy = -1; sy <= 1; sy += 2)
            for (int sh = -1; sh <= 1; sh += 2)   // sh: lambung kiri/kanan
            {
              const math::Vector3d pBody(sx * halfL,
                                         sh * this->hullY + sy * this->hullWid / 2.0,
                                         zBot);
              // hanya 4 titik per lambung: pakai sy utk sisi dalam/luar
              const math::Vector3d rW = R.RotateVector(pBody);
              const math::Vector3d pW = poseOpt->Pos() + rW;
              const double depth = this->waterLevel - pW.Z();
              double frac = depth / this->hullHt;
              frac = std::max(0.0, std::min(1.0, frac));
              if (frac <= 0.0)
                continue;
              double fz = fFull * frac;
              const double vz = ((*velOpt) + angOpt->Cross(rW)).Z();
              fz -= this->heaveDampPt * vz;
              const math::Vector3d f(0, 0, fz);
              fTot += f;
              tTot += rW.Cross(f);
            }
        this->link.AddWorldWrench(_ecm, fTot, tTot);
      }
    }
  }

  public: void PostUpdate(const gazebo::UpdateInfo &_info,
                          const gazebo::EntityComponentManager &_ecm) override
  {
    if (this->sock < 0 || _info.paused || !this->pendingReply || !this->haveRemote)
      return;
    const double simTime =
        std::chrono::duration<double>(_info.simTime).count();
    if (simTime <= this->lastSentSimTime)   // timestamp HARUS naik utk SITL
      return;

    auto poseOpt = this->link.WorldPose(_ecm);
    auto velOpt  = this->link.WorldLinearVelocity(_ecm);
    auto angOpt  = this->link.WorldAngularVelocity(_ecm);
    if (!poseOpt || !velOpt || !angOpt)
      return;
    const math::Pose3d &pose = *poseOpt;
    const math::Quaterniond &q = pose.Rot();          // badan FLU -> dunia ENU
    const math::Vector3d aW =
        this->link.WorldLinearAcceleration(_ecm).value_or(math::Vector3d::Zero);

    // Gaya spesifik (pembacaan akselerometer) = a - g, lalu ke badan FLU.
    const math::Vector3d g(0, 0, -9.80665);
    const math::Vector3d fB = q.RotateVectorReverse(aW - g);
    const math::Vector3d wB = q.RotateVectorReverse(*angOpt);

    // FLU -> FRD dan ENU -> NED.
    const double gyro[3]  = { wB.X(), -wB.Y(), -wB.Z() };
    const double accel[3] = { fB.X(), -fB.Y(), -fB.Z() };
    const double posN[3]  = { pose.Pos().Y(),  pose.Pos().X(), -pose.Pos().Z() };
    const double velN[3]  = { velOpt->Y(),     velOpt->X(),    -velOpt->Z() };

    // Attitude NED/FRD:  M' = C * M(q) * D,  C = ENU->NED,  D = FRD->FLU.
    const math::Matrix3d M(q);
    const math::Matrix3d C(0, 1, 0,   1, 0, 0,   0, 0, -1);
    const math::Matrix3d D(1, 0, 0,   0, -1, 0,  0, 0, -1);
    math::Quaterniond qn((C * M * D));

    char json[512];
    int len = std::snprintf(json, sizeof(json),
        "\n{\"timestamp\":%.6f,"
        "\"imu\":{\"gyro\":[%.7f,%.7f,%.7f],"
        "\"accel_body\":[%.7f,%.7f,%.7f]},"
        "\"position\":[%.7f,%.7f,%.7f],"
        "\"attitude\":[%.7f,%.7f,%.7f],"
        "\"velocity\":[%.7f,%.7f,%.7f]}\n",
        simTime,
        gyro[0], gyro[1], gyro[2],
        accel[0], accel[1], accel[2],
        posN[0], posN[1], posN[2],
        qn.Roll(), qn.Pitch(), qn.Yaw(),
        velN[0], velN[1], velN[2]);

    ::sendto(this->sock, json, len, 0,
             reinterpret_cast<sockaddr *>(&this->remote), sizeof(this->remote));
    this->lastSentSimTime = simTime;
    this->pendingReply = false;
  }

  public: ~AsvJsonBridge() override
  {
    if (this->sock >= 0)
      ::close(this->sock);
  }
};

}  // namespace asv

IGNITION_ADD_PLUGIN(asv::AsvJsonBridge,
                    ignition::gazebo::System,
                    asv::AsvJsonBridge::ISystemConfigure,
                    asv::AsvJsonBridge::ISystemPreUpdate,
                    asv::AsvJsonBridge::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(asv::AsvJsonBridge, "asv::AsvJsonBridge")
