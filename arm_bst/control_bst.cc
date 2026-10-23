// control_bst.cc
// Brazo de 4 GDL: pesa 5 cubos con el sensor F/T de la muneca, guarda (peso, posicion)
// en un Arbol Binario de Busqueda y te deja elegir en un menu cual cubo levantar.
//
// Requiere Gazebo corriendo con:   gz sim -r worlds/brazo_cubos.sdf
// Ejecutar desde la carpeta del proyecto:  ./build/control_bst [catalogo.csv] [pesos.csv]
//
// Archivos:  datos/cubos.csv (catalogo, se lee al iniciar)   datos/pesos.csv (pesos medidos)

#include <gz/msgs/double.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/wrench.pb.h>
#include <gz/transport/Node.hh>

#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "bst.h"
#include "cubo.h"
#include "sorts.h"

using namespace std::chrono_literals;

// ---------------- Constantes del brazo y de la escena (deben coincidir con el SDF) ----------------
constexpr double PI = 3.14159265358979;
constexpr double GRAV = 9.81;              // = <gravity> del SDF
constexpr double L1 = 0.40;                // hombro -> codo
constexpr double L2 = 0.35;                // codo -> muneca
constexpr double H0 = 0.18;                // altura del hombro

constexpr double Z_CUBO = 0.11;            // centro del cubo sobre el pedestal (tapa del pedestal = 0.08)
constexpr double LARGO_DEDOS = 0.12;       // muneca -> punta de los dedos (mano hacia abajo)
// Gripper tipo PALA: cada dedo tiene un labio en la punta que debe quedar JUSTO DEBAJO del cubo.
constexpr double ESPESOR_LABIO = 0.015;    // = alto del labio en el SDF
constexpr double HOLGURA_LABIO = 0.002;    // la cara superior del labio queda 2 mm bajo la base del cubo
// Punta (cara inferior del labio) a 0.063; muneca a 0.183. Los labios bajan por los costados del
// pedestal angosto (2.4 cm), cierran y quedan debajo del cubo (6 cm). Al subir lo cargan.
constexpr double Z_AGARRE = (Z_CUBO - 0.03) - HOLGURA_LABIO - ESPESOR_LABIO + LARGO_DEDOS;  // 0.183
constexpr double Z_SEGURO = 0.36;          // altura para viajar y levantar (dedos a >= 0.24 m)
constexpr bool   USAR_ATTACH = false;      // true: suelda el cubo a la mano al cerrar (DetachableJoint); false: solo la jaula
constexpr double GRIPPER_ABIERTO = 0.0;
constexpr double GRIPPER_CERRADO = 0.023;  // placas a 32 mm del centro (cubo a 30): 2 mm de holgura, SIN apretar
                                           // (el SDF ademas tiene tope mecanico en 0.024)

const char *RUTA_CATALOGO = "datos/cubos.csv";
const char *RUTA_PESOS = "datos/pesos.csv";
std::string rutaPesosActual = RUTA_PESOS;   // la fija main() (puede venir como argumento)

// Auxiliares propios (en este programa no se usa la libreria <algorithm>)
inline int    maxInt(int a, int b) { return a > b ? a : b; }
inline double maxDbl(double a, double b) { return a > b ? a : b; }
inline double acotar(double x, double lo, double hi) { return x < lo ? lo : (x > hi ? hi : x); }

// Catalogo de cubos (donde esta cada cubo en el SDF + peso nominal). Se llena al iniciar
// leyendo datos/cubos.csv con cargarCatalogo().
std::vector<InfoCubo> INFO;
inline int nCubos() { return static_cast<int>(INFO.size()); }

using Q = std::array<double, 4>;
using Vec3 = std::array<double, 3>;

const char *color(const Cubo &c) { return INFO[c.id - 1].color.c_str(); }

// ---------------- Control del brazo ----------------
class Brazo
{
public:
  Brazo()
  {
    for (const auto &j : {"joint1", "joint2", "joint3", "joint4", "finger_l_joint", "finger_r_joint"})
      pubs_[j] = node_.Advertise<gz::msgs::Double>(std::string("/model/arm/joint/") + j + "/0/cmd_pos");
    if (USAR_ATTACH)
    for (int n = 1; n <= nCubos(); ++n)
    {
      attachPub_[n] = node_.Advertise<gz::msgs::Empty>("/grasp/cubo" + std::to_string(n) + "/attach");
      detachPub_[n] = node_.Advertise<gz::msgs::Empty>("/grasp/cubo" + std::to_string(n) + "/detach");
    }
    node_.Subscribe("/wrist_ft", &Brazo::OnWrench, this);
    std::this_thread::sleep_for(1s);
    // Los DetachableJoint arrancan con los cubos atados a la mano: soltarlos todos (varias veces por si
    // el primer mensaje se pierde mientras se descubren los topics).
    if (USAR_ATTACH)
      for (int k = 0; k < 3; ++k)
      {
        for (int n = 1; n <= nCubos(); ++n) Detach(n);
        std::this_thread::sleep_for(200ms);
      }
  }

  // Suelda / suelta el cubo n a la mano (equivalente a gazebo_grasp_fix en Gazebo Harmonic)
  void Attach(int n) { gz::msgs::Empty m; attachPub_[n].Publish(m); }
  void Detach(int n) { gz::msgs::Empty m; detachPub_[n].Publish(m); }

  void MoverJoint(const std::string &joint, double rad)
  {
    gz::msgs::Double msg;
    msg.set_data(rad);
    pubs_[joint].Publish(msg);
  }

  void Gripper(double pos)
  {
    MoverJoint("finger_l_joint", pos);
    MoverJoint("finger_r_joint", pos);
    grip_ = pos;
  }

  // Abre/cierra el gripper de forma GRADUAL. Un comando escalon hace que los dedos choquen
  // con el cubo a ~0.25 m/s y lo avientan fuera del pedestal; con la rampa llegan despacio y
  // los dedos se acercan despacio y se detienen con holgura.
  void MoverGripper(double destino, double seg)
  {
    int pasos = maxInt(1, static_cast<int>(seg * 50));
    double ini = grip_;
    for (int k = 1; k <= pasos; ++k)
    {
      double s = static_cast<double>(k) / pasos;
      s = s * s * (3.0 - 2.0 * s);
      Gripper(ini + (destino - ini) * s);
      std::this_thread::sleep_for(20ms);
    }
  }

  // Cinematica inversa plana: muneca en (r, zw), mano apuntando hacia abajo
  bool IK(double q1, double r, double zw, Q &q) const
  {
    double dz = zw - H0;
    double c3 = (r * r + dz * dz - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
    if (c3 < -1.0 || c3 > 1.0) return false;
    double q3 = std::acos(c3);
    double q2 = std::atan2(r, dz) - std::atan2(L2 * std::sin(q3), L1 + L2 * std::cos(q3));
    double q4 = PI - (q2 + q3);
    if (std::fabs(q2) > 1.8 || std::fabs(q3) > 2.4 || std::fabs(q4) > 2.8) return false;
    q = {q1, q2, q3, q4};
    return true;
  }

  // Interpolacion en articulaciones con arranque/frenado suave
  void IrJoints(const Q &destino, double seg)
  {
    int pasos = maxInt(1, static_cast<int>(seg * 50));
    Q ini = q_;
    for (int k = 1; k <= pasos; ++k)
    {
      double s = static_cast<double>(k) / pasos;
      s = s * s * (3.0 - 2.0 * s);
      Q q;
      for (int i = 0; i < 4; ++i) q[i] = ini[i] + (destino[i] - ini[i]) * s;
      Enviar(q);
      std::this_thread::sleep_for(20ms);
    }
    q_ = destino;
  }

  // Lleva la muneca a (q1, r, zw). Si seg <= 0 calcula la duracion segun lo que hay que girar.
  bool IrAPunta(double q1, double r, double zw, double seg = -1.0)
  {
    Q q;
    if (!IK(q1, r, zw, q))
    {
      std::cerr << "Punto inalcanzable: r=" << r << " z=" << zw << "\n";
      return false;
    }
    if (seg <= 0.0)
    {
      double dmax = 0.0;
      for (int i = 0; i < 4; ++i) dmax = maxDbl(dmax, std::fabs(q[i] - q_[i]));
      seg = acotar(dmax / 0.5, 1.5, 6.0);
    }
    IrJoints(q, seg);
    r_ = r;
    zw_ = zw;
    enPunta_ = true;
    return true;
  }

  // Sube/baja la muneca en linea recta (q1 y r constantes)
  bool LineaVertical(double zw_final, double seg)
  {
    int pasos = maxInt(1, static_cast<int>(seg * 50));
    double z0 = zw_;
    for (int k = 1; k <= pasos; ++k)
    {
      double s = static_cast<double>(k) / pasos;
      s = s * s * (3.0 - 2.0 * s);
      double z = z0 + (zw_final - z0) * s;
      Q q;
      if (!IK(q_[0], r_, z, q))
      {
        std::cerr << "Punto inalcanzable en la linea vertical (z=" << z << ")\n";
        return false;
      }
      Enviar(q);
      q_ = q;
      zw_ = z;
      std::this_thread::sleep_for(20ms);
    }
    return true;
  }

  // Viaja a la vertical de un cubo SIEMPRE a altura segura (no roza otros cubos)
  bool ViajarA(double q1, double r)
  {
    if (enPunta_ && zw_ < Z_SEGURO - 1e-3 && !LineaVertical(Z_SEGURO, 1.5)) return false;
    return IrAPunta(q1, r, Z_SEGURO);
  }

  // Baja, cierra el gripper y sube con el cubo hasta la altura segura.
  // ANTES de bajar mide la TARA: la fuerza con la mano vacia (gripper cerrado) en la misma pose
  // en la que luego se pesa el cubo. Restar ambos vectores deja solo el peso del cubo y cancela
  // cualquier fuerza constante que reporte el sensor (peso del brazo, offsets, etc.).
  bool TomarCubo(const Cubo &c)
  {
    Gripper(GRIPPER_ABIERTO);
    if (!ViajarA(c.q1, c.r)) return false;
    std::this_thread::sleep_for(800ms);          // que joint1 termine de asentarse antes de bajar
    MoverGripper(GRIPPER_CERRADO, 1.0);          // misma configuracion de dedos que al cargar el cubo
    std::this_thread::sleep_for(2000ms);         // que se calme la vibracion
    MedirTara(1.5);
    MoverGripper(GRIPPER_ABIERTO, 1.0);
    std::this_thread::sleep_for(500ms);
    if (!LineaVertical(Z_AGARRE, 2.0)) return false;
    std::this_thread::sleep_for(700ms);
    MoverGripper(GRIPPER_CERRADO, 2.0);          // cierre gradual
    std::this_thread::sleep_for(1000ms);         // los dedos terminan de asentarse
    if (USAR_ATTACH)
    {
      Attach(c.id);
      cuboAgarrado_ = c.id;
      std::this_thread::sleep_for(500ms);
    }
    return LineaVertical(Z_SEGURO, 2.0);
  }

  // Devuelve el cubo a su pedestal (el brazo sigue en la misma vertical) y sube
  bool SoltarCubo()
  {
    if (!LineaVertical(Z_AGARRE, 2.0)) return false;
    std::this_thread::sleep_for(1s);
    if (cuboAgarrado_ > 0)                       // primero se suelta el cubo, luego se abren los dedos
    {
      for (int k = 0; k < 2; ++k) { Detach(cuboAgarrado_); std::this_thread::sleep_for(150ms); }
      cuboAgarrado_ = 0;
      std::this_thread::sleep_for(500ms);
    }
    MoverGripper(GRIPPER_ABIERTO, 1.0);          // apertura gradual
    std::this_thread::sleep_for(500ms);
    return LineaVertical(Z_SEGURO, 1.5);
  }

  void Home()
  {
    if (cuboAgarrado_ > 0) { Detach(cuboAgarrado_); cuboAgarrado_ = 0; }   // nunca dejar un cubo soldado
    Gripper(GRIPPER_ABIERTO);
    if (enPunta_ && zw_ < Z_SEGURO - 1e-3) LineaVertical(Z_SEGURO, 1.5);
    IrJoints({0, 0, 0, 0}, 5.0);
    enPunta_ = false;
  }

  // Promedia el VECTOR fuerza durante 'segundos' (la vibracion, que cambia de signo, se cancela).
  // Devuelve false si no llegaron datos.
  bool PromediarFuerza(double segundos, Vec3 &out)
  {
    double sx = 0.0, sy = 0.0, sz = 0.0;
    int n = 0;
    auto fin = std::chrono::steady_clock::now() +
               std::chrono::milliseconds(static_cast<int>(segundos * 1000));
    while (std::chrono::steady_clock::now() < fin)
    {
      {
        std::lock_guard<std::mutex> lock(mtx_);
        if (hayDato_) { sx += fx_; sy += fy_; sz += fz_; ++n; }
      }
      std::this_thread::sleep_for(10ms);
    }
    if (n == 0) return false;
    out = {sx / n, sy / n, sz / n};
    return true;
  }

  static double Norma(const Vec3 &v) { return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

  // Mide la fuerza con la mano VACIA (la tara de este cubo)
  void MedirTara(double segundos)
  {
    taraValida_ = PromediarFuerza(segundos, tara_);
    if (taraValida_)
      std::cout << "    Tara (mano vacia): |F0| = " << std::fixed << std::setprecision(3)
                << Norma(tara_) << " N\n";
  }

  // Masa del cubo en gramos = |F_con_cubo - F_tara| / g   (NAN si no hay datos)
  double MedirMasaGramos(double segundos)
  {
    Vec3 f1;
    if (!taraValida_ || !PromediarFuerza(segundos, f1)) return NAN;
    Vec3 d = {f1[0] - tara_[0], f1[1] - tara_[1], f1[2] - tara_[2]};
    std::cout << "    |F con cubo| = " << std::fixed << std::setprecision(3) << Norma(f1)
              << " N,  |F - tara| = " << Norma(d) << " N\n";
    return Norma(d) / GRAV * 1000.0;
  }

private:
  void Enviar(const Q &q)
  {
    MoverJoint("joint1", q[0]);
    MoverJoint("joint2", q[1]);
    MoverJoint("joint3", q[2]);
    MoverJoint("joint4", q[3]);
  }

  void OnWrench(const gz::msgs::Wrench &msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    fx_ = msg.force().x(); fy_ = msg.force().y(); fz_ = msg.force().z();
    hayDato_ = true;
  }

  gz::transport::Node node_;
  std::map<std::string, gz::transport::Node::Publisher> pubs_;
  std::map<int, gz::transport::Node::Publisher> attachPub_, detachPub_;
  int cuboAgarrado_ = 0;  // id del cubo soldado a la mano (0 = ninguno)
  std::mutex mtx_;
  double fx_ = 0.0, fy_ = 0.0, fz_ = 0.0;
  Vec3 tara_ = {0, 0, 0};
  bool taraValida_ = false;
  bool hayDato_ = false;
  Q q_ = {0, 0, 0, 0};
  double r_ = 0.0, zw_ = 0.0;
  bool enPunta_ = false;  // false = en la pose "home" (brazo vertical)
  double grip_ = 0.0;     // ultima posicion ordenada a los dedos
};

// ---------------- Presentacion ----------------
void imprimirTabla(const BST<Cubo> &arbol)
{
  if (arbol.empty()) { std::cout << "  (no hay cubos registrados, usa la opcion 1)\n"; return; }
  std::cout << "\n  Pos  Cubo  Color      Peso medido   Real     Error    q1      r\n"
            << "  ---------------------------------------------------------------------\n";
  int pos = 0;
  arbol.inorderApply([&](const Cubo &c) {
    float real = INFO[c.id - 1].pesoReal;
    std::cout << "  " << std::setw(3) << pos++ << "  " << std::setw(4) << c.id << "  "
              << std::left << std::setw(9) << color(c) << std::right << "  "
              << std::fixed << std::setprecision(1) << std::setw(9) << c.peso << " g "
              << std::setw(6) << real << " g " << std::showpos << std::setw(6) << (c.peso - real)
              << std::noshowpos << " g " << std::setprecision(2) << std::setw(6) << c.q1
              << std::setw(6) << c.r << "\n";
  });
  std::cout << "\n";
}

// ---------------- Lectura de teclado ----------------
bool leerLinea(const std::string &prompt, std::string &s)
{
  std::cout << prompt << std::flush;
  return static_cast<bool>(std::getline(std::cin, s));
}

bool leerNumero(const std::string &prompt, double &out)
{
  std::string s;
  if (!leerLinea(prompt, s)) return false;
  try { out = std::stod(s); } catch (...) { out = NAN; }
  return true;
}

// ---------------- Archivos ----------------
bool guardarPesosEnArchivo(const std::vector<Cubo> &lecturas, const std::string &ruta)
{
  std::string error;
  if (!guardarPesos(ruta, lecturas, error))
  {
    std::cerr << "No se guardaron los pesos: " << error << "\n";
    return false;
  }
  std::cout << "Pesos guardados en '" << ruta << "' (" << lecturas.size() << " cubos)\n";
  return true;
}

// Lee los pesos del archivo y reconstruye el arbol sin mover el brazo
bool cargarPesosDeArchivo(BST<Cubo> &arbol, std::vector<Cubo> &lecturas, const std::string &ruta)
{
  std::vector<Cubo> leidos;
  std::string error;
  if (!cargarPesos(ruta, INFO, leidos, error))
  {
    std::cerr << "No se pudieron cargar los pesos: " << error << "\n";
    return false;   // el arbol anterior no se toca
  }
  arbol.clear();
  for (const Cubo &c : leidos) arbol.add(c);
  lecturas = leidos;
  std::cout << "Cargados " << arbol.size() << " cubos desde '" << ruta << "'. Altura del arbol: "
            << arbol.height() << "\n";
  imprimirTabla(arbol);
  return true;
}

// Ordena las lecturas con Merge Sort, lo compara con el recorrido inorden del BST y permite
// buscar un peso con Busqueda Binaria
void ordenarYBuscar(const BST<Cubo> &arbol, const std::vector<Cubo> &lecturas)
{
  if (lecturas.empty()) { std::cout << "No hay lecturas (usa la opcion 1 o 12).\n"; return; }
  Sorts<Cubo> sorts;
  std::vector<Cubo> orden = lecturas;
  sorts.ordenaMerge(orden);

  std::cout << "\nLecturas ordenadas con Merge Sort:\n  ";
  for (const Cubo &c : orden) std::cout << c << "  ";
  std::cout << "\n";

  std::vector<Cubo> inorden;
  arbol.inorderApply([&](const Cubo &c) { inorden.push_back(c); });
  bool igual = inorden.size() == orden.size();
  for (size_t i = 0; igual && i < orden.size(); ++i) igual = orden[i].id == inorden[i].id;
  std::cout << "Coincide con el recorrido inorden del BST: " << (igual ? "si" : "NO") << "\n";

  double g;
  if (!leerNumero("Peso a buscar con Busqueda Binaria (g): ", g) || std::isnan(g))
  {
    std::cout << "Peso no valido.\n";
    return;
  }
  Cubo clave{0, static_cast<float>(g), 0.0, 0.0};
  int pos = sorts.busqBinaria(orden, clave);
  const Cubo *enArbol = arbol.find(clave);
  if (pos >= 0)
    std::cout << "Busqueda binaria: posicion " << pos << " -> cubo " << orden[pos].id << " ("
              << color(orden[pos]) << ", " << std::fixed << std::setprecision(1) << orden[pos].peso << " g)\n";
  else
    std::cout << "Busqueda binaria: no hay un cubo de " << g << " g (tolerancia " << TOL_G << " g)\n";
  std::cout << "BST (find):       " << (enArbol ? "cubo " + std::to_string(enArbol->id) : std::string("no encontrado"))
            << "\n";
}

// ---------------- Acciones ----------------
// Llena el arbol Y el vector 'lecturas' (en orden de id, SIN ordenar: es la entrada de Merge Sort)
bool escanear(Brazo &brazo, BST<Cubo> &arbol, std::vector<Cubo> &lecturas)
{
  arbol.clear();
  lecturas.clear();
  std::cout << "\n== Escaneo: tomo cada cubo, lo peso y lo devuelvo a su pedestal ==\n";
  for (int i = 0; i < nCubos(); ++i)
  {
    Cubo c{i + 1, 0.0f, INFO[i].q1, INFO[i].r};
    std::cout << "Cubo " << c.id << " (" << color(c) << ")\n";
    if (!brazo.TomarCubo(c)) return false;
    std::this_thread::sleep_for(2500ms);  // que el brazo se estabilice antes de medir
    double g = brazo.MedirMasaGramos(1.5);
    if (!brazo.SoltarCubo()) return false;
    if (std::isnan(g))
    {
      std::cerr << "    No llegan datos de /wrist_ft, se omite este cubo\n";
      continue;
    }
    c.peso = static_cast<float>(g);
    std::cout << "    Peso medido: " << std::fixed << std::setprecision(1) << g << " g\n";
    arbol.add(c);
    lecturas.push_back(c);
  }
  std::cout << "\nArbol construido: " << arbol.size() << " cubos, altura " << arbol.height() << "\n";
  imprimirTabla(arbol);
  guardarPesosEnArchivo(lecturas, rutaPesosActual);
  return true;
}

void levantar(Brazo &brazo, const Cubo &c)
{
  std::cout << "\nLevantando cubo " << c.id << " (" << color(c) << ", peso guardado "
            << std::fixed << std::setprecision(1) << c.peso << " g) en q1="
            << std::setprecision(2) << c.q1 << " r=" << c.r << "\n"
            << std::setprecision(1);
  if (!brazo.TomarCubo(c)) { std::cerr << "No se pudo levantar el cubo.\n"; return; }
  std::this_thread::sleep_for(2500ms);
  double g = brazo.MedirMasaGramos(1.5);
  if (!std::isnan(g))
    std::cout << "    Peso ahora: " << g << " g   (guardado: " << c.peso << " g)\n";
  std::cout << "Cubo en el aire. Presiona Enter para devolverlo a su pedestal... ";
  std::string basura;
  std::getline(std::cin, basura);
  brazo.SoltarCubo();
  std::cout << "Listo.\n";
}

void menu()
{
  std::cout << "\n=================== MENU ===================\n"
               " 1) Re-escanear y pesar todos los cubos\n"
               " 2) Listar cubos ordenados por peso (inorden)\n"
               " 3) Ver el arbol BST y sus recorridos (visit)\n"
               " 4) Levantar un cubo por ID\n"
               " 5) Levantar un cubo por peso (busqueda en el BST)\n"
               " 6) Levantar el MAS LIGERO  (minimo del BST)\n"
               " 7) Levantar el MAS PESADO  (maximo del BST)\n"
               " 8) Listar cubos en un rango de peso\n"
               " 9) Ver nivel y ancestros de un cubo en el arbol\n"
               "10) Ordenar lecturas (Merge Sort) y buscar peso (Busqueda Binaria)\n"
               "11) Guardar los pesos en archivo (datos/pesos.csv)\n"
               "12) Cargar los pesos desde archivo (sin escanear)\n"
               " 0) Salir\n"
               "============================================\n";
}

int main(int argc, char **argv)
{
  // Rutas opcionales:  control_bst [catalogo.csv] [pesos.csv]
  std::string rutaCatalogo = argc > 1 ? argv[1] : RUTA_CATALOGO;
  std::string rutaPesos = argc > 2 ? argv[2] : RUTA_PESOS;
  rutaPesosActual = rutaPesos;

  // 1) Leer el catalogo de cubos del archivo ANTES de mover el brazo
  std::string error;
  if (!cargarCatalogo(rutaCatalogo, INFO, error))
  {
    std::cerr << "No se pudo cargar el catalogo: " << error << "\n"
              << "Ejecuta el programa desde la carpeta del proyecto o pasa la ruta como argumento.\n";
    return 1;
  }
  std::cout << "Catalogo cargado de '" << rutaCatalogo << "': " << nCubos() << " cubos\n";

  Brazo brazo;
  BST<Cubo> arbol;
  std::vector<Cubo> lecturas;   // pesos en orden de id, sin ordenar (entrada de Merge Sort)

  // 2) Si hay pesos guardados, ofrecer cargarlos en vez de escanear (~1.5 min)
  bool cargado = false;
  if (std::ifstream(rutaPesos).good())
  {
    std::string r;
    if (leerLinea("Se encontro '" + rutaPesos + "'. Cargar esos pesos en lugar de escanear? (s/n): ", r) &&
        !r.empty() && (r[0] == 's' || r[0] == 'S'))
      cargado = cargarPesosDeArchivo(arbol, lecturas, rutaPesos);
  }
  if (!cargado)
  {
    std::cout << "Brazo conectado. Escaneando los " << nCubos() << " cubos (aprox. 1.5 min)...\n";
    if (!escanear(brazo, arbol, lecturas)) { std::cerr << "El escaneo fallo.\n"; return 1; }
  }

  std::string linea;
  while (true)
  {
    menu();
    double op;
    if (!leerNumero("Opcion: ", op)) break;  // Ctrl+D
    if (std::isnan(op)) { std::cout << "Opcion no valida.\n"; continue; }

    int opcion = static_cast<int>(op);
    if (opcion == 0) break;

    switch (opcion)
    {
      case 1:
        if (!escanear(brazo, arbol, lecturas)) std::cerr << "El escaneo fallo.\n";
        break;

      case 2:
        imprimirTabla(arbol);
        break;

      case 3:
        std::cout << "\nArbol (raiz a la izquierda, mayores arriba):\n";
        arbol.printTree();
        std::cout << "\nRecorridos (preorder, inorder, postorder, nivel por nivel):\n"
                  << arbol.visit() << "\n";
        std::cout << "\ntamano = " << arbol.size() << ", altura = " << arbol.height() << "\n";
        break;

      case 4:
      {
        double id;
        if (!leerNumero("ID del cubo (1-" + std::to_string(nCubos()) + "): ", id)) break;
        int i = static_cast<int>(id);
        const Cubo *c = std::isnan(id) ? nullptr
                        : arbol.findIf([i](const Cubo &x) { return x.id == i; });
        if (c) levantar(brazo, *c);
        else std::cout << "No hay un cubo con ese ID registrado.\n";
        break;
      }

      case 5:
      {
        double g;
        if (!leerNumero("Peso buscado en gramos: ", g) || std::isnan(g)) { std::cout << "Peso no valido.\n"; break; }
        Cubo clave{0, static_cast<float>(g), 0.0, 0.0};
        const Cubo *c = arbol.find(clave);
        if (c)
        {
          std::cout << "Encontrado en el BST: cubo " << c->id << " (" << color(*c) << ", "
                    << std::fixed << std::setprecision(1) << c->peso << " g)\n";
          levantar(brazo, *c);
        }
        else
        {
          const Cubo *cerca = nullptr;
          arbol.inorderApply([&](const Cubo &x) {
            if (!cerca || std::fabs(x.peso - g) < std::fabs(cerca->peso - g)) cerca = &x;
          });
          std::cout << "No hay ningun cubo de " << g << " g (tolerancia " << TOL_G << " g).\n";
          if (cerca)
          {
            std::cout << "El mas cercano es el cubo " << cerca->id << " (" << color(*cerca) << ", "
                      << std::fixed << std::setprecision(1) << cerca->peso << " g). Levantarlo? (s/n): ";
            std::getline(std::cin, linea);
            if (!linea.empty() && (linea[0] == 's' || linea[0] == 'S')) levantar(brazo, *cerca);
          }
        }
        break;
      }

      case 6:
      {
        const Cubo *c = arbol.minimum();
        if (c) levantar(brazo, *c); else std::cout << "Arbol vacio.\n";
        break;
      }

      case 7:
      {
        const Cubo *c = arbol.maximum();
        if (c) levantar(brazo, *c); else std::cout << "Arbol vacio.\n";
        break;
      }

      case 8:
      {
        double lo, hi;
        if (!leerNumero("Peso minimo (g): ", lo) || !leerNumero("Peso maximo (g): ", hi)) break;
        if (std::isnan(lo) || std::isnan(hi)) { std::cout << "Valores no validos.\n"; break; }
        std::cout << "Cubos entre " << lo << " y " << hi << " g:\n";
        int n = 0;
        arbol.range(Cubo{0, static_cast<float>(lo), 0, 0}, Cubo{0, static_cast<float>(hi), 0, 0},
                    [&](const Cubo &c) {
                      std::cout << "  Cubo " << c.id << " (" << color(c) << "): "
                                << std::fixed << std::setprecision(1) << c.peso << " g\n";
                      ++n;
                    });
        if (n == 0) std::cout << "  (ninguno)\n";
        break;
      }

      case 9:
      {
        double id;
        if (!leerNumero("ID del cubo (1-" + std::to_string(nCubos()) + "): ", id)) break;
        int i = static_cast<int>(id);
        const Cubo *c = std::isnan(id) ? nullptr
                        : arbol.findIf([i](const Cubo &x) { return x.id == i; });
        if (c)
          std::cout << "Cubo " << *c << " (" << color(*c) << ")\n"
                    << "  Nivel en el arbol: " << arbol.whatlevelamI(*c) << "\n"
                    << "  Ancestros (de la raiz hacia abajo): " << arbol.ancestors(*c) << "\n";
        else
          std::cout << "No hay un cubo con ese ID registrado.\n";
        break;
      }

      case 10:
        ordenarYBuscar(arbol, lecturas);
        break;

      case 11:
        if (lecturas.empty()) std::cout << "No hay pesos que guardar.\n";
        else guardarPesosEnArchivo(lecturas, rutaPesos);
        break;

      case 12:
        cargarPesosDeArchivo(arbol, lecturas, rutaPesos);
        break;

      default:
        std::cout << "Opcion no valida.\n";
    }
  }

  std::cout << "Regresando a casa...\n";
  brazo.Home();
  return 0;
}
